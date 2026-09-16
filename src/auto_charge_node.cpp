/**
 * auto_charge_node.cpp
 *
 * 服务机器人视觉引导自主回充。
 *
 * 用底部相机识别充电桩上的 AR 码，得到机器人相对充电桩的位姿偏差，
 * 通过三阶段状态机把机器人导引到充电桩上：
 *
 *   WAIT_MARKER -> ALIGN -> BACK_TO_FIXED -> BLIND_DOCK -> DONE
 *
 * 设计要点见 README.md。
 */

#include <ros/ros.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <relative_move/SetRelativeMove.h>

#include <cmath>
#include <deque>
#include <string>
#include <vector>

namespace {

constexpr double kEps = 1e-3;

struct Pose2D {
  double lateral = 0.0;   // 横向偏差 (m)，0 = 正对充电桩中线
  double distance = 0.0;  // 前后距离 (m)
  double yaw = 0.0;       // 朝向偏差 (rad)，0 = 正对
  bool valid = false;
};

enum class Phase { kIdle, kDone, kFailed };

const char* PhaseName(Phase p) {
  switch (p) {
    case Phase::kIdle:   return "IDLE";
    case Phase::kDone:   return "DONE";
    case Phase::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace

class AutoChargeNode {
 public:
  AutoChargeNode() : nh_("~") {
    loadParams();
    marker_sub_ = nh_.subscribe(marker_topic_, 1, &AutoChargeNode::markerCallback, this);
    move_client_ = nh_.serviceClient<relative_move::SetRelativeMove>(move_service_);
  }

  bool run() {
    start_time_ = ros::Time::now();
    ROS_INFO("========================================");
    ROS_INFO(" Auto Charge  |  marker: %s", marker_topic_.c_str());
    ROS_INFO("========================================");

    if (!waitForService()) return finish(false);
    if (!alignToMarker()) return finish(false);
    if (!backToFixedDistance()) return finish(false);
    if (!blindDock()) return finish(false);

    return finish(true);
  }

 private:
  // ---------- 参数 ----------
  void loadParams() {
    nh_.param<std::string>("marker_topic", marker_topic_, "/base_camera/ar_pose_marker");
    nh_.param<std::string>("move_service", move_service_, "relative_move");
    nh_.param<std::string>("global_frame", global_frame_, "odom");

    nh_.param("x_tolerance", x_tolerance_, 0.03);
    nh_.param("yaw_tolerance", yaw_tolerance_, 0.05);
    nh_.param("max_align_iter", max_align_iter_, 8);
    nh_.param("stall_threshold", stall_threshold_, 0.05);

    nh_.param("fixed_distance", fixed_distance_, 0.20);
    nh_.param("docking_distance", docking_distance_, 0.35);

    nh_.param("step_size", step_size_, 0.05);
    nh_.param("step_delay", step_delay_, 0.30);
    nh_.param("settle_time", settle_time_, 0.30);

    nh_.param("samples_per_measure", samples_per_measure_, 5);
    nh_.param("marker_timeout", marker_timeout_, 3.0);
    nh_.param("marker_stale_sec", marker_stale_sec_, 0.5);

    nh_.param("service_retry", service_retry_, 2);
  }

  // ---------- AR 码回调 ----------
  // 相机在底盘后下方，AR 码位于机器人后方，因此：
  //   position.x -> 横向偏差（相机光学系 +x 为右）
  //   position.z -> 前后距离
  //   orientation -> 朝向偏差
  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    if (msg->markers.empty()) return;

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if ((ros::Time::now() - stamp).toSec() > marker_stale_sec_) {
      ROS_WARN_THROTTLE(2.0, "[MARKER] drop stale sample (%.2f s old)", (ros::Time::now() - stamp).toSec());
      return;
    }

    const auto& pose = msg->markers.front().pose.pose;
    Pose2D p;
    p.lateral = pose.position.x;
    p.distance = pose.position.z;

    // 四元数 -> 偏航角（只取 yaw 分量）
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    p.yaw = std::atan2(siny_cosp, cosy_cosp);

    p.valid = true;
    samples_.push_back(p);
    while (samples_.size() > static_cast<size_t>(samples_per_measure_)) {
      samples_.pop_front();
    }
  }

  // ---------- 测量 ----------
  // 清空历史样本后重新采集，保证每次决策用的都是动作之后的新数据。
  // 取多帧均值抑制单帧噪声：单帧抖动可能有 1cm 量级，与 3cm 容差同阶。
  bool measure(Pose2D& out) {
    samples_.clear();
    const ros::Time deadline = ros::Time::now() + ros::Duration(marker_timeout_);
    ros::Rate rate(50.0);

    while (ros::ok() && ros::Time::now() < deadline) {
      ros::spinOnce();
      if (samples_.size() >= static_cast<size_t>(samples_per_measure_)) break;
      rate.sleep();
    }

    if (samples_.empty()) {
      ROS_ERROR("[MEASURE] AR marker not visible (timeout %.1f s)", marker_timeout_);
      out.valid = false;
      return false;
    }

    const double n = static_cast<double>(samples_.size());
    double sum_lat = 0.0, sum_dist = 0.0, sum_yaw = 0.0;
    for (const Pose2D& s : samples_) {
      sum_lat += s.lateral;
      sum_dist += s.distance;
      sum_yaw += s.yaw;
    }
    out.lateral = sum_lat / n;
    out.distance = sum_dist / n;
    out.yaw = sum_yaw / n;
    out.valid = true;

    if (samples_.size() < static_cast<size_t>(samples_per_measure_)) {
      ROS_WARN("[MEASURE] only %zu/%d samples, result may be noisy",
               samples_.size(), samples_per_measure_);
    }
    return true;
  }

  void logPose(const Pose2D& p, const std::string& tag) {
    ROS_INFO("[%s] lateral %+6.1f cm | distance %6.1f cm | yaw %+6.2f deg",
             tag.c_str(), p.lateral * 100.0, std::abs(p.distance) * 100.0,
             p.yaw * 180.0 / M_PI);
  }

  // ---------- 运动 ----------
  // approach: 正值 = 朝充电桩方向后退；dtheta: 旋转；lateral: 横移
  bool callRelativeMove(double approach, double lateral, double dtheta) {
    relative_move::SetRelativeMove srv;
    srv.request.global_frame = global_frame_;
    srv.request.goal.x = -approach;  // 负 x 为后退，靠近充电桩
    srv.request.goal.y = lateral;
    srv.request.goal.theta = dtheta;
    srv.request.is_omni = 1;
    srv.request.avoid_obstacle = false;
    srv.request.finishStopObstacle = false;

    for (int attempt = 0; attempt <= service_retry_; ++attempt) {
      if (move_client_.call(srv)) {
        if (srv.response.success) return true;
        ROS_WARN("[MOVE] rejected: %s", srv.response.message.c_str());
      } else {
        ROS_WARN("[MOVE] call failed (attempt %d/%d)", attempt + 1, service_retry_ + 1);
      }
      ros::Duration(0.2).sleep();
    }
    ROS_ERROR("[MOVE] giving up after %d attempts", service_retry_ + 1);
    return false;
  }

  // 前后位移必须分段：AR 码装位低，一次大位移会让它脱出相机视野，
  // 最后一段相当于盲走。分段后每步都留出重新检测的机会。
  bool moveInSteps(double approach, double lateral, double dtheta) {
    if (std::abs(dtheta) > kEps) {
      ROS_INFO("[MOVE] rotate %.2f deg", dtheta * 180.0 / M_PI);
      if (!callRelativeMove(0.0, 0.0, dtheta)) return false;
      ros::Duration(settle_time_).sleep();
    }

    if (std::abs(lateral) > kEps) {
      ROS_INFO("[MOVE] lateral %.1f cm", lateral * 100.0);
      if (!callRelativeMove(0.0, lateral, 0.0)) return false;
      ros::Duration(settle_time_).sleep();
    }

    if (std::abs(approach) > kEps) {
      const double sign = (approach > 0.0) ? 1.0 : -1.0;
      double remaining = std::abs(approach);
      int steps = 0;
      ROS_INFO("[MOVE] approach %.1f cm in steps of %.1f cm",
               approach * 100.0, step_size_ * 100.0);
      while (remaining > kEps) {
        const double step = (remaining > step_size_) ? step_size_ : remaining;
        if (!callRelativeMove(sign * step, 0.0, 0.0)) {
          ROS_ERROR("[MOVE] step %d failed", steps + 1);
          return false;
        }
        remaining -= step;
        ++steps;
        if (remaining > kEps) ros::Duration(step_delay_).sleep();
      }
      ROS_INFO("[MOVE] approach done in %d steps", steps);
    }
    return true;
  }

  // ---------- 阶段 1：对准 ----------
  // 横向偏差与朝向偏差在相机坐标系下是耦合的：转动会改变横向读数。
  // 因此每次只修正一个自由度，然后重新测量；同时修正两个量会在目标附近震荡。
  // 优先修朝向，因为朝向未收敛时横向读数本身不可信。
  bool alignToMarker() {
    ROS_INFO("---------- Stage 1: ALIGN ----------");
    Pose2D p;
    double prev_norm = -1.0;
    int stall_count = 0;

    for (int i = 0; i < max_align_iter_; ++i) {
      if (!measure(p)) return false;
      logPose(p, "ALIGN");

      const double x_err = p.lateral;
      const double yaw_err = p.yaw;
      const double norm = std::abs(x_err) / x_tolerance_ + std::abs(yaw_err) / yaw_tolerance_;

      if (std::abs(x_err) <= x_tolerance_ && std::abs(yaw_err) <= yaw_tolerance_) {
        ROS_INFO("[ALIGN] converged in %d iteration(s), residual x=%.1f cm yaw=%.2f deg",
                 i + 1, std::abs(x_err) * 100.0, std::abs(yaw_err) * 180.0 / M_PI);
        return true;
      }

      // 归一化残差连续两轮没有实质改善，说明迭代已经不再收敛，
      // 继续下去只会浪费时间并带着残余偏差进入盲倒。
      if (prev_norm > 0.0 && (prev_norm - norm) < stall_threshold_) {
        ++stall_count;
        ROS_WARN("[ALIGN] residual stalled (%.2f -> %.2f), %d/2", prev_norm, norm, stall_count);
        if (stall_count >= 2) {
          ROS_ERROR("[ALIGN] no progress, aborting instead of docking with residual error");
          return false;
        }
      } else {
        stall_count = 0;
      }
      prev_norm = norm;

      if (std::abs(yaw_err) > yaw_tolerance_) {
        if (!moveInSteps(0.0, 0.0, -yaw_err)) return false;
      } else {
        if (!moveInSteps(0.0, x_err, 0.0)) return false;
      }
    }

    ROS_ERROR("[ALIGN] not converged after %d iterations, aborting", max_align_iter_);
    return false;
  }

  // ---------- 阶段 2：倒退到固定点 ----------
  // 退到固定距离后，剩余位移就是一个定值，可以不再依赖视觉。
  bool backToFixedDistance() {
    ROS_INFO("---------- Stage 2: BACK TO FIXED POINT ----------");
    Pose2D p;
    if (!measure(p)) return false;
    logPose(p, "BACK");

    const double current = std::abs(p.distance);
    const double need_back = current - fixed_distance_;
    ROS_INFO("[BACK] %.1f cm -> target %.1f cm (back %.1f cm)",
             current * 100.0, fixed_distance_ * 100.0, need_back * 100.0);

    if (need_back <= 0.01) {
      ROS_INFO("[BACK] already within fixed point");
      return true;
    }
    if (!moveInSteps(need_back, 0.0, 0.0)) return false;

    // 复核（非致命：此距离下 AR 码可能已接近视野边缘）
    Pose2D q;
    if (measure(q)) {
      logPose(q, "BACK+");
    } else {
      ROS_WARN("[BACK] marker not visible after move, continue with blind offset");
      samples_.clear();
    }
    return true;
  }

  // ---------- 阶段 3：盲倒对接 ----------
  // 这一段的位移不再由视觉反馈，因此进入前必须最后确认一次目标可见，
  // 否则宁可中止，也不要盲走撞上充电桩。
  bool blindDock() {
    ROS_INFO("---------- Stage 3: BLIND DOCK ----------");
    Pose2D p;
    if (!measure(p)) {
      ROS_ERROR("[DOCK] marker lost before blind move, aborting to avoid collision");
      return false;
    }
    logPose(p, "DOCK");

    ROS_INFO("[DOCK] blind backing %.1f cm", docking_distance_ * 100.0);
    if (!moveInSteps(docking_distance_, 0.0, 0.0)) return false;

    ROS_INFO("[DOCK] docking completed");
    return true;
  }

  bool waitForService() {
    ROS_INFO("[INIT] waiting for service '%s' ...", move_service_.c_str());
    if (!ros::service::waitForService(move_service_, ros::Duration(10.0))) {
      ROS_ERROR("[INIT] service '%s' unavailable", move_service_.c_str());
      return false;
    }
    ROS_INFO("[INIT] service ready");
    ros::Duration(0.5).sleep();
    return true;
  }

  bool finish(bool ok) {
    const double elapsed = (ros::Time::now() - start_time_).toSec();
    ROS_INFO("========================================");
    ROS_INFO(" Auto Charge %s   |   elapsed %.1f s", ok ? "[SUCCESS]" : "[FAILED]", elapsed);
    ROS_INFO(" phase: %s", PhaseName(ok ? Phase::kDone : Phase::kFailed));
    ROS_INFO("========================================");
    return ok;
  }

  // ---------- 成员 ----------
  ros::NodeHandle nh_;
  ros::Subscriber marker_sub_;
  ros::ServiceClient move_client_;
  std::deque<Pose2D> samples_;

  std::string marker_topic_, move_service_, global_frame_;

  double x_tolerance_ = 0.03;
  double yaw_tolerance_ = 0.05;
  int max_align_iter_ = 8;
  double stall_threshold_ = 0.05;

  double fixed_distance_ = 0.20;
  double docking_distance_ = 0.35;

  double step_size_ = 0.05;
  double step_delay_ = 0.30;
  double settle_time_ = 0.30;

  int samples_per_measure_ = 5;
  double marker_timeout_ = 3.0;
  double marker_stale_sec_ = 0.5;

  int service_retry_ = 2;

  ros::Time start_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "auto_charge_node");
  AutoChargeNode node;
  return node.run() ? 0 : 1;
}