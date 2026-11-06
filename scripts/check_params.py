#!/usr/bin/env python3
"""
check_params.py —— 启动前校验 config/charge_params.yaml

用法：
    python3 scripts/check_params.py
    python3 scripts/check_params.py --file config/charge_params.yaml
    python3 scripts/check_params.py --strict      # 把警告也当失败

退出码：
    0 = 全部通过（可能有警告）
    1 = 有错误，或 --strict 下有警告

为什么需要它：
    ROS 参数有三个特点和直觉相反的地方：

    1. 参数名打错不会报错。rosparam 只是把 yaml 里的键塞进参数服务器，
       节点按名字去取，取不到就用代码里的默认值。所以
       `x_tolerence`（少一个 a）不会报错，只会安静地不生效。
    2. 参数之间的一致性没人管。fixed_distance 和 docking_distance 写反了，
       节点照样启动、照样运动，只是往反方向倒。
    3. 越界值只在撞上时才暴露。容差写到 0.5 m，要等到上桩失败才发现，
       而那时候你已经在查视觉算法了。

    这三件事的共同点是：**错误不在启动时报，而在几十分钟后的现场暴露**。
    这个脚本把它们提前到启动前，用一秒换那几十分钟。
"""

import argparse
import sys
from pathlib import Path

# 期望的键名与类型。新增参数时同步维护这里，否则会被当成拼写错误报出来。
SPEC = {
    # --- 接口 ---
    "marker_topic": str,
    "move_service": str,
    "global_frame": str,
    # --- 对准容差 ---
    "x_tolerance": float,
    "yaw_tolerance": float,
    "max_align_iter": int,
    "stall_threshold": float,
    # --- 距离控制 ---
    "fixed_distance": float,
    "docking_distance": float,
    # --- 分段运动 ---
    "step_size": float,
    "step_delay": float,
    "settle_time": float,
    # --- 测量 ---
    "samples_per_measure": int,
    "marker_timeout": float,
    "marker_stale_sec": float,
    # --- 可靠性 ---
    "service_retry": int,
}

errors = []
warns = []
oks = 0


def ok(msg):
    global oks
    oks += 1
    print(f"  [ OK ]  {msg}")


def warn(msg):
    warns.append(msg)
    print(f"  [WARN]  {msg}")


def error(msg):
    errors.append(msg)
    print(f"  [FAIL]  {msg}")


def check_range(cfg, key, lo=None, hi=None, lo_open=True, why=""):
    """检查数值是否落在 [lo, hi] 内。why 说明越界会怎样。"""
    val = cfg[key]
    bad = False
    if lo is not None:
        bad |= (val <= lo) if lo_open else (val < lo)
    if hi is not None:
        bad |= val > hi
    if bad:
        bound = []
        if lo is not None:
            bound.append(f"{'>' if lo_open else '>='} {lo}")
        if hi is not None:
            bound.append(f"<= {hi}")
        warn(f"{key} = {val} 超出建议范围（{' 且 '.join(bound)}）。{why}")
    else:
        ok(f"{key} = {val}")


def main():
    ap = argparse.ArgumentParser(description="校验 charge_params.yaml")
    ap.add_argument("--file", default="config/charge_params.yaml")
    ap.add_argument("--strict", action="store_true", help="警告也算失败")
    args = ap.parse_args()

    path = Path(args.file)
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[1] / path

    print(f"参数自检：{path}\n")

    if not path.exists():
        print(f"  [FAIL]  文件不存在：{path}")
        return 1

    try:
        import yaml
    except ImportError:
        print("  [FAIL]  缺少 PyYAML。安装：pip install pyyaml 或 sudo apt install python3-yaml")
        return 1

    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if not isinstance(cfg, dict):
        print("  [FAIL]  yaml 顶层不是键值对，检查缩进")
        return 1

    # ---- 1. 拼写检查：最容易被忽略、也最贵的一类错误 ----
    print("[1] 键名")
    unknown = sorted(set(cfg) - set(SPEC))
    missing = sorted(set(SPEC) - set(cfg))
    for k in unknown:
        error(
            f"未知参数 '{k}'（值 {cfg[k]!r}）。ROS 不会为拼错的参数报错，"
            f"它会静默使用代码默认值 —— 你以为改了，其实没改。"
        )
    for k in missing:
        warn(f"缺少参数 '{k}'，节点会退回代码内默认值")
    if not unknown and not missing:
        ok(f"{len(SPEC)} 个参数齐全，无未知键")

    # ---- 2. 类型 ----
    print("\n[2] 类型")
    typed = {}
    for k, typ in SPEC.items():
        if k not in cfg:
            continue
        v = cfg[k]
        if typ is str:
            if not isinstance(v, str) or not v.strip():
                error(f"{k} 应为非空字符串，实际是 {v!r}")
            else:
                typed[k] = v
        else:
            # yaml 里 8 会被解析成 int，0.03 是 float，两种都接受
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                error(f"{k} 应为数字，实际是 {v!r}")
            else:
                typed[k] = typ(v)
                ok(f"{k} = {typed[k]}")
    cfg.update(typed)

    # ---- 3. 单项取值范围 ----
    print("\n[3] 取值范围")
    if "x_tolerance" in cfg:
        check_range(cfg, "x_tolerance", 0, 0.10, why="容差大于 10cm 时，对准结束时机器人可能仍偏在桩外，盲倒会撞偏")
    if "yaw_tolerance" in cfg:
        check_range(cfg, "yaw_tolerance", 0, 0.15, why="朝向容差过大时，盲倒过程中车体会斜着蹭桩")
    if "max_align_iter" in cfg:
        check_range(cfg, "max_align_iter", 0, 20, lo_open=False, why="迭代上限过高时，一次失败会拖很久，现场不如早点报错重试")
    if "stall_threshold" in cfg:
        check_range(cfg, "stall_threshold", 0, 1.0, why="归一化残差的改善量，超出 0~1 没有意义")
    if "fixed_distance" in cfg:
        check_range(cfg, "fixed_distance", 0, 0.60, why="退得太远会超出充电桩的可视范围")
    if "step_size" in cfg:
        check_range(cfg, "step_size", 0, 0.15, why="单步过大时 AR 码可能在一步内脱出视野，正是分段要避免的情况")
    if "step_delay" in cfg:
        check_range(cfg, "step_delay", 0, 3.0, why="延时过长会让一次回充耗时失控")
    if "settle_time" in cfg:
        check_range(cfg, "settle_time", 0, 3.0, why="同上")
    if "samples_per_measure" in cfg:
        check_range(cfg, "samples_per_measure", 0, 20, lo_open=False, why="采样过多会拖慢每次决策")
    if "marker_timeout" in cfg:
        check_range(cfg, "marker_timeout", 0, 30.0, why="超时过长时，找不到 AR 码会一直卡住而不报错")
    if "marker_stale_sec" in cfg:
        check_range(cfg, "marker_stale_sec", 0, 2.0, why="过大的话旧数据会被当成新数据用")
    if "service_retry" in cfg:
        check_range(cfg, "service_retry", -1, 10, lo_open=False, why="重试次数过多会掩盖真实故障")

    # ---- 4. 参数之间的一致性：脚本真正不可替代的部分 ----
    print("\n[4] 参数间一致性")

    if "docking_distance" in cfg and "fixed_distance" in cfg:
        fd, dd = cfg["fixed_distance"], cfg["docking_distance"]
        if dd <= fd:
            error(
                f"docking_distance ({dd} m) 必须大于 fixed_distance ({fd} m)。"
                f"盲倒是从「对准后的固定点」走到桩上，前一段必须比后一段短，"
                f"否则方向反了 —— 节点不会报错，车会往后退。"
            )
        else:
            ok(f"盲倒行程 {dd - fd:.2f} m（fixed {fd} → docking {dd}），方向正确")

    if "step_size" in cfg and "fixed_distance" in cfg:
        ss, fd = cfg["step_size"], cfg["fixed_distance"]
        if ss >= fd:
            warn(f"step_size ({ss} m) 不小于 fixed_distance ({fd} m)，一步就退过了目标点")
        else:
            ok(f"倒退分段 {fd / ss:.1f} 步，每步 {ss} m")

    if "step_size" in cfg and "docking_distance" in cfg and "fixed_distance" in cfg:
        ss = cfg["step_size"]
        travel = cfg["docking_distance"] - cfg["fixed_distance"]
        if ss > travel > 0:
            warn(f"step_size ({ss} m) 大于盲倒总行程 ({travel:.2f} m)，最后一段会一步走完，分段失去意义")

    if "marker_stale_sec" in cfg and "marker_timeout" in cfg:
        stale, timeout = cfg["marker_stale_sec"], cfg["marker_timeout"]
        if stale > timeout:
            warn(f"marker_stale_sec ({stale} s) 大于 marker_timeout ({timeout} s)，时效判定永远轮不到生效")

    if "x_tolerance" in cfg and "step_size" in cfg:
        if cfg["x_tolerance"] > cfg["step_size"]:
            warn(
                f"x_tolerance ({cfg['x_tolerance']} m) 大于 step_size ({cfg['step_size']} m)，"
                f"对准精度比单步位移还粗，最后一步可能跨过容差带"
            )

    # ---- 汇总 ----
    print("\n" + "-" * 52)
    print(f"通过 {oks} 项 | 警告 {len(warns)} 项 | 错误 {len(errors)} 项")

    if errors:
        print("\n结论：有错误，不要启动。先把上面 [FAIL] 的项改掉。")
        return 1
    if warns and args.strict:
        print("\n结论：--strict 模式下警告也算失败。")
        return 1
    if warns:
        print("\n结论：可以启动，但建议先看一遍 [WARN] 的项。")
        return 0
    print("\n结论：全部通过，可以启动。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
