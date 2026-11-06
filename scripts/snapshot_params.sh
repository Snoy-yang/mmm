#!/usr/bin/env bash
#
# snapshot_params.sh —— 给当前参数存一份带时间戳的快照
#
# 用法（在仓库任意位置）：
#   bash scripts/snapshot_params.sh
#
# 产出：
#   experiments/<YYYYMMDD-HHMMSS>/params.yaml   参数原文
#   experiments/<YYYYMMDD-HHMMSS>/meta.txt      时间、提交号、分支、是否改了未提交
#
# 为什么需要它：
#   参数和实验结论必须能对上。只记「我把容差调小了」没有意义，
#   要能回答「哪一版参数、哪一次提交、哪个场景」。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_ROOT/config/charge_params.yaml"
STAMP="$(date +%Y%m%d-%H%M%S)"
DEST_DIR="$REPO_ROOT/experiments/$STAMP"

if [ ! -f "$SRC" ]; then
  echo "错误：找不到 $SRC" >&2
  exit 1
fi

mkdir -p "$DEST_DIR"
cp "$SRC" "$DEST_DIR/params.yaml"

COMMIT="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo no-git)"
BRANCH="$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
DIRTY="$(git -C "$REPO_ROOT" status --porcelain -- config/charge_params.yaml 2>/dev/null || true)"

{
  echo "snapshot_time: $(date -Iseconds)"
  echo "git_commit:    $COMMIT"
  echo "git_branch:    $BRANCH"
  echo "params_dirty:  ${DIRTY:-clean}"
  echo "source:        config/charge_params.yaml"
} > "$DEST_DIR/meta.txt"

REL="experiments/$STAMP"
echo "已保存参数快照：$REL/"
echo "  params.yaml  $(wc -l < "$DEST_DIR/params.yaml") 行"
echo "  meta.txt     提交 $COMMIT / 分支 $BRANCH"

if [ -n "$DIRTY" ]; then
  echo
  echo "注意：charge_params.yaml 有未提交的改动。"
  echo "      快照记的是工作区当前内容；请确认这些改动会和快照一起提交，"
  echo "      否则别人拉到的是旧参数、配新快照，对不上。"
fi

echo
echo "下一步：在 docs/EXPERIMENTS.md 的记录表里加一行，指向 $REL/"
