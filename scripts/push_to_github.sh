#!/usr/bin/env bash
# 推送到 GitHub
#
# 前置（需要先在网页完成，只需做一次）：
#   1. fork https://github.com/Tfly6/Mid360_px4_sim_plugin  ->  自己的账号
#   2. fork https://github.com/hku-mars/FAST_LIO            ->  自己的账号
#   3. 新建仓库 https://github.com/new ，名称 px4-mid360-fastlio-sim
#      不要勾选 Add README / .gitignore / License（本地已有，勾了会冲突）
#
# 用法：
#   bash scripts/push_to_github.sh
#   GH_USER=别的用户名 MAIN_REPO=别的仓库名 bash scripts/push_to_github.sh
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GH_USER="${GH_USER:-TIANN-ENG}"
MAIN_REPO="${MAIN_REPO:-px4-mid360-fastlio-sim}"
PLUGIN_BRANCH="${PLUGIN_BRANCH:-px4-sim-fixes}"
LIO_BRANCH="${LIO_BRANCH:-mid360-px4-sim}"

remote_exists() { timeout 20 git ls-remote "git@github.com:$1.git" >/dev/null 2>&1; }

# 确保该仓库的 origin 指向自己的 fork（缺失就补上）
# 否则会因为 "fatal: 'origin' does not appear to be a git repository" 而中断
ensure_origin() {
  local dir="$1" slug="$2" url="git@github.com:$2.git"
  if git -C "$dir" remote | grep -qx origin; then
    git -C "$dir" remote set-url origin "$url"
  else
    git -C "$dir" remote add origin "$url"
  fi
  echo "  ${dir/#$HOME/~}  origin -> $url"
}

echo "== [1/3] 检查远程仓库是否已创建 =="
missing=0
for slug in "$GH_USER/Mid360_px4_sim_plugin" "$GH_USER/FAST_LIO" "$GH_USER/$MAIN_REPO"; do
  printf "  %-46s " "$slug"
  if remote_exists "$slug"; then echo "✅"; else echo "❌ 尚未创建"; missing=1; fi
done

if [ "$missing" = 1 ]; then
  cat <<EOF

缺少远程仓库，请先在网页完成这三步：
  fork  https://github.com/Tfly6/Mid360_px4_sim_plugin/fork
  fork  https://github.com/hku-mars/FAST_LIO/fork
  新建  https://github.com/new   （名称 $MAIN_REPO，不要初始化 README/.gitignore/License）

完成后重新执行： bash scripts/push_to_github.sh
EOF
  exit 1
fi

echo
echo "== [2/3] 推送第三方改动分支 =="
ensure_origin "$REPO_DIR/src/Mid360_px4_sim_plugin" "$GH_USER/Mid360_px4_sim_plugin"
ensure_origin "$REPO_DIR/src/FAST_LIO"              "$GH_USER/FAST_LIO"
git -C "$REPO_DIR/src/Mid360_px4_sim_plugin" push -u origin "$PLUGIN_BRANCH" || exit 1
git -C "$REPO_DIR/src/FAST_LIO"              push -u origin "$LIO_BRANCH"    || exit 1

echo
echo "== [3/3] 推送主仓库 =="
ensure_origin "$REPO_DIR" "$GH_USER/$MAIN_REPO"
git -C "$REPO_DIR" push -u origin main || exit 1

cat <<EOF

全部完成 🎉

  主仓库   https://github.com/$GH_USER/$MAIN_REPO
  插件分支 https://github.com/$GH_USER/Mid360_px4_sim_plugin/tree/$PLUGIN_BRANCH
  LIO分支  https://github.com/$GH_USER/FAST_LIO/tree/$LIO_BRANCH

别忘了在仓库首页右上齿轮里填 About 与 Topics（见 README 或 docs）。
EOF
