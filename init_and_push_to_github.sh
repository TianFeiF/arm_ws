#!/usr/bin/env bash
# init_and_push_to_github.sh
# 交互式将本目录初始化为 git 仓库并推送到 GitHub。
#
# 用法:
#   bash init_and_push_to_github.sh
#
# 不会执行任何破坏性操作:
#   - 已是 git 仓库时,跳过 git init
#   - 远端 origin 已存在时,询问是否覆盖
#   - 默认分支若已有同名分支,直接复用
#
# 依赖:
#   - git
#   - (可选)gh CLI:有则自动调用 gh 创建远端仓库;没有则提示手动在网页创建。

set -eu

# ─────────────── 工具函数 ───────────────
color() { printf '\033[%sm%s\033[0m\n' "$1" "$2"; }
info()  { color "1;34" "[INFO]  $*"; }
ok()    { color "1;32" "[ OK ]  $*"; }
warn()  { color "1;33" "[WARN]  $*"; }
err()   { color "1;31" "[ERR ]  $*" >&2; }

# 带默认值的 read。$1=提示, $2=默认值(可空), 结果写到全局变量 REPLY_VAL
prompt() {
    local question="$1" default="${2:-}" answer
    if [[ -n "$default" ]]; then
        read -r -p "$question [$default]: " answer
        REPLY_VAL="${answer:-$default}"
    else
        while true; do
            read -r -p "$question: " answer
            if [[ -n "$answer" ]]; then
                REPLY_VAL="$answer"; break
            fi
            warn "不能为空,请重新输入。"
        done
    fi
}

# 是 / 否提示。$1=问题, $2=默认 y|n。返回 0 表示 yes。
confirm() {
    local question="$1" default="${2:-n}" answer hint
    [[ "$default" == "y" ]] && hint="Y/n" || hint="y/N"
    read -r -p "$question [$hint]: " answer
    answer="${answer:-$default}"
    [[ "${answer,,}" == "y" || "${answer,,}" == "yes" ]]
}

# ─────────────── 前置检查 ───────────────
info "检查 git ..."
command -v git >/dev/null 2>&1 || { err "未安装 git。请先 sudo apt install git"; exit 1; }

HAS_GH=0
if command -v gh >/dev/null 2>&1; then
    HAS_GH=1
    info "检测到 gh CLI ($(gh --version | head -1))"
else
    warn "未检测到 gh CLI,稍后需要您手动在 GitHub 网页创建空仓库。"
fi

# ─────────────── 收集信息 ───────────────
echo
color "1;36" "=========== 仓库信息 ==========="

# git 身份
CUR_NAME=$(git config --global user.name 2>/dev/null || true)
CUR_EMAIL=$(git config --global user.email 2>/dev/null || true)
prompt "Git user.name"  "${CUR_NAME:-}"; GIT_NAME="$REPLY_VAL"
prompt "Git user.email" "${CUR_EMAIL:-}"; GIT_EMAIL="$REPLY_VAL"

# GitHub 信息
prompt "GitHub 用户名 (例如 TianFeiF)" ""; GH_USER="$REPLY_VAL"
prompt "仓库名称" "$(basename "$(pwd)")"; REPO_NAME="$REPLY_VAL"
prompt "仓库描述(可空,直接回车跳过)" " "; REPO_DESC="$REPLY_VAL"
[[ "$REPO_DESC" == " " ]] && REPO_DESC=""

# 可见性
echo
echo "仓库可见性:"
echo "  1) public  公开"
echo "  2) private 私有"
prompt "选择 1 或 2" "2"; VIS_CHOICE="$REPLY_VAL"
case "$VIS_CHOICE" in
    1|public)  VISIBILITY="public" ;;
    2|private) VISIBILITY="private" ;;
    *) err "无效选项: $VIS_CHOICE"; exit 1 ;;
esac

# 协议
echo
echo "推送协议:"
echo "  1) SSH    git@github.com (推荐,需要 SSH key)"
echo "  2) HTTPS  https://github.com/... (需要 PAT 或 gh auth)"
prompt "选择 1 或 2" "1"; PROTO_CHOICE="$REPLY_VAL"
case "$PROTO_CHOICE" in
    1|ssh)   REMOTE_URL="git@github.com:${GH_USER}/${REPO_NAME}.git" ;;
    2|https) REMOTE_URL="https://github.com/${GH_USER}/${REPO_NAME}.git" ;;
    *) err "无效选项: $PROTO_CHOICE"; exit 1 ;;
esac

# 分支
prompt "默认分支名" "main"; BRANCH="$REPLY_VAL"

# 首次 commit message
prompt "首次 commit message" "Initial commit"; COMMIT_MSG="$REPLY_VAL"

# 汇总确认
echo
color "1;36" "=========== 即将执行 ==========="
cat <<EOF
  工作目录:      $(pwd)
  git user:      ${GIT_NAME} <${GIT_EMAIL}>
  GitHub 用户:   ${GH_USER}
  仓库名:        ${REPO_NAME}
  描述:          ${REPO_DESC:-(无)}
  可见性:        ${VISIBILITY}
  远端 URL:      ${REMOTE_URL}
  默认分支:      ${BRANCH}
  首次 commit:   ${COMMIT_MSG}
  gh CLI 创建远端: $([[ $HAS_GH -eq 1 ]] && echo yes || echo no, 手动创建)
EOF
echo
confirm "确认执行?" "y" || { warn "已取消。"; exit 0; }

# ─────────────── 执行 ───────────────

# 1) git 全局配置
info "写入 git 全局身份 ..."
git config --global user.name  "$GIT_NAME"
git config --global user.email "$GIT_EMAIL"

# 2) 准备 .gitignore
if [[ ! -f .gitignore ]]; then
    info "未发现 .gitignore,创建 ROS 2 通用模板 ..."
    cat > .gitignore <<'GITIGNORE'
# colcon
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/

# IDE
.vscode/
.idea/
*.swp
*~

# OS
.DS_Store
Thumbs.db

# 大文件类型
*.AppImage
*.zip
*.tar.gz
*.bag
*.mcap

# 日志/临时
*.log
outputs_*/
terminal_logs.txt
return.txt
GITIGNORE
    ok ".gitignore 已创建"
else
    info ".gitignore 已存在,跳过创建"
fi

# 3) git init
if [[ ! -d .git ]]; then
    info "git init -b $BRANCH ..."
    git init -b "$BRANCH" >/dev/null
else
    info "已是 git 仓库,跳过 init"
    CUR_BRANCH=$(git symbolic-ref --quiet --short HEAD 2>/dev/null || echo "")
    if [[ -n "$CUR_BRANCH" && "$CUR_BRANCH" != "$BRANCH" ]]; then
        if confirm "当前分支是 '$CUR_BRANCH',切换/重命名为 '$BRANCH'?" "y"; then
            git branch -M "$BRANCH"
        fi
    elif [[ -z "$CUR_BRANCH" ]]; then
        # 还没有任何 commit,设置初始分支名
        git symbolic-ref HEAD "refs/heads/$BRANCH"
    fi
fi

# 4) 大文件预警
info "检查 >50MB 文件 ..."
BIG=$(find . -type f -size +50M \
    -not -path "./.git/*" -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" 2>/dev/null || true)
if [[ -n "$BIG" ]]; then
    warn "发现以下大文件,GitHub 单文件 100MB 上限:"
    echo "$BIG" | sed 's/^/    /'
    confirm "继续(将这些文件一并 add)?" "n" || {
        warn "请把这些文件加入 .gitignore 或使用 Git LFS,然后重新运行本脚本。"
        exit 0
    }
fi

# 5) add + commit
info "git add ..."
git add -A
if git diff --cached --quiet; then
    warn "暂存区为空,可能所有文件都被 .gitignore 排除。"
    exit 1
fi

if git rev-parse --verify HEAD >/dev/null 2>&1; then
    info "已存在 commit 历史,跳过首次 commit"
else
    info "git commit ..."
    git commit -m "$COMMIT_MSG" >/dev/null
    ok "首次 commit 已创建"
fi

# 6) 远端 origin
if git remote get-url origin >/dev/null 2>&1; then
    EXISTING_URL=$(git remote get-url origin)
    if [[ "$EXISTING_URL" == "$REMOTE_URL" ]]; then
        info "remote origin 已指向 $REMOTE_URL,跳过"
    else
        warn "remote origin 已存在: $EXISTING_URL"
        if confirm "覆盖为 $REMOTE_URL ?" "n"; then
            git remote set-url origin "$REMOTE_URL"
        else
            err "已中止。请手动处理 remote 后再运行。"; exit 1
        fi
    fi
else
    info "添加 remote origin -> $REMOTE_URL"
    git remote add origin "$REMOTE_URL"
fi

# 7) 创建远端仓库
if [[ $HAS_GH -eq 1 ]]; then
    # 检查是否已存在
    if gh repo view "${GH_USER}/${REPO_NAME}" >/dev/null 2>&1; then
        info "远端仓库已存在,跳过创建"
    else
        info "通过 gh 创建远端仓库 ..."
        # gh repo create 需要登录态
        if ! gh auth status >/dev/null 2>&1; then
            warn "gh 未登录,正在启动 gh auth login ..."
            gh auth login
        fi
        gh_args=(repo create "${GH_USER}/${REPO_NAME}" "--${VISIBILITY}")
        [[ -n "$REPO_DESC" ]] && gh_args+=(--description "$REPO_DESC")
        gh "${gh_args[@]}"
        ok "远端仓库已创建"
    fi
else
    cat <<MANUAL

请在浏览器手动创建一个空仓库:
    https://github.com/new
        名称:     ${REPO_NAME}
        可见性:   ${VISIBILITY}
        不要勾选 README / .gitignore / license

创建完成后回到此处按回车继续推送。
MANUAL
    read -r -p ""
fi

# 8) push
info "git push -u origin $BRANCH ..."
if git push -u origin "$BRANCH"; then
    ok "推送成功!"
    echo
    echo "  仓库地址: https://github.com/${GH_USER}/${REPO_NAME}"
else
    err "push 失败。常见原因:"
    cat <<HINTS
    1) SSH key 未配置:
       ssh-keygen -t ed25519 -C "${GIT_EMAIL}"
       cat ~/.ssh/id_ed25519.pub  → 粘贴到 https://github.com/settings/keys
    2) 远端非空(创建时勾了 README):
       git pull --rebase origin ${BRANCH} && git push
    3) HTTPS 凭据过期:
       git config --global credential.helper store
       下次 push 时输入用户名 + PAT
HINTS
    exit 1
fi
