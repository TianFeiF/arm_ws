# 将本地项目上传到 GitHub 指南

适用于 `arm_ws` 这类 ROS 2 工作区(也适用于绝大多数本地工程)。下文按"首次上传"完整流程编写;后续日常推送(`git commit && git push`)不在本文档范围。

---

## 前置条件

| 项目 | 检查命令 | 缺失时安装 |
|---|---|---|
| Git | `git --version` | `sudo apt install git` |
| GitHub 账号 | — | https://github.com/signup |
| 认证(SSH 或 PAT) | 见下节 | 见下节 |
| (可选)gh CLI | `gh --version` | `sudo apt install gh` 或 [安装文档](https://cli.github.com) |

---

## 第 1 步:配置认证

GitHub 在 2021 年后**禁止用账号密码 push**。三选一:

### A. SSH key(推荐,一次配置长期生效)
```bash
# 1) 生成 key(如果 ~/.ssh/id_ed25519.pub 已存在可跳过)
ssh-keygen -t ed25519 -C "chunyvtian@gmail.com"

# 2) 复制公钥
cat ~/.ssh/id_ed25519.pub

# 3) 粘贴到 https://github.com/settings/keys → New SSH key

# 4) 测试
ssh -T git@github.com   # 显示 "Hi <username>! ..." 即成功
```
之后 remote URL 使用 `git@github.com:USER/REPO.git`。

### B. Personal Access Token(PAT)
```bash
# 1) 浏览器打开 https://github.com/settings/tokens
#    Generate new token (classic) → 勾选 'repo' 作用域 → 生成 → 立即复制
# 2) push 时输入用户名 + 用 PAT 替代密码,凭据可缓存:
git config --global credential.helper store
```
remote URL 使用 `https://github.com/USER/REPO.git`。

### C. gh CLI 登录(最省事)
```bash
gh auth login
# 选 GitHub.com → HTTPS → Login with a web browser → 粘贴一次性验证码
```
之后 `git` 和 `gh` 都用同一份凭据。

---

## 第 2 步:配置本地 git 身份

```bash
git config --global user.name  "你的名字"
git config --global user.email "chunyvtian@gmail.com"   # 必须与 GitHub 账号邮箱一致,否则 commit 不归属本人
git config --global init.defaultBranch main
```

---

## 第 3 步:整理工作目录

### 检查 `.gitignore`
ROS 2 工作区**必须忽略**:
```
build/
install/
log/
*.pyc
__pycache__/
.vscode/
.idea/
```
检查根目录 `.gitignore` 是否覆盖以上项目,没有则补上。

### 排除大文件 / 敏感文件
GitHub 单文件限制 100 MB,仓库总大小建议 1 GB 内。先排查:
```bash
find . -type f -size +50M -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" -not -path "./.git/*"
```
查到的 `.zip`、`.AppImage`、`.bag` 等要么加入 `.gitignore`,要么用 [Git LFS](https://git-lfs.com/)。

凭证类绝对不能 commit:
```bash
grep -rIn -E "password|secret|token|api[_-]?key" --include="*.{py,yaml,json,sh,cpp}" . 2>/dev/null | grep -v ".git/"
```

---

## 第 4 步:在 GitHub 创建空仓库

⚠️ **不要**勾选 "Initialize this repository with a README/.gitignore/license",否则第一次 push 时需要 `--force` 或额外 merge。

- 网页方式:https://github.com/new → 填名字 → Create repository
- 命令行方式:
  ```bash
  gh repo create my-arm-ws --public --source=. --remote=origin --description "ARM 7DoF + EtherCAT MoveIt2 workspace"
  ```
  `gh repo create` 会一次性完成:创建远端 + 添加 remote。

---

## 第 5 步:初始化本地仓库并首次推送

```bash
cd ~/arm_ws

# 1) 初始化(如果已经是 git 仓库可跳过)
git init -b main

# 2) 暂存所有(被 .gitignore 排除的不会进来)
git add .

# 3) 检查将要 commit 的文件,确认没有意外的大文件 / 凭证
git status
git ls-files | xargs -I{} du -h "{}" | sort -hr | head -20

# 4) 首次 commit
git commit -m "Initial commit: armv7 EtherCAT MoveIt2 workspace"

# 5) 添加远端(SSH 示例。HTTPS 用 https://github.com/USER/REPO.git)
git remote add origin git@github.com:USER/REPO.git

# 6) 推送
git push -u origin main
```

`-u` 设置上游分支,以后 `git push`/`git pull` 不用再带参数。

---

## 第 6 步:验证

```bash
# 远端记录
git log --oneline -5

# 浏览器打开
gh repo view --web        # 需要 gh CLI
# 或手动: https://github.com/USER/REPO
```

确认:
- 文件树正确,没有 `build/install/log/`。
- 首屏 README 渲染正常(如未写 README,GitHub 会显示仓库结构)。

---

## 常见问题

### push 报 `Permission denied (publickey)`
SSH key 没加到 GitHub,或者 remote URL 用了 SSH 但本地没配 key。
```bash
ssh -T git@github.com   # 看错误细节
ssh-add -l              # 看 agent 里加载了哪些 key
```

### push 报 `! [rejected] main -> main (fetch first)`
远端有内容(可能创建仓库时勾了 README)。处理:
```bash
git pull --rebase origin main
git push
```

### push 报 `remote: error: File ... is XXX MB; this exceeds GitHub's file size limit of 100.00 MB`
有大文件已经被 commit 到历史里,即便 `.gitignore` 加了也没用,需要从历史移除:
```bash
git rm --cached path/to/bigfile
echo "path/to/bigfile" >> .gitignore
git commit -m "Remove large file"
# 如果在最近一次 commit 之前就引入,需要 git-filter-repo:
# pip install git-filter-repo
# git filter-repo --path path/to/bigfile --invert-paths
git push --force-with-lease   # 仅当你知道这是空仓库 / 自己一个人用时再 force
```

### 想把现有目录覆盖式上传到一个已有内容的远端
```bash
git push --force-with-lease origin main
```
**慎用**,会丢远端历史。仅在确认远端是空仓库或测试库时再用。

---

## 附:`.gitignore` 模板(ROS 2 + 通用)

```gitignore
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

# 大文件类型(按需启用)
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
```
