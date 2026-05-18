# `init_and_push_to_github.sh` 使用教程

交互式脚本,把当前目录初始化为 git 仓库并推送到 GitHub。本文档教您从零到一跑通它。

---

## 一、第一次使用前要准备什么

### 1. 安装依赖

```bash
# 必需
sudo apt update
sudo apt install -y git

# 推荐(脚本会自动用它创建远端仓库,免去手动到网页操作)
sudo apt install -y gh
```

`gh` 在 Ubuntu 22.04 官方源里就有,如果版本太旧可换官方源:
```bash
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list
sudo apt update && sudo apt install gh
```

### 2. 准备 GitHub 认证(二选一)

#### 方式 A:gh CLI 登录(最省事)
```bash
gh auth login
# 选项依次:GitHub.com → HTTPS → Y → Login with a web browser
# 终端会显示一个 8 位验证码,例如 ABCD-1234,记下来
# 按回车,浏览器打开 https://github.com/login/device,粘贴验证码、点 Authorize
```
登录后 `gh auth status` 应显示 `Logged in to github.com as <你的用户名>`。

#### 方式 B:SSH key
```bash
# 生成
ssh-keygen -t ed25519 -C "chunyvtian@gmail.com"   # 一路回车即可
# 拷贝公钥
cat ~/.ssh/id_ed25519.pub
# 粘贴到 https://github.com/settings/keys → New SSH key → 保存
# 测试
ssh -T git@github.com   # 看到 "Hi <username>!" 即成功
```

> 两者可以共存。脚本里推送协议选 SSH 走 key,选 HTTPS 走 gh CLI 的凭据。

---

## 二、运行脚本

```bash
cd ~/arm_ws
./init_and_push_to_github.sh
```

如果提示 `Permission denied`,说明没加可执行权限:
```bash
chmod +x init_and_push_to_github.sh
```

---

## 三、逐项交互说明

下面按运行顺序列出每个提示,标注含义、默认值、示例。`[xxx]` 表示直接按回车采用默认值。

### 1) 环境自检
```
[INFO]  检查 git ...
[INFO]  检测到 gh CLI (gh version 2.x.x ...)
```
没装 git 直接退出。没装 gh 会 `[WARN]`,后续要您手动在网页建仓库。

### 2) Git 身份
```
Git user.name [TianFeiF]:
Git user.email [chunyvtian@gmail.com]:
```
- 如果您之前 `git config --global` 设置过,会显示现值,直接回车即可。
- 第一次用 git 时这两项必填,否则 commit 会被拒绝。
- **email 必须和 GitHub 账号的邮箱一致**,否则 GitHub 不会把 commit 归属到您的头像。

### 3) GitHub 信息
```
GitHub 用户名 (例如 TianFeiF):
仓库名称 [arm_ws]:
仓库描述(可空,直接回车跳过) [ ]:
```
- 用户名就是您 GitHub 主页 URL 里的那一段:`https://github.com/<这里>`。
- 仓库名默认取当前目录名,可自定义。**注意大小写敏感,不要含空格**,用 `-` 或 `_` 分隔。
- 描述可空。回车跳过即可,会显示在仓库首页副标题。

### 4) 可见性
```
仓库可见性:
  1) public  公开
  2) private 私有
选择 1 或 2 [2]:
```
- `1` = 公开,任何人都能看。
- `2` = 私有(默认),只有您和被邀请的协作者能看。
- 含商用代码 / 内部数据 / 客户配置时**务必选 2**。

### 5) 推送协议
```
推送协议:
  1) SSH    git@github.com (推荐,需要 SSH key)
  2) HTTPS  https://github.com/... (需要 PAT 或 gh auth)
选择 1 或 2 [1]:
```
- `1` SSH:适合配过 SSH key 的机器。一次配置长期有效,push 不再要求输入凭据。
- `2` HTTPS:适合受限网络(公司里 22 端口被封)。需要 `gh auth login` 或 PAT。

### 6) 默认分支
```
默认分支名 [main]:
```
直接回车用 `main`。如果您单位习惯 `master`,在这里改。

### 7) 首次 commit message
```
首次 commit message [Initial commit]:
```
回车即用默认。建议写得有信息量,例如 `Initial commit: armv7 7DoF MoveIt2 + IgH EtherCAT integration`。

### 8) 汇总确认
```
=========== 即将执行 ===========
  工作目录:      /home/tian/arm_ws
  git user:      TianFeiF <chunyvtian@gmail.com>
  GitHub 用户:   TianFeiF
  仓库名:        arm_ws
  描述:          ARM 7DoF EtherCAT workspace
  可见性:        private
  远端 URL:      git@github.com:TianFeiF/arm_ws.git
  默认分支:      main
  首次 commit:   Initial commit
  gh CLI 创建远端: yes

确认执行? [Y/n]:
```
**这是唯一的"反悔点"**。检查无误回车;有任何错误按 `n` 退出重新跑。

### 9) 大文件检查
```
[INFO]  检查 >50MB 文件 ...
[WARN]  发现以下大文件,GitHub 单文件 100MB 上限:
    ./isaac-sim-standalone-5.0.0-linux-x86_64.zip
    ./WeChatLinux_x86_64.AppImage
继续(将这些文件一并 add)? [y/N]:
```
**这一步非常关键**:
- 如果列出来的文件确实是您要 commit 的(很少见),按 `y`。但单文件超过 100MB 时 push 会直接失败。
- 通常应按 `n` 退出,把这些文件加入 `.gitignore`,或者用 Git LFS,然后重新跑脚本。

### 10) 远端创建 + 推送
```
[INFO]  通过 gh 创建远端仓库 ...
✓ Created repository TianFeiF/arm_ws on GitHub
[INFO]  git push -u origin main ...
Enumerating objects: 1234, done.
...
To github.com:TianFeiF/arm_ws.git
 * [new branch]      main -> main
branch 'main' set up to track 'origin/main'.
[ OK ]  推送成功!

  仓库地址: https://github.com/TianFeiF/arm_ws
```
浏览器打开链接即可看到您的工程。

如果没装 `gh`,这一步会停下来:
```
请在浏览器手动创建一个空仓库:
    https://github.com/new
        名称:     arm_ws
        可见性:   private
        不要勾选 README / .gitignore / license

创建完成后回到此处按回车继续推送。
```
按要求建好(**关键:不要勾任何初始化选项**),回到终端按回车继续。

---

## 四、常见场景

### 场景 1:第一次推一个新工程
按上面流程走一遍即可。

### 场景 2:之前手动 `git init` 过,现在想用脚本接手
脚本会检测到 `.git` 已存在并跳过 `git init`,也会检测到分支名和 `origin`,不会破坏现有状态。直接运行即可。

### 场景 3:仓库名想改
重新运行脚本,在"仓库名称"步骤填新名字。脚本会发现 `origin` 已存在并询问:
```
[WARN]  remote origin 已存在: git@github.com:TianFeiF/old_name.git
覆盖为 git@github.com:TianFeiF/new_name.git ? [y/N]:
```
按 `y` 覆盖。然后会调用 `gh` 创建新远端并 push。

### 场景 4:GitHub 上已经手动建过同名仓库
- 如果是空仓库(没勾 README):脚本检测到后跳过创建,直接 push,成功。
- 如果是非空仓库:`git push -u origin main` 会失败,提示 `! [rejected] main -> main (fetch first)`。处理:
  ```bash
  git pull --rebase origin main
  git push -u origin main
  ```

### 场景 5:脚本中途 Ctrl+C 终止了
不会留下任何不可逆的痕迹。本地最多多了 `.git/` 和一份 commit,可以:
```bash
rm -rf .git           # 完全恢复成裸目录
# 然后重新跑脚本
```

---

## 五、报错排查

### `Permission denied (publickey)`
SSH 协议但 key 没配。两条路:
1. 跑 [前置准备 - 方式 B](#方式-bssh-key) 配 SSH key。
2. 重跑脚本,推送协议选 HTTPS,先 `gh auth login`。

### `fatal: could not read Username for 'https://github.com'`
HTTPS 协议但没有凭据。`gh auth login` 即可。

### `error: failed to push some refs ... fetch first`
远端非空。`git pull --rebase origin main && git push`。

### `remote: error: File XXX is 123.45 MB; this exceeds GitHub's file size limit`
有文件超 100MB。**第一次 push 时遇到这个,清理最简单**:
```bash
git reset --soft HEAD~1     # 撤销 commit 但保留改动
git rm --cached path/to/bigfile
echo "path/to/bigfile" >> .gitignore
git commit -m "Initial commit"
git push -u origin main
```

### `! [rejected] main -> main (non-fast-forward)`
本地和远端历史分叉。**仅当确认远端可以被覆盖时**:
```bash
git push --force-with-lease origin main
```

### gh `HTTP 422: Repository creation failed (name already exists)`
您 GitHub 上已有同名仓库。重跑脚本时换个名字,或者先去 GitHub 删掉旧仓库。

---

## 六、后续日常推送

首次成功后,以后的更新不需要再跑这个脚本,标准 git 流程即可:
```bash
cd ~/arm_ws
git status              # 看改了什么
git add <文件>          # 或 git add -A
git commit -m "fix: xxx"
git push                # 因为首次 push 已 -u,这里不用带参数
```

需要把新机器的代码同步下来:
```bash
git pull
```

---

## 七、卸载 / 反悔

如果您想把这次操作完全撤销(本地 + 远端):

```bash
# 1) 删除远端仓库
gh repo delete TianFeiF/arm_ws --yes
# 或者去 https://github.com/TianFeiF/arm_ws/settings 滚动到底部 "Danger Zone" 手动删

# 2) 删除本地 git 元数据
cd ~/arm_ws
rm -rf .git

# 3) (可选)删除脚本生成的 .gitignore
# rm .gitignore     # 仅当确认是脚本生成的、之前没有时
```

工作目录回到运行脚本之前的状态。
