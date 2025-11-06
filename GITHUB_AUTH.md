# GitHub 登入認證指南

## 🔐 方法 1：使用 Personal Access Token（最簡單）

### 步驟 1：創建 Token

1. 訪問：https://github.com/settings/tokens
2. 點擊 **"Generate new token"** → **"Generate new token (classic)"**
3. 設置：
   - **Note**: `ROS2 Unity Bridge`
   - **Expiration**: 選擇期限（建議90天或自定義）
   - **Scopes**: 勾選 **`repo`** (完整倉庫權限)
4. 點擊 **"Generate token"**
5. **立即複製token**（只顯示一次！格式類似：`ghp_xxxxxxxxxxxxxxxxxxxx`）

### 步驟 2：在容器中使用 Token 推送

```bash
cd /root/ros2_ws

# 直接推送，會提示輸入認證
git push -u origin main
```

**輸入提示時：**
- **Username**: `lkzaxx`
- **Password**: 貼上你的 Personal Access Token（不是你的GitHub密碼！）

### 或者直接在URL中嵌入Token（一次性設置）

```bash
cd /root/ros2_ws

# 替換 <YOUR_TOKEN> 為你的實際token
git remote set-url origin https://ghp_YOUR_TOKEN_HERE@github.com/lkzaxx/UNITYtoROS2.git

# 然後推送（不需要再輸入認證）
git push -u origin main
```

---

## 🔑 方法 2：使用 GitHub CLI（gh）

### 安裝 GitHub CLI（如果還沒安裝）

```bash
# 在容器中安裝
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null
apt update
apt install gh -y
```

### 登入 GitHub

```bash
# 啟動認證流程
gh auth login

# 選擇：
# 1. GitHub.com
# 2. HTTPS
# 3. 選擇認證方式（瀏覽器或token）
# 4. 完成認證
```

---

## 🔐 方法 3：使用 Git Credential Helper（保存認證）

### 設置憑證助手

```bash
cd /root/ros2_ws

# 設置憑證助手（會保存認證信息）
git config --global credential.helper store

# 或者使用緩存（15分鐘有效）
git config --global credential.helper cache

# 推送時輸入一次認證，之後會自動保存
git push -u origin main
```

**輸入：**
- Username: `lkzaxx`
- Password: 你的 Personal Access Token

---

## 📝 方法 4：手動設置環境變數（適用於腳本）

```bash
# 設置環境變數（臨時）
export GIT_ASKPASS=echo
export GIT_USERNAME=lkzaxx
export GIT_TOKEN=ghp_YOUR_TOKEN_HERE

# 或者在URL中嵌入
git remote set-url origin https://${GIT_TOKEN}@github.com/lkzaxx/UNITYtoROS2.git
```

---

## 🚀 快速開始（推薦流程）

```bash
cd /root/ros2_ws

# 1. 設置憑證助手（保存認證）
git config --global credential.helper store

# 2. 推送（會提示輸入認證）
git push -u origin main

# 輸入：
# Username: lkzaxx
# Password: [貼上你的Personal Access Token]
```

**之後推送就不需要再輸入認證了！**

---

## ⚠️ 重要提醒

1. **Personal Access Token 不是密碼**：使用token時，密碼欄位要輸入token，不是GitHub密碼
2. **Token只顯示一次**：創建後立即複製保存
3. **安全性**：不要將token提交到代碼倉庫中
4. **Token過期**：如果token過期，需要重新創建

---

## 🔍 驗證認證是否成功

```bash
# 測試遠程連接
git ls-remote origin

# 如果成功，會顯示遠程分支列表
```

---

## 📚 相關資源

- [創建Personal Access Token](https://github.com/settings/tokens)
- [Git認證文檔](https://git-scm.com/book/en/v2/Git-Tools-Credential-Storage)
- [GitHub CLI文檔](https://cli.github.com/manual/)

