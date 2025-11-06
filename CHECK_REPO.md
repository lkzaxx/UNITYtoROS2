# 檢查和創建 GitHub 倉庫指南

## 🔍 當前問題

看到錯誤訊息：
```
remote: Repository not found.
fatal: Authentication failed
```

這表示：
1. 倉庫可能不存在於 GitHub
2. 或者認證失敗

## ✅ 解決步驟

### 步驟 1：確認倉庫是否存在

訪問：https://github.com/lkzaxx/UNITYtoROS2

**如果看到 404 錯誤**，說明倉庫不存在，需要創建。

**如果倉庫存在**，繼續步驟 2。

### 步驟 2：創建倉庫（如果不存在）

1. 訪問：https://github.com/new
2. 設置：
   - **Repository name**: `UNITYtoROS2`
   - **Description**: `Unity to ROS2 Bridge Communication`
   - **Visibility**: 
     - Public（公開，任何人都能看到）
     - Private（私有，只有你可以看到）
   - ⚠️ **重要**：**不要**勾選以下選項：
     - ❌ Add a README file
     - ❌ Add .gitignore
     - ❌ Choose a license
   （因為你已經有代碼了）
3. 點擊 **"Create repository"**

### 步驟 3：創建 Personal Access Token

1. 訪問：https://github.com/settings/tokens
2. 點擊 **"Generate new token"** → **"Generate new token (classic)"**
3. 設置：
   - **Note**: `UNITYtoROS2 - Full Access`
   - **Expiration**: 選擇期限（建議 90 天或更長）
   - **Scopes**: **必須勾選**：
     - ✅ **`repo`** (完整倉庫權限) - **最重要！**
       - 這包括所有子權限：repo:status, repo_deployment, public_repo, repo:invite, security_events
   - 不要只勾選 `public_repo`，要勾選完整的 `repo`
4. 點擊 **"Generate token"**
5. **立即複製token**（格式：`ghp_xxxxxxxxxxxxxxxxxxxx`）
   - ⚠️ 只顯示一次，請立即保存！

### 步驟 4：在容器中設置認證並推送

```bash
cd /root/ros2_ws

# 方法A：直接在URL中嵌入token（推薦）
git remote set-url origin https://ghp_YOUR_TOKEN@github.com/lkzaxx/UNITYtoROS2.git

# 測試連接
git ls-remote origin

# 如果成功，推送
git push -u origin main
```

或者：

```bash
# 方法B：使用憑證助手（會提示輸入）
git config --global credential.helper store
git push -u origin main

# 輸入：
# Username: lkzaxx
# Password: [貼上你的Personal Access Token]
```

## 🔐 完整的推送流程

```bash
# 1. 確認當前狀態
cd /root/ros2_ws
git status

# 2. 設置遠程（使用新token）
git remote set-url origin https://ghp_YOUR_NEW_TOKEN@github.com/lkzaxx/UNITYtoROS2.git

# 3. 測試連接
git ls-remote origin

# 4. 如果看到分支列表，說明成功，可以推送
git push -u origin main
```

## ⚠️ 常見錯誤

### 錯誤 1: "Repository not found"
- **原因**: 倉庫不存在
- **解決**: 在 GitHub 上創建倉庫

### 錯誤 2: "Authentication failed"
- **原因**: Token 無效或權限不足
- **解決**: 創建新 token，確保勾選 `repo` 權限

### 錯誤 3: "Write access to repository not granted"
- **原因**: Token 只有讀取權限
- **解決**: 創建新 token，勾選完整的 `repo` 權限

## 🎯 快速檢查清單

在推送前確認：

- [ ] GitHub 倉庫已創建：https://github.com/lkzaxx/UNITYtoROS2
- [ ] Personal Access Token 已創建（有 `repo` 權限）
- [ ] Token 已複製並保存
- [ ] 遠程 URL 已更新（包含新 token）
- [ ] 測試連接成功（`git ls-remote origin`）

## 🚀 一鍵推送腳本

```bash
# 在容器中執行（替換 <YOUR_TOKEN>）
cd /root/ros2_ws
git remote set-url origin https://ghp_YOUR_TOKEN@github.com/lkzaxx/UNITYtoROS2.git
git ls-remote origin && git push -u origin main
```

