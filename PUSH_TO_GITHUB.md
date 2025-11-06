# 推送代碼到 GitHub 指南

## 🔐 方法 1：使用 Personal Access Token（推薦）

### 步驟 1：創建 GitHub Personal Access Token

1. 訪問：https://github.com/settings/tokens
2. 點擊 "Generate new token" → "Generate new token (classic)"
3. 設置：
   - Note: `ROS2 Unity Bridge`
   - Expiration: 選擇合適的期限
   - Scopes: 勾選 `repo` (完整倉庫權限)
4. 點擊 "Generate token"
5. **複製token**（只顯示一次！）

### 步驟 2：使用 Token 推送

```bash
# 在容器中執行
cd /root/ros2_ws

# 推送時使用token作為密碼
git push -u origin main

# 用戶名：lkzaxx
# 密碼：貼上你的 Personal Access Token
```

或者直接在URL中使用token：

```bash
git remote set-url origin https://<你的token>@github.com/lkzaxx/UNITYtoROS2.git
git push -u origin main
```

## 🔑 方法 2：使用 SSH 密鑰

### 步驟 1：生成 SSH 密鑰（如果還沒有）

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
# 按Enter使用默認路徑
# 設置密碼（可選）
```

### 步驟 2：複製公鑰

```bash
cat ~/.ssh/id_ed25519.pub
# 複製輸出的內容
```

### 步驟 3：添加到 GitHub

1. 訪問：https://github.com/settings/keys
2. 點擊 "New SSH key"
3. 貼上公鑰內容
4. 保存

### 步驟 4：更改遠程URL為SSH

```bash
cd /root/ros2_ws
git remote set-url origin git@github.com:lkzaxx/UNITYtoROS2.git
git push -u origin main
```

## 📦 方法 3：在 Windows 上推送（如果容器無法認證）

### 步驟 1：從容器導出到Windows

```powershell
# 在Windows PowerShell中
docker cp ros2_jazzy:/root/ros2_ws C:\Users\lkzax\ros2_ws_git
```

### 步驟 2：在Windows上配置Git

```powershell
cd C:\Users\lkzax\ros2_ws_git

# 設置用戶信息
git config user.name "lkzaxx"
git config user.email "你的郵箱@example.com"

# 推送（使用GitHub Desktop或命令行）
git push -u origin main
```

## 🚀 快速推送命令（使用Token）

在容器中執行：

```bash
cd /root/ros2_ws

# 方法A：交互式輸入（推薦首次使用）
git push -u origin main
# 用戶名：lkzaxx
# 密碼：貼上你的Personal Access Token

# 方法B：直接在URL中嵌入（不推薦，但方便）
git remote set-url origin https://<YOUR_TOKEN>@github.com/lkzaxx/UNITYtoROS2.git
git push -u origin main
```

## ✅ 驗證推送成功

推送成功後，訪問：
https://github.com/lkzaxx/UNITYtoROS2

應該能看到所有文件！

