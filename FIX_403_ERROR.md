# 修復 GitHub 403 錯誤指南

## ❌ 錯誤訊息
```
remote: Write access to repository not granted.
fatal: unable to access 'https://github.com/lkzaxx/UNITYtoROS2.git/': The requested URL returned error: 403
```

## 🔍 可能原因

1. **Token權限不足** - Token沒有 `repo` 權限
2. **Token過期或無效** - Token已過期或被撤銷
3. **認證方式錯誤** - 使用了錯誤的認證方法
4. **倉庫不存在** - GitHub上沒有這個倉庫
5. **倉庫權限問題** - 沒有該倉庫的寫入權限

## ✅ 解決方案

### 方案 1：檢查並重新創建 Token（最常見）

#### 步驟 1：創建新的 Personal Access Token

1. 訪問：https://github.com/settings/tokens
2. 點擊 **"Generate new token"** → **"Generate new token (classic)"**
3. **重要設置**：
   - **Note**: `UNITYtoROS2 - Full Access`
   - **Expiration**: 選擇合適期限（建議90天或更長）
   - **Scopes**: **必須勾選以下權限**：
     - ✅ `repo` (完整倉庫權限) - **最重要！**
     - ✅ `workflow` (如果需要GitHub Actions)
   - 不要只勾選 `public_repo`，要勾選完整的 `repo`
4. 點擊 **"Generate token"**
5. **立即複製token**（格式：`ghp_xxxxxxxxxxxxxxxxxxxx`）

#### 步驟 2：清除舊的認證並重新設置

```bash
cd /root/ros2_ws

# 清除保存的認證
git config --global --unset credential.helper
rm -f ~/.git-credentials

# 使用新token設置遠程URL
git remote set-url origin https://ghp_YOUR_NEW_TOKEN@github.com/lkzaxx/UNITYtoROS2.git

# 或者使用交互式方式（會提示輸入）
git remote set-url origin https://github.com/lkzaxx/UNITYtoROS2.git
git push -u origin main
# 用戶名：lkzaxx
# 密碼：貼上你的新token
```

### 方案 2：檢查倉庫是否存在

```bash
# 測試遠程連接
git ls-remote origin

# 如果返回404，說明倉庫不存在，需要先在GitHub上創建
```

如果倉庫不存在：
1. 訪問：https://github.com/new
2. 創建新倉庫：
   - Repository name: `UNITYtoROS2`
   - 選擇 Public 或 Private
   - **不要**初始化README（因為你已經有代碼）
3. 創建後再推送

### 方案 3：使用 SSH 方式（推薦長期使用）

#### 步驟 1：生成SSH密鑰

```bash
# 生成SSH密鑰
ssh-keygen -t ed25519 -C "lkzaxx.work@gmail.com"

# 按Enter使用默認路徑
# 設置密碼（可選，建議設置）
```

#### 步驟 2：查看公鑰

```bash
cat ~/.ssh/id_ed25519.pub
# 複製輸出的內容
```

#### 步驟 3：添加到GitHub

1. 訪問：https://github.com/settings/keys
2. 點擊 **"New SSH key"**
3. **Title**: `ROS2 Docker Container`
4. **Key**: 貼上剛才複製的公鑰
5. 點擊 **"Add SSH key"**

#### 步驟 4：更改遠程URL為SSH

```bash
cd /root/ros2_ws

# 更改為SSH方式
git remote set-url origin git@github.com:lkzaxx/UNITYtoROS2.git

# 測試連接
ssh -T git@github.com

# 如果看到 "Hi lkzaxx! You've successfully authenticated..." 就成功了

# 推送
git push -u origin main
```

### 方案 4：檢查倉庫權限

確認：
1. 你確實是 `lkzaxx` 這個GitHub帳號
2. 倉庫 `UNITYtoROS2` 存在且你有權限
3. 訪問：https://github.com/lkzaxx/UNITYtoROS2 確認倉庫存在

### 方案 5：完全重置認證

```bash
cd /root/ros2_ws

# 清除所有認證
git config --global --unset credential.helper
rm -f ~/.git-credentials
rm -f ~/.gitconfig

# 重新設置用戶信息
git config --global user.name "lkzaxx"
git config --global user.email "lkzaxx.work@gmail.com"

# 設置憑證助手
git config --global credential.helper store

# 重新推送
git push -u origin main
# 輸入用戶名：lkzaxx
# 輸入密碼：你的新Personal Access Token
```

## 🔍 診斷步驟

```bash
# 1. 檢查遠程URL
git remote -v

# 2. 測試連接
git ls-remote origin

# 3. 檢查認證
git config --list | grep credential

# 4. 查看Git配置
git config --list
```

## ✅ 最可能的解決方法

**99%的情況是Token權限問題！**

1. 創建**新的**Personal Access Token
2. **確保勾選了 `repo` 權限**（不是 `public_repo`）
3. 清除舊認證
4. 使用新token重新推送

## 🚀 快速修復命令

```bash
cd /root/ros2_ws

# 清除舊認證
git config --global --unset credential.helper
rm -f ~/.git-credentials

# 使用新token（替換 <NEW_TOKEN>）
git remote set-url origin https://ghp_NEW_TOKEN@github.com/lkzaxx/UNITYtoROS2.git

# 推送
git push -u origin main
```

## 📝 驗證步驟

推送成功後，訪問：
https://github.com/lkzaxx/UNITYtoROS2

應該能看到你的代碼！

