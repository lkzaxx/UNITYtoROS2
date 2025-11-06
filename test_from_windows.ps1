# Windows PowerShell 測試腳本
# 測試連接到容器 192.168.65.6

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "容器連線測試工具" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

$containerIP = "192.168.65.6"
$ports = @(7400, 7401, 7402, 7403, 7404)

Write-Host "1. 測試基本網路連線 (Ping):" -ForegroundColor Yellow
$pingResult = Test-Connection -ComputerName $containerIP -Count 2 -Quiet
if ($pingResult) {
    Write-Host "   ✅ Ping 成功" -ForegroundColor Green
} else {
    Write-Host "   ❌ Ping 失敗" -ForegroundColor Red
}
Write-Host ""

Write-Host "2. 測試 TCP 埠連線:" -ForegroundColor Yellow
foreach ($port in $ports) {
    $tcpTest = Test-NetConnection -ComputerName $containerIP -Port $port -InformationLevel Quiet -WarningAction SilentlyContinue
    if ($tcpTest) {
        Write-Host "   ✅ TCP $port - 可連線" -ForegroundColor Green
    } else {
        Write-Host "   ❌ TCP $port - 無法連線" -ForegroundColor Red
    }
}
Write-Host ""

Write-Host "3. 測試 UDP 埠連線 (發送測試):" -ForegroundColor Yellow
foreach ($port in $ports) {
    try {
        $udp = New-Object System.Net.Sockets.UdpClient
        $endpoint = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Parse($containerIP), $port)
        $bytes = [System.Text.Encoding]::ASCII.GetBytes("test")
        $udp.Send($bytes, $bytes.Length, $endpoint) | Out-Null
        $udp.Close()
        Write-Host "   ✅ UDP $port - 訊息已發送" -ForegroundColor Green
    } catch {
        Write-Host "   ❌ UDP $port - 錯誤: $_" -ForegroundColor Red
    }
}
Write-Host ""

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "測試完成" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
