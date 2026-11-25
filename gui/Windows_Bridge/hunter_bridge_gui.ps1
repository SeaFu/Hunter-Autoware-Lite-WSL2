#requires -version 5.1
# Hunter Device Bridge - Windows 11 / PowerShell WinForms, Phase 1/2.
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()
$ErrorActionPreference='Stop'
$script:root=Split-Path -Parent $MyInvocation.MyCommand.Path
$script:relayScript=Join-Path $script:root 'udp_relay.ps1'
$script:statusFile=Join-Path $script:root 'bridge_status.json'
$script:relayProc=$null
$script:busid=''

function Say([string]$message) {
    $script:logBox.AppendText((Get-Date -Format 'HH:mm:ss')+'  '+$message+[Environment]::NewLine)
    $script:logBox.SelectionStart=$script:logBox.TextLength
    $script:logBox.ScrollToCaret()
}
function LabelAt([string]$text,[int]$x,[int]$y,[int]$w=160) {
    $lab=New-Object System.Windows.Forms.Label
    $lab.Text=$text;$lab.Location=New-Object System.Drawing.Point($x,$y)
    $lab.Size=New-Object System.Drawing.Size($w,28)
    return $lab
}
function TextAt([string]$text,[int]$x,[int]$y,[int]$w=210) {
    $t=New-Object System.Windows.Forms.TextBox
    $t.Text=$text;$t.Location=New-Object System.Drawing.Point($x,$y)
    $t.Size=New-Object System.Drawing.Size($w,26)
    return $t
}
function ButtonAt([string]$title,[int]$x,[int]$y,[int]$w=140) {
    $b=New-Object System.Windows.Forms.Button
    $b.Text=$title;$b.Location=New-Object System.Drawing.Point($x,$y)
    $b.Size=New-Object System.Drawing.Size($w,36)
    $b.FlatStyle='Flat'
    $b.BackColor=[System.Drawing.Color]::FromArgb(45,73,97)
    $b.ForeColor=[System.Drawing.Color]::White
    return $b
}
function Get-Usb {
    try {
        $output=(& usbipd.exe list 2>&1 | Out-String)
        if ($LASTEXITCODE -ne 0) {throw $output}
        $line=($output -split "`r?`n" | Where-Object {$_ -match '1d50:606f'} | Select-Object -First 1)
        if ($line -and $line -match '^\s*([0-9]+-[0-9.]+)\s+1d50:606f') {
            $script:busid=$Matches[1]
            $script:canStatus.Text='CAN：'+$line.Trim()
            Say ('找到 candleLight：BUSID '+$script:busid)
        } else {
            $script:busid=''
            $script:canStatus.Text='CAN：找不到 VID:PID 1d50:606f'
            Say '找不到 USB-CAN。請確認 USB 接線與 usbipd list。'
        }
    } catch {Say ('USB 掃描失敗：'+$_)}
}
function Get-WslIP {
    $distro=$script:distroBox.Text.Trim()
    if (-not $distro) {Say 'WSL 發行版不得空白。';return}
    try {
        $output=(& wsl.exe -d $distro -- sh -c 'ip -4 -o addr show dev eth0' 2>&1 | Out-String)
        if ($LASTEXITCODE -ne 0) {throw $output}
        if ($output -match '\binet\s+((?:\d{1,3}\.){3}\d{1,3})/') {
            $script:wslBox.Text=$Matches[1]
            Say ('已偵測 WSL2 eth0 IP：'+$script:wslBox.Text)
        } else { Say ('無法解析 eth0 IPv4：'+$output.Trim()) }
    } catch {Say ('WSL IP 偵測失敗：'+$_)}
}
function Attach-Can {
    if (-not $script:busid) {Get-Usb}
    if (-not $script:busid) {return}
    # usbipd can print informational lines to stderr; PowerShell 5.1 can
    # mistake those lines for terminating errors under Stop preference.
    # Trust the verified status from usbipd list instead of stderr text.
    Get-Usb
    if ($script:canStatus.Text -match '\bAttached\b') {
        Say ('USB-CAN '+$script:busid+' 已是 Attached；不重複掛載。')
        return
    }
    $stdOut=[System.IO.Path]::GetTempFileName()
    $stdErr=[System.IO.Path]::GetTempFileName()
    try {
        $exe=(Get-Command usbipd.exe -ErrorAction Stop).Source
        $proc=Start-Process -FilePath $exe -ArgumentList @('attach','--wsl','--busid',$script:busid) -Wait -PassThru -RedirectStandardOutput $stdOut -RedirectStandardError $stdErr
        $output=((Get-Content -LiteralPath $stdOut -Raw -ErrorAction SilentlyContinue) + [Environment]::NewLine + (Get-Content -LiteralPath $stdErr -Raw -ErrorAction SilentlyContinue)).Trim()
        if ($output) {Say ('usbipd attach: '+$output)}
        Get-Usb
        if ($script:canStatus.Text -match '\bAttached\b') {
            Say ('USB-CAN '+$script:busid+' Attach 成功（usbipd list 已確認）。')
        } else {
            Say ('Attach 尚未完成（exit code='+$proc.ExitCode+'）；請檢查上述輸出及 Shared/Attached 狀態。')
        }
    } catch {Say ('Attach 操作未完成：'+$_);Get-Usb}
    finally {
        Remove-Item -LiteralPath $stdOut,$stdErr -Force -ErrorAction SilentlyContinue
    }
}
function Bind-Can {
    if (-not $script:busid) {Get-Usb}
    if (-not $script:busid) {return}
    try {
        $exe=(Get-Command usbipd.exe -ErrorAction Stop).Source
        Say ('要求 Windows 管理員權限，Bind BUSID '+$script:busid)
        $p=Start-Process -FilePath $exe -ArgumentList @('bind','--busid',$script:busid) -Verb RunAs -Wait -PassThru
        Say ('Bind 結束，exit code='+$p.ExitCode)
        Get-Usb
    } catch {Say ('Bind 未完成（可能取消 UAC）：'+$_)}
}
function Detach-Can {
    if (-not $script:busid) {Get-Usb}
    if (-not $script:busid) {return}
    $output=(& usbipd.exe detach --busid $script:busid 2>&1 | Out-String)
    Say ('Detach：'+$output.Trim())
    Get-Usb
}
function Start-Relay {
    if ($script:relayProc -and -not $script:relayProc.HasExited) {Say 'Relay 已在執行。';return}
    $ip=$script:wslBox.Text.Trim();$lidar=$script:lidarBox.Text.Trim()
    try {
        [void][System.Net.IPAddress]::Parse($ip)
        [void][System.Net.IPAddress]::Parse($lidar)
        $port=[int]$script:portBox.Text.Trim()
        if ($port -lt 1 -or $port -gt 65535) {throw 'UDP Port 超出有效範圍'}
    } catch {Say ('參數不正確：'+$_);return}
    try {
        if (Test-Path -LiteralPath $script:statusFile) {Remove-Item -LiteralPath $script:statusFile -Force}
        $arg='-NoProfile -ExecutionPolicy Bypass -File "'+$script:relayScript+'" -WslIp '+$ip+' -LidarIp '+$lidar+' -Port '+$port+' -StatusFile "'+$script:statusFile+'"'
        $script:relayProc=Start-Process -FilePath 'powershell.exe' -ArgumentList $arg -PassThru -WindowStyle Hidden
        Say ('Relay 啟動請求：'+$lidar+':'+$port+' → '+$ip+':'+$port+'，PID='+$script:relayProc.Id)
    } catch {Say ('Relay 啟動失敗：'+$_)}
}
function Stop-Relay {
    if ($script:relayProc -and -not $script:relayProc.HasExited) {
        try {Stop-Process -Id $script:relayProc.Id -Force -ErrorAction Stop;Say '已停止本 GUI 啟動的 UDP Relay。'}
        catch {Say ('停止 Relay 失敗：'+$_)}
    }
    $script:relayProc=$null
    $script:relayStatus.Text='UDP Relay：已停止'
}
function Install-WslGui {
    $distro=$script:distroBox.Text.Trim();$user=$script:userBox.Text.Trim()
    $folder=[System.IO.Path]::GetFullPath((Join-Path $script:root '..\WSL2'))
    try {
        # Avoid passing Windows backslashes through wsl.exe to wslpath.
        # On some Windows/WSL versions native argument marshaling strips the
        # backslashes and produces: wslpath: C:Users...WSL2.
        if ($folder -match '^([A-Za-z]):\\(.*)$') {
            $drive=$Matches[1].ToLowerInvariant()
        } else {
            throw ('本版只支援放在磁碟機路徑（例如 C:\... 或 D:\...）的測試包：'+$folder)
        }
        $relative=$Matches[2].Replace('\','/')
        $linuxSource='/mnt/'+$drive+'/'+$relative
        Say ('WSL 安裝來源：'+$linuxSource)
        # $1 is the source directory; do not evaluate user-supplied paths as shell script.
        $linuxCmd='set -e; src="$1"; test -f "$src/hunter_online_gui.py" || { echo "找不到 GUI 程式: $src" >&2; exit 2; }; mkdir -p "$HOME/hunter_online_gui"; cp -a -- "$src/." "$HOME/hunter_online_gui/"; chmod +x "$HOME/hunter_online_gui/start_hunter_online_gui.sh"'
        $result=(& wsl.exe -d $distro -u $user -- bash -c $linuxCmd _ $linuxSource 2>&1 | Out-String)
        if ($LASTEXITCODE -ne 0) {throw $result}
        Say ('已安裝 WSL GUI 到 '+$user+':~/hunter_online_gui')
    } catch {Say ('安裝失敗：'+$_)}
}
function Open-WslGui {
    $distro=$script:distroBox.Text.Trim();$user=$script:userBox.Text.Trim()
    try {
        $args='-d "'+$distro+'" -u "'+$user+'" -- bash -lc "python3 ~/hunter_online_gui/hunter_online_gui.py"'
        [void](Start-Process -FilePath 'wsl.exe' -ArgumentList $args -WindowStyle Hidden -PassThru)
        Say '已要求啟動 WSL2 GUI。如果沒有視窗，請檢查是否已完成安裝及 PyQt5 依賴。'
    } catch {Say ('啟動 WSL GUI 失敗：'+$_)}
}

$form=New-Object System.Windows.Forms.Form
$form.Text='Hunter Device Bridge - Windows 11'
$form.Size=New-Object System.Drawing.Size(860,770)
$form.StartPosition='CenterScreen'
$form.BackColor=[System.Drawing.Color]::FromArgb(16,24,38)
$form.ForeColor=[System.Drawing.Color]::White
$form.Font=New-Object System.Drawing.Font('Microsoft JhengHei UI',10)

$title=LabelAt 'HUNTER DEVICE BRIDGE  |  Windows' 22 18 780
$title.Font=New-Object System.Drawing.Font('Microsoft JhengHei UI',16,[System.Drawing.FontStyle]::Bold)
$title.ForeColor=[System.Drawing.Color]::FromArgb(75,219,198)
$form.Controls.Add($title)
$form.Controls.Add((LabelAt 'WSL 發行版' 26 70 105))
$script:distroBox=TextAt 'Autoware-18.04' 130 68 180; $form.Controls.Add($script:distroBox)
$form.Controls.Add((LabelAt 'Linux 使用者' 325 70 110))
$script:userBox=TextAt 'cobe' 440 68 135; $form.Controls.Add($script:userBox)

$canTitle=LabelAt '01  USB-CAN (candleLight / 1d50:606f)' 26 119 760
$canTitle.Font=New-Object System.Drawing.Font('Microsoft JhengHei UI',12,[System.Drawing.FontStyle]::Bold)
$form.Controls.Add($canTitle)
$script:canStatus=LabelAt 'CAN：尚未偵測' 26 152 790;$script:canStatus.AutoSize=$false;$form.Controls.Add($script:canStatus)
$scan=ButtonAt '掃描 USB-CAN' 26 187 160;$form.Controls.Add($scan);$scan.Add_Click({Get-Usb})
$bind=ButtonAt 'Bind（管理員）' 198 187 160;$form.Controls.Add($bind);$bind.Add_Click({Bind-Can})
$attach=ButtonAt 'Attach CAN' 370 187 145;$form.Controls.Add($attach);$attach.Add_Click({Attach-Can})
$detach=ButtonAt 'Detach' 527 187 140;$form.Controls.Add($detach);$detach.Add_Click({Detach-Can})

$relayTitle=LabelAt '02  VLP-16 UDP Relay (Windows → WSL2)' 26 251 760
$relayTitle.Font=New-Object System.Drawing.Font('Microsoft JhengHei UI',12,[System.Drawing.FontStyle]::Bold)
$form.Controls.Add($relayTitle)
$form.Controls.Add((LabelAt 'LiDAR IP' 26 295 115))
$script:lidarBox=TextAt '192.168.1.14' 145 293 185;$form.Controls.Add($script:lidarBox)
$form.Controls.Add((LabelAt 'WSL2 eth0 IP' 351 295 155))
$script:wslBox=TextAt '' 510 293 195;$form.Controls.Add($script:wslBox)
$form.Controls.Add((LabelAt 'UDP Port' 26 331 115))
$script:portBox=TextAt '2368' 145 329 100;$form.Controls.Add($script:portBox)
$ipButton=ButtonAt '自動取得 WSL IP' 351 326 196;$form.Controls.Add($ipButton);$ipButton.Add_Click({Get-WslIP})
$relayStart=ButtonAt 'Start Relay' 26 372 160;$form.Controls.Add($relayStart);$relayStart.Add_Click({Start-Relay})
$relayStop=ButtonAt 'Stop Relay' 198 372 160;$form.Controls.Add($relayStop);$relayStop.Add_Click({Stop-Relay})
$script:relayStatus=LabelAt 'UDP Relay：尚未啟動' 26 419 795;$script:relayStatus.AutoSize=$false;$form.Controls.Add($script:relayStatus)

$wslTitle=LabelAt '03  WSL2 GUI' 26 462 760
$wslTitle.Font=New-Object System.Drawing.Font('Microsoft JhengHei UI',12,[System.Drawing.FontStyle]::Bold)
$form.Controls.Add($wslTitle)
$install=ButtonAt '安裝 / 更新 WSL GUI' 26 498 220;$form.Controls.Add($install);$install.Add_Click({Install-WslGui})
$open=ButtonAt '開啟 WSL GUI' 259 498 180;$form.Controls.Add($open);$open.Add_Click({Open-WslGui})
$quick=ButtonAt '快速啟動 Bridge' 452 498 220;$form.Controls.Add($quick);$quick.Add_Click({Get-Usb;Attach-Can;Get-WslIP;Start-Relay})
$note=LabelAt '注意：VeloView 請先停止接收；CAN Bind 需要 UAC。關閉本視窗會停止此視窗啟動的 UDP Relay。' 26 550 790
$form.Controls.Add($note)
$script:logBox=New-Object System.Windows.Forms.TextBox
$script:logBox.Multiline=$true;$script:logBox.ReadOnly=$true
$script:logBox.ScrollBars='Vertical';$script:logBox.WordWrap=$false
$script:logBox.Location=New-Object System.Drawing.Point(26,590)
$script:logBox.Size=New-Object System.Drawing.Size(785,130)
$script:logBox.BackColor=[System.Drawing.Color]::FromArgb(7,17,32)
$script:logBox.ForeColor=[System.Drawing.Color]::White
$form.Controls.Add($script:logBox)

$timer=New-Object System.Windows.Forms.Timer
$timer.Interval=1300
$timer.Add_Tick({
    if ($script:relayProc -and $script:relayProc.HasExited) {
        Say ('Relay 程序已結束 (exit='+$script:relayProc.ExitCode+')')
        $script:relayProc=$null
    }
    if ($script:relayProc -and (Test-Path -LiteralPath $script:statusFile)) {
        try {
            $status=Get-Content -LiteralPath $script:statusFile -Raw -Encoding UTF8 | ConvertFrom-Json
            $age=([DateTime]::UtcNow-[DateTime]::Parse($status.updated_utc).ToUniversalTime()).TotalSeconds
            if ($script:relayProc -and $age -lt 6) {
                $script:relayStatus.Text='UDP Relay：'+$status.state+' | Forwarded '+$status.forwarded+' | Ignored '+$status.ignored+' | '+$status.wsl_ip+':'+$status.port
            } elseif ($script:relayProc) {
                $script:relayStatus.Text='UDP Relay：心跳逾時，請重新啟動 Relay'
            }
            if ($status.state -eq 'error') { $script:relayStatus.Text='UDP Relay：錯誤 '+$status.detail }
        } catch {}
    }
})
$timer.Start()
$form.Add_FormClosing({Stop-Relay;$timer.Stop()})
Say 'Bridge GUI 已啟動。建議先掃描 USB-CAN、讀取 WSL IP，再啟動 Relay。'
[void]$form.ShowDialog()
