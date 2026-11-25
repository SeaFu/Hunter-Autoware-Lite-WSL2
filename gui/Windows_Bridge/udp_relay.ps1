# PowerShell 5.1 UDP receive/forward worker, launched by hunter_bridge_gui.ps1.
# Forwards only source 192.168.1.14 (configurable) from Windows UDP 2368 to WSL2.
param(
    [Parameter(Mandatory=$true)][string]$WslIp,
    [string]$LidarIp='192.168.1.14',
    [int]$Port=2368,
    [Parameter(Mandatory=$true)][string]$StatusFile
)
$ErrorActionPreference='Stop'
$receiver=$null
$sender=$null
$count=0L
$ignored=0L
$lastPacket=''
$lastWrite=[DateTime]::MinValue

function Publish-Status([string]$state, [string]$detail) {
    $payload=[ordered]@{
        state=$state
        detail=$detail
        wsl_ip=$WslIp
        lidar_ip=$LidarIp
        port=$Port
        forwarded=$script:count
        ignored=$script:ignored
        last_packet=$script:lastPacket
        updated_utc=[DateTime]::UtcNow.ToString('o')
        pid=$PID
    }
    $temp="$StatusFile.tmp.$PID"
    $payload | ConvertTo-Json -Compress | Set-Content -LiteralPath $temp -Encoding UTF8
    Move-Item -LiteralPath $temp -Destination $StatusFile -Force
}

try {
    [void][System.Net.IPAddress]::Parse($WslIp)
    [void][System.Net.IPAddress]::Parse($LidarIp)
    $receiver=New-Object System.Net.Sockets.UdpClient($Port)
    $receiver.Client.ReceiveBufferSize=4194304
    $receiver.Client.ReceiveTimeout=1000
    $sender=New-Object System.Net.Sockets.UdpClient
    $remote=New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any,0)
    Publish-Status 'running' 'UDP Relay 已啟動；等待封包'
    while ($true) {
        try {
            $data=$receiver.Receive([ref]$remote)
            if ($remote.Address.ToString() -eq $LidarIp) {
                [void]$sender.Send($data,$data.Length,$WslIp,$Port)
                $count++
                $lastPacket=[DateTime]::UtcNow.ToString('o')
            } else {
                $ignored++
            }
        } catch [System.Net.Sockets.SocketException] {
            if ($_.Exception.ErrorCode -ne 10060) { throw }
        }
        if (([DateTime]::UtcNow-$lastWrite).TotalSeconds -ge 1.5) {
            Publish-Status 'running' '轉送中'
            $lastWrite=[DateTime]::UtcNow
        }
    }
} catch {
    try { Publish-Status 'error' $_.Exception.Message } catch {}
    throw
} finally {
    if ($null -ne $receiver) { $receiver.Close() }
    if ($null -ne $sender) { $sender.Close() }
}
