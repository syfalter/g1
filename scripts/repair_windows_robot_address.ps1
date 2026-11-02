param(
    [Parameter(Mandatory=$true)][string]$MacAddress,
    [string]$Address = '192.168.123.99'
)
$ErrorActionPreference = 'Stop'
$addressValue = [System.Net.IPAddress]::Parse($Address)
if ($addressValue.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) {
    throw 'An IPv4 address is required.'
}
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script in an elevated Windows PowerShell.'
}
$adapters = @(Get-NetAdapter | Where-Object { $_.MacAddress -eq $MacAddress })
if ($adapters.Count -ne 1 -or $adapters[0].Status -ne 'Up') {
    throw 'Expected exactly one connected adapter with the supplied MAC address.'
}
$adapter = $adapters[0]
$existing = @(Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -eq $Address })
if (@($existing | Where-Object { $_.InterfaceIndex -ne $adapter.ifIndex }).Count) {
    throw 'Requested address is already assigned to another Windows adapter.'
}
$directory = Join-Path $PSScriptRoot '../data/task_panel/network_repair'
New-Item -ItemType Directory -Force -Path $directory | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4 |
    Select-Object InterfaceIndex,InterfaceAlias,IPAddress,PrefixLength,PrefixOrigin |
    ConvertTo-Json | Set-Content (Join-Path $directory "$stamp-before.json")
if (-not $existing.Count) {
    # No default gateway: this interface is only for the robot subnet.
    New-NetIPAddress -InterfaceIndex $adapter.ifIndex -IPAddress $Address -PrefixLength 24 | Out-Null
}
Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4 |
    Select-Object InterfaceIndex,InterfaceAlias,IPAddress,PrefixLength,AddressState |
    ConvertTo-Json | Set-Content (Join-Path $directory "$stamp-after.json")
