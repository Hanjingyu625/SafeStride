[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$InterfaceAlias,

    [ValidateSet('Direct', 'Dhcp')]
    [string]$Mode = 'Direct'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Open PowerShell as Administrator and run this script again.'
}

$adapter = Get-NetAdapter -Name $InterfaceAlias -ErrorAction Stop
if ($adapter.Status -eq 'Disabled') {
    Enable-NetAdapter -Name $InterfaceAlias -Confirm:$false
}

$firewallName = 'SafeStride ROS 2 DDS (Direct Ethernet)'
$pcAddress = '10.42.0.1'
$piAddress = '10.42.0.2'

if ($Mode -eq 'Direct') {
    $addressOwner = Get-NetIPAddress -AddressFamily IPv4 -IPAddress $pcAddress -ErrorAction SilentlyContinue |
        Where-Object { $_.InterfaceAlias -ne $InterfaceAlias }
    if ($addressOwner) {
        throw "$pcAddress is already assigned to another adapter: $($addressOwner.InterfaceAlias)"
    }

    Set-NetIPInterface -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -Dhcp Disabled
    $existing = Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -IPAddress $pcAddress -ErrorAction SilentlyContinue
    if (-not $existing) {
        New-NetIPAddress -InterfaceAlias $InterfaceAlias -IPAddress $pcAddress -PrefixLength 24 | Out-Null
    }

    $profile = Get-NetConnectionProfile -InterfaceAlias $InterfaceAlias -ErrorAction SilentlyContinue
    if ($profile) {
        Set-NetConnectionProfile -InterfaceAlias $InterfaceAlias -NetworkCategory Private
    }

    Get-NetFirewallRule -DisplayName $firewallName -ErrorAction SilentlyContinue |
        Remove-NetFirewallRule -ErrorAction SilentlyContinue
    New-NetFirewallRule `
        -DisplayName $firewallName `
        -Direction Inbound `
        -Action Allow `
        -Enabled True `
        -Profile Any `
        -Protocol UDP `
        -LocalAddress $pcAddress `
        -RemoteAddress '10.42.0.0/24' | Out-Null

    Write-Host "Direct Ethernet configured: PC $pcAddress/24, Raspberry Pi $piAddress/24"
    Write-Host "Next: ping $piAddress"
    Write-Host "Then: ssh <pi-user>@$piAddress"
}
else {
    Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -IPAddress $pcAddress -ErrorAction SilentlyContinue |
        Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
    Set-NetIPInterface -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -Dhcp Enabled
    Set-DnsClientServerAddress -InterfaceAlias $InterfaceAlias -ResetServerAddresses
    Get-NetFirewallRule -DisplayName $firewallName -ErrorAction SilentlyContinue |
        Remove-NetFirewallRule -ErrorAction SilentlyContinue

    Write-Host "DHCP restored on adapter '$InterfaceAlias'."
}
