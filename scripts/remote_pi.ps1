[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PiUser,

    [string]$PiHost = '10.42.0.2',

    [string]$Workspace = '',

    [ValidateSet('Info', 'Build', 'Test', 'Run', 'InstallService', 'Start', 'Stop', 'Restart', 'Status', 'Logs', 'Topics')]
    [string]$Action = 'Info'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ($PiUser -notmatch '^[a-z_][a-z0-9_-]*$') {
    throw 'PiUser contains unsupported characters.'
}
if ($PiHost -notmatch '^[A-Za-z0-9.-]+$') {
    throw 'PiHost must be an IPv4 address or a simple DNS/mDNS host name.'
}
if ([string]::IsNullOrWhiteSpace($Workspace)) {
    $Workspace = "/home/$PiUser/SafeStride"
}
if ($Workspace -notmatch '^/[A-Za-z0-9._/-]+$') {
    throw 'Workspace must be an absolute Linux path without spaces or shell characters.'
}

$ssh = Get-Command ssh.exe -ErrorAction Stop
$target = "$PiUser@$PiHost"
$rosEnvironment = "source /opt/ros/jazzy/setup.bash; if [[ -f $Workspace/install/setup.bash ]]; then source $Workspace/install/setup.bash; fi; unset ROS_LOCALHOST_ONLY; export ROS_DOMAIN_ID=42; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"

$commands = @{
    Info = "hostname; ip -brief -4 address; systemctl is-active ssh.service; if [[ -f /opt/ros/jazzy/setup.bash ]]; then bash -lc 'source /opt/ros/jazzy/setup.bash; printf ROS_DISTRO=; printenv ROS_DISTRO'; else echo 'ROS 2 Jazzy is not installed'; fi"
    Build = "cd $Workspace; bash scripts/build.sh"
    Test = "cd $Workspace; bash scripts/test.sh"
    Run = "cd $Workspace; bash scripts/run.sh"
    InstallService = "cd $Workspace; bash scripts/install_service.sh; sudo systemctl enable --now safestride.service"
    Start = 'sudo systemctl start safestride.service'
    Stop = 'sudo systemctl stop safestride.service'
    Restart = 'sudo systemctl restart safestride.service'
    Status = 'systemctl status safestride.service --no-pager'
    Logs = 'sudo journalctl -u safestride.service -n 200 --no-pager'
    Topics = "bash -lc '$rosEnvironment; ros2 daemon stop >/dev/null 2>&1 || true; echo ROS_NODES; ros2 node list; echo ROS_TOPICS; ros2 topic list'"
}

$interactiveActions = @('InstallService', 'Start', 'Stop', 'Restart', 'Logs')
$sshArguments = @('-o', 'ConnectTimeout=5')
if ($interactiveActions -contains $Action) {
    $sshArguments += '-t'
}
$sshArguments += @($target, $commands[$Action])

Write-Host "[$Action] $target ($Workspace)"
& $ssh.Source @sshArguments
if ($LASTEXITCODE -ne 0) {
    throw "SSH command failed with exit code $LASTEXITCODE."
}
