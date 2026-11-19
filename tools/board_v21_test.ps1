param()
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Push-Location $repoRoot
try {
    $cc=Join-Path $repoRoot 'build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe'
    if(!(Test-Path -LiteralPath $cc)){throw 'Host Zig compiler is missing'}
    $cases=@(
        @{Name='sensor_boot_test';Args=@('-Itools/sensor_boot_fakes','-Itools/host_fakes/system','-Imain','-Imain/include','-Itools/host_fakes','tools/sensor_boot_test.c')},
        @{Name='board_power_test';Args=@('-DCONFIG_EXO_BOARD_V21=1','-Itools/board_power_fakes','-Imain/include','-Itools/host_fakes','main/exo_board_power.c','tools/board_power_test.c')},
        @{Name='board_attitude_test';Args=@('-Itools/board_attitude_fakes','-Imain','main/exo_attitude.c','tools/board_attitude_test.c')},
        @{Name='board_sensor_protocol_test';Args=@('-Imain/include','tools/board_sensor_protocol_test.c','main/exo_ble_protocol.c')},
        @{Name='rs05_profile_test';Args=@('-Itools/host_fakes/system','-Imain/include','-Itools/host_fakes','tools/rs05_profile_test.c')},
        @{Name='rs01_peak_profile_test';Args=@('-Itools/host_fakes/system','-Imain/include','-Itools/host_fakes','tools/rs01_peak_profile_test.c')},
        @{Name='rs01_feedback_frames_test';Args=@('-Itools/host_fakes/system','-Imain/include','-Itools/host_fakes','tools/phase10_feedback_frames_replay.c')}
    )
    foreach($case in $cases){
        $exe=Join-Path $repoRoot ('build/'+$case.Name+'.exe')
        & $cc cc -std=c11 -Wall -Wextra -Werror @($case.Args) -lm -o $exe
        if($LASTEXITCODE -ne 0){throw ('Compile failed: '+$case.Name)}
        & $exe
        if($LASTEXITCODE -ne 0){throw ('Test failed: '+$case.Name)}
    }
} finally {Pop-Location}
