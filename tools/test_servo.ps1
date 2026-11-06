param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$servoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Push-Location $servoRoot
try {
    New-Item -ItemType Directory -Force -Path "build/servo_tests" | Out-Null
    $common = @("-std=c99", "-O2", "-Wall", "-Wextra", "-Wno-unused-but-set-variable",
                "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
                "-Itests/servo_host", "-IUser")
    $cases = @(
        @{ Name="driver"; Source="test_servo.c"; Define=@() },
        @{ Name="boot"; Source="test_boot.c"; Define=@("-DTEST_SERVO_BOOT") },
        @{ Name="console"; Source="test_console.c"; Define=@() },
        @{ Name="missing"; Source="test_config.c"; Define=@("-DTEST_SERVO_MISSING") },
        @{ Name="calibrated"; Source="test_config.c"; Define=@("-DTEST_SERVO_CALIBRATED") },
        @{ Name="inverted"; Source="test_config.c"; Define=@("-DTEST_SERVO_INVERT") },
        @{ Name="bad_config"; Source="test_config.c"; Define=@("-DTEST_SERVO_BAD_CONFIG") }
    )
    foreach ($case in $cases) {
        $binary = Join-Path $servoRoot ("build/servo_tests/" + $case.Name + ".exe")
        $defines = $case.Define
        & $Compiler @common @defines ("tests/servo_host/" + $case.Source) -o $binary
        if ($LASTEXITCODE -ne 0) { throw ("Compilation failed: " + $case.Name) }
        & $binary
        if ($LASTEXITCODE -ne 0) { throw ("Test failed: " + $case.Name) }
    }
} finally { Pop-Location }
