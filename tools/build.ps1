# Command line build for the CH32V307 FreeRTOS robot firmware.
#
#   .\tools\build.ps1
#   .\tools\build.ps1 -ToolchainDir "C:\path\to\toolchain\bin"
#
# The MounRiver Studio build into obj\ is untouched; this script only exists so
# the project can be verified without the IDE.

param(
    [string]$ToolchainDir = $env:MRS_TOOLCHAIN_DIR,
    [string]$BuildDir = "build"
)

$ErrorActionPreference = "Stop"

if (-not $ToolchainDir) {
    $ToolchainDir = "E:\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC\bin"
}

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$build = Join-Path $root $BuildDir
$cc = Join-Path $ToolchainDir "riscv-none-embed-gcc.exe"
$objcopy = Join-Path $ToolchainDir "riscv-none-embed-objcopy.exe"
$size = Join-Path $ToolchainDir "riscv-none-embed-size.exe"

foreach ($tool in @($cc, $objcopy, $size)) {
    if (-not (Test-Path -LiteralPath $tool)) {
        throw "Toolchain binary not found: $tool"
    }
}

New-Item -ItemType Directory -Force -Path $build | Out-Null

$includeDirs = @(
    "Debug",
    "Core",
    "User",
    "Peripheral/inc",
    "FreeRTOS",
    "FreeRTOS/include",
    "FreeRTOS/portable/Common",
    "FreeRTOS/portable/GCC/RISC-V",
    "FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions",
    "FreeRTOS/portable/MemMang"
)
$includes = $includeDirs | ForEach-Object { "-I" + (Join-Path $root $_) }

$cflags = @(
    "-march=rv32imacxw",
    "-mabi=ilp32",
    "-msmall-data-limit=8",
    "-msave-restore",
    "-fmax-errors=20",
    "-Os",
    "-fmessage-length=0",
    "-fsigned-char",
    "-ffunction-sections",
    "-fdata-sections",
    "-fno-common",
    "-Wunused",
    "-Wuninitialized",
    "-g",
    "-std=gnu99"
)

$cSources = @()
$cSources += Get-ChildItem (Join-Path $root "Core") -Filter *.c -File
$cSources += Get-ChildItem (Join-Path $root "Debug") -Filter *.c -File
$cSources += Get-ChildItem (Join-Path $root "FreeRTOS") -Filter *.c -File
$cSources += Get-Item (Join-Path $root "FreeRTOS/portable/GCC/RISC-V/port.c")
$cSources += Get-Item (Join-Path $root "FreeRTOS/portable/MemMang/heap_4.c")
$cSources += Get-ChildItem (Join-Path $root "Peripheral/src") -Filter *.c -File
$cSources += Get-ChildItem (Join-Path $root "User") -Filter *.c -File -Recurse

$asmSources = @(
    (Join-Path $root "Startup/startup_ch32v30x_D8C.S"),
    (Join-Path $root "FreeRTOS/portable/GCC/RISC-V/portASM.S")
)

$objects = @()

foreach ($source in ($cSources + $asmSources)) {
    if ($source -is [string]) {
        $path = $source
    }
    else {
        $path = $source.FullName
    }

    $relative = $path.Substring($root.Length + 1)
    $objectName = ($relative -replace "[\\/]", "_") + ".o"
    $objectPath = Join-Path $build $objectName

    Write-Host ("CC  " + $relative)
    & $cc @cflags @includes -c $path -o $objectPath
    if ($LASTEXITCODE -ne 0) {
        throw "Compile failed: $relative"
    }

    $objects += $objectPath
}

$elf = Join-Path $build "CH32V307-FreeRTOS.elf"
$map = Join-Path $build "CH32V307-FreeRTOS.map"
$hex = Join-Path $build "CH32V307-FreeRTOS.hex"

Write-Host "LD  CH32V307-FreeRTOS.elf"
& $cc @cflags -T (Join-Path $root "Ld/Link.ld") -nostartfiles -Xlinker --gc-sections `
    "-Wl,-Map,$map" --specs=nano.specs --specs=nosys.specs `
    -o $elf @objects -lm
if ($LASTEXITCODE -ne 0) {
    throw "Link failed"
}

& $objcopy -O ihex $elf $hex
if ($LASTEXITCODE -ne 0) {
    throw "objcopy failed"
}

& $size --format=berkeley $elf
