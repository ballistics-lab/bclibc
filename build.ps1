# build.ps1 - для Windows
param(
    [ValidateSet("Release", "Debug")]
    [string]$Configuration = "Release"
)

Write-Host ">>> Building bclibc for Windows..." -ForegroundColor Green

$BuildDir = "build"
$Generator = "Visual Studio 17 2022"
$Arch = "x64"

# Create build directory
if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

Set-Location $BuildDir

# Configure CMake
Write-Host "--- Configuring project ---" -ForegroundColor Green
cmake -G $Generator -A $Arch -DCMAKE_BUILD_TYPE=$Configuration ..

# Build
Write-Host "--- Building targets (Core & FFI) ---" -ForegroundColor Green
cmake --build . --config $Configuration

# Validate
Write-Host "------------------------------------------" -ForegroundColor Green
$LibCore = "lib\${Configuration}\bclibc_core.lib"
$LibFFI = "bin\${Configuration}\bclibc_ffi.dll"

if ((Test-Path $LibCore) -and (Test-Path $LibFFI)) {
    Write-Host "SUCCESS: Libraries generated successfully!" -ForegroundColor Green
    Write-Host "Static core:  $BuildDir\$LibCore"
    Write-Host "Shared FFI:   $BuildDir\$LibFFI"
    
    # Check exported symbols (optional)
    Write-Host "--- Exported FFI symbols: ---" -ForegroundColor Green
    & dumpbin /exports $LibFFI | Select-String "BCLIBCFFI_"
} else {
    Write-Host "ERROR: Build failed, libraries not found." -ForegroundColor Red
    exit 1
}

Set-Location ..