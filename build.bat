@echo off
REM build.bat - для Windows (CMD)

setlocal enabledelayedexpansion
set CONFIGURATION=%1
if "%CONFIGURATION%"=="" set CONFIGURATION=Release
if not "%CONFIGURATION%"=="Release" if not "%CONFIGURATION%"=="Debug" (
    echo Usage: build.bat [Release^|Debug]
    exit /b 1
)

echo >>> Building bclibc for Windows (%CONFIGURATION%)...

set BUILD_DIR=build
set GENERATOR="Visual Studio 17 2022"
set ARCH=x64

if not exist %BUILD_DIR% mkdir %BUILD_DIR%
cd %BUILD_DIR%

echo --- Configuring project ---
cmake -G %GENERATOR% -A %ARCH% -DCMAKE_BUILD_TYPE=%CONFIGURATION% ..

echo --- Building targets (Core & FFI) ---
cmake --build . --config %CONFIGURATION%

echo ------------------------------------------
set LIB_CORE=lib\%CONFIGURATION%\bclibc_core.lib
set LIB_FFI=bin\%CONFIGURATION%\bclibc_ffi.dll

if exist %LIB_CORE% if exist %LIB_FFI% (
    echo SUCCESS: Libraries generated successfully!
    echo Static core:  %BUILD_DIR%\%LIB_CORE%
    echo Shared FFI:   %BUILD_DIR%\%LIB_FFI%
    
    echo --- Exported FFI symbols: ---
    dumpbin /exports %LIB_FFI% | findstr "BCLIBCFFI_"
) else (
    echo ERROR: Build failed, libraries not found.
    exit /b 1
)

cd ..