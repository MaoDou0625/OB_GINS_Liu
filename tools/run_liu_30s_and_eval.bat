@echo off
setlocal enabledelayedexpansion

REM Defaults (override via args or env)
set VS_PATH_DEFAULT=D:\VisualStudio\2022\VC\Auxiliary\Build\vcvars64.bat
set VCPKG_TOOLCHAIN_DEFAULT=D:/vcpkgforOB/vcpkg/scripts/buildsystems/vcpkg.cmake
set YAML_DEFAULT=OB_GINS\config\run_liu_30s_no_odo.yaml
set NAV_DEFAULT=OB_GINS\out\liu_30s_no_odo\OB_GINS_TXT.nav
set TRU_DEFAULT=OB_GINS\dataset\truth.nav
set OUT_DEFAULT=OB_GINS\out\liu_30s_no_odo\nav_vs_truth_200hz.csv

REM Read env overrides if provided
set VS_PATH=!VS_PATH!
set VCPKG_TOOLCHAIN=!VCPKG_TOOLCHAIN!
set YAML=!YAML!
set NAV=!NAV!
set TRU=!TRU!
set OUT=!OUT!

REM Parse command-line options
:parse
if "%~1"=="" goto parsed
if /I "%~1"=="--vs"        ( set VS_PATH=%~2 & shift & shift & goto parse )
if /I "%~1"=="--toolchain" ( set VCPKG_TOOLCHAIN=%~2 & shift & shift & goto parse )
if /I "%~1"=="--yaml"      ( set YAML=%~2 & shift & shift & goto parse )
if /I "%~1"=="--nav"       ( set NAV=%~2 & shift & shift & goto parse )
if /I "%~1"=="--truth"     ( set TRU=%~2 & shift & shift & goto parse )
if /I "%~1"=="--out"       ( set OUT=%~2 & shift & shift & goto parse )
if /I "%~1"=="--help"  ( goto :help )
if /I "%~1"=="-h"      ( goto :help )
echo Unknown option: %1
goto :help

:parsed
if not defined VS_PATH set VS_PATH=%VS_PATH_DEFAULT%
if not defined VCPKG_TOOLCHAIN set VCPKG_TOOLCHAIN=%VCPKG_TOOLCHAIN_DEFAULT%
if not defined YAML set YAML=%YAML_DEFAULT%
if not defined NAV set NAV=%NAV_DEFAULT%
if not defined TRU set TRU=%TRU_DEFAULT%
if not defined OUT set OUT=%OUT_DEFAULT%

echo Using VS script: %VS_PATH%
echo Using vcpkg toolchain: %VCPKG_TOOLCHAIN%
echo YAML: %YAML%
echo NAV OUT: %NAV%
echo TRUTH: %TRU%
echo ERR CSV: %OUT%

call "%VS_PATH%"
if errorlevel 1 (
  echo Failed to initialize MSVC environment.
  exit /b 1
)

REM Configure & build
cmake -S OB_GINS -B OB_GINS\build -G "NMake Makefiles" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_TOOLCHAIN_FILE=%VCPKG_TOOLCHAIN% ^
  -DVCPKG_TARGET_TRIPLET=x64-windows
if errorlevel 1 exit /b 1

cmake --build OB_GINS\build -- -s
if errorlevel 1 exit /b 1

REM Run demo
OB_GINS\bin\ob_gins.exe %YAML%
if errorlevel 1 exit /b 1

REM 200 Hz evaluation vs truth.nav
python OB_GINS\tools\compare_nav_truth_200hz.py %NAV% %TRU% --out %OUT%

endlocal
exit /b %errorlevel%

:help
echo Usage: %~nx0 [--vs "^<vcvars64.bat^>"] [--toolchain "^<vcpkg cmake toolchain^>"] ^
echo        [--yaml ^<config.yaml^>] [--nav ^<OB_GINS_TXT.nav^>] [--truth ^<truth.nav^>] [--out ^<csv^>]
exit /b 0
