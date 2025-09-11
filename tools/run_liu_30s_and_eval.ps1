param(
  [string]$vs = 'D:\VisualStudio\2022\VC\Auxiliary\Build\vcvars64.bat',
  [string]$toolchain = 'D:/vcpkgforOB/vcpkg/scripts/buildsystems/vcpkg.cmake',
  [string]$yaml = 'OB_GINS/config/run_liu_30s_no_odo.yaml',
  [string]$nav = 'OB_GINS/out/liu_30s_no_odo/OB_GINS_TXT.nav',
  [string]$truth = 'OB_GINS/dataset/truth.nav',
  [string]$out = 'OB_GINS/out/liu_30s_no_odo/nav_vs_truth_200hz.csv'
)

$ErrorActionPreference = 'Stop'

Write-Host "Using VS script: $vs"
Write-Host "Using vcpkg toolchain: $toolchain"
Write-Host "YAML: $yaml"; Write-Host "NAV OUT: $nav"; Write-Host "TRUTH: $truth"; Write-Host "ERR CSV: $out"

# Build a single cmd.exe command so that MSVC env persists for all steps
$segments = @(
  ('call "{0}"' -f $vs),
  ('cmake -S OB_GINS -B OB_GINS\build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="{0}" -DVCPKG_TARGET_TRIPLET=x64-windows' -f $toolchain),
  'cmake --build OB_GINS\build -- -s',
  ('OB_GINS\bin\ob_gins.exe "{0}"' -f $yaml),
  ('python OB_GINS\tools\compare_nav_truth_200hz.py "{0}" "{1}" --out "{2}"' -f $nav, $truth, $out)
)
$cmd = $segments -join ' && '

Write-Host "Running: cmd /c $cmd"
& cmd.exe /c $cmd
if ($LASTEXITCODE -ne 0) { throw "Pipeline failed with exit code $LASTEXITCODE" }

Write-Host "Done. Errors CSV at: $out"

