Param(
  [switch]$Clean
)

$ErrorActionPreference = 'Stop'

$root = Resolve-Path '.'
$src = Join-Path $root 'OB_GINS'
$bd  = Join-Path $src  'build_debug'
$br  = Join-Path $src  'build_release'

if($Clean){
  if(Test-Path $bd){ Remove-Item -Recurse -Force $bd }
  if(Test-Path $br){ Remove-Item -Recurse -Force $br }
}

Write-Host '=== Configure Debug ===' -ForegroundColor Cyan
cmake -S $src -B $bd -DCMAKE_BUILD_TYPE=Debug
Write-Host '=== Build Debug ===' -ForegroundColor Cyan
cmake --build $bd --config Debug -j 8

Write-Host '=== Configure Release ===' -ForegroundColor Cyan
cmake -S $src -B $br -DCMAKE_BUILD_TYPE=Release
Write-Host '=== Build Release ===' -ForegroundColor Cyan
cmake --build $br --config Release -j 8

Write-Host 'Done. Binaries will be under OB_GINS/bin/(Release|Debug)' -ForegroundColor Green

