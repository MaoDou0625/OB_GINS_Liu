Param(
  [string]$Yaml = 'OB_GINS/config/ob_gins_wheel.yaml'
)

$ErrorActionPreference = 'Stop'

function Get-YamlValue([string]$path, [string]$key){
  # very simple YAML scalar reader: key: "value"
  $re = '^[\s]*' + [regex]::Escape($key) + ':[\s]*"(.*)"'
  foreach($ln in Get-Content $path){
    if($ln -match $re){ return $Matches[1] }
  }
  return $null
}

Write-Host "YAML:" $Yaml
if(!(Test-Path $Yaml)){ throw "YAML not found: $Yaml" }

$outputpath = Get-YamlValue $Yaml 'outputpath'
$gnssfile   = Get-YamlValue $Yaml 'gnssfile'
if(-not $outputpath){ throw "outputpath not found in YAML" }
if(-not $gnssfile){ throw "gnssfile not found in YAML" }

# Prefer Release binary if available
$exe = $null
$candidates = @(
  (Join-Path 'OB_GINS/bin/Release' 'ob_gins.exe'),
  (Join-Path 'OB_GINS/bin' 'ob_gins.exe'),
  (Join-Path 'OB_GINS/bin/Debug' 'ob_gins.exe')
)
foreach($c in $candidates){ if(Test-Path $c){ $exe=$c; break } }
if(-not $exe){ throw "Binary not found in: $($candidates -join ', ') (build first)" }

# Run navigation
Write-Host "Running OB_GINS..." -ForegroundColor Cyan
& $exe $Yaml

# Prepare truth 200Hz path
$truth_nav = $null
if($gnssfile -match '^(.*)[/\\]GINS_all\.pos$'){
  $truth_nav = (Join-Path $Matches[1] 'truth_200hz.nav')
} else {
  $truth_nav = [IO.Path]::Combine([IO.Path]::GetDirectoryName($gnssfile), 'truth_200hz.nav')
}

# If missing, convert from GINS.bin in same folder
if(!(Test-Path $truth_nav)){
  $bin = [IO.Path]::Combine([IO.Path]::GetDirectoryName($gnssfile), 'GINS.bin')
  Write-Host "Converting truth from:" $bin -ForegroundColor Yellow
  python OB_GINS/tools/convert_gins_bin.py $bin
}

# Compare and plot
$nav = Join-Path $outputpath 'OB_GINS_TXT.nav'
if(!(Test-Path $nav)){ throw "Nav not found: $nav" }
New-Item -ItemType Directory -Force -Path $outputpath | Out-Null
$csv = Join-Path $outputpath 'nav_truth_xyz.csv'
$png = Join-Path $outputpath 'nav_truth_xyz.png'
python OB_GINS/tools/compare_nav_truth_xyz.py $nav $truth_nav --out $csv --plot $png

Write-Host "Saved CSV:" $csv -ForegroundColor Green
Write-Host "Saved PNG:" $png -ForegroundColor Green
