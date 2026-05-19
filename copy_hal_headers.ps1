# Copy missing STM32F4 HAL headers into this project.
# Run from: Motor_4_integration folder. Optional: -SourceInc "C:\path\to\...\Inc"

param([string]$SourceInc)

$projectInc = Join-Path $PSScriptRoot "Drivers\STM32F4xx_HAL_Driver\Inc"

# If user passed path, use it
if ($SourceInc -and (Test-Path $SourceInc)) {
    if (Test-Path (Join-Path $SourceInc "stm32f4xx_hal_adc.h")) { $sourceInc = $SourceInc }
}

# Otherwise search common locations (shallow search to avoid slow scan)
if (-not $sourceInc) {
    $candidates = @()
    $packBase = Join-Path $env:USERPROFILE "STM32Cube\Repository\Packs\STMicroelectronics"
    if (Test-Path $packBase) {
        Get-ChildItem -Path $packBase -Directory -ErrorAction SilentlyContinue | ForEach-Object {
            $incPath = Join-Path $_.FullName "Drivers\STM32F4xx_HAL_Driver\Inc"
            if (Test-Path (Join-Path $incPath "stm32f4xx_hal_adc.h")) { $candidates += $incPath }
        }
    }
    $repoBase = Join-Path $env:USERPROFILE "STM32Cube\Repository"
    if (Test-Path $repoBase) {
        Get-ChildItem -Path $repoBase -Directory -Filter "STM32Cube_FW_F4*" -ErrorAction SilentlyContinue | ForEach-Object {
            $incPath = Join-Path $_.FullName "Drivers\STM32F4xx_HAL_Driver\Inc"
            if (Test-Path (Join-Path $incPath "stm32f4xx_hal_adc.h")) { $candidates += $incPath }
        }
    }
    if ($candidates.Count -gt 0) { $sourceInc = $candidates[0] }
}

if (-not $sourceInc) {
    Write-Host "Could not find STM32CubeF4 HAL Inc folder automatically." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Please either:" -ForegroundColor Cyan
    Write-Host "  1. Download STM32CubeF4 from https://www.st.com/en/embedded-software/stm32cubef4.html"
    Write-Host "  2. Unzip it, then run this script again with the path:"
    Write-Host "     .\copy_hal_headers.ps1 -SourceInc 'C:\path\to\STM32Cube_FW_F4_x.x.x\Drivers\STM32F4xx_HAL_Driver\Inc'"
    Write-Host ""
    Write-Host "  3. Or copy the folder manually - see COPY_MISSING_HAL_HEADERS.md"
    exit 1
}

Write-Host "Found HAL Inc at: $sourceInc" -ForegroundColor Green
Write-Host "Copying all .h files to: $projectInc" -ForegroundColor Green

if (-not (Test-Path $projectInc)) {
    New-Item -ItemType Directory -Path $projectInc -Force | Out-Null
}

$count = 0
Get-ChildItem -Path $sourceInc -Filter "*.h" | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination $projectInc -Force
    $count++
}
Write-Host "Copied $count header file(s). Done." -ForegroundColor Green
Write-Host "Rebuild your project in STM32CubeIDE (Project -> Clean, then Build)."
