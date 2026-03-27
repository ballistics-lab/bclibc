Write-Host ">>> Cleaning build directory..." -ForegroundColor Yellow

$BuildDir = "build"
if (Test-Path $BuildDir) {
    Remove-Item -Recurse -Force $BuildDir
    Write-Host "   Removed $BuildDir" -ForegroundColor Green
} else {
    Write-Host "   Build directory not found" -ForegroundColor Yellow
}

Write-Host ">>> Clean complete!" -ForegroundColor Green
