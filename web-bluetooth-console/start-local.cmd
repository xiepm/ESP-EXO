@echo off
setlocal
title ESP-EXO Stage 9 BLE Acceptance
cd /d "%~dp0"

where node.exe >nul 2>nul
if errorlevel 1 (
  echo Node.js was not found. Please install Node.js 22 or newer first.
  echo https://nodejs.org/
  pause
  exit /b 1
)

if not exist "node_modules\" (
  echo Installing local dependencies for the first run...
  call npm install
  if errorlevel 1 (
    echo Installation failed. Check the network connection and try again.
    pause
    exit /b 1
  )
)

start "" /min powershell.exe -NoProfile -WindowStyle Hidden -Command "$url='http://localhost:4173/exo'; for($i=0;$i -lt 60;$i++){try{$response=Invoke-WebRequest -UseBasicParsing -Uri $url -TimeoutSec 1;if($response.StatusCode -eq 200){Start-Process $url;break}}catch{};Start-Sleep -Milliseconds 500}"

echo.
echo ESP-EXO Stage 9 BLE Acceptance
echo Local address: http://localhost:4173/exo
echo Use Windows Chrome or Edge. If another browser opens, copy the URL above.
echo Keep this window open while using the acceptance page.
echo Press Ctrl+C to stop the local server.
echo.

call npm run local
