@echo off
echo Killing ADB server...
start "" /b adb kill-server

echo Waiting for 5 seconds...
timeout /t 5 /nobreak >nul

echo Starting ADB server...
adb start-server

echo ADB restart process completed.