@echo off
cd /d "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0hunter_bridge_gui.ps1"
if errorlevel 1 pause
