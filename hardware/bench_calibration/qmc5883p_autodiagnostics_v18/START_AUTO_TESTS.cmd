@echo off
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\run_auto_tests.ps1" -Port COM3 -Suite triage -PowerMode warm
exit /b %ERRORLEVEL%
