@echo off
"%~dp0bin\MediaPipe2ManoQt.exe" --root "%~dp0." --mode interaction %*
if errorlevel 1 pause
