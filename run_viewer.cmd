@echo off
"%~dp0Bin\MediaPipe2ManoQt.exe" --root "%~dp0." --mode viewer %*
if errorlevel 1 pause
