@echo off
cd /d "%~dp0"
Bin\MediaPipe2ManoQt.exe --mode parallel_ik %*
