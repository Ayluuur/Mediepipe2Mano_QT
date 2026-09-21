@echo off
cd /d "%~dp0"
bin\MediaPipe2ManoQt.exe --mode parallel_ik %*
