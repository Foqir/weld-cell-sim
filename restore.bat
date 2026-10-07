@echo off
REM Restores the ready image from backup\weld-cell-sim.tar (no internet needed).
cd /d "%~dp0"
docker load -i backup\weld-cell-sim.tar
pause
