@echo off
REM Builds the weld-cell-sim image stage by stage with backups in the backup\ folder.
REM Does not touch the px4-vtol-sim (UAV) project or its image.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1"
pause
