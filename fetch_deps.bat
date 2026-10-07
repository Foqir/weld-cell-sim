@echo off
REM Clones the official HD Hyundai Robotics packages into ros2_ws\src at the pinned commits (see hdr.repos).
REM Run once before build.bat. Already cloned packages are only switched to the pinned commit.
cd /d "%~dp0ros2_ws\src"
call :get hdr_description   8512ee3a110e9e75967d3e1fda8477d0ba6fcd05
call :get hdr_client_driver 047edf62ec9bb9bb04aaf788f58fc9ce92cd2816
call :get hdr_ros2_driver   38d627fc8a3edccd96a4b34f46122a817de9ed2b
call :get hdr_simulation_gz 1014ffe7be3efa3a301d64708be674082f36e64f
echo Done.
pause
exit /b
:get
if not exist %1 git clone https://github.com/hyundai-robotics/%1.git %1
git -C %1 fetch --quiet origin
git -C %1 checkout --quiet %2 && echo   [ok] %1 @ %2
exit /b
