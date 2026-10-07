@echo off
REM Usage: start.bat [hdr20_17^|hh020^|hdr35_20] [--demo] [--headless] [--no-rviz] [--offset DX DY DYAW]
REM Starts the welding cell (Gazebo + robot + MoveIt + RViz) in one container. Type 'exit' to stop.
cd /d "%~dp0"
if not exist shared mkdir shared
docker rm -f weld-cell-sim >nul 2>&1
docker run -it --rm --name weld-cell-sim ^
  -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix ^
  -v /run/desktop/mnt/host/wslg:/mnt/wslg ^
  -e DISPLAY=:0 ^
  -e WAYLAND_DISPLAY=wayland-0 ^
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir ^
  -e LIBGL_ALWAYS_SOFTWARE=1 ^
  -v "%~dp0shared:/root/shared" ^
  weld-cell-sim:latest /opt/scripts/start_all.sh %*
