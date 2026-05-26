@echo off
setlocal
cd /d "%~dp0"
echo Starting URDF Viewer at http://localhost:8000
echo Press Ctrl+C in this window to stop the server.
python -m http.server 8000
