@echo off
setlocal enabledelayedexpansion

:: Set the image name
set IMAGE_NAME=arena-cuda-ros

:: Get the current directory and append packages folders
set ARENA_MSGS_PATH=%cd%\arena_msgs

:: Run the Docker container
echo Running Docker container...
docker run -it --rm ^
    --gpus=all ^
    -v "%ARENA_MSGS_PATH%:/home/dev_ws/src/arena_msgs" ^
    -p 10000:10000 ^
    -p 5005:5005 ^
    -p 8765:8765 ^
    %IMAGE_NAME%
