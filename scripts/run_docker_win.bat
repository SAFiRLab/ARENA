@echo off
setlocal enabledelayedexpansion

:: Set the image name
set IMAGE_NAME=arena-cuda-ros

:: Get the current directory and append packages folders
set ARENA_MSGS_PATH=%cd%\arena_msgs
set ARENA_CORE_PATH=%cd%\arena_core

:: Run the Docker container
:: -v "%ARENA_CORE_PATH%:/home/dev_ws/src/arena_core" ^
echo Running Docker container...
docker run -it --rm ^
    --gpus=all ^
    -v "%ARENA_MSGS_PATH%:/home/dev_ws/src/arena_msgs" ^
    -v "%ARENA_CORE_PATH%:/home/dev_ws/src/arena_core" ^
    -p 10000:10000 ^
    -p 5005:5005 ^
    -p 8765:8765 ^
    %IMAGE_NAME%
