@echo off
setlocal enabledelayedexpansion

:: Set the image name
set IMAGE_NAME=arena-cuda-ros

:: Run the Docker container
echo Running Docker container...
docker run -it --rm ^
    --gpus=all ^
    -p 10000:10000 ^
    -p 5005:5005 ^
    -p 8765:8765 ^
    %IMAGE_NAME%
