#!/bin/bash
set -e

# Set the image name
IMAGE_NAME="arena-cuda-ros"


# Run the Docker container
echo "Running Docker container..."
docker run -it --rm \
    --gpus=all \
    --network=host \
    "$IMAGE_NAME"
