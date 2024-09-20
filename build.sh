#!/usr/bin/env bash

# Enable Docker BuildKit
export DOCKER_BUILDKIT=1

# Default version to "latest" if not provided
VERSION=${1:-latest}

# Build the Docker image with the specified version tag
docker build --network=host -t jetbot_riva_voice:$VERSION .