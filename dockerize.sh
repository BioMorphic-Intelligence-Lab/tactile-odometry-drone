#!/bin/bash
docker buildx build --platform linux/arm32,linux/amd64 -t michatud/tactile-odometry-drone --push .
