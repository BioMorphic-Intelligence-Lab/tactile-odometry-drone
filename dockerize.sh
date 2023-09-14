#!/bin/bash
docker buildx build --platform linux/arm64,linux/amd64 -t michatud/tactile-odometry-drone -f trackball_and_imu.devel.dockerfile --push .
#docker buildx build --platform linux/arm64,linux/amd64 -t antbre/test-trajectory -f controller.devel.dockerfile --push .
