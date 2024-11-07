#!/bin/bash

echo "pull aichallenge-2024"
cd ~/aic/aichallenge-2024/
git branch --show-current
git pull

echo "pull MPC"
cd ~/aic/aichallenge-2024/aichallenge/workspace/src/multi_purpose_mpc_ros/
git branch --show-current
git pull

echo "pull aic_tools"
cd ~/aic/aichallenge-2024/aichallenge/workspace/src/aic_tools/
git branch --show-current
git pull

cd ~/aic/aichallenge-2024
