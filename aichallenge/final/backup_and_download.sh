#!/bin/bash

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

AIC_LOCAL_DIR="/home/$USER/aic/aichallenge-2024"
AIC_REMOTE_DIR="/home/$AIC_USER/aichallenge-2024"

# backup src, roslog, and rosbag on remote
DATE=$(date '+%Y%m%d_%H%M%S')
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR/aichallenge/; mkdir -p backup/$DATE; tar zcvf backup/$DATE/src.tar.gz -C workspace src'" || true
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR/aichallenge/; tar zcvf backup/$DATE/rosbag2_autoware.tar.gz rosbag2_autoware'" || true
ssh -t "$AIC_HOSTNAME" "CONTAINER_NAME=\$(docker ps --filter \"name=aichallenge-\" --format \"{{.Names}}\"); docker cp \$CONTAINER_NAME:/home/$AIC_USER/.ros/log $AIC_REMOTE_DIR/aichallenge/backup/$DATE/roslog"
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR/aichallenge/backup/$DATE/; tar zcvf roslog.tar.gz roslog; rm -rf roslog'" || true

# transfer backup from remote to local
scp -r $AIC_HOSTNAME:$AIC_REMOTE_DIR/aichallenge/backup/$DATE $AIC_LOCAL_DIR/aichallenge/backup/
