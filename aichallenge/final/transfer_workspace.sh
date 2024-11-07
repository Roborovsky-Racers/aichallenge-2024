#!/bin/bash

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

AIC_LOCAL_DIR="/home/$USER/aic/aichallenge-2024"
AIC_REMOTE_DIR="/home/$AIC_USER/aichallenge-2024"

# transfer config files
cd $AIC_LOCAL_DIR
scp run_vehicle_tmux.sh run_script.tmux $AIC_HOSTNAME:$AIC_REMOTE_DIR/
scp aichallenge/Makefile $AIC_HOSTNAME:$AIC_REMOTE_DIR/aichallenge/
scp -r aichallenge/final $AIC_HOSTNAME:$AIC_REMOTE_DIR/aichallenge/

# backup workspace/src on remote
DATE=$(date '+%Y%m%d_%H%M%S')
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR/aichallenge/; mkdir -p backup; if [ -d workspace/src ]; then tar zcvf backup/src_$DATE.tar.gz workspace/src && rm -rf workspace/src; else echo \"workspace/src does not exist\"; fi'" || true

# transfer workspace/src
cd $AIC_LOCAL_DIR/aichallenge/workspace
tar zcvf src.tar.gz src
scp -r src.tar.gz $AIC_HOSTNAME:$AIC_REMOTE_DIR/aichallenge/workspace/
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR/aichallenge/workspace/; tar zxvf src.tar.gz && rm src.tar.gz'"
rm src.tar.gz

# add source of aicrc to remote bashrc
SOURCE_CMD="source ~/aichallenge-2024/aichallenge/final/rc/aicrc"
ssh -t "$AIC_HOSTNAME" "bash -c 'if ! grep -Fxq \"$SOURCE_CMD\" ~/.bashrc; then echo \"$SOURCE_CMD\" >> ~/.bashrc; fi'"
