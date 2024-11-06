#!/bin/bash

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

cd $SCRIPT_DIR/../

# scp -r final $AIC_HOSTNAME:~/aic2024/final/

# add source of aicrc to remote bashrc
# ssh -t $AIC_HOSTNAME "exec bash -c 'if ! grep -Fxq "source ~/aichallenge-2024/final/rc/aicrc" ~/.bashrc; then echo "source ~/aichallenge-2024/final/rc/aicrc" >> ~/.bashrc; fi"

ssh -t "$AIC_HOSTNAME" "bash -c 'if ! grep -Fxq \"source ~/aichallenge-2024/final/rc/aicrc\" ~/.bashrc; then echo \"source ~/aichallenge-2024/final/rc/aicrc\" >> ~/.bashrc; fi'"

