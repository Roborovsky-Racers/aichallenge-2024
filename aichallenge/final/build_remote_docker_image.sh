#!/bin/bash

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

AIC_REMOTE_DIR="/home/$AIC_USER/aichallenge-2024"

# build docker image on remote
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR; ./docker_build.sh dev'"
