#!/bin/bash

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

ssh -t $AIC_HOSTNAME "exec bash --rcfile ~/aichallenge-2024/final/aicrc"
