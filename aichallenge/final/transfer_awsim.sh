#!/bin/bash

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

AIC_LOCAL_DIR="/home/$USER/aic/aichallenge-2024"
AIC_REMOTE_DIR="/home/$AIC_USER/aichallenge-2024"

# transfer AWSIM_PU
cd $AIC_LOCAL_DIR/aichallenge/simulator
tar zcvf AWSIM_CPU.tar.gz AWSIM_CPU
scp -r AWSIM_CPU.tar.gz $AIC_HOSTNAME:$AIC_REMOTE_DIR/aichallenge/simulator/
ssh -t "$AIC_HOSTNAME" "bash -c 'cd $AIC_REMOTE_DIR/aichallenge/simulator/;tar zxvf AWSIM_CPU.tar.gz && rm AWSIM_CPU.tar.gz'"
rm AWSIM_CPU.tar.gz
