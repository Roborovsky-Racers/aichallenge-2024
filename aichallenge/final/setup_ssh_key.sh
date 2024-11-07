#!/bin/bash

IDENTITY_FILE="id_aic"

set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
"$SCRIPT_DIR/utils/check_aic_env.sh"

# Hostセクションが存在し、かつIdentityFileが含まれていない場合のみ追加
if grep -q "Host $AIC_HOSTNAME" ~/.ssh/config; then
    if ! grep -A 10 "Host $AIC_HOSTNAME" ~/.ssh/config | grep -q "IdentityFile ~/.ssh/$IDENTITY_FILE"; then
        # 既存のHostセクションにIdentityFileを追加
        sed -i "/Host $AIC_HOSTNAME/a \    IdentityFile ~/.ssh/$IDENTITY_FILE" ~/.ssh/config
    fi
fi

ssh-copy-id -i ~/.ssh/id_aic $AIC_HOSTNAME
