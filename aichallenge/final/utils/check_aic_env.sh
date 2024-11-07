#!/bin/bash
VARIABLE_NAME="AIC_HOSTNAME"
if [ -z "${!VARIABLE_NAME}" ]; then
    echo -e "\e[31mError: Env var $VARIABLE_NAME is not set\e[0m" >&2
    exit 1
fi
echo -e "\e[32m$VARIABLE_NAME is ${!VARIABLE_NAME}\e[0m" >&2

VARIABLE_NAME="AIC_USER"
if [ -z "${!VARIABLE_NAME}" ]; then
    echo -e "\e[31mError: Env var $VARIABLE_NAME is not set\e[0m" >&2
    exit 1
fi
echo -e "\e[32m$VARIABLE_NAME is ${!VARIABLE_NAME}\e[0m" >&2
