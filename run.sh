#!/bin/bash

source $HOME/aic/venv_aic/bin/activate

cd $HOME/aic/aichallenge-2024 && sh docker_build.sh dev

sh $HOME/aic/aichallenge-2024/docker_run.sh dev cpu
# sh $HOME/aic/aichallenge-2024/docker_run.sh dev gpu