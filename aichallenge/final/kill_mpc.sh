#!/bin/bash

alias mpc_kill="ps -ef | grep -e mpc_controller | grep -v grep | awk '{cmd = \"\"; for(i=8;i<=NF;i++) cmd = cmd \$i \" \"; print \$2, cmd}' | while read pid cmd; do echo \"kill process: PID \$pid Command: \$cmd\"; kill -9 \$pid; done"
mpc_kill