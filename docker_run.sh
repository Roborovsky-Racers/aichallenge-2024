#!/bin/bash

target=${1}
device=${2}

case "${target}" in
"eval")
    volume="output:/output"
    ;;
"dev")
    volume="output:/output aichallenge:/aichallenge $HOME/aic/autoware-practice:/autoware-practice"
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

case "${device}" in
"cpu")
    opts=""
    ;;
"gpu")
    opts="--nvidia"
    ;;
*)
    echo "invalid argument (use 'gpu' or 'cpu')"
    exit 1
    ;;
esac

mkdir -p output

# shellcheck disable=SC2086
rocker ${opts} --x11 --device /dev/dri --user --net host --privileged --volume ${volume} -- "aichallenge-2024-${target}"
