#/bin/sh

thispath=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)

xhost +local:root

nvidia-docker run -it \
       --volume="$thispath/../..:/root/local" \
       --volume="$thispath/bashrc:/root/bashrc" \
       --volume="$HOME/.gitconfig:/root/.gitconfig:ro" \
       --volume="$HOME/.ssh:/root/.ssh:ro" \
       --env="DISPLAY" \
       --publish 11311:11311 \
       --network host \
       lgp-exec /bin/bash

xhost -local:root
