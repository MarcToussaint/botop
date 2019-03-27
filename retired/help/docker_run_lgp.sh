#/bin/sh

thispath=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)

xhost +local:root

#nvidia-docker run  --name spartan  -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw  -v /home/christina/spartan:/home/christina/spartan  -v ~/.ssh:/home/christina/.ssh  -v ~/.spartan-docker/christina-spartan-build:/home/christina/.spartan-build  --user christina  -v /home/christina/data/spartan:/home/christina/spartan/data_volume   -p 11311:11311  --network host  --privileged -v /dev/bus/usb:/dev/bus/usb  --rm  --ulimit rtprio=30 -it christina-spartan 

docker run -it \
       --volume="$thispath/../..:/root/local" \
       --volume="$thispath/docker.bashrc:/root/bashrc" \
       --volume="$HOME/.gitconfig:/root/.gitconfig:ro" \
       --volume="$HOME/.ssh:/root/.ssh:ro" \
       --env="DISPLAY" \
       --publish 11311:11311 \
       --network host \
       christina-lgp /bin/bash

xhost -local:root


