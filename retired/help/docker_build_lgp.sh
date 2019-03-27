
#/bin/sh

docker build \
       --tag christina-lgp \
       --network host \
-f full.dockerfile .
