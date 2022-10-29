#!/bin/bash

image="velodynescan_to_pc2"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)