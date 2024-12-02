#!/bin/bash
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <datasets-dir> <output-dir> [display]"
  echo "display can be 0 or 1 (default)"
  exit 1
fi

data_dir=$1
if [ ! -d "$data_dir" ]; then
  echo "Data directory does not exist: $data_dir."
  exit 2
fi

out_dir=$2
if [ ! -d "$out_dir" ]; then
  echo "Output directory does not exist: $out_dir."
  exit 2
fi

# Explanations of arguments to the below commands are given in 
# https://stackoverflow.com/questions/43015536/xhost-command-for-docker-gui-apps-eclipse
if [ "$3" = "0" ]; then
  docker run --gpus all -it -v $data_dir:/root/datasets -v $out_dir:/root/results photoslam:latest /bin/bash -c "/bin/bash"
else
  xhost +local:root;
  docker run  --gpus all -it -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $data_dir:/root/datasets -v $out_dir:/root/results  photoslam:latest /bin/bash -c "/bin/bash"
  xhost -local:root;
fi
