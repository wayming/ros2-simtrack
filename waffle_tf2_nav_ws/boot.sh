xhost +local:root

docker stop waffle_tf2_navigation
docker build \
  --build-arg HOST_UID=$(id -u) \
  --build-arg HOST_GID=$(id -g) \
  -t waffle_tf2_navigation:latest .

docker rm waffle_tf2_navigation
docker run -d --rm \
  --name waffle_tf2_navigation \
  --network=host \
  --device=/dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/../:/ros2_ws \
  --user $(id -u):$(id -g) \
  waffle_tf2_navigation:latest