xhost +local:root

docker stop turtle_auto_patrol
docker build \
  --build-arg HOST_UID=$(id -u) \
  --build-arg HOST_GID=$(id -g) \
  -t turtle_auto_patrol:latest .

docker rm turtle_auto_patrol
docker run -d --rm \
  --name turtle_auto_patrol \
  --network=host \
  --device=/dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/../:/ros2_ws \
  --user $(id -u):$(id -g) \
  turtle_auto_patrol:latest