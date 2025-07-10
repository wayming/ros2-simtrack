xhost +local:root

docker stop turtle_monitor
docker build -t turtle_monitor:latest .
docker run -d --rm \
  --name turtle_monitor \
  --network=host \
  --device=/dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/../:/ros2_ws \
  turtle_monitor:latest