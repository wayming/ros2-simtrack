xhost +local:root

docker stop turtle_commander
docker build -t turtle_commander:latest .
docker run -d --rm \
  --name turtle_commander \
  --network=host \
  --device=/dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ..:/ros2_ws \
  turtle_commander:latest