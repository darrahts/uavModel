docker run -it --rm --privileged --env="QT_X11_NO_MITSHM=1" \
	--env=LOCAL_USER_ID="$(id -u)" \
    --env="DISPLAY" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--name=gazebo_cont gazebo_img

