container_name=${1:-imperio}
docker run -it \
	--rm \
	--network=host \
	--env-file docker/run.env \
	--device /dev/snd:/dev/snd \
	-e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
	-v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
	-v ~/.config/pulse/cookie:/root/.config/pulse/cookie \
	-v $(pwd):/home/imperio \
	--name $container_name \
	imperio bash
