docker run -it \
	--rm \
	--network=host \
	--env-file docker/run.env \
	--device /dev/snd:/dev/snd \
	-e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
	-v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
	-v ~/.config/pulse/cookie:/root/.config/pulse/cookie \
	--name imperio \
	imperio bash
