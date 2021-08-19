FROM ros:noetic-ros-core

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

COPY --from=pykaldi/pykaldi /kaldi /kaldi

ARG PYKALDI_VER=0.1.2
ARG PYKALDI_PYTHON_VER=3.6
COPY --from=pykaldi/pykaldi \
    /usr/local/lib/python${PYKALDI_PYTHON_VER}/dist-packages/pykaldi-${PYKALDI_VER}-py${PYKALDI_PYTHON_VER}-linux-x86_64.egg/ \
    /usr/lib/python3/dist-packages/

RUN mv /usr/lib/python3/dist-packages/EGG-INFO/ \
    /usr/lib/python3/dist-packages/pykaldi-${PYKALDI_VER}-py${PYKALDI_PYTHON_VER}-linux-x86_64.egg-info && \
    echo 'export LD_LIBRARY_PATH=/kaldi/src/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo 'export KALDI_ROOT=/kaldi' >> ~/.bashrc && \
    cat /kaldi/tools/config/common_path.sh >> ~/.bashrc

ARG HR_MSGS=head-noetic-hr_msgs_0.3.16_amd64.deb
ARG HR_MSGS_DIR=docker/externals
COPY ${HR_MSGS_DIR}/${HR_MSGS} /home/
RUN apt-get update -y && \
    apt-get install -y \
        libatlas-base-dev \
        portaudio19-dev \
        libsndfile1 \
        libasound2 \
        libasound2-plugins \
        pulseaudio \
        pulseaudio-utils \
        alsa-base \
        alsa-oss \
        alsa-utils \
        python3-pip \
        wget && \
    dpkg -i /home/${HR_MSGS} && \
        rm -rf /home/${HR_MSGS} && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/imperio

WORKDIR /home/imperio

ARG URL=https://www.dropbox.com/s/eecyok5h1pfn7vy/librispeech_tgsmall.tar.gz?dl=1

SHELL ["/bin/bash", "-c"]
RUN export MODEL=$(export TEMP="${URL##*/}"; echo "${TEMP%%.*}") && \
    export HASH=$(echo -n ${URL} | md5sum); export HASH=${HASH%% *} && \
    export DIR=/root/.cache/sonorus/kaldi_model_${HASH} && \
    mkdir /root/.cache && \
    mkdir /root/.cache/sonorus && \
    mkdir ${DIR} && \
    cd ${DIR} && \
        echo -e "${DIR}/${MODEL}\t${URL}" > url.txt && \
        wget -O ${MODEL}.tar.gz ${URL} && \
        tar -xzvf ${MODEL}.tar.gz && \
        rm ${MODEL}.tar.gz && \
    cd /home/imperio && \
    pip install -r requirements.txt 
