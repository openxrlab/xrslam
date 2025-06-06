FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i s:/archive.ubuntu.com:/mirrors.tuna.tsinghua.edu.cn/ubuntu:g /etc/apt/sources.list \
    && cat /etc/apt/sources.list \
    && apt-get clean

RUN apt-get update && apt-get install -y \
    build-essential libssl-dev libatlas-base-dev git wget python3 python3-pip apt-file \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libavcodec-dev ffmpeg

RUN apt-get remove -y gcc \
    && apt-file update && apt-get install -y software-properties-common \
    && add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update && apt-get install -y gcc-9 g++-9 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9

RUN wget https://github.com/Kitware/CMake/releases/download/v3.19.0/cmake-3.19.0.tar.gz \
    && tar -zxvf cmake-3.19.0.tar.gz && rm -rf cmake-3.19.0.tar.gz \
    && cd cmake-3.19.0 && ./bootstrap && make -j4 && make install

RUN git clone https://github.com/openxrlab/xrslam.git \
    && cd xrslam && cmake -B build -DXRSLAM_PC_HEADLESS_ONLY=ON && cmake --build build -j4
