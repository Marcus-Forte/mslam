
FROM ubuntu:noble

RUN apt-get update && apt-get install -y git build-essential libflann-dev libboost-all-dev gdb cmake clangd clang-format clang-tidy libtbb-dev libeigen3-dev && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /deps

RUN git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc

RUN mkdir /deps/grpc/build && cd /deps/grpc/build && cmake -DgRPC_INSTALL=ON \
    -DgRPC_BUILD_TESTS=OFF \
    .. && make -j8 install

RUN git clone https://github.com/PointCloudLibrary/pcl.git -b pcl-1.14.1

RUN  mkdir /deps/pcl/build && cd /deps/pcl/build && cmake -DWITH_VTK=OFF -DWITH_OPENGL=OFF  \ 
  -DBUILD_segmentation=OFF -DBUILD_registration=OFF -DBUILD_recognition=OFF .. && \
  make -j4 install && \
  rm -rf /deps/*