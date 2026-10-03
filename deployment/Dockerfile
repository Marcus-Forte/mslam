FROM mdnf1992/cpp-dev AS build

WORKDIR /tmp
COPY . .

# Raspberry Pi 5 (Cortex-A76) build. Override with e.g. --build-arg PRESET=gcc.
ARG PRESET=pi5
RUN cmake --preset ${PRESET} -DBUILD_TESTING=OFF \
        -DCMAKE_C_COMPILER=/usr/bin/gcc-16 -DCMAKE_CXX_COMPILER=/usr/bin/g++-16 && \
    cmake --build --preset build-${PRESET} && \
    cmake --install build/${PRESET}

# Can be distroless
FROM debian:trixie-slim AS runtime

# gcc-16 needs libstdc++6 >= 16 (GLIBCXX_3.4.35), which trixie lacks: install it from sid.
RUN printf 'Types: deb\nURIs: http://deb.debian.org/debian\nSuites: sid\nComponents: main\nSigned-By: /usr/share/keyrings/debian-archive-keyring.pgp\n' > /etc/apt/sources.list.d/sid.sources && \
    apt-get update && apt-get install -y \
    libstdc++6 \
    libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 libgnutls30 libdw1 libunwind8 libi2c0 libjpeg62-turbo libgomp1 libegl1 libgles2 \
    libboost-log1.83.0 libboost-thread1.83.0 libboost-system1.83.0 libboost-iostreams1.83.0 libboost-atomic1.83.0 libboost-serialization1.83.0 \
    libflann1.9 liborc-0.4-0 libjsoncpp26 \
    && rm -rf /var/lib/apt/lists/*

ARG PRESET=pi5
COPY --from=build /tmp/build/${PRESET}/install/ /usr/local/

RUN ldconfig