FROM mdnf1992/cpp-dev AS build

WORKDIR /tmp
COPY . .

# Raspberry Pi 5 (Cortex-A76) build. Override with e.g. --build-arg PRESET=gcc.
ARG PRESET=pi5
RUN cmake --preset ${PRESET} -DBUILD_TESTING=OFF \
        -DCMAKE_C_COMPILER=/usr/bin/gcc-16 -DCMAKE_CXX_COMPILER=/usr/bin/g++-16 && \
    cmake --build --preset build-${PRESET} && \
    cmake --install build/${PRESET}

# Keep the runtime glibc compatible with the sid-based build image.
FROM debian:sid-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    libfmt10 libjsoncpp26 libspdlog1.15 \
    && rm -rf /var/lib/apt/lists/*

ARG PRESET=pi5
COPY --from=build /tmp/build/${PRESET}/install/ /usr/local/

RUN ldconfig