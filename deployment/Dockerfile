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
    apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    libfmt10 libjsoncpp26 libspdlog1.15 \
    && rm -rf /var/lib/apt/lists/*

ARG PRESET=pi5
COPY --from=build /tmp/build/${PRESET}/install/ /usr/local/

RUN ldconfig