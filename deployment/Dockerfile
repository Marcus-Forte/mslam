FROM mdnf1992/cpp-dev AS build

WORKDIR /tmp
COPY . .

RUN cmake --preset default && \
    cmake --build build/default && \
    cmake --install build/default


# Can be distroless
FROM debian:trixie-slim AS runtime

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libatomic1 \
    libboost-atomic1.83.0 \
    libboost-filesystem1.83.0 \
    libboost-iostreams1.83.0 \
    libboost-serialization1.83.0 \
    libboost-system1.83.0 \
    libflann1.9 \
    libglib2.0-0t64 \
    libgstreamer-plugins-base1.0-0 \
    libgstreamer1.0-0 \
    libjpeg62-turbo \
    libjsoncpp26 \
    libpng16-16t64 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=build /tmp/build/default/install/ /usr/local/

RUN ldconfig