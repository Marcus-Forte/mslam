FROM mdnf1992/cpp-dev AS build

WORKDIR /tmp
COPY . .

# Raspberry Pi 5 (Cortex-A76) build with GCC (the presets set the compiler).
# --build-arg PRESET=pi5-sycl builds the SYCL backend too (DPC++).
ARG PRESET=pi5
RUN cmake --preset ${PRESET} -DBUILD_TESTING=OFF && \
    cmake --build --preset build-${PRESET} && \
    cmake --install build/${PRESET}
# SYCL builds: the install's runtime-dependency scan finds libsycl and the UR
# loader (linked), but the loader dlopen()s its native_cpu adapter, which needs
# libumf.
RUN if grep -q "MSLAM_WITH_SYCL:BOOL=ON" build/${PRESET}/CMakeCache.txt; then \
      cp -a /opt/sycl/lib/libur_adapter_native_cpu.so* /opt/sycl/lib/libumf.so* \
          build/${PRESET}/install/lib/; \
    fi

# Keep the runtime glibc compatible with the sid-based build image.
FROM debian:sid-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    libfmt10 libjsoncpp26 libspdlog1.15 \
    libtbb12 \
    libhwloc15 libudev1 \
    && rm -rf /var/lib/apt/lists/*

ARG PRESET=pi5
COPY --from=build /tmp/build/${PRESET}/install/ /usr/local/

RUN ldconfig