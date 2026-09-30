# Requires BuildKit (Docker Engine's bundled Dockerfile frontend).
# Official ubuntu:22.04 digest verified by docker pull on 2026-09-30.
ARG UBUNTU_IMAGE=ubuntu:22.04@sha256:\
b8b6ee6aa931ecd9d0d952abc34dc0e5f7c6a30c6bb71b079fe399fde0329c02
ARG OPENVINO_VERSION=2026.3.1

FROM ${UBUNTU_IMAGE} AS base
ARG UBUNTU_IMAGE
ARG APP_UID=1000
ARG APP_GID=1000
SHELL ["/bin/bash", "-euo", "pipefail", "-c"]
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
LABEL org.opencontainers.image.title="XTYF-AutoAim offline environment" \
      org.opencontainers.image.base.name="${UBUNTU_IMAGE}"
RUN test "$(dpkg --print-architecture)" = amd64 \
    && test "$APP_UID" -gt 0 && test "$APP_GID" -gt 0 \
    && groupadd --gid "$APP_GID" autoaim \
    && useradd --uid "$APP_UID" --gid "$APP_GID" --create-home autoaim \
    && mkdir -p /workspace/src /workspace/build /models /data /config /output \
    && chown autoaim:autoaim /workspace/build /output

FROM base AS dev
ARG OPENVINO_VERSION
COPY install_dependence.sh /tmp/install_dependence.sh
COPY tools/container/runtime-packages.sh /tmp/runtime-packages.sh
RUN rm -f /etc/apt/apt.conf.d/docker-clean
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    DEBIAN_FRONTEND=noninteractive bash /tmp/install_dependence.sh \
      --yes --skip-camera --openvino-version "$OPENVINO_VERSION" \
    && mkdir -p /opt/autoaim-environment \
    && bash /tmp/runtime-packages.sh "$OPENVINO_VERSION" \
         > /opt/autoaim-environment/runtime-packages.txt \
    && dpkg-query -W > /opt/autoaim-environment/dev-packages.txt \
    && cmake --version > /opt/autoaim-environment/cmake.txt \
    && g++ --version > /opt/autoaim-environment/compiler.txt \
    && rm -rf /var/lib/apt/lists/* /tmp/install_dependence.sh /tmp/runtime-packages.sh
USER autoaim
WORKDIR /workspace/src
CMD ["bash"]

FROM dev AS build
USER root
COPY . /workspace/src
ARG CXX_STANDARD=17
ARG BUILD_JOBS=2
RUN cmake -S /workspace/src -B /workspace/build/release \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD="$CXX_STANDARD" \
      -DAUTOAIM_OPENVINO=ON -DAUTOAIM_HIKROBOT=OFF -DBUILD_TESTING=OFF \
      -DCMAKE_INSTALL_PREFIX=/opt/autoaim \
    && cmake --build /workspace/build/release --parallel "$BUILD_JOBS" \
    && cmake --install /workspace/build/release --component Runtime

# Only this explicitly selected target reads the named local SDK context.
# The bind mount is read-only and is not retained in the image layer.
FROM dev AS sdk-check
USER root
COPY . /workspace/src
ARG MVS_INCLUDE=include
ARG MVS_LIBRARY=lib/64/libMvCameraControl.so
ARG BUILD_JOBS=2
RUN --mount=type=bind,from=mvs,source=.,target=/opt/MVS \
    test -f "/opt/MVS/$MVS_INCLUDE/MvCameraControl.h" \
    && test -f "/opt/MVS/$MVS_LIBRARY" \
    && cmake -S /workspace/src -B /workspace/build/sdk \
      -DCMAKE_BUILD_TYPE=Debug -DAUTOAIM_OPENVINO=OFF -DBUILD_TESTING=OFF \
      -DAUTOAIM_HIKROBOT=ON -DAUTOAIM_HIKROBOT_INCLUDE="/opt/MVS/$MVS_INCLUDE" \
      -DAUTOAIM_HIKROBOT_LIBRARY="/opt/MVS/$MVS_LIBRARY" \
    && cmake --build /workspace/build/sdk --target autoaim_hal --parallel "$BUILD_JOBS"
USER autoaim
CMD ["bash"]

FROM base AS runtime
USER root
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=dev /usr/share/keyrings/xtyf-openvino.gpg /usr/share/keyrings/xtyf-openvino.gpg
COPY --from=dev /etc/apt/sources.list.d/xtyf-openvino.list \
    /etc/apt/sources.list.d/xtyf-openvino.list
COPY --from=dev /opt/autoaim-environment /opt/autoaim-environment
# CPU and IR are loaded dynamically, so ldd alone is not a dependency inventory.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive xargs -r apt-get install -y --no-install-recommends \
         < /opt/autoaim-environment/runtime-packages.txt \
    && dpkg-query -W > /opt/autoaim-environment/runtime-packages-installed.txt \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/autoaim /opt/autoaim
ENV PATH="/opt/autoaim/bin:${PATH}"
USER autoaim
WORKDIR /output
CMD ["bash"]
