ARG DOCKER_TAG=latest
FROM espressif/idf:${DOCKER_TAG}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

# Installer udev + python/pip, puis installer les dépendances Python ROS tooling

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
      udev \
      python3 \
      python3-pip \
      python3-setuptools \
      python3-wheel \
      ca-certificates && \
    python3 -m pip install --no-cache-dir --break-system-packages \
      catkin_pkg \
      lark-parser \
      colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]