FROM robotis/ai-worker:amd64-2.3.1@sha256:99748390fd747165625000ab1ee36184516f2d59f21f7fc931474e899f341e03
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb fluxbox xdotool x11-utils ffmpeg python3-pil mesa-utils \
    python3-reportlab fonts-ipafont-gothic poppler-utils \
    && rm -rf /var/lib/apt/lists/*
ENV DISPLAY=:99 QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=64 \
    QT_QPA_PLATFORM=xcb
WORKDIR /project
# Deliberately bypass vendor /init: no hardware/agent services in this simulation.
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "sleep infinity"]
