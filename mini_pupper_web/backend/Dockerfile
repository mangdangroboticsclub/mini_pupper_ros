FROM python:3.8

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    udev \
    ffmpeg \
    libsm6 \
    libxext6 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY ./requirements.txt ./

RUN pip install pip --upgrade && pip install -r requirements.txt
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

COPY ./mini_pupper_webrtc/ ./mini_pupper_webrtc/
COPY ./log_conf.yaml ./
COPY ./models/ /root/.cache/blobconverter/

EXPOSE 8080

CMD uvicorn mini_pupper_webrtc.main:app --host 0.0.0.0 --port 8080 --log-config log_conf.yaml
