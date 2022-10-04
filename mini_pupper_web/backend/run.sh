#!/bin/bash

source .venv/bin/activate
uvicorn mini_pupper_webrtc.main:app --host 0.0.0.0 --port 8080 --log-config log_conf.yaml
