#!/bin/bash

IP=$1
cp calibration_settings.template.yaml calibration_settings.yaml && \
[ -f ../minipupper_control_v2/config/calibration_settings.template.yaml ] && cp calibration_settings.template.yaml calibration_settings.yaml
echo "HOST_IP=${IP}" > .env
