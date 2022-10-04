#!/bin/bash

sudo apt-get install -y python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip3 install pip --upgrade
pip3 install wheel
pip3 install -r requirements.txt
