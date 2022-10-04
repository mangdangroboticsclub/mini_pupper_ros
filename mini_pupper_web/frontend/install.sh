#!/bin/bash

curl -sL https://deb.nodesource.com/setup_14.x | sudo bash -
sudo apt-get install -y nodejs
sudo npm install --global http-server
npm install
cp .env.example .env
