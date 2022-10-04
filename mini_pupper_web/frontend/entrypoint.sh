#!/bin/bash
set -e

cd /app
./import-meta-env --example .env.example

cd /app/build
nginx -g "daemon off;"
