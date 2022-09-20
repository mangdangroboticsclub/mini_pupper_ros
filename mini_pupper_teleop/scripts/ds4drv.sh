#!/bin/bash
set -eu

# launch ds4drv if not exists
ps aux | grep -v grep | grep ds4drv || ds4drv
