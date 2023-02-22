#!/bin/sh

export DISPLAY=:0

Xvfb :0 -screen 0 1920x1080x16 &
x11vnc -passwd password -display :0 -N -forever &

exec "$@"
