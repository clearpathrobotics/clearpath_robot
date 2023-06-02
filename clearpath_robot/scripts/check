#!/bin/bash
filename="/etc/clearpath/robot.yaml"

m1=$(md5sum "$filename")

while true; do
  sleep 1

  m2=$(md5sum "$filename")

  if [ "$m1" != "$m2" ] ; then
    echo "robot.yaml has changed." >&2 
    exit 1
  fi
done