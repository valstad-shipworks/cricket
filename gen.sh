#!/bin/bash

ROBOT_NAME=$1

if [ -z "$ROBOT_NAME" ]; then
  echo "Usage: $0 <robot_name>"
  exit 1
fi

JSON_FILE="/mount/resources/${ROBOT_NAME}.json"
OUTPUT_FILE="/mount/${ROBOT_NAME}.hh"

docker run --rm -v "$(pwd):/mount" --user "$(id -u):$(id -g)" cricket:latest "${JSON_FILE}" -t "${OUTPUT_FILE}"
clang-format -i "${ROBOT_NAME}.hh"