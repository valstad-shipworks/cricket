#!/bin/bash

ROBOT_NAME=$1

if [ -z "$ROBOT_NAME" ]; then
  echo "Usage: $0 <robot_name>"
  exit 1
fi

# if a "-b" flag is passed build tthe docker image aswell
if [ "$2" == "--docker-build" ]; then
  docker build -t cricket:latest .
fi
if [ "$2" == "--docker" ] || [ "$2" == "--docker-build" ]; then
  JSON_FILE="/mount/resources/${ROBOT_NAME}.json"
  OUTPUT_FILE="/mount/${ROBOT_NAME}.hh"

  docker run --rm -v "$(pwd):/mount" --user "$(id -u):$(id -g)" cricket:latest "${JSON_FILE}" -t "${OUTPUT_FILE}"
  clang-format -i "${ROBOT_NAME}.hh"
else
  /opt/conda/bin/micromamba env create -f ./environment.yaml
  /opt/conda/bin/micromamba activate cricket

  JSON_FILE="./resources/${ROBOT_NAME}.json"
  OUTPUT_FILE="./${ROBOT_NAME}.hh"
  cmake -GNinja -Bbuild .
  cmake --build build
  ./build/fkcc_gen "${JSON_FILE}" -t "${OUTPUT_FILE}"
  clang-format -i "${ROBOT_NAME}.hh"

  /opt/conda/bin/micromamba deactivate
fi