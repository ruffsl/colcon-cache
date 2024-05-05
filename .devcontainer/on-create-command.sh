#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

git config --global --add safe.directory "*"

COLCON_CI_URL=https://github.com/colcon/ci/archive/refs/heads/main.zip
curl -sSL $COLCON_CI_URL > /tmp/main.zip
unzip /tmp/main.zip -d /tmp/
GITHUB_ACTION_PATH=/tmp/ci-main

PKG_NAME=$(pip list -l -e --format json | jq '.[0].name' -r | tr _ -)
cp -a ${GITHUB_ACTION_PATH}/constraints{,-heads,-pins}.txt ./
sed -i.orig "s/^${PKG_NAME}@.*//g" constraints-heads.txt
# Install dependencies, including 'test' extras, as well as pytest-cov
python -m pip install -U -e .[test] pytest-cov -c constraints.txt
