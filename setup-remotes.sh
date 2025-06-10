#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

echo "Adding origin and upstream remotes..."

git remote rm upstream 2>/dev/null
git remote add upstream https://github.com/ArduPilot/ardupilot.git 2>/dev/null
git remote rm origin 2>/dev/null
git remote add origin https://github.com/inertiallabs/ardupilot.git

cd "$SCRIPT_DIR/modules/mavlink" || exit 1

git remote rm upstream 2>/dev/null
git remote add upstream https://github.com/ArduPilot/mavlink.git 2>/dev/null
git remote rm origin 2>/dev/null
git remote add origin https://github.com/inertiallabs/mavlink.git

echo "Fetching origin and upstream"
git fetch origin
git fetch upstream

cd "$SCRIPT_DIR" || exit 1

git fetch origin
git fetch upstream

echo "Done."
