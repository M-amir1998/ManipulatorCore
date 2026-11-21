#!/usr/bin/env bash
# Installs the build, analysis and Python prerequisites on Ubuntu 22.04/24.04 (CI agents, containers).
set -euo pipefail

sudo_cmd=""
if [ "$(id -u)" -ne 0 ]; then sudo_cmd="sudo"; fi

${sudo_cmd} apt-get update
${sudo_cmd} apt-get install -y --no-install-recommends \
  build-essential cmake ninja-build git ca-certificates \
  clang clang-format clang-tidy \
  python3 python3-venv python3-pip \
  libeigen3-dev

python3 -m venv .venv
. .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev]"
echo "Done. Configure with: cmake --preset debug"
