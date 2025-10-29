#!/bin/bash

GIT_ROOT=$(git rev-parse --show-toplevel)
BENSCHMARK_DIR=${GIT_ROOT}/testdata/wake-word-benchmark
TESTRUNS_ROOT=${GIT_ROOT}/testdata/mic_streaming

if [ ! -d "${BENSCHMARK_DIR}" ]; then
  cd ${GIT_ROOT}/testdata
  git clone https://github.com/Picovoice/wake-word-benchmark.git
fi

if [ ! -d "${TESTRUNS_ROOT}" ]; then
  mkdir ${TESTRUNS_ROOT}
fi
