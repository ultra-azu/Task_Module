#!/bin/bash

set -e -v

cd /opt
git clone --depth=1 https://github.com/linghu8812/tensorrt_inference.git

cd tensorrt_inference
git config --global user.email "None@none"
git config --global user.name "Anonymous"
git am /0001-Use-yaml-cpp-cmake-package.patch

pushd depends
wget https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.zip
unzip yaml-cpp-0.6.3.zip
mkdir yaml-cpp-yaml-cpp-0.6.3/build
pushd yaml-cpp-yaml-cpp-0.6.3/build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=ON ..
make -j"$(nproc)"
make install
#Go back to tensorrt_inference
pushd +2

cd project
mkdir build
cd build

cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 ..
make -j"$(nproc)"

