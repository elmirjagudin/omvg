#!/bin/sh
cd build && cmake \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DCMAKE_BUILD_TYPE=Debug \
    ..
