#!/bin/bash

# Build KISSFFT library
git submodule update --init --recursive
cd vendor/kissfft/kissfft
rm -rf build
mkdir build
cd build
cmake -DKISSFFT_STATIC=ON -DKISSFFT_TEST=OFF ..
make -j4

# Copy the build library to the main directory
mkdir ../../macos
cp libkissfft-float.a ../../macos/

cd ../../..