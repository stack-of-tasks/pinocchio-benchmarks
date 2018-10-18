#!/bin/bash

set -e
set -x

[[ -d build || -d prefix ]] && rm -rf build prefix

export PREFIX=${1:-$PWD/prefix}  # with bash / zsh
export PKG_CONFIG_PATH=${PREFIX}/lib/pkgconfig:/opt/ros/kinetic/lib/pkgconfig
export LD_LIBRARY_PATH=${PREFIX}/lib:/opt/ros/kinetic/lib:${LD_LIBRARY_PATH}
export CXX="ccache clang++-6.0"
export CC="ccache clang-6.0"
BUILD_TYPE=Release

mkdir -p build/eigen
pushd build/eigen
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    ../../libs/eigen
make -j8 install
popd

mkdir -p build/pinocchio
pushd build/pinocchio
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF -DINSTALL_DOCUMENTATION=OFF \
    ../../libs/pinocchio
make -j8 install
popd

#export CXX="ccache g++"
#export CC="ccache gcc"
#mkdir -p build/metapod
#pushd build/metapod
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    #-DCMAKE_PREFIX_PATH=$PREFIX \
    #-DDOXYGEN_USE_MATHJAX=YES \
    #../../libs/metapod
#make -j8 install
#popd
#export CXX="ccache clang++-6.0"
#export CC="ccache clang-6.0"

mkdir -p build/kdl
pushd build/kdl
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DENABLE_TESTS=ON -DCMAKE_CXX_STANDARD=98 \
    ../../libs/kdl/orocos_kdl
make -j8 install
popd

mkdir -p build/kdl_parser
pushd build/kdl_parser
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH="$PREFIX;/opt/ros/kinetic" \
    ../../libs/kdl_parser/kdl_parser
make -j8 install
popd

if [[ ! -d libs/rbdl ]]
then
    wget https://bitbucket.org/rbdl/rbdl/get/default.zip
    unzip default.zip
    rm default.zip
    mv rbdl-rbdl-* libs/rbdl
fi

mkdir -p build/rbdl
pushd build/rbdl
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH="$PREFIX;/opt/ros/kinetic" \
    -DRBDL_BUILD_ADDON_URDFREADER=ON \
    ../../libs/rbdl
make -j8 install
popd

#mkdir -p build/google-benchmark
#pushd build/google-benchmark
#cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    #-DCMAKE_PREFIX_PATH=$PREFIX \
    #-DBENCHMARK_ENABLE_GTEST_TESTS=OFF \
    #../../libs/benchmark
#make -j8 install
#popd

mkdir -p build/pinocchio-benchmarks
pushd build/pinocchio-benchmarks
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    ../..
make -j8 install
popd
