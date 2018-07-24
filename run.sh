#!/bin/bash

set -e
set -x

rm -rf build prefix

export PREFIX=${1:-$PWD/prefix}  # with bash / zsh
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
export CXX="ccache clang++-6.0"
export CC="ccache clang-6.0"

mkdir -p build/pinocchio
pushd build/pinocchio
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF -DINSTALL_DOCUMENTATION=OFF \
    ../../libs/pinocchio
make -j8 install
popd

#mkdir -p build/kdl
#pushd build/kdl
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    #../../libs/kdl/orocos_kdl
#make -j8 install
#popd

#mkdir -p build/kdl_parser
#pushd build/kdl_parser
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    #../../libs/kdl_parser/kdl_parser
#make -j8 install
#popd

if [[ ! -d libs/rbdl ]]
then
    wget https://bitbucket.org/rbdl/rbdl/get/default.zip
    unzip default.zip
    rm default.zip
    mv rbdl-rbdl-* libs/rbdl
fi

mkdir -p build/rbdl
pushd build/rbdl
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DRBDL_BUILD_ADDON_URDFREADER=ON \
    ../../libs/rbdl
make -j8 install
popd

mkdir -p build/google-benchmark
pushd build/google-benchmark
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DBENCHMARK_ENABLE_GTEST_TESTS=OFF \
    ../../libs/benchmark
    #-DBENCHMARK_ENABLE_LTO=true \
    #-DLLVMAR_EXECUTABLE="/usr/bin/llvm-ar-6.0" \
    #-DLLVMNM_EXECUTABLE="/usr/bin/llvm-nm-6.0" \
    #-DLLVMRANLIB_EXECUTABLE="/usr/bin/llvm-ranlib-6.0" \
make -j8 install
popd

mkdir -p build/pinocchio-benchmarks
pushd build/pinocchio-benchmarks
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    ../..
make -j8 install
popd

$PREFIX/bin/rbdl-test
$PREFIX/bin/pinocchio-test
#$PREFIX/bin/kdl-test
echo "don't forget to run 'sudo cpupower frequency-set --governor performance'"
$PREFIX/bin/pinocchio-benchmark
$PREFIX/bin/rbdl-benchmark
echo "don't forget to run 'sudo cpupower frequency-set --governor powersave'"
