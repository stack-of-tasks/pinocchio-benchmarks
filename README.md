# Benchmarking Pinocchio

## Setup your environment

You need both eigen2 (for KDL) and eigen3 (for RBDL & Pinocchio):
```
sudo apt install -qqy libeigen{2,3}-dev
```

You have to choose the `PREFIX` in which you want to install Pinocchio, RBDL & KDL:


```
export PREFIX=$PWD/prefix  # with bash / zsh
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
# OR
set -x PREFIX $PWD/prefix  # with fish
set -x LD_LIBRARY_PATH $PREFIX/lib:$LD_LIBRARY_PATH
```

## Pinocchio

### Downloading

If you did not clone this repository with the `--recursive` options, you will have to get the submodules with:
```
git submodule update --init --recursive
```

### Installing

```
mkdir -p build/pinocchio
pushd build/pinocchio
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF ../../libs/pinocchio
make -j8 install
popd
```

## KDL

### Installing

```
mkdir -p build/kdl
pushd build/kdl
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib ../../libs/kdl/orocos_kdl
make -j8 install
popd
```

## RBDL

### Downloading

RBDL uses mercurial, and git-hg can't clone it, so we can't provide a git submodule, and you have to download it:

```
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
rm default.zip
mv rbdl-rbdl-* libs/rbdl
```

### Installing

```
mkdir -p build/rbdl
pushd build/rbdl
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DRBDL_BUILD_ADDON_URDFREADER=ON ../../libs/rbdl
make -j8 install
popd
```

## Google Benchmark

### Installing

```
mkdir -p build/google-benchmark
pushd build/google-benchmark
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DBENCHMARK_ENABLE_GTEST_TESTS=OFF ../../benchmark
make -j8 install
popd
```

## Benchmarks

### Installing

```
mkdir -p build/pinocchio-benchmarks
pushd build/pinocchio-benchmarks
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_PREFIX_PATH=$PREFIX ../..
make -j8 install
popd
```

### Running

```
./prefix/bin/rbdl-bench models/simple_humanoid.urdf
./prefix/bin/rbdl-bench models/romeo/romeo_description/urdf/romeo.urdf
./prefix/bin/pinocchio-bench models/simple_humanoid.urdf
./prefix/bin/pinocchio-bench models/romeo/romeo_description/urdf/romeo.urdf
sudo cpupower frequency-set --governor performance
./prefix/bin/pinocchio-benchmark
./prefix/bin/rbdl-benchmark
sudo cpupower frequency-set --governor powersave
```
