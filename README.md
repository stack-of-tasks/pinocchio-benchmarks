# Benchmarking Pinocchio

Using docker:

```
docker run --rm -it gepetto/pinocchio-benchmarks
```

## Install Pinocchio & KDL binaries

```
sudo apt install -qqy robotpkg-pinocchio liborocos-kdl-dev libkdl-parser-dev
```

## Configure your prefix

```
export PREFIX=$PWD/prefix  # with bash / zsh
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
# OR
set -x PREFIX $PWD/prefix  # with fish
set -x LD_LIBRARY_PATH $PREFIX/lib:$LD_LIBRARY_PATH
```

## Install RBDL

```
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
rm default.zip
mv rbdl-rbdl-* libs/rbdl
mkdir -p build/rbdl
pushd build/rbdl
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DRBDL_BUILD_ADDON_URDFREADER=ON ../../libs/rbdl
make -j8 install
popd
```

## Install Google Benchmark

```
mkdir -p build/google-benchmark
pushd build/google-benchmark
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DBENCHMARK_ENABLE_GTEST_TESTS=OFF ../../libs/benchmark
make -j8 install
popd
```

## Install

```
mkdir -p build/pinocchio-benchmarks
pushd build/pinocchio-benchmarks
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_PREFIX_PATH=$PREFIX ../..
make -j8 install
popd
```

### Running

```
sudo cpupower frequency-set --governor performance
$PREFIX/bin/kdl-test
$PREFIX/bin/rbdl-test
$PREFIX/bin/pinocchio-test
$PREFIX/bin/benchmarks-pinocchio
$PREFIX/bin/benchmarks-rbdl
$PREFIX/bin/benchmarks-kdl
sudo cpupower frequency-set --governor powersave
```

Results are stored in `data/hostname_lib_algo_model.txt`
