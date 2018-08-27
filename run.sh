#!/bin/bash

set -e
set -x

export PREFIX=${1:-$PWD/prefix}  # with bash / zsh
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH

$PREFIX/bin/rbdl-test
$PREFIX/bin/pinocchio-test
#$PREFIX/bin/kdl-test
$PREFIX/bin/pinocchio-benchmark
$PREFIX/bin/rbdl-benchmark
$PREFIX/bin/benchmarks-pinocchio
$PREFIX/bin/benchmarks-rbdl
