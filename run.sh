#!/bin/bash

set -e
set -x

export PREFIX=${1:-$PWD/prefix}  # with bash / zsh
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH

$PREFIX/bin/rbdl-test
$PREFIX/bin/pinocchio-test
$PREFIX/bin/kdl-test
#$PREFIX/bin/metapod-test
$PREFIX/bin/julia-test.jl

$PREFIX/bin/benchmarks-pinocchio
$PREFIX/bin/benchmarks-rbdl
$PREFIX/bin/benchmarks-kdl
#$PREFIX/bin/benchmarks-metapod
#$PREFIX/bin/benchmarks-julia.jl
