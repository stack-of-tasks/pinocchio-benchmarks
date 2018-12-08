#!/bin/bash

set -e
set -x

export PREFIX=${1:-$PWD/prefix}  # with bash / zsh
export LD_LIBRARY_PATH=${PREFIX}/lib:/opt/ros/kinetic/lib:${LD_LIBRARY_PATH}

$PREFIX/bin/rbdl-test > /tmp/rbdl.txt
$PREFIX/bin/pinocchio-test > /tmp/pinocchio.txt
#$PREFIX/bin/kdl-test > /tmp/kdl.txt
#$PREFIX/bin/metapod-test
#$PREFIX/bin/julia-test.jl

$PREFIX/bin/benchmarks-pinocchio
$PREFIX/bin/benchmarks-rbdl
#$PREFIX/bin/benchmarks-kdl
#$PREFIX/bin/benchmarks-metapod
#$PREFIX/bin/benchmarks-julia.jl
