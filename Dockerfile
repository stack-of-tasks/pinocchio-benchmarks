FROM ubuntu:18.04

RUN mkdir /benchmarks
WORKDIR /benchmarks

RUN apt update -qqy \
 && apt install -qqy \
    ccache \
    clang-6.0 \
    cmake \
    libboost-all-dev \
    libeigen3-dev \
    liburdfdom-dev

ADD . .

RUN ./compile.sh

CMD ./run.sh
