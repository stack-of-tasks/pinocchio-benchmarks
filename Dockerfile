ARG IMAGE=ubuntu
ARG TAG=18.04

FROM ${IMAGE}:${TAG}

RUN mkdir /benchmarks
WORKDIR /benchmarks

RUN apt update -qqy \
 && apt install -qqy \
    ccache \
    clang-6.0 \
    cmake \
    libboost-all-dev \
    liburdfdom-dev

ADD . .

RUN ./compile.sh

CMD ./run.sh
