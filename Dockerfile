# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Copy source files to directory
COPY . .

# Install dependency
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install libsdl2-dev cmake g++ make -y

# Build recipe
RUN cd RVO2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    cd ../../rds && \
    make
