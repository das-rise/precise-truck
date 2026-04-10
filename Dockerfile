# Multi-stage Dockerfile for WayWise
# Optimized for C++ builds with minimal runtime image

# Build stage
FROM ubuntu:22.04 AS builder

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    qtbase5-dev \
    libqt5serialport5-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    wget \
    libgpiod-dev \
    && rm -rf /var/lib/apt/lists/*

# Install MAVSDK
RUN wget --no-check-certificate https://github.com/mavlink/MAVSDK/releases/download/v2.14.1/libmavsdk-dev_2.14.1_ubuntu22.04_amd64.deb \
    && dpkg -i libmavsdk-dev_2.14.1_ubuntu22.04_amd64.deb \
    && rm libmavsdk-dev_2.14.1_ubuntu22.04_amd64.deb

# Set working directory
WORKDIR /precise-truck

# Copy source code
COPY . .

# Initialize submodules and build
RUN git submodule update --init \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j$(nproc)

# Runtime stage
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies only
RUN apt-get update && apt-get install -y \
    libqt5core5a \
    libqt5network5 \
    libqt5serialport5 \
    libqt5printsupport5 \
    libqt5widgets5 \
    libboost-program-options1.74.0 \
    libboost-system1.74.0 \
    libatomic1 \
    && rm -rf /var/lib/apt/lists/*

# Copy MAVSDK libraries from builder
COPY --from=builder /usr/lib/libmavsdk*.so* /usr/lib/

# Update library cache
RUN ldconfig

# Create non-root user for running the application
RUN useradd -m -s /bin/bash precisetruck

# Copy built executables
COPY --from=builder /precise-truck/build/PreciseTruck /home/precisetruck/
RUN chown -R precisetruck:precisetruck /home/precisetruck/

# Copy configuration files from project
COPY --from=builder /precise-truck/config /home/precisetruck/config
RUN chown -R precisetruck:precisetruck /home/precisetruck/config

# Set user and working directory
USER precisetruck
WORKDIR /home/precisetruck

# Labels for container metadata
LABEL org.opencontainers.image.title="PRECISE Truck"
LABEL org.opencontainers.image.description="An autonomous Truck based on WayWise for Precise Security Chaos Engineering experiments."
LABEL org.opencontainers.image.source="https://github.com/das-rise/precise-truck"
LABEL org.opencontainers.image.vendor="RISE Research Institutes of Sweden"
LABEL org.opencontainers.image.licenses="GPL-3.0"