FROM nvcr.io/nvidia/isaac-lab:2.3.1 AS base

USER root

RUN apt update && apt install -y \
    mesa-utils

WORKDIR /workspace

SHELL ["bash", "-ic"]
