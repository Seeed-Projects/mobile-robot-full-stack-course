FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    jupyterlab

# Set the working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
