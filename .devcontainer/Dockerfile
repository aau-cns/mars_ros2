FROM ros:humble-ros-base
# Install the xz-utils package

# Set the timezone to UTC
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install essential build tools
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    g++ \
    clang \
    clang-tidy \
    cmake \
    python3-pip \
    libgtest-dev \
    doxygen \
    graphviz \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -y \
    vim \
    git \
    clang-format \
    && rm -rf /var/lib/apt/lists/*
    
ENV SHELL /bin/bash

CMD ["/bin/bash"]