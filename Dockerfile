FROM python:3.8

WORKDIR /usr/src/colcon-cache

COPY requirements.txt .
COPY test/requirements.txt ./test/
RUN pip install -r test/requirements.txt

COPY . .
RUN colcon build \
        --symlink-install

RUN colcon test && \
    colcon test-result --verbose
