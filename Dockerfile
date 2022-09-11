FROM python:3

WORKDIR /usr/src/app

COPY . .
RUN pip install -e .[test]

RUN colcon build \
        --symlink-install

RUN colcon test && \
    colcon test-result --verbose
