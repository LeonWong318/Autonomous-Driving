# Copyright (C) 2024 OpenDLV

FROM alpine:3.19 as builder

RUN apk update && \
    apk --no-cache add \
        cmake \
        g++ \
        make \
        linux-headers \
        opencv-dev


ADD . /opt/sources
RUN mkdir /opt/build /opt/out
WORKDIR /opt/build
RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/opt/out /opt/sources
RUN make
RUN make install


FROM alpine:3.19
RUN apk update && apk --no-cache add \
    libopencv_highgui \
    libopencv_core \
    ttf-dejavu


COPY --from=builder /opt/out/ /usr
ENTRYPOINT ["/usr/bin/project-task-1"]