FROM alpine:3.19 as builder
RUN apk update && \
    apk --no-cache add \
    cmake \
    g++ \
    make \
    linux-headers \
    opencv-dev

ADD . /opt/sources
WORKDIR /opt/sources
RUN rm -rf build && \
    mkdir -p build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/build-dest .. && \
    make && make install

FROM alpine:3.19

# Install only the parts of OpenCV required to run to minimize image size
RUN apk update && apk --no-cache add \
    libopencv_core \
    libopencv_aruco \
    libopencv_imgcodecs \
    libopencv_highgui

WORKDIR /usr/bin
COPY --from=builder /tmp/build-dest/ /usr/
ENTRYPOINT ["convert-video-to-img"]
