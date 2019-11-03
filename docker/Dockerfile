FROM ros:melodic

MAINTAINER Nikita Maslov <wkernelteam@gmail.com>

RUN rm /etc/apt/apt.conf.d/docker-clean

RUN apt-get update && apt-get -y install nano vim bash-completion \
    libcairomm-1.0-dev && rm -rf /tmp/* /var/tmp/*

RUN mkdir /docker /userdata
COPY bashrc entrypoint.sh /docker/
