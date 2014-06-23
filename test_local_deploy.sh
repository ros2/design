#!/bin/sh

docker build -t "ros_tutorials" .

docker run -v `pwd`:/tmp/jekyll -w /tmp/jekyll -i -t -p 0.0.0.0:4000:4000 ros_tutorials