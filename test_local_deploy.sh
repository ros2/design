#!/bin/sh

docker run --rm -v `pwd`:/srv/jekyll -i -t -p 127.0.0.1:4000:4000 jekyll/jekyll:pages
