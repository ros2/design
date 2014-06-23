FROM ubuntu:trusty
MAINTAINER Tully Foote<tfoote@osrfoundation.org>


ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update
RUN apt-get install -q -y curl net-tools python python-yaml build-essential
RUN apt-get install -q -y ruby-dev
RUN apt-get install -q -y nodejs

RUN make --version

RUN gem install jekyll jekyll-sitemap --no-rdoc --no-ri

EXPOSE 4000
VOLUME /tmp/jekyll
WORKDIR /tmp/jekyll

CMD jekyll serve -w --baseurl='' -d /tmp/_site