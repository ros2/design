# ROS 2 design

This repository is a [Jekyll](http://jekyllrb.com/) website hosted on [Github Pages](http://pages.github.com/) at http://design.ros2.org/.

The repository/website is meant to be a point around which users can collaborate on the ROS 2 design efforts as well as capture those discussions for posterity.

The best place for discussing these topics is [the Next Generation ROS category on the ROS Discourse](https://discourse.ros.org/c/ng-ros).

## Working Locally

You can run the site locally by running this command in this repository:

```
jekyll serve --watch --baseurl=''
```

And navgiating to your browser to:

[http://localhost:4000/](http://localhost:4000/)


## Working Locally (Docker)

If you don't want to (or can't) install Ruby and other Jekyll dependencies directly on your machine, you can run the server via Docker. Navigate to the directory containing this repository, and start the server via

```
docker run -v $PWD:/srv/jekyll -p 4000:4000 jekyll/jekyll jekyll serve --watch --incremental
```

You can then navigate your browser to:

[http://localhost:4000/](http://localhost:4000/)


## Site Setup

Site is a Jekyll website with `design.ros2.org` as the `CNAME`.

The site requires no static generation outside of github's static jekyll generation, which means that changes are published to the site as soon as they are pushed (can take up to 10 minutes for github to update the site).

The github login (for showing pull requests) also requires that https://github.com/prose/gatekeeper is setup in heroku, and the url for that is http://auth.design.ros2.org/authenticate/TEMP_TOKEN. Because of the free Heroku instance, the first time someone logins in after a period of time, there is a delay.
