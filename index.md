---
layout: default
title: ROS2 Design
---

# ROS 2.0 Design

This site is repository of articles which are designed to inform and guide the ROS 2.0 design efforts.
The goal of the ROS 2.0 project is to leverage what is great about ROS 1.x and improve what isn't.

# Articles

Here is a list of the articles (white papers) we have written which should serve as an excellent entry point for anyone wanting to join the conversation about any of the topics below:

{% for p in site.pages %}
<!-- {{ p.url }} -->
    {% if p.url contains 'articles/' %}
- <a href="{{ p.url }}">{{ p.title }}</a>
    {% endif %}
{% endfor %}
