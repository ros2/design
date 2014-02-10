---
layout: default
title: ROS2 Design
---

# ROS 2.0 Design

This site is repository of articles which are designed to inform and guide the ROS 2.0 design efforts.
The goal of the ROS 2.0 project is to leverage what is great about ROS 1.x and improve what isn't.

If you would like to contribute, checkout the [contribute]({{ site.baseurl }}/contribute) page to learn how.

# Articles

Here is a list of the articles (white papers) which have been written so far. These articles should serve as an excellent entry point for anyone wanting to join the conversation about any of the ROS 2.0 topics.

{% for p in site.pages %}
    {% if p.url contains 'articles/' %}
----
#### [{{ p.title }}]({{ site.baseurl }}{{ p.url }})

{{ p.abstract }}
    {% endif %}
{% endfor %}
----
