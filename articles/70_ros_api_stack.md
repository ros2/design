---
layout: default
title: ROS 2 API Stack
permalink: articles/ros_api_stack.html
abstract:
  This article describes the API's that will be defined for the ROS Client Libraries.
author: '[William Woodall](https://github.com/wjwwood)'
published: false
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


## The ROS API Stack

The purpose of this article is to layout the different API's which are in the ROS Client Library.
Some of the API's are user facing and others are intermediate API's, but all will be part of specification of ROS.

![ROS API's Stack](/img/ros_api_stack/stack.png)

The above diagram summarizes the API's which will be defined and how they build on top of each other in a stack below the user's applications.
Only the very top level API's, like `rclc`, `rclcpp`, and `rclpy`, are exposed to the user in most cases.
The `rclc` API is exposed most frequently to developers who wish to build a new client library, e.g. `rcljava` or `rclruby`, and reuse as much of the existing implementation as possible.
The `ros_middleware_interface` is what a middleware implementation must implement in order to have ROS utilize it, and so this is the interface which must be implemented for new DDS vendors to be supported.
This article will go into more detail on each of these layers and how they interact.

### 
