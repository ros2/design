---
layout: default
title: Embedded ROS 2.0
abstract:
  This article captures requirements, design ideas and related works on a tiny ROS 2.0 stack for microcontrollers.
published: true
author: '[Ingo Luetkebohle](https://github.com/iluetkeb), [Ralph Lange](https://github.com/ralph-lange), ... add many more from Embedded ROS2 Interest Group'
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

While standard ROS2 can run on Linux-class embedded devices, smaller microcontrollers (MCU) are still a challenge. For example, ARM Cortex-M3 and -M4 class MCUs are popular for motion control and sensor integration, but have little RAM and storage. Also, they are often running with small batteries, e.g. for environmental sensors or small consumer robots, which requires efficient power management. We aim to address these devices through a combination of specialized, but ROS2 interoperable, communication, as well as the use of small Real-Time Operating Systems (RTOS).

This article documents requirements, design ideas and related works on this undertaking.



## Requirements



*   Link to OFERA requirements deliverable for details



## Prior and on-going works

### EU project OFERA

https://github.com/microROS


### ROS2 library for OpenCR by ROBOTIS

https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps/arduino/opencr_arduino/opencr/libraries/ROS2


### ROS1-based approaches

*   rosserial
*   mROS


## First Design Ideas

Our diagram ...
