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

While standard ROS2 can run on Linux-class embedded devices, smaller microcontrollers (MCU) are still a challenge. For example, ARM Cortex-M3 and -M4 class MCUs are popular for motion control and sensor integration, but have little RAM and storage. Also, they are often running with small batteries, e.g. for environmental sensors or small consumer robots, which requires efficient power management. We aim to address these devices through a combination of specialized, ROS2-interoperable stack as well as the use of small Real-Time Operating Systems (RTOS).

This article documents requirements, design ideas and related works on this undertaking.



## Requirements

*   **Seamless integration with ROS2**
    *   ...

*   **Portability of ROS2-based software**
    *   ...

*   **Support of a broad spectrum of device classes**
    *   ...

*   **Support of prevalent communication protocols**
    *   ...

*   **Modularity**
    *   ...

*   **MCU-specific base functionalities**
    *   ...

In the EU project OFERA, we compiled a longer list of requirements to an embedded ROS2 stack, which might serve as a good basis for the discussion. Please see Section 5 in [ofera.eu/deliverables/D1.7_Requirements.pdf](http://ofera.eu/deliverables/D1.7_Requirements.pdf).



## Prior and on-going works

### EU project OFERA

The EU project OFERA (Open Framework for Embedded Robot Applications) aims at a ROS2-compatible stack for MCUs in the range of STM32F4 or STM32L1, i.e. with possibly less than 100kB RAM. The project partners currently investigate of using the ROS2 rmw, rcl and rclcpp layers as-is on the micro-XRCE implementation of the upcoming XRCE-DDS standard. In parallel, a more modular approach in the style of rosserial is investigated. In the project's use-cases, NuttX is considered as primary choice for the RTOS.

*   [ofera.eu](http://ofera.eu/) -- project website
*   [github.com/microROS](https://github.com/microROS) -- first project results, including extensions for NuttX for various communication protocols and micro-XRCE
*   [github.com/eProsima/Micro-XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) -- the implementation of XRCE-DSS standard by the project partner eProsima


### ROS2 library for OpenCR by ROBOTIS

... *add description* ...

*   [github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps/arduino/opencr_arduino/opencr/libraries/ROS2](https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps/arduino/opencr_arduino/opencr/libraries/ROS2)


### ROS1-based approaches

*   [rosserial](http://wiki.ros.org/rosserial) -- well-known and widely used in the ROS community ...
*   [mROS](https://github.com/tlk-emb/mROS/) -- a new work on bringing ROS1 concepts (including nodes and the ROS1 middleware) on stronger MCUs, cf. also
    *Hideki Takase, Tomoya Mori, Kazuyoshi Takagi and Naofumi Takagi: 'Work-in-Progress: Design Concept of a Lightweight Runtime Environment for Robot Software Components onto Embedded Devices' in Proc. of ESWEEK, Torino, Italy, September 2018.*


## First Design Ideas

... *diagram from last OFERA meeting* ...

... *possibly also the diagram with the API features and the vertical bars indicating the depth of implementation* ...
