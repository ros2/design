---
layout: default
title: Embedded ROS 2.0
abstract:
  This article captures requirements, design ideas and related works on a tiny ROS 2.0 stack for microcontrollers.
published: true
author: 'In alphabetic order: [Adam Dąbrowski](https://github.com/adamdbrw), [Borja Outerelo](https://github.com/BorjaOuterelo), [Ingo Lütkebohle](https://github.com/iluetkeb), [Ralph Lange](https://github.com/ralph-lange), [Víctor Mayoral Vilches](https://github.com/vmayoral), ... and many more from Embedded ROS 2 Interest Group. Please feel free to add your name.'
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

While standard ROS 2 can run on Linux-class embedded devices, smaller microcontrollers (MCU) are still a challenge. For example, ARM Cortex-M3 and -M4 class MCUs are popular for motion control and sensor integration, but have little RAM and storage. ARM Cortex-M3 is widely used in many sensors due to its optimized power consumption capabilities and ultra low power modes. These microcontrollers are often running with small batteries, e.g. for environmental sensors or small consumer robots, which requires efficient power management. We aim to address these devices through a combination of specialized, ROS 2-interoperable stack as well as the use of small Real-Time Operating Systems (RTOS).

This article documents requirements, design ideas and related works on this undertaking.

## Requirements

In this section, we first describe basic requirements to the embedded ROS stack -- as a basis for discussion. This list shall be detailed by the feedback of the community.

*   **Seamless integration with ROS 2:** The embedded ROS 2 stack shall integrate seamlessly with standard ROS 2 stacks and nodes running on stronger microprocessors. For a standard ROS 2 node, communication (topics, services) from and to software running on the embedded ROS 2 stack should be transparent. The same should hold for other core concepts such as parameters, graph introspection, and run-time reconfiguration by the node lifecycle.

*   **Portability of ROS 2-based software:** The embedded ROS 2 stack shall resemble or directly use the ROS 2 API -- more precisely the rclcpp API -- to facilitate porting standard ROS 2 nodes to MCUs.

*   **Support of a broad spectrum of device classes and RTOS:** The embedded ROS 2 stack shall support a broad range of MCU device classes, starting from a few tens kilobyte of RAM. This implies suitable abstractions to be able to adapt the stack to specific hardware features/mechanisms as well as to replace modules or layers with optimized implementations.

    Similarly, the stack shall be adaptable to different RTOS (e.g., NuttX, FreeRTOS, Zephyr) and provide abstractions with regard to the application software to facilitate porting application components to between RTOSs.

*   **Support of prevalent communication technologies:** To cover the broad range of use-cases for MCUs in robotics, the embedded ROS 2 stack shall be usable with the default ROS 2 middleware standard DDS, simple (custom) serial communication protocols just as common wireless technologies like Bluetooth Low Energy

*   **High Modularity:** The embedded ROS 2 stack shall be highly modular, not only to allow for adaptation to specific hardware features and different RTOS, but also to allow the integration with existing frameworks and to be able to create customized sub-stacks. For example, it shall be usable in the style of rosserial, thus bringing basically the message serialization to the MCU only. Similarly, it should be possible to derive a sub-stack that provides node graph features and publish/subscribe but no parameters and services, and so on.

*   **MCU-specific core functionalities and mechanisms:** Finally, the embedded ROS 2 stack shall include novel core functionalities and mechanisms that are specific for MCUs. These include power management functionalities and mechanisms for static initialization and memory management.

In the EU project OFERA, we compiled a longer list of requirements to an embedded ROS 2 stack, which might serve as a good basis for the discussion. Please see Section 5 in [ofera.eu/deliverables/D1.7_Requirements.pdf](http://ofera.eu/deliverables/D1.7_Requirements.pdf).



## Prior and on-going works

### ROS2-based approaches
*   [EU project OFERA](http://ofera.eu/): The EU project OFERA (Open Framework for Embedded Robot Applications) aims at a ROS 2-compatible stack for MCUs in the range of STM32F4 or STM32L1, i.e. with possibly less than 100kB RAM. The project partners currently investigate of using the ROS 2 rmw, rcl and rclcpp layers as-is on the micro-XRCE-DDS implementation of the upcoming XRCE-DDS standard. In parallel, a more modular approach in the style of rosserial is investigated. In the project's use-cases, NuttX is considered as primary choice for the RTOS. Beyond the project page, additional details of the project results can be found at [http://micro-ros.com](http://micro-ros.com).

*   [ROS 2 library for OpenCR by ROBOTIS](https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ROS2-micrortps/arduino/opencr_arduino/opencr/libraries/ROS2)


### ROS1-based approaches

*   [rosserial](http://wiki.ros.org/rosserial) -- well-known and widely used in the ROS community ...

*   [mROS](https://github.com/tlk-emb/mROS/) -- a new work on bringing ROS1 concepts (including nodes and the ROS1 middleware) on stronger MCUs, cf. also
    *Hideki Takase, Tomoya Mori, Kazuyoshi Takagi and Naofumi Takagi: 'Work-in-Progress: Design Concept of a Lightweight Runtime Environment for Robot Software Components onto Embedded Devices' in Proc. of ESWEEK, Torino, Italy, September 2018.*
*   [freeRTPS](TODO): TODO

*   [ros2_embedded_nuttx](TODO): TODO

## First Design Ideas

* This is our proposed architecture:

![micro-ROS](/img/embedded/micro-ROS_proposed_architecture.png)

... *possibly also the diagram with the API features and the vertical bars indicating the depth of implementation* ...