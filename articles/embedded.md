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

While standard ROS 2 can run on Linux-class embedded devices, smaller microcontrollers (MCU) are still a challenge. For example, ARM Cortex-M3 and -M4 class MCUs are popular for motion control and sensor integration, but have little RAM and storage. ARM Cortex-M3 is widely used in many sensors due to its optimized power consumption capabilities and ultra low power modes. These microcontrollers are often running with small batteries, e.g. for environmental sensors or small consumer robots, which requires efficient power management. We aim to address these devices through a combination of specialized, ROS 2-interoperable stack as well as the use of Real-Time Operating Systems (RTOS).

This article documents requirements, design ideas and related works on this undertaking.

## Wishlist

A number of things that would be great to have, lets call it a wishlist. Feel free to add stuff ;-)

### W-SEAMLESS: *Seamless integration with ROS&nbsp;2*

The embedded ROS 2 stack shall integrate seamlessly with standard 
ROS 2 stacks and nodes running on more powerful microprocessors. For 
a standard ROS 2 node, communication (topics, services) from and to
software running on the embedded ROS 2 stack should be transparent. 
The same should hold for other core concepts such as parameters, 
graph introspection, and run-time reconfiguration by the node lifecycle.

### W-PORTABILITY: *Portability of ROS 2-based software*

The embedded ROS 2 stack shall resemble or directly use the ROS 2 API 
-- more precisely the rclcpp API -- to facilitate porting standard 
ROS 2 nodes to MCUs.

### W-DEVICES *Support of a broad spectrum of device classes*

The embedded ROS 2 stack shall support a broad range of MCU device 
classes, starting from a few tens kilobyte of RAM.

### W-RTOS: *Support of a broad spectrum of RTOS(s)* 

The stack shall be adaptable to different RTOS (e.g., NuttX, FreeRTOS, 
Zephyr) and provide abstractions with regard to the application 
software to facilitate porting application components to between RTOSs.

### W-COMM: *Support of prevalent communication technologies* 

To cover the broad range of use-cases for MCUs in robotics, the embedded 
ROS 2 stack shall be usable with the default ROS 2 middleware standard 
DDS, simple (custom) serial communication protocols just as common 
wireless technologies like 6LoWPAN

### W-MODULARITY: *High Modularity* 

The embedded ROS 2 stack shall be highly modular, not only to allow for 
adaptation to specific hardware features and different RTOS, but also to
allow the integration with existing frameworks and to be able to create 
customized sub-stacks. For example, it shall be usable in the style of 
rosserial, thus bringing basically the message serialization to the MCU 
only. Similarly, it should be possible to derive a sub-stack that provides 
node graph features and publish/subscribe but no parameters and services, 
and so on.

### W-CONTROL: *Support control-oriented applications*  

MCUs are great for control, and micro-ROS should be as well. This usually 
means hard real-time performance with low jitter and small response times.

### W-POWER *Make low-power modes possible* 

MCUs are often used in battery-powered applications, and/or in applications
with a large amount of standby time. Therefore, the stack should make it 
easily possible to save power.

### W-STATIC: *Use static initialization* 

Static initialization is less error-prone, easier to analyze, and shifts 
memory usage from RAM to flash. It is a requirement for going to very small 
resource use. 

### W-BOOT: *Quick boot times compared to native ROS 2 machines* 

Microcontrollers provide with the capability of booting very quickly
(typically, in the order of tenths of milliseconds or less). This enhances
the existing landscape of ROS 2-native devices which in the best cases, 
require a few seconds to boot an embedded Linux.

### W-INFOMODEL: *A common interface that facilitates interoperability across different vendors* 

An information model for robot devices, adapted by vendors of robot modules, 
would lower costs of integration of robotic systems. Within the OFERA project 
we propose and use the 
[Hardware Robot Information Model (HRIM)](https://acutronicrobotics.com/modularity/hrim/).|

### More Details / Ideas

In the EU project OFERA, we compiled a longer list of requirements to an embedded 
ROS 2 stack, which might serve as a good basis for the discussion. Please see 
Section 5 in 
[OFERA deliverable D1.7_Requirements.pdf](http://ofera.eu/storage/deliverables/OFERA_D1.7_Requirements.pdf).


## Questions

To determine how feasible that is, and to come up with the actual requirements and/or design, a few questions come to mind. I've cross-referenced them to the relevant wishlist items (most of the there's more than one -- this suggests areas where trade-offs may be required, or alternatives pursued).

| Key | Related Wish | Question |
|-----|-------------|----------|
| Q-RTOS | W-RTOS, W-PORTABILITY | Which RTOS(s) do we use as the basis? |
| Q-BUILD | W-RTOS, W-PORTABILITY | How do we handle the RTOS(s) respective build-systems? |
| Q-LANG | W-PORTABILITY | Which language should be used, and at what spec level? |
| Q-API | W-PORTABILITY, W-MODULARITY, W-STATIC, W-POWER | How should the API look in general?|
| Q-PERF | W-PORTABILITY, W-DEVICES, W-CONTROL, W-STATIC | What are the performance implications of the API?|
| Q-COMM | W-COMM, W-SEAMLESS| Which communication/middleware technology is used?|

## Analyses and Experiments

To answer these questions, the OFERA EU project as well as several others have already undertaken or are planning exploratory work.

*Meta-Note*: Please only add a short description here, linking to more detailed pages if necessary.

| Key | Related Question | Action | Description |
|-----|------------------|--------|-------------|
| A-RTOS | Q-RTOS | RTOS Proof-of-Concept | Provide a proof-of-concept RTOS. In the OFERA project, we chose [NuttX](http://nuttx.org/), because it is largely POSIX compatible and thus eases porting. There are also experiments based on [RIOT](https://www.riot-os.org/) (cf. [github.com/astralien3000/riot-ros2](https://github.com/astralien3000/riot-ros2)) and FreeRTOS (cf. [github.com/ros2/ros2_embedded_freertos](https://github.com/ros2/ros2_embedded_freertos)) |
| A-BUILD-META | Q-BUILD | Meta-Build | Explores a meta-build approach to transform ROS 2 CMakeLists.txt to RTOS-specific build instructions. |
| A-BUILD-NUTTX | Q-BUILD | NuttX-specific build | OFERA has integrated (parts of) micro-ROS directly as an app and as a library in the NuttX build. |
| A-BUILD-ARDUINO | Q-BUILD | Arduino Build | ROBOTIS has explored building all the libraries using the Arduino IDE, cf. [github.com/ROBOTIS-GIT/OpenCR](https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps). This required some manual changes and thus does not scale, but can get you off the ground. |
| A-NUTTX-LIBCXX | Q-LANG | C++11/higher support for NuttX | Build libxx from the LLVM project on NuttX, as a pre-requisite to building rclcpp. |
| A-PERF-RCLCPP-RESOURCE | Q-PERF | Determine resource use of rclcpp | |
| A-DDS-XRCE | Q-COMM | Use of DDS-XRCE standard protocol | This OMG standard defines the protocol used by microcontrollers to publish and subscribe data to a DDS Domain, standard in ROS 2. OFERA and ROBOTIS have demonstrated that it is a suitable protocol to seamlessly communicate with microcontrollers. | [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/) |
| A-Micro-XRCE-DDS | Q-COMM | Middleware usage | OFERA, Robotis and others have integrated Micro XRCE-DDS middleware as part of their solutions. This middleware provides an implementation of DDS-XRCE standard. Integrations have been done on top of different RTOSs, NuttX and FreeRTOS and using different underlying transports. | [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) |

## Prior and on-going works

### ROS2-based approaches
*   [**EU project OFERA**](http://ofera.eu/): The EU project OFERA (Open Framework for Embedded Robot Applications) aims at a ROS 2-compatible stack for MCUs in the range of STM32F4 or STM32L1, i.e. with possibly less than 100kB RAM. The project partners currently investigate of using the ROS 2 rmw, rcl and rclcpp layers as-is on the Micro XRCE-DDS implementation of the upcoming XRCE-DDS standard. In parallel, a more modular approach in the style of rosserial is investigated. In the project's use-cases, NuttX is considered as primary choice for the RTOS. Beyond the project page, additional details of the project results can be found at [microros.github.io/micro-ROS/](https://microros.github.io/micro-ROS/).

* [**Hardware Robot Operating System (H-ROS)**](https://acutronicrobotics.com/modularity/H-ROS/) is an a standardized software and hardware infrastructure to create modular robot hardware. H-ROS is actively being used within the OFERA EU project to benchmark and prototype the capabilities of the ROS 2 stack against the ROS 2 embedded stack. In addition, H-ROS implements selected components of the ROS 2.0 stack for microcontrollers.

*   [**ROS 2 library for OpenCR by ROBOTIS**](https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps/arduino/opencr_arduino/opencr/libraries/ROS2): Tailored and optimized implementation of the ROS 2 publish/subscribe and clock API for the Micro XRCE-DDS (formerly micro-RTPS) implementation of the upcoming XRCE-DDS middleware standard running on an STM32F7.

*   [**XEL Network by ROBOTIS**](https://xelnetwork.readthedocs.io):
  Product which communicate with ROS 2 (DDS) through DDS-XRCE using Micro XRCE-DDS in the firmware of their CommXel board.
  This CommXel board manages the rest of the boards conforming the XEL Network and interface them to a ROS 2 space.
  The CommXel could use Ethernet or UART to communicate using DDS-XRCE.

*   [**freeRTPS**](https://github.com/ros2/freertps): A free, portable, minimalist implementation of the RTPS protocol for microcontrollers such as the STM32F7 developed at the OSRF. FreeRTPS shall allow to run ROS 2 with standard DDS as-is on stronger MCUs. This project has been discontinued in 2016.

*   [**ros2_embedded_nuttx**](https://github.com/ros2/ros2_embedded_nuttx): Early port (in 2014) of ROS 2 alpha for the STM32F4Discovery board and the STM3240G eval board running the RTOS NuttX developed by Víctor Mayoral Vilches and Esteve Fernandez at the OSRF.

*   [**Renesas GR-ROSE**](http://gadget.renesas.com/ja/event/2018/pm_rose.html):
  Renesas have integrated their GR-ROSE platform with ROS 2 using DDS-XRCE protocol.
  They use Micro XRCE-DDS implementation on top of FreeRTOS.
  A sample can be found in their forums [renesas forum ](https://japan.renesasrulz.com/gr_user_forum_japanese/f/gr-rose/5201/ros-2-micro-rtps).
  They have integrated Micro XRCE-DDS middleware as part of their [online web compiler](http://tool-cloud2.renesas.com/index.php) for the GR-ROSE platform.

### ROS1-based approaches

*   [**rosserial**](http://wiki.ros.org/rosserial): Well-known and widely used in the ROS community.

*   [**mROS**](https://github.com/tlk-emb/mROS/): A new work on bringing ROS1 concepts (including nodes and the ROS1 middleware) on stronger MCUs, cf.
    *Hideki Takase, Tomoya Mori, Kazuyoshi Takagi and Naofumi Takagi: 'Work-in-Progress: Design Concept of a Lightweight Runtime Environment for Robot Software Components onto Embedded Devices' in Proc. of ESWEEK, Torino, Italy, September 2018.*

## Design Discussion

The following figure may serve as a starting point for the design discussion. It depicts the major layers from the real-time operating system to the application, in the style the ROS 2 standard stack.

![micro-ROS](/img/embedded/features_with_dependencies.png)

At the same time, the diagram illustrates the possible feature set of the client library -- ideally in a modular fashion so that different profiles can be derived from it. The vertical bar at each feature gives an indication of the dependencies with lower layers and thus on the portability to different RTOS and middlewares.

In the OFERA project, a more detailed diagram has been developed, which can be found at [microros.github.io/micro-ROS/](https://microros.github.io/micro-ROS/).
