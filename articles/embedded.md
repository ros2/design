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

While ROS 2 can run on Linux-class embedded devices, smaller 
microcontrollers (MCU) are still a challenge. For example, ARM Cortex-M3 
and -M4 class MCUs are popular for motion control and sensor integration, 
but have little RAM and storage. ARM Cortex-M3 is widely used in many 
sensors due to its optimized power consumption capabilities and ultra low 
power modes. These microcontrollers are often running with small batteries, 
e.g. for environmental sensors or small consumer robots, which requires
efficient power management. The embedded ROS 2 effort addresses these devices.

This article documents feature wishes, requirements, design ideas and 
related work on this undertaking.

## Important Differences / Assumptions

One of the major difference for the embedded stack is to be able to
run not (only) on Linux, but also on Real-Time Operating Systems, or
even "bare metal" (no OS).

## Wishlist

A number of things that would be great to have, lets call it a wishlist. Feel free to add stuff ;-)

### W-SEAMLESS: *Seamless integration with ROS&nbsp;2*

Support all the communication and configuration mechanisms expected,
e.g. topics, services, parameters (including dynamic modification),
the life-cycle, etc.

It should not be noticeable to the rest of the system that part of
it runs on a micro-controller.

### W-PORTABILITY: *Portability of ROS 2-based software*

Use the same API as on ROS 2.

Note that the feasibility of this wish has been questioned.

### W-DEVICES *Support of a broad spectrum of device classes*

Support a broad range of MCU device classes, starting from a 
few tens kilobyte of RAM, up to megabytes.

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


## Questions

To determine how feasible that is, and to come up with the actual 
requirements and/or design, a few questions come to mind. We've 
cross-referenced them to the relevant wishlist items (most of the 
time there's more than one -- this suggests areas where trade-offs 
may be required, or alternatives pursued).


### Q-RTOS: *Which RTOS(s) do we use as the basis?*

#### Related: W-RTOS, W-PORTABILITY

### Q-BUILD: *How do we handle the RTOS(s) respective build-systems?*

#### Related: W-RTOS, W-PORTABILITY 

### Q-LANG: *Which language(s) should be used, and at what spec level?*

#### Related: W-PORTABILITY 

### Q-API: *How should the API look in general?*

#### Related: W-PORTABILITY, W-MODULARITY, W-STATIC, W-POWER

### Q-PERF: * What are the performance implications of the API?*

#### Related: W-PORTABILITY, W-DEVICES, W-CONTROL, W-STATIC

### Q-COMM: *Which communication/middleware technology is used?*

#### Related: W-COMM, W-SEAMLESS


## Analyse and Experiment Actions

To answer these questions, the OFERA EU project as well as several 
others have already undertaken or are planning exploratory work.

*Meta-Note*: Please only add a short description here, linking to 
more detailed pages if necessary.

### A-RTOS: *RTOS Proof-of-Concept* 

#### Questions: *Q-RTOS*

Provide a proof-of-concept RTOS. In the OFERA project, we chose 
[NuttX](http://nuttx.org/), because it is largely POSIX compatible 
and thus eases porting. There are also experiments based on 
[RIOT](https://www.riot-os.org/) (cf. 
[github.com/astralien3000/riot-ros2](https://github.com/astralien3000/riot-ros2))
and FreeRTOS (cf. 
[github.com/ros2/ros2_embedded_freertos](https://github.com/ros2/ros2_embedded_freertos)) |

### A-BUILD-META: *Meta-Build*

#### Questions: Q-BUILD 

Explores a meta-build approach to transform ROS 2 CMakeLists.txt to 
RTOS-specific build instructions. |

###  A-BUILD-NUTTX: *NuttX-specific build*

#### Questions: Q-BUILD 

OFERA has integrated (parts of) micro-ROS directly as an app and 
as a library in the NuttX build.

### A-BUILD-ARDUINO: *Arduino Build*

### Questions: Q-BUILD  

ROBOTIS has explored building all the libraries using the Arduino 
IDE, cf. [github.com/ROBOTIS-GIT/OpenCR](https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps). 
This required some manual changes and thus does not scale, but 
can get you off the ground.

###  A-NUTTX-LIBCXX: *C++11/higher support for NuttX*

#### Questions: Q-LANG  

Build libxx from the LLVM project on NuttX, as a pre-requisite to building rclcpp.

### A-PERF-RCLCPP-RESOURCE: *Determine resource use of rclcpp*

#### Questions: Q-PERF 

There are doubts whether rclcpp can really fit tiny MCUs, but we'll only 
know once we tried and measured it.

### A-DDS-XRCE: *Use of DDS-XRCE standard protocol*

#### Questions: Q-COMM  

This OMG standard defines a protocol for use by microcontrollers to 
publish and subscribe data to a DDS Domain, standard in ROS 2. OFERA 
and ROBOTIS have demonstrated that it is a suitable protocol to seamlessly
communicate with microcontrollers. See [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/)

### A-Micro-XRCE-DDS: *Micro-XRCE Middleware usage*

#### Questions: Q-COMM  

OFERA, Robotis and others have integrated Micro XRCE-DDS middleware as 
part of their solutions. This middleware provides an implementation of 
DDS-XRCE standard. Integrations have been done on top of different RTOSs, 
NuttX and FreeRTOS and using different underlying transports. 

See [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) |

## Prior and on-going works

### ROS2-based approaches
*   [**EU project OFERA**](http://ofera.eu/): The EU project OFERA 
(Open Framework for Embedded Robot Applications) aims at a ROS 2-compatible 
stack for MCUs in the range of STM32F4 or STM32L1, i.e. with possibly less 
than 100kB RAM. The project partners currently investigate of using the 
ROS 2 rmw, rcl and rclcpp layers as-is on the Micro XRCE-DDS implementation 
of the upcoming XRCE-DDS standard. In parallel, a more modular approach in 
the style of rosserial is investigated. In the project's use-cases, NuttX 
is considered as primary choice for the RTOS. Beyond the project page, 
additional details of the project results can be found at 
[microros.github.io/micro-ROS/](https://microros.github.io/micro-ROS/).

* [**Hardware Robot Operating System (H-ROS)**](https://acutronicrobotics.com/modularity/H-ROS/) 
is an a standardized software and hardware infrastructure to create modular 
robot hardware. H-ROS is actively being used within the OFERA EU project to 
benchmark and prototype the capabilities of the ROS 2 stack against the ROS 2 
embedded stack. In addition, H-ROS implements selected components of the ROS 
2.0 stack for microcontrollers.

*   [**ROS 2 library for OpenCR by ROBOTIS**](https://github.com/ROBOTIS-GIT/OpenCR/tree/feature-ros2-micrortps/arduino/opencr_arduino/opencr/libraries/ROS2): 
Tailored and optimized implementation of the ROS 2 publish/subscribe 
and clock API for the Micro XRCE-DDS (formerly micro-RTPS) implementation 
of the upcoming XRCE-DDS middleware standard running on an STM32F7.
This initiative has been factored out under the [ros2arduino](https://github.com/ROBOTIS-GIT/ros2arduino) name
[discourse.ros.org announcement](https://discourse.ros.org/t/ros2arduino-discussion-for-development-ros2-library-for-arduino/6498).

*   [**XEL Network by ROBOTIS**](https://xelnetwork.readthedocs.io):
  Product which communicate with ROS 2 (DDS) through DDS-XRCE using Micro 
  XRCE-DDS in the firmware of their CommXel board.
  This CommXel board manages the rest of the boards conforming the XEL 
  Network and interface them to a ROS 2 space.
  The CommXel could use Ethernet or UART to communicate using DDS-XRCE.

*   [**freeRTPS**](https://github.com/ros2/freertps): A free, portable, 
minimalist implementation of the RTPS protocol for microcontrollers such as 
the STM32F7 developed at the OSRF. FreeRTPS shall allow to run ROS 2 with 
standard DDS as-is on stronger MCUs. This project has been discontinued in 2016.

*   [**ros2_embedded_nuttx**](https://github.com/ros2/ros2_embedded_nuttx): 
Early port (in 2014) of ROS 2 alpha for the STM32F4Discovery board and the 
STM3240G eval board running the RTOS NuttX developed by Víctor Mayoral Vilches 
and Esteve Fernandez at the OSRF.

*   [**Renesas GR-ROSE**](http://gadget.renesas.com/ja/event/2018/pm_rose.html):
  Renesas have integrated their GR-ROSE platform with ROS 2 using DDS-XRCE protocol.
  They use Micro XRCE-DDS implementation on top of FreeRTOS.
  A sample can be found in their forums 
  [renesas forum ](https://japan.renesasrulz.com/gr_user_forum_japanese/f/gr-rose/5201/ros-2-micro-rtps).
  They have integrated Micro XRCE-DDS middleware as part of their
  [online web compiler](http://tool-cloud2.renesas.com/index.php) for the GR-ROSE platform.

*   [**Renesas RX65N MCU**](https://www.renesas.com/us/en/about/press-center/news/2018/news20181029.html):
  Renesas announce that their RX65N Microcontrollers Support DDS-XRCE using eProsimas' Micro XRCE-DDS implementation.
  All software used in this demonstration will be open sourced and be available in Q4 2018.

*    [**Amazon aws-ros-client-library-microcontrollers**](https://github.com/awslabs/aws-ros-client-library-microcontrollers)
  Amazon open source project where an initial draft of a rcluc and rmwu layers interfaces have been designed.
  The protocol used is DDS-XRCE and uses Micro XRCE-DDS implementation.
  [discourse.ros.org announcement](https://discourse.ros.org/t/ros2arduino-discussion-for-development-ros2-library-for-arduino/6498/12)

### ROS1-based approaches

*   [**rosserial**](http://wiki.ros.org/rosserial): Well-known and widely used 
in the ROS community.

*   [**mROS**](https://github.com/tlk-emb/mROS/): A new 
    work on bringing ROS1 concepts (including nodes and the 
    ROS1 middleware) on stronger MCUs, cf.
    *Hideki Takase, Tomoya Mori, Kazuyoshi Takagi and Naofumi Takagi: 
    'Work-in-Progress: Design Concept of a Lightweight Runtime Environment 
    for Robot Software Components onto Embedded Devices' in 
    Proc. of ESWEEK, Torino, Italy, September 2018.*

## Design Discussion

The following figure may serve as a starting point for the design discussion.
It depicts the major layers from the real-time operating system to the 
application, in the style the ROS 2 standard stack.

![micro-ROS](/img/embedded/features_with_dependencies.png)

At the same time, the diagram illustrates the possible feature set of the 
client library -- ideally in a modular fashion so that different profiles 
can be derived from it. The vertical bar at each feature gives an indication
of the dependencies with lower layers and thus on the portability to 
different RTOS and middlewares.

In the OFERA project, a more detailed diagram has been developed, which can 
be found at [microros.github.io/micro-ROS/](https://microros.github.io/micro-ROS/).
