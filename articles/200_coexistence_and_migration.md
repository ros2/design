---
layout: default
title: Coexistence with and Migration from ROS 1
permalink: articles/coexistence_and_migration.html
abstract:
  This article summarizes how the two major versions of ROS can coexist and be used together.
  Additionally it covers the different aspects of migrating ROS packages from ROS 1 to ROS 2.
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
published: true
categories: Overview
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Goals

### Separately usable

* One version should not be affected by other versions choices / drawbacks.

* ROS 1 users shouldn't be worried about ROS 2 development potentially negatively affecting ROS 1.

  * No changes to existing ROS 1 packages should be necessary to continue to work.

* ROS 2 users shouldn't need to make compromises due to ROS 1 choices / limitations / design decisions.

### Support dual-homed applications

To support creating dual-homed applications like the [ros1_bridge](https://github.com/ros2/ros1_bridge) ROS 1 and ROS 2 must be usable within a single application.

### Minimize migration effort

To ease the migration path the goal is to minimize the effort necessary where possible and when this goal doesn't conflict with the previously mentioned goals.

## Technical Constraints

### Namespace of ROS packages

The first and very important question is about the namespace of ROS packages.
Should ROS 1 and ROS 2 packages use separate namespaces or should they share the same namespace?
For example can there be two packages names `foo` - one in ROS 1 and the other in ROS2 or do they have to be named differently.

In the case that they use separate namespaces that does not prevent the release of a package `bar` into both, ROS 1 as well as ROS 2.

Having them share a single namespace makes it difficult for users to distinguish them.

Using separate namespaces allows e.g. to release a package `foo` in ROS 2 which provides the same API (but different implementation) as the ROS 1 package with the same name.
This way downstream packages don't need to change when switching between the ROS 1 and ROS 2 version of the package.

**Decision:** Use separate namespaces for packages in ROS 1 and ROS 2 (this is currently already the case).

### Namespace of rosdep keys

The next question is very similar and is about the namespace of the `rosdep` keys.
Should ROS 1 and ROS 2 use a different set of `rosdep` rules or should they share the same database?

#### Same key vs. different key

If a ROS package declares a dependency on the `rosdep` key `python-foo` that maps on e.g. Ubuntu Xenial to the Python 2 package `foo`.
In order to work for ROS 2 it needs to depend on the Python 3 version of the package instead.

Currently the mapping in `rosdep` only depends on the platform and doesn't consider which ROS distribution is being used.

If the migration to ROS 2 requires the key to be changed to `python3-foo` that implies that any package which depends on Python packages does need a modification in the manifest.

The same conceptional problems already exist in ROS 1.
When being built on platforms which use Python 3 they already map the keys to Python 3 packages.
If this problem would be address within `rosdep` to consider the Python version as part of the platform that would allow Python 2 as well as Python 3 packages to be resolved using the same `rosdep` key.
This would also allow building / using / testing ROS 1 with Python 3.

Another example is the mapping of `rosdep` key to the Qt 4 or Qt 5 version of a dependency.
Package try to avoid changing the rosdep key used for their dependencies but rosdep only considers the platform without the context of the ROS distribution.

**Proposal:** Change request for `rosdep` to consider information beyond the platform to determine the mapping.

**Decision:** With that change the `rosdep` rules should share a single namespace (this is currently already the case).

### Package manifest

In ROS 1 two versions of the package manifest are supported: format 1 specified in [REP 127](http://www.ros.org/reps/rep-0127.html) and format 2 specified in [REP 140](http://www.ros.org/reps/rep-0140.html).
While ROS 2 only supports format 2 there is no technical limitation behind that design decision.
It simply reduces the effort to distinguish the different semantics of some depend tags in various parts of the software.

It might be the case that a to-be-specified format 3 will be created to add support for the desired features to describe [dependency groups]() as well as declaring an [ABI version]().

#### ROS dependencies

While a ROS 1 package might depend on `roscpp` and `message_generation` a ROS 2 package depends on `rclcpp` and `rosidl_message_generation` instead.

#### Build type

Almost all build types for ROS 1 and ROS 2 are disjoint except the build type `cmake` which can be processed by both build tools.
Since it is not desired to change existing `catkin` / `ament_cmake` packages to use the build type `cmake` those are currently different for each ROS version.

### CMake version

ROS 2 requires CMake version 3.5+.
It should be easily possible to update existing CMake code to be compatible with various versions of CMake.

### Python version

ROS 2 requires Python version 3.5+.
It should be easily possible to update existing Python code to be compatible with Python 2.7 as well as Python 3.5+.

### CMake API

ROS 1 packages use the CMake API provided by [catkin](https://github.com/ros/catkin).
ROS 2 packages use a slightly different CMake API provided by [ament_cmake](https://github.com/ament/ament_cmake).
The rational for the changes and the different name are described in [this article](http://design.ros2.org/articles/ament.html).

**Idea:** Create a ROS 2 package names `catkin` which provides the same CMake API as the ROS 1 package (minus unavailable features like the devel space) by mapping the calls to `ament_cmake` API.
In addition it is expected that existing packages need to call a new function at the end of their CMake code to account for the fact that `ament_package` needs to be invoked *after* all targets where `catkin_package` needs to be invoked *before* them.

### Message / service definitions

In ROS 2 message / service definitions are expected to be in the subdirectory `msg` / `srv`.
This implicitly ensures that message and service names are unique.
There is no technical reason behind that design decision rather than simplifying the implementation.

**Idea:** Allow arbitrary locations in the source space.

Currently ROS 2 already supports additional feature in these files (e.g. default values) which are not available in ROS 1.
In the future ROS 2 might support a different file format to satisfy the requested feature requests (like grouping constants in enums, optional fields, file / constant / field comments, etc.).
If that is the case a backward compatible parser to read existing `.msg` / `.srv` files will likely be available.

### Generated message / service API

In order to satisfy the goal of dual-homed applications the generated code of ROS 1 and ROS 2 need to be side-by-side usable.

For C++ that is already the case.
In ROS 1 the header files are generated in namespace specific to the package name.
In ROS 2 the header files are placed into a sub-namespace names `msg` / `srv`.
Beside the different path / namespace the data structure also uses different types for e.g. the shared pointers (Boost vs. C++11).

For Python that is not the case.

**Proposal:** Change the location of the generated Python modules from using `msg` / `srv` as the subdirectory to `message` / `service`.
At the same time updating the subdirectory name of the C++ headers would make sense to be similar.

For backward compatibility the same API as for ROS 1 could be generated into a separate location.
The user could opt-in to use these to reduce the migration effort but by sacrificing the dual-home support.
In order to use those ROS 1 shim APIs the user would to add another path to the include directories / `PYTHONPATH`.

**Idea:** Implement these additional message generators and provide a mechanism to easily use those interfaces.

## Different categories of changes

For the migration two different categories of changes need to be considered separately:

* Changes which can be applied to ROS 1 packages to either make it compatible with ROS 2 or minimize the necessary changes.
* Changes which are necessary to make the package work for ROS 2 but which are not compatible with ROS 1.

If the second set of changes would be empty a package could be used as a ROS 1 package as well as ROS 2 packages using the same sources.
Currently the previous sections mention a few different items which fall into the second category therefore it doesn't seem realistic to achieve compatibility using the exact same sources.
The following sections assume that it is not feasible to maintain a single source for the ROS 1 as well as the ROS 2 version of a package.

### Changes applicable to ROS 1 packages

* Update the package manifest to format 2.

* In C++ introduce typedefs for all message / service classes as well as their pointer types.
  This will allow to easily change the typedef to a different type in ROS 2.

* Update Python code to also target Python 3.5+.

### Changes necessary for migrating a package to ROS 2

* Update the dependencies in the manifest.

* Update the CMake code to find the updated dependencies.

* The API of the message / service classes has changed:

  * Different headers / modules to include / import.
  * Different classes / methods / types.
  * C++11 interface using e.g. `std::shared_ptr` rather then relying on `Boost`.

* The API of the ROS client libraries has changed (`roscpp` -> `rclcpp`, `rospy` -> `rclpy`):

  * Different headers / modules to include / import.
  * Different classes / methods / types.

## Maintenance

When considering the migration of a package to ROS 2 a few things should be considered:

* Is it planned to support a ROS 1 as well as a ROS 2 version of the package for longer time?

  If not, the following options will likely matter less since only one version is maintained for longer.

If yes, these two variations of the packages can be managed in different ways:

* Both versions can be stored on separate branches within the same repository.

  * (+) The code branches are closely together under the same "roof".
  * (+) Changes between the two branches can be easily ported both ways.
  * (-) Likely less discoverable for users.
  * (-) It might be unclear which version reported issues are referring to.

* One of the versions is stored in a forked repository.

  * (-) The separation makes the packages appear less correlated.
  * (+) Changes between the two branches can be easily ported both ways.
  * (+) More obvious for users (assuming there is comparable infrastructure like distributions file, wiki, etc.).
  * (+) It might be unclear which version reported issues are referring to.

* Both versions are stored in separate unrelated repositories.

  * No obvious advantages over the other two approaches.
    This is likely a good choice when the ROS 2 version of the package is a "full rewrite".

In any case it is important to try getting the current maintainers involved in the process to avoid alienating existing contributors.

## Migration

### API shims

As described in the above section about the [CMake API](#cmake-api) it is possible to create ROS 2 packages mimicking the same name and API as existing ROS 1 packages.
Depending how different the ROS 1 interface is compared to the ROS 2 interface it should be mapped to that becomes more or less feasible.
In the case of `catkin` and `ament_cmake` the functionality is very close and therefore an API shim is realistic.
But even in that case some changes might be necessary as mentioned for a new required call at the end of the CMake code.

The same idea would also work for higher level packages like `roscpp`.
But the amount of API makes this task much more challenging.
Additionally in some cases the existing ROS 1 API might not be mappable without changes to the ROS 2 interface.
In those case it is still required to update user land code even when using the "API shim".
For each case it needs to be carefully evaluated if the effort necessary to implement such an API shim as well as the potentially remaining issues are justified by the advantage it provides.

### "General" Steps

* Apply changes compatible with ROS 1 first.

* Create a `ros2` branch in the same repository or fork the repository.
  In the case of a fork it is still recommended to create a `ros2` branch (and making it the default) to make it easier to port changes between the two repositories.

* Apply ROS 2 specific patches.

* In the future port relevant fixes from either version to the other.
  The more significant the two branches/ forks diverge the more effort / less likely these ports will become.
