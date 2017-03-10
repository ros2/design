---
layout: default
title: A universal build tool
permalink: articles/build_tool.html
abstract:
  This article describes the rationale for a universal build tool.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
categories: Overview
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Preface

ROS is developed with a federated model.
It consists of many separately developed and maintained packages.

The "manual" approach to build a set of packages consists of building all packages in their topological order one by one.
For each package the documentation usually describes what the dependencies are, how to setup the environment to build the package as well as how to setup the environment afterwards to use the package.
From an efficiency point of view that manual process does not scale well.

This article describes the steps from the current build tools used in the ROS ecosystem to a single universal build tool which automates the process and works for various different use cases.

## Goal

The goal of the build tool is to build a set of packages with a single invocation automating the process.
It should work with ROS 1 packages as well as ROS 2 packages which provide the necessary information in their manifest files.
It would be great if the tool could also be used to build other packages (e.g. Gazebo including its dependencies).

### Out of Scope

The build tool does not cover the steps necessary to fetch the sources of the to-be-built packages.
This can be done with existing tools (e.g. [rosinstall_generator](http://wiki.ros.org/rosinstall_generator), [vcstool](https://github.com/dirk-thomas/vcstool)).

The build tool also does not provide a mechanism to install any dependencies required to build the packages.
In the ROS ecosystem [rosdep](http://wiki.ros.org/rosdep) can be used for this.

The build tool also does not create binary packages (e.g. a Debian package).
In the ROS ecosystem [bloom](http://wiki.ros.org/bloom) is used to generate the required metadata and then platform dependent tools like `dpkg-buildpackage` build binary packages.

## Build Tool vs. Build System

A build tool operates on a set of packages.
It determines the dependency graph and invokes the package specific build tool of each package in topological order.
The build tool itself should know as little as possible about the build system used for a specific package.
Just enough in order to know how to setup the environment for it, invoke the build, and setup the environment to use the built package.

The build system on the other hand operates on a single package.
In the case a package uses e.g. `CMake` the responsibility of the tool comes down to invoke the common steps `cmake`, `make`, `make install` for this package.
As another example for a package using `Autotools` the steps could look like `configure`, `make`, `make install`.

### Environment Setup

A very important part beside the actual build of a package is the environment setup.
For a CMake package e.g. a dependency discovered using the CMake function `find_package` needs to be either in a common location or the location must be provided through the environment variable `CMAKE_PREFIX_PATH`.
On the other hand after a package has been built and installed the environment might need to be extended to use the package.
E.g. when a package installs a shared library in a non-default location the environment variable `LD_LIBRARY_PATH` (or `PATH` on Windows) needs to be extended to include the containing folder.

The functionality to setup these environment variables can be provided by either the build tool or the build system.
In the latter case the build tool only needs to know how the build system exposes the environment setup in order to reuse it.

Considering the use case that a user might want to invoke the build system of each package manually it is beneficial if the build system already provides as much of the environment setup as possible.
That avoids that the user has to manually take care of the environment setup when not using a build tool.

## Existing Build Systems

In the following the build systems being used in the ROS ecosystem are briefly described.

### catkin

[catkin](http://wiki.ros.org/catkin) is based on CMake and provides a set of convenience functions to make writing CMake packages easier.
It automates the generation of CMake config files as well as pkg-config files.
It additionally provides functions to register different kinds of tests.

### ament_cmake

[ament_cmake](https://github.com/ament/ament_cmake) is an evolution of `catkin` and is also based on CMake.
The main difference between `ament_cmake` and `catkin` is described in [another article](http://design.ros2.org/articles/ament.html).
In the context of the build tool the biggest difference is that `ament_cmake` generates package-specific files to setup the environment to use the package after it has been built and installed.

### Python setuptools

`setuptools` is the common tool to package Python packages.
A Python package uses a `setup.py` file to describe the dependencies as well as how and what to build and install.
In ROS 2 a package can be a "vanilla" Python package whereas in ROS 1 any Python functionality is triggered from a CMake file.

## Existing Build Tools

Several different build tools are already being used in the ROS ecosystem.
Their method of operating is being described in the following subsections together with their advantages as well as disadvantages.

### catkin_make

`catkin_make` is provided by the ROS package `catkin` which contains the build system for ROS 1.
It has been designed as the successor of `rosbuild` for ROS Fuerte.

The tool invokes CMake only a single time and uses CMake's `include` function to process all packages in a single context.
While this is an efficient approach since all targets across all packages can be parallelized it comes with significant disadvantages.
Due to the single context all function names, targets and tests share a single namespace across all packages and on a larger scale this easily leads to collisions.
The single context is also subject to side effects between the packages and sometimes requires to add additional target dependencies across package boundaries.

### catkin_make_isolated

`catkin_make_isolated` is provided by the ROS package `catkin` which contains the build system for ROS 1.
It was developed after `catkin_make` to address the problems involved with building multiple packages in a single CMake context.

The tool only supports CMake-based packages and builds each package in topological order using the command sequence common for CMake packages: `cmake`, `make`, `make install`.
While each package can parallelize the build of its targets the packages are processed sequentially even if they are not (recursive) dependencies of each other.

### ament_tools

`ament_tools` is provided by a standalone Python 3 package used to build ROS 2 packages.
It was developed to bootstrap the ROS 2 project, is therefore only targeting Python 3, and works on Linux, MacOS and Windows.
In addition to CMake packages it also supports building Python packages and can infer meta information without requiring an explicit package manifest.
The tool performs an "isolated" build like `catkin_make_isolated` (one CMake invocation per package) but also parallelizes the build of packages which have no (recursive) dependencies on each other.

### catkin_tools

[catkin_tools](https://catkin-tools.readthedocs.io/) is provided by a standalone Python package used to build ROS 1 packages.
It was developed after `catkin_make` / `catkin_make_isolated` to build packages in parallel as well as provide significant usability improvements.
The tool supports building CMake packages and builds them in isolation as well as supports parallelizing the process across packages.

### Naming

The existing build tools are all named by the build system they are supporting.
In general it should be possible for a build tool to support multiple different build systems.
Therefore a name for a build tool being derived from a single build system might mislead the users that the tool only works for that specific build system.
To avoid confusion of the user it would be beneficial if the build tool would have a different name to avoid implying an undesired correlation.

## Rationale

Currently each of the existing tools performs similar tasks and duplicates a significant amount of the logic.
Certain features are only available in some of the tools while other tools lack those.

The reason to work on a single universal build tool comes down to reducing the effort necessary for development and maintenance.
Additionally this makes new features developed for one ROS version / build system available to the other supported ROS versions / build systems.

## Requirements

The unified build tool should provide a superset of the functionality provided by the existing tools.
In the following a few use cases are described as well as desired software criteria.

### Use Cases

The following uses cases should be satisfied by the unified build tool.

#### Build ROS 1 workspaces

The tool needs to be able to build ROS 1 workspaces which can already be built using `catkin_make_isolated` / `catkin_tools`.

#### Build ROS 2 workspaces

The tool needs to be able to build ROS 2 workspaces which can already be built using `ament_tools`.

#### Build Gazebo including dependencies

After cloning the repositories containing Gazebo and all its dependencies (excluding system packages) the tool needs to be able to build the set of packages.
Meta information not inferable from the sources can be provided externally without adding or modifying any files in the workspace.
After the build a single file can be sourced / invoked to setup the environment to use Gazebo (e.g. `GAZEBO_MODEL_PATH`).

## Software Criteria

The tool aims to support a variety of build systems, use cases, and platforms.
The above mentioned ones are mainly driven by the needs in the ROS ecosystem but the tool should also be usable outside the ROS ecosystem (e.g. for Gazebo).
Therefore it should be designed in a way which enables extending its functionality.

Assuming that the tool will be implemented in Python (since that is the case for existing tools) the entry point mechanism provides a convenient way to make the software extensible.
Extensions don't even have to be integrated into the Python package containing the core logic of the build tool but can easily be provided by additional Python packages.
This approach will not only foster a modular design and promote clear interfaces but enable external contributions without requiring them to be integrated in a single monolithic package.

Several well known software principles apply:

- Separation of concerns
- Single Responsibility principle
- Principle of Least Knowledge
- Don’t repeat yourself
- Keep it stupid simple
- "Not paying for what you don't use"

### Extension Points

The following items are possible extension points to provide custom functionality:

- contribute `verbs` to the command line tool (e.g. `build`, `test`)
- contribute command line options for specific features (e.g. `build`, `test`)
- discovery of packages (e.g. recursively crawling a workspace)
- identification of packages and their meta information (e.g. from a `package.xml` file)
- process a package (e.g. build a CMake package, test a Python package)
- execution model (e.g. sequential processing, parallel processing)
- output handling (e.g. console output, logfiles, status messages, notifications)
- setup the environment (e.g. `sh`, `bash`, `bat`)
- completion (e.g. `bash`, `Powershell`)

## Alternative Approaches

There are two different approaches possible to reach the goal of a universal build tool.

- Incrementally evolve one of the existing tools to satisfy the described goals.
- Implement a new build tool "from scratch" (while existing building blocks can still be "copied" from the existing solutions).

## Proposal

*to be decided*
