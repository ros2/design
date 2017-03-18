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

In the ROS ecosystem teams typically solve problems by programming in many software packages at the same time.
This is in contrast to workflows where teams work on a single software package at a time only, and all dependencies are installed and updated by package managers.
Such a workflow is impracticable without a tool that automatically rebuilds many packages in topological order of dependency.
The rosbuild tool enabled this workflow first and was a critical factor in the success of ROS.

The ROS1 ecosystem has catkin (and the deprecated rosbuild) for this purpose.
The ROS2 ecosystem has ament.

Both catkin and ament toolsets have 2 conceptually separate parts, a build tool part that deals with a single package at a time, and a build system part that deals with invoking build tools in the right order.

Both ament and catkin toolsets have largely similar code for the build system part (more details follow below), but their build tool parts are very different from each other, requiring quite different build descriptors.

This article describes the steps to unify the build systems of ROS1 and ROS2, but not their build tools.

## Goal

The goal of the build tool is to build a set of packages with a single invocation automating the process.
It should work with:
* ROS 1 packages
* ROS 2 packages
* other software sources when sufficient meta information is externally provided.

The last point allows to build non-ROS dependencies of ROS packages (e.g. Gazebo including its dependencies).

In the ROS ecosystems several tools already exist which support this use case (see below).
Each of the existing tools performs similar tasks and duplicates a significant amount of the logic.
As a consequence of being developed separately certain features are only available in some of the tools while other tools lack those.

The reason to work on a single universal build tool comes down to reducing the effort necessary for development and maintenance.
Additionally this makes new features developed for one ROS version / build system available to the other supported ROS versions / build systems.

### Out of Scope

The build tool does not cover the steps necessary to fetch the sources of the to-be-built packages.
There are already tools to help with this.
For example, the list of repositories that need to be fetched is provided either by a hand crafted `.rosinstall` or `.repos` file or by using [rosinstall_generator](http://wiki.ros.org/rosinstall_generator) to generate one.
The list of repositories can then be fetched with one of several tools, like [rosinstall](http://wiki.ros.org/rosinstall) or [wstool](http://wiki.ros.org/wstool) in the case of a `.rosinstall` file, or [vcstool](https://github.com/dirk-thomas/vcstool) in the case of a `.repos` file.

The build tool also does not provide a mechanism to install any dependencies required to build the packages.
In the ROS ecosystem [rosdep](http://wiki.ros.org/rosdep) can be used for this.

The build tool also does not create binary packages (e.g. a Debian package).
In the ROS ecosystem [bloom](http://wiki.ros.org/bloom) is used to generate the required metadata and then platform dependent tools like `dpkg-buildpackage` build binary packages.

## Build Tool vs. Build System vs. Package manager

A build system schedules and executes atomic tasks for creating runnable software from sources.
It determines the dependency graph and invokes the specific build tool for each package in topological order.
Examples are Gnu Make, cmake, python setuptools.

A build tool schedules and invokes the build systems for separate source trees (packages) in topological order of dependency.
Example are rosbuild, catkin_make, ament.

A package manager downloads, optionally builds, then installs released packages from origin locations.
Examples are dpkg, rpm, homebrew, portage, robotpkg.

### Examples of Build Tools vs. Build Systems

|                                             | Build System | Build Tool |
|---------------------------------------------|:------------:|:----------:|
| CMake project                               |       x      |            |
| catkin (CMake that calls catkin macros)     |       x      |            |
| ament_cmake (CMake that calls ament macros) |       x      |            |
| catkin_make                                 |              |      x     |
| catkin_make_isolated                        |              |      x     |
| catkin_tools                                |              |      x     |
| ament_tools                                 |              |      x     |


The following describes the essential functional requirements for a build tool, and further functional and non-functional requirements that are to be decided.
For example, in order for a CMake project to discover a dependency using the CMake function `find_package`, the CMake module (e.g. `FindFoo.cmake`) or the CMake config file (e.g. `FooConfig.cmake`) for that dependency must either be in a prefix that CMake searches implicitly (e.g. `/usr`) or the location must be provided through the environment variable `CMAKE_PREFIX_PATH` / `CMAKE_MODULE_PATH`.

In addition to building a package on top of another package (using `find_package` in the case of CMake), you may need to adjust the environment in order to run an executable from a package.
For example, when a package installs a shared library in a non-default location then the environment variable `LD_LIBRARY_PATH` (or `PATH` on Windows) needs to be extended to include the containing folder before trying to run executables that load that library at runtime.

The functionality to setup these environment variables can be provided by either the build tool or the build system.
In the latter case the build tool only needs to know how the build system exposes the environment setup in order to reuse it.

### Development Environment Setup

Invoking a buildsystem for a package implies also setting up environment variables before the process.
Examples are the `CMAKE_PREFIX_PATH` and the `LD_LIBRARY_PATH`.
For consistency of the build result, those variables should be restricted, allowing the build to only access declared dependencies.

For various reasons, it is beneficial to also allow developers to easily manually invoke the buildsystem for one package.
This requires the buildsystem to provide the environment setup by itself.

### Subtask invocation

Build systems define various tasks, such as testing, compiling, generating documentation, installing.
A build tool must provide it's own abstract set of tasks to be mapped to the tasks of different build systems.

### Isolated installation

For the ROS ecosystem it is important that packages can be installed to any target location in a filesystem.

After a package has been built and installed to a target location, the environment might need to be extended to use the package.

### Convenience features

* Build Parallelity
* Invocation from various locations, roscd
* Workspace chaining
* rosbuild compatibility (ROS_PACKAGE_PATH)


## Existing Build Systems

In the following the build systems being used in the ROS ecosystem are briefly described.

### cmake

cmake is a popular build system that runs on various operating systems.

### catkin

[catkin](http://wiki.ros.org/catkin) is an addition to CMake and provides a set of convenience functions to make writing CMake packages easier.
It automates the generation of CMake config files as well as pkg-config files.
It additionally provides functions to register different kinds of tests.

### ament_cmake

[ament_cmake](https://github.com/ament/ament_cmake) is an evolution of `catkin` and is an addition to CMake.
The main difference between `ament_cmake` and `catkin` is described in [another article](http://design.ros2.org/articles/ament.html).
In the context of the build tool the biggest difference is that `ament_cmake` generates package-specific files to setup the environment to use the package after it has been built and installed.

### Python setuptools

`setuptools` is the common tool to package Python packages.
A Python package uses a `setup.py` file to describe the dependencies as well as how and what to build and install.
In ROS 2 a package can be a "vanilla" Python package whereas in ROS 1 any Python functionality is triggered from a CMake file.

## Existing Build Tools

Several different build tools are already being used in the ROS ecosystem.
Their method of operating is being described in the following subsections together with their advantages as well as disadvantages.

### rosbuild

Rosbuild was the first build system for ROS, it was based on Makefiles.
Rosbuild did not provide any install target, and kept sources and build results in the same folder structure.
Rosbuild was replaced because it did not support packaging, cross-compiling, and because build speed was an issue.

### catkin_make

`catkin_make` replaced rosbuild for ROS1, it is provided by the ROS package `catkin`.
It has been designed as the successor of `rosbuild` since ROS Fuerte.

The tool invokes CMake only a single time and uses CMake's `add_subdirectory` function to process all packages in a single context.

The tool uses CMake's `include` function to process all packages in a single cmake context.
While this is an efficient approach since all targets across all packages can be parallelized it comes with significant disadvantages.
Due to the single context all function names, targets and tests share a single namespace across all packages and on a larger scale this easily leads to collisions.
The single context is also subject to side effects between the packages and sometimes requires adding additional target dependencies across package boundaries.

### catkin_make_isolated

`catkin_make_isolated` is included with `catkin`.
It was developed after `catkin_make` to address the problems involved with building multiple packages in a single CMake context.

The tool only supports CMake-based packages and builds each package in topological order using the command sequence common for CMake packages: `cmake`, `make`, `make install`.
While each package can parallelize the build of its targets the packages are processed sequentially even if they are not (recursive) dependencies of each other.

### catkin_tools

[catkin_tools](https://catkin-tools.readthedocs.io/) is provided by a standalone Python package used to build catkin packages.
It was developed after `catkin_make` / `catkin_make_isolated` to build packages in parallel as well as provide significant usability improvements.
The tool supports building CMake packages and builds them in isolation as well as supports parallelizing the process across packages.


### ament_tools

`ament_tools` is provided by a standalone Python 3 package used to build ROS 2 packages.
It was developed to bootstrap the ROS 2 project, is therefore only targeting Python 3, and works on Linux, MacOS and Windows.
In addition to CMake packages it also supports building Python packages and can infer meta information without requiring an explicit package manifest.
The tool performs an "isolated" build like `catkin_make_isolated` and `catkin_tools` (one CMake invocation per package) and also parallelizes the build of packages which have no (recursive) dependencies on each other (like `catkin_tools`).

## Naming

The existing build tools are all named by the build system they are supporting.
In general it should be possible for a build tool to support multiple different build systems.
Therefore a name for a build tool being derived from a single build system might mislead the users that the tool only works for that specific build system.
To avoid confusion of the user the build tool should have a different unrelated name to avoid implying an undesired correlation.

## Requirements

The unified build tool should provide a superset of the functionality provided by the existing tools.
In the following a few use cases are described as well as desired software criteria.

Other use cases which are not explicitly covered but are already supported by the existing tools (e.g. cross-compilation, `DESTDIR` support, building CMake packages without a manifest) should continue to work with the unified build tool.

### Use Cases

The following uses cases should be satisfied by the unified build tool.

#### Build ROS 1 workspaces

The tool needs to be able to build ROS 1 workspaces which can already be built using `catkin_make_isolated` / `catkin_tools`.
It is up to the implementation to decide if it only supports the standard CMake workflow or also the *custom devel space concept* of `catkin`.

#### Build ROS 2 workspaces

The tool needs to be able to build ROS 2 workspaces which can already be built using `ament_tools`.

#### Build Gazebo including dependencies

After cloning the repositories containing Gazebo and all its dependencies (excluding system packages) the tool needs to be able to build the set of packages.
Meta information not inferable from the sources can be provided externally without adding or modifying any files in the workspace.
After the build a single file can be sourced / invoked to setup the environment to use Gazebo (e.g. `GAZEBO_MODEL_PATH`).

#### Mixing different build systems

The build tool will support using different build systems within a single workspace.
If these packages inter-operate with each other correctly depends also to a large degree on the build system.
The build tool should ensure that it doesn't prevent that use case.

### Software Criteria

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

## Possible Approaches

In terms of flexibility neither of the existing build tools can already support the superset of features described in this article.
There are two different paths possible to reach the goal of a universal build tool:

### Evolve catkin_tools

One approach is to incrementally evolve one of the existing tools to satisfy the described goals.
Since `catkin_tools` is in many aspects the most complete build tool it should be one being evolved.
While `ament_tools` has a few features `catkin_tools` currently lacks (e.g. plain CMake support without a manifest, Windows support) the feature richness of `catkin_tools` makes it a better starting point.

The following items are highlighting some of the necessary efforts (not a complete list):

- Refactor the software architecture of the existing code base to support the flexibility sketched by the extension points listed above.
- Move `catkin` specific concepts out of the core of the build tool into a catkin specific extension (e.g. manifest format, *devel space*).
- Support for ROS 2 which includes:

  - Support for Python 3 and Windows.
  - Support for pure Python packages as well as packages without an in-source manifest file.
  - Environment setup of `ament` packages

- Rename the tool to use a name unrelated to one build system.
- Investigate if a feature like continued support of the *devel space* is feasible since it doesn't apply to other build system and might be complicated to separate without sacrificing usability.

### Start "from scratch"

Another approach is to implement the necessary software architecture to enable the desired flexibility and modularity "from scratch".
Then fill in the features step-by-step by porting existing building blocks from the existing solutions.
Some items to highlight the necessary efforts (not a complete list):

- Create the software architecture to support the flexibility sketched by the extension points listed above which will be easier "from scratch" than for an existing code base.
- Port / reimplement many of the features existing in the other build tools. It will take a non trivial amount of time to reach the feature level of e.g. `catkin_tools`
- Thorough test the functionality and write documentation for developers as well as users.

## Proposal

*to be decided*
