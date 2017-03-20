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

## Goal

The goal of the build tool is to build a set of packages with a single invocation automating the process.
It should work with ROS 1 packages as well as ROS 2 packages which provide the necessary information in their manifest files.
It should also work with packages that do not provide manifest files themselves, given that the meta information is externally provided.
This will allow the build tool to be utilized for non-ROS dependencies of ROS packages (e.g. Gazebo including its dependencies).

In the ROS ecosystems several tools already exist which support this use case (see below).
Each of the existing tools performs similar tasks and duplicates a significant amount of the logic.
As a consequence of being developed separately certain features are only available in some of the tools while other tools lack those.

The reason to work on a single universal build tool comes down to reducing the effort necessary for development and maintenance.
Additionally this makes new features developed for one ROS version / build system available to the other supported ROS versions / build systems.

### Current Workspace and tool combinations

| Concept Layer | ROS2                               | ROS1 (catkin)                  | ROS1 (rosbuild)         |
|--------------|------------------------------------|--------------------------------|-------------------------|
| Workspace    | ROS2 ament workspace               | ROS1 catkin workspace          | ROS1 rosbuild workspace |
| Build tool   | ament_tools                        | catkin_tools, catkin_make, cmi | rosbuild                |
| Build system | ament_cmake, cmake, python         | catkin, cmake                  | rosbuild                |
| Package      | ROS2 package, cmake/python project | ROS1 package, cmake project    | ros1 rosbuild package   |

(Note catkin_make cannot build pure cmake projects)

### Phase 1 goal: unify build systems (not rosbuild)

| Concept Layer     | ROS2                                | ROS1 (catkin)                  |
|-------------------|-------------------------------------|--------------------------------|
| Workspace         | ROS2 ament workspace                | ROS1 catkin workspace          |
| Build tool        | *{universal build tool}*            | *{universal build tool}*       |
| Workspace adapter | *ament workspace adapter*           | *catkin workspace adapter*     |
| Build system      | ament_cmake, cmake, python          | catkin, cmake                  |
| Package           | ROS2 package, cmake/python project  | ROS1 package, cmake project    |

This phase seems realistic to achieve in predictable time because both catkin_tools and ament_tools already have a large overlap.

A coarse pseudo-code looks like this

1. Detect workspace folder to use (e.g. current folder, or parent folder with marker file, ...)
2. Load workspace adapter for this workspaces buildsystem
3. Ask plugin/wrapper for package declarations (name, folder and dependencies of each package)
4. Create DAG of packages for building the workspace
5. Invoke buildsystem plugin command for each package in build order

### Phase 2 goal: unify workspaces

| Concept Layer         | All ROS / non-ROS                         |
|-----------------------|-------------------------------------------|
| Build tool            | *{universal build tool}*                  |
| Workspace             | *{universal workspace}*                   |
| Build system adapter  | for catkin, ament, cmake, python, ...     |
| Build system          | catkin, ament_cmake, cmake, python ...    |
| Package               | ROS1/ROS2 packages, cmake/python projects |

Note this kind of homogenous workspace would likely require a changes to catkin and the cmake makros, also requiring users to migrate.


### Out of Scope

The build tool does not cover the steps necessary to fetch the sources of the to-be-built packages.
There are already tools to help with this.
For example, the list of repositories that need to be fetched is provided either by a hand crafted `.rosinstall` or `.repos` file or by using [rosinstall_generator](http://wiki.ros.org/rosinstall_generator) to generate one.
The list of repositories can then be fetched with one of several tools, like [rosinstall](http://wiki.ros.org/rosinstall) or [wstool](http://wiki.ros.org/wstool) in the case of a `.rosinstall` file, or [vcstool](https://github.com/dirk-thomas/vcstool) in the case of a `.repos` file.

The build tool also does not provide a mechanism to install any dependencies required to build the packages.
In the ROS ecosystem [rosdep](http://wiki.ros.org/rosdep) can be used for this.

The build tool also does not create binary packages (e.g. a Debian package).
In the ROS ecosystem [bloom](http://wiki.ros.org/bloom) is used to generate the required metadata and then platform dependent tools like `dpkg-buildpackage` build binary packages.

## Build Tool vs. Build System

A build tool operates on a set of packages.
It determines the dependency graph and invokes the specific build tool for each package in topological order.
The build tool itself should know as little as possible about the build system used for a specific package.
Just enough in order to know how to setup the environment for it, invoke the build, and setup the environment to use the built package.

The build system on the other hand operates on a single package.
In the case a package uses e.g. `CMake` the responsibility of the tool comes down to invoke the common steps `cmake`, `make`, `make install` for this package.
As another example for a package using `Autotools` the steps could look like `configure`, `make`, `make install`.

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

### Environment Setup

A very important part beside the actual build of a package is the environment setup.
For example, in order for a CMake project to discover a dependency using the CMake function `find_package`, the CMake module (e.g. `FindFoo.cmake`) or the CMake config file (e.g. `FooConfig.cmake`) for that dependency must either be in a prefix that CMake searches implicitly (e.g. `/usr`) or the location must be provided through the environment variable `CMAKE_PREFIX_PATH` / `CMAKE_MODULE_PATH`.

In addition to building a package on top of another package (using `find_package` in the case of CMake), you may need to adjust the environment in order to run an executable from a package.
For example, when a package installs a shared library in a non-default location then the environment variable `LD_LIBRARY_PATH` (or `PATH` on Windows) needs to be extended to include the containing folder before trying to run executables that load that library at runtime.

The functionality to setup these environment variables can be provided by either the build tool or the build system.
In the latter case the build tool only needs to know how the build system exposes the environment setup in order to reuse it.

Considering the use case that a user might want to invoke the build system of each package manually it is beneficial if the build system already provides as much of the environment setup as possible.
That avoids forcing the user to manually take care of the environment setup when not using a build tool.

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

The tool invokes CMake only a single time and uses CMake's `add_subdirectory` function to process all packages in a single context.
While this is an efficient approach since all targets across all packages can be parallelized it comes with significant disadvantages.
Due to the single context all function names, targets and tests share a single namespace across all packages and on a larger scale this easily leads to collisions.
The single context is also subject to side effects between the packages and sometimes requires adding additional target dependencies across package boundaries.

### catkin_make_isolated

`catkin_make_isolated` is provided by the ROS package `catkin` which contains the build system for ROS 1.
It was developed after `catkin_make` to address the problems involved with building multiple packages in a single CMake context.

The tool only supports CMake-based packages and builds each package in topological order using the command sequence common for CMake packages: `cmake`, `make`, `make install`.
While each package can parallelize the build of its targets the packages are processed sequentially even if they are not (recursive) dependencies of each other.

### catkin_tools

[catkin_tools](https://catkin-tools.readthedocs.io/) is provided by a standalone Python package used to build ROS 1 packages.
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
There are multiple different paths possible to reach the goal of a universal build tool which fall into two categories:

- One approach is to incrementally evolve one of the existing tools to satisfy the described goals.
- Another approach would be to start "from scratch".

### Evolve catkin_make, catkin_make_isolated, or ament_tools

Since neither of these three build tools has the feature richness of `catkin_tools` it is considered strictly less useful to starting building upon one of these build tools.

### Evolve catkin_tools

Since `catkin_tools` is in many aspects the most complete build tool it should be the one being evolved.
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
