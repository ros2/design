---
layout: default
title: The meta build system "ament"
permalink: articles/ament.html
abstract:
  This article describes the meta build system "ament".
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
While this comes with a lot of advantages it makes the process of building several interdependent packages more complex.

In order to build a set of interdependent packages the user must build them in the correct topological order.
The user must look at the documentation of each package to determine the dependencies, being careful to build those packages before hand, and to determine the instructions for building that package.
Finally the user must build and install each package and do any environment configuration before proceeding to build the next package.

ROS mitigates that complexity with package and build system conventions as well as tools to automate the build process.
Wherever possible ROS uses standard tools, e.g. CMake and Python setuptools.
Where these tools lack built-in support for what ROS needs to do, the additional functionality is provided by ROS.

### Origin of ament

ament is an evolution of [catkin](http://wiki.ros.org/catkin).
The word *ament* is actually a synonym of *catkin*.
For more information about the differences please read [below](#how-is-ament-different-from-catkin).

## ament

ament is a meta build system to improve building applications which are split into separate packages.
It consists of two major parts:

- a *build system* (e.g. CMake, Python setuptools) to configure, build, and install a single package
- a *tool* to invoke the build of individual packages in their topological order

The tool relies on meta information about the packages to determine their dependencies and their build type.
This meta information is defined in a manifest file called `package.xml` which is specified in [REP 140](http://www.ros.org/reps/rep-0140.html).

Each package is built separately with its own build system.
In order to make the output of one package available to other packages each package can extend the environment in a way that downstream packages can find and use its artifacts and resources.
If the resulting artifacts are installed into `/usr`, for example, it might not be necessary to alter the environment at all since these folders are commonly being searched by various tools.

### The ament command line tool

[ament_tools](https://github.com/ament/ament_tools) is a Python package which provides the command line tool `ament` to build, test, install, and uninstall packages.
It is similar to [catkin_tools](https://github.com/catkin/catkin_tools) and builds each package in a workspace in topological order.
Support for different build systems is integrated through extension points which allows to contribute support for other build types without change the ament tool itself.

While it currently does not build packages in parallel that feature will be added in the future to speed up the build process.
The goal is to reuse common functionality from catkin_tools by making it available through a third package which can be used by both tools.

### Integrate arbitrary build systems

Each package can utilize a different build system to perform the steps of configuring, building, and installing.
The build type is defined in each package manifest using the [build_type](http://www.ros.org/reps/rep-0140.html#build-type) tag in the export section.

Currently supported are CMake and Python but support for others (e.g. autotools, plain Makefiles) will likely be added in the future.
For each build system the native steps are being applied separately by the ament tool.

#### Build type: ament_cmake

The CMake package `ament_cmake` provides several convenience functions to make it easier to write CMake-based packages:

- It generates a CMake config file for the package.
  That allows passing information (e.g. about include directories and libraries) to downstream packages.
  Additionally it makes it easy to pass along information from recursive dependencies (and e.g. takes care about the ordering of include directories).

- It provides an easy interface to register tests and ensure that JUnit-compatible result files are generated for those.
  Currently it supports a few different testing frameworks like `nosetests`, `gtest`, and `gmock`.

- It allows a package to generate environment hooks to extend the environment e.g. by extending the `PATH`.

- It provides a CMake API to read and write [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md) entries.
  The index is built at build time and provides efficient access to information like the available packages, messages, etc.

- It provides an uninstall target for convenience.

Most of these features are implemented in separate packages.
The CMake code uses an extension point system to foster modularity of the code.
This enables others to add further features without requiring to alter existing functionality in the core of ament_cmake.

#### Build type: cmake

The CMake build type uses plain CMake and gives all the flexibility as well as responsibility to the developer.

#### Build type: ament_python

The Python build type allows packages to use setuptools with only a `setup.py` file.
It uses the standard Python work flow to build Python packages.

The ament tool will copy the package manifest into the install location.
It will also extend the environment variables `PYTHONPATH` and `PATH` to include the Python modules and executables provided by the package.

### Environment creation

Commonly the install space should follow the [Filesystem Hierarchy Standard (FHS)](http://www.pathname.com/fhs/).
But that is just a recommendation and not enforced by ament.

Depending on where the packages are being installed it might be necessary to setup the environment in order to find all resources.
E.g. the location of installed executables should be on the `PATH`, installed Python code should be on the `PYTHONPATH` etc.

Therefore each package can provide a shell script to setup the environment to match its needs.
These package specific scripts are provided in the folder `<prefix>/share/<pkg-name>`.
The `local_setup.*` files will update environment variables as specified by the package.
Even when the package is built without the ament tool these setup files are being generated.

The different shell scripts in the root of the install space are generated by the ament tool.
The `local_setup.*` files only iterate over all packages in the install space (by reading the list of `packages` in the ament index) and source their package specific setup files.
The `setup.*` files also consider workspaces outside of this install space (by reading the list of `parent_prefixp_path` in the ament index) and source them before the `local_setup.*` files.

### Optional symlinked install

It is very important to maximize the efficiency of the development cycle of changing code, building, and installing it and then run it to confirm the changes.
Commonly the installation steps involves copying some resources from the source space to their final destination in the install location.
ament provides an option to use symbolic links instead (if the platform supports that).
This enables the developer to change the resources in the source space and skipping the installation step in many situations.

For CMake packages this is achieved by optionally overriding the CMake `install()` function.
For Python packages the [development mode](http://pythonhosted.org//setuptools/setuptools.html#development-mode) is used to install the package.
The symlinked install is an optional feature and must be enabled explicitly by the developer using the command line option `--symlink-install`.

### ament linters

ament provides a set of linters to check that source code complies with the ROS 2 style guidelines.
The usage of these linters is optional but it is very easy to integrate them as part of the automated tests of a package.

Linters can also check non-style related criteria.
E.g. [cppcheck](http://cppcheck.sourceforge.net/) is used to statically analyze C / C++ code and check for semantic bugs.

### ament_auto

ament_auto is similar to catkin_simple.
It aims to simplify writing CMake code for an ament package.
It avoid repetition of dependencies by extracting them from the manifest.
Additionally it automates several steps by relying on conventions, e.g.:

- all information from dependencies are used for compiling and linking of all target
- if a package has an `include` folder it adds that folder to the include directories, installs all headers from this folder, and exports the include folder to downstream packages
- if the package contains interface definitions in the `msg` and / or `srv` subfolder they will be processed automatically
- all libraries and executables are being installed to default locations

For an example see [below](#how-is-ament-different-from-catkin).

### Additional resources

The implementation of ament is spread across several repositories:

- [ament_package](https://github.com/ament/ament_package) is a Python package providing an API to parse the files containing the meta information.

- [ament_cmake](https://github.com/ament/ament_cmake) contains a set of packages which provide CMake based functionality.
  The features are split across several packages in order to ensure that they are cleanly separated from each other.

- [ament_lint](https://github.com/ament/ament_lint) contains a set of packages which perform linting tasks.
  For a set of linters a ROS 2 specific configuration is provided which can be used via the command line as well as from within CMake (commonly as a CTest).

- [ament_tools](https://github.com/ament/ament_tools) is a Python package which provides command line tools to build, test, install, and uninstall packages.

- [ament_index](https://github.com/ament/ament_index) contains a set of packages which provide API to access the [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md).
  Currently it only contains a Python implementation but other languages like C++ should follow.

## How is *ament* different from *catkin*

catkin has been used for ROS 1 since ROS Groovy.
It was developed as a replacement for [rosbuild](http://wiki.ros.org/rosbuild) which was used from the beginning of ROS.
Therefore various features of catkin have been designed to be similar to rosbuild.

### Why not continue to use *catkin*

catkin has many advantages over rosbuild.
Those cover out-of-source builds, automatic CMake config file generation, an installation target and many more.

Over time, however, feedback about the shortcomings of catkin has been collected.
Also additional tools have been developed (like catkin_simple and catkin_tools) to improve the user / developer experience.

This feedback was used to develop the next iteration of catkin which was then called ament.
A few of these cases are mentioned below and all addressed by ament:

#### CMake centric

catkin is based around CMake and even packages only containing Python code are being processed via CMake.
Because of that a setup.py file in a catkin package can only utilize a small subset of features from setuptools.
Common features like extension points are not supported which makes it more difficult to deploy a package on Windows.

#### *Devel space*

catkin has the feature to provide a so called [devel space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space) after building a set of packages.
That folder is providing a fully working ROS environment without the need to install the packages.
Avoiding copying any files allows users to e.g. edit Python code and immediately try running the code.

While this is a very convenient feature and speeds up the development process it comes at a cost.
The necessary logic in catkin increases its complexity significantly.
Additionally the CMake code in every ROS package has to make sure to handle the *devel space* correctly which puts an extra effort on every ROS developer.

ament provides the same advantage using the optional feature of *symlinked installs* without the extra complexity for each ROS package.

#### *CMAKE_PREFIX_PATH*

catkin relies directly on the `CMAKE_PREFIX_PATH` environment variable to store the prefixes of multiple workspaces.
This has been considered not a good approach since it interferes with other values set in the variable and is a CMake specific build variable.
Therefore ament uses a separate environment variable (`AMENT_PREFIX_PATH`) for that purpose which is used at runtime.
At build time of CMake packages the CMake specific variable can be derived from the generic ament variable.

#### catkin_simple

The ROS package [catkin_simple](https://github.com/catkin/catkin_simple) was the attempt to make the common cases of developing ROS packages easier.
While it is able to reduce the complexity of the CMake code in some ROS packages it fails conceptually in other cases.
Some of the limitations were due to core catkin design decisions like the order and position of calling certain CMake functions etc.

E.g. the function `catkin_package()` must be invoked *before* any target in order to setup the appropriate location for the build targets in the devel space.
But in order to automatically perform several tasks in the code generation step this functionality needs to happen *after* all target have been defined.

With the different design of ament it becomes possible to implement a package similar to catkin_simple which can actually work reliably in all the cases where catkin_simple fails.

#### Building within a single CMake context

catkin allows users to build multiple packages within a single CMake context (using `catkin_make`).
While this significantly speeds up the process it is not scalable at large.
This is due to the fact that all packages share the same CMake namespace.
Therefore the package might have colliding target names or fail to build correctly due to side effects between the packages.
ament does not provide that feature due to these drawbacks.

### Additional improvements in ament over catkin

catkin is a single monolithic package providing various features and functionalities.
E.g. it integrates support for `gtest` which makes it very difficult to also optionally support `gmock` since only one tool can be selected at a time.

ament on the other hand is designed to be very modular.
Almost every feature is provided by a separate package and can be optionally used.
The CMake part of ament also features an extension point system to extend it with further functionality (see above).

### Why not evolve *catkin* with the necessary features

One important goal of ROS 2 is to [*not* disrupt ROS 1](http://design.ros2.org/articles/why_ros2.html) in any way.
Any changes to catkin would potentially introduce regressions into ROS 1 which is undesired.

Additionally some necessary changes would require a different usage of catkin in every ROS package.
Releasing such a change into ROS 1 in a future distribution would imply a significant effort for each developer which should also be avoided.
Not to mention the effort to update the documentation and tutorials and making the users aware of the subtle changes.

### Why use a new name instead of continuing to call it *catkin*

For the use case of building a bridge between ROS 1 and ROS 2 it must be possible to access both code bases in a single project.
With both build systems having the same name they would not be distinguishable on a CMake level (which of the two packages is found with `find_package(catkin)`?).
Therefore the package names of both build system must be different.

The newly developed build system has similar CMake functions but they have partly different arguments and / or are being called in different locations.
With both system called the same (or even very similar) a significant user confusion was expected.
Naming the new build system `catkin2` was considered, but the similarity would still have led to user confusion.
Therefore a different name has been selected to clarify the different context and behavior.

### Why has *catkin_pkg* been forked to *ament_package*

ament as well as ROS 2 in general is using Python 3 exclusively.
`catkin_pkg` as well as many other ROS 1 Python packages cannot be side-by-side installed for Python 2 and Python 3 at the same time.
So requiring python3-catkin-pkg for ROS 2 would collide with python-catkin-pkg for ROS 1 and would make both ROS versions not installable at the same time.

ament_package only implements the specification [REP 140](http://www.ros.org/reps/rep-0140.html).
It does not support *package.xml* files with an older format.

Additionally it does provide the templates for all the environment hooks so that each package can generate its own environment and does not rely on an external tool to setup the environment.
Beside that the package provides none of the additional functionalities of catkin_pkg (no crawling, no topological ordering, etc.).

### Example CMakeLists.txt files

The following CMake files are examples showing the similarities and differences between catkin and ament:

- [CMakeLists.txt](https://gist.github.com/dirk-thomas/596a53bbb922c4d6988a) using catkin
- [CMakeLists.txt](https://gist.github.com/dirk-thomas/83ae6bfbfc94e06cd519) using ament
- [CMakeLists.txt](https://gist.github.com/dirk-thomas/a76f952d05e7b21b0128) using ament_auto
