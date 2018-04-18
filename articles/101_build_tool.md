---
layout: default
title: A universal build tool
permalink: articles/build_tool.html
abstract:
  This article describes a universal build tool for ROS 1 and ROS 2.
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

In the ROS ecosystem the software is separated into numerous packages.
It is very common that a developer is working on multiple packages at the same time.
This is in contrast to workflows where a developer only works on a single software package at a time, and all dependencies are provided once but not being iterated on.

The "manual" approach to build a set of packages consists of building all packages in their topological order one by one.
For each package the documentation usually describes what the dependencies are, how to setup the environment to build the package as well as how to setup the environment afterwards to use the package.
Such a workflow is impracticable at scale without a tool that automates that process.

A build tool performs the task of building a set of packages with a single invocation.
For ROS 1 multiple different tools provide support for this, namely `catkin_make`, `catkin_make_isolated`, and `catkin_tools`.
For ROS 2 up to the Ardent release the build tool providing this functionality is called `ament_tools`.

This article describes the steps to unify these build tools as well as extend the field of application.

## Goal

The goal of a unified build tool is to build a set of packages with a single invocation.
It should work with ROS 1 packages as well as ROS 2 packages which provide the necessary information in their manifest files.
It should also work with packages that do not provide manifest files themselves, given that the necessary meta information is provided externally.
This will allow the build tool to be utilized for non-ROS packages (e.g. Gazebo including its ignition dependencies).

In the ROS ecosystems several tools already exist which support this use case (see below).
Each of the existing tools performs similar tasks and duplicates a significant amount of the logic.
As a consequence of being developed separately certain features are only available in some of the tools while other tools lack those.

The reason to use a single universal build tool comes down to reducing the effort necessary for development and maintenance.
Additionally this makes newly developed features available for all the use cases.

### Build Tool vs. Build System

Since this article focuses on the build tool the distinction to a build system needs to be clarified.

A build tool operates on a set of packages.
It determines the dependency graph and invokes the specific build system for each package in topological order.
The build tool itself should know as little as possible about the build system used for a specific package.
Just enough in order to know how to setup the environment for it, invoke the build, and setup the environment to use the built package.
The existing ROS build tools are: `catkin_make`, `catkin_make_isolated`, `catkin_tools`, and `ament_tools`.

The build system on the other hand operates on a single package.
Examples are `Make`, `CMake`, `Python setuptools`, or `Autotools` (which isn't used in ROS atm).
A CMake package is e.g. build by invoking these steps: `cmake`, `make`, `make install`.

`catkin` as well as `ament_cmake` are based on CMake and offer some convenience functions described below.

### Environment Setup

A very important part beside the actual build of a package is the environment setup.
For example, in order for a CMake project to discover a dependency using the CMake function `find_package`, the CMake module (e.g. `FindFoo.cmake`) or the CMake config file (e.g. `FooConfig.cmake`) for that dependency must either be in a prefix that CMake searches implicitly (e.g. `/usr`) or the location must be provided through the environment variable `CMAKE_PREFIX_PATH` / `CMAKE_MODULE_PATH`.

In addition to building a package on top of another package (using `find_package` in the case of CMake), you may need to adjust the environment in order to run an executable from a package.
For example, when a package installs a shared library in a non-default location then the environment variable `LD_LIBRARY_PATH` (or `PATH` on Windows) needs to be extended to include the containing folder before trying to run executables that load that library at runtime.

The functionality to setup these environment variables can be provided by either the build tool or the build system.
In the latter case the build tool only needs to know how the build system exposes the environment setup in order to reuse it.

Considering the use case that a user might want to invoke the build system of each package manually it is beneficial if the build system already provides as much of the environment setup as possible.
That avoids forcing the user to manually take care of the environment setup when not using a build tool.

### Out of Scope

To clarify the scope of this article a few related topics are explicitly enumerated even though they are not being considered.

#### Build System

Any build system related functionality (which is not directly relevant for the build tool) is not considered in this article.

##### Mixing Different Build Systems

The unified build tool will support different build systems in order to satisfy the described goals.
If packages using different build system inter-operate with each other correctly depends also to a large degree on the build system.
While the build tool should ensure that it doesn't prevent that use case this article will not cover the use case of mixing multiple build systems in a single workspace (e.g. ROS 1 packages using `catkin` with ROS 2 packages using `ament_cmake`).

#### Fetch Source Code

The build tool does not cover the steps necessary to fetch the sources of the to-be-built packages.
There are already tools to help with this.
For example, the list of repositories that need to be fetched is provided either by a hand crafted `.rosinstall` or `.repos` file or by using [rosinstall_generator](http://wiki.ros.org/rosinstall_generator) to generate one.
The list of repositories can then be fetched with one of several tools, like [rosinstall](http://wiki.ros.org/rosinstall) or [wstool](http://wiki.ros.org/wstool) in the case of a `.rosinstall` file, or [vcstool](https://github.com/dirk-thomas/vcstool) in the case of a `.repos` file.

#### Install Dependencies

The build tool also does not provide a mechanism to install any dependencies required to build the packages.
In the ROS ecosystem [rosdep](http://wiki.ros.org/rosdep) can be used for this.

#### Create Binary Packages

The build tool also does not create binary packages (e.g. a Debian package).
In the ROS ecosystem [bloom](http://wiki.ros.org/bloom) is used to generate the required metadata and then platform dependent tools like `dpkg-buildpackage` build binary packages.

## Existing Build Systems

In the following the build systems being used in the ROS ecosystem are briefly described.

### CMake

[CMake](https://cmake.org/) is a cross-platform build system generator.
Projects specify their build process with platform-independent `CMakeLists.txt` files.
Users build a project by using CMake to generate a build system for a native tool on their platform, e.g. `Makefiles` or `Visual Studio projects`.

### catkin

[catkin](http://wiki.ros.org/catkin) is based on CMake and provides a set of convenience functions to make writing CMake packages easier.
It automates the generation of CMake config files as well as pkg-config files.
It additionally provides functions to register different kinds of tests.

A package using `catkin` specifies its meta data in a manifest file named `package.xml`.
The latest format of the manifest file is specified in the [ROS REP 149](http://www.ros.org/reps/rep-0149.html).

### ament_cmake

[ament_cmake](https://github.com/ament/ament_cmake) is an evolution of `catkin` and is also based on CMake.
The main difference between `ament_cmake` and `catkin` is described in [another article](http://design.ros2.org/articles/ament.html).
In the context of the build tool the biggest difference is that `ament_cmake` generates package-specific files to setup the environment to use the package after it has been built and installed.

A package using `ament_cmake` uses the same manifest file as `catkin` (except that it requires format version 2 or higher).

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

`catkin_make` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.

### catkin_make_isolated

`catkin_make_isolated` is provided by the ROS package `catkin` which contains the build system for ROS 1.
It was developed after `catkin_make` to address the problems involved with building multiple packages in a single CMake context.

The tool only supports CMake-based packages and builds each package in topological order using the command sequence common for CMake packages: `cmake`, `make`, `make install`.
While each package can parallelize the build of its targets the packages are processed sequentially even if they are not (recursive) dependencies of each other.

`catkin_make_isolated` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.
- Plain CMake packages with a `package.xml` file.

### catkin_tools

[catkin_tools](https://catkin-tools.readthedocs.io/) is provided by a standalone Python package used to build ROS 1 packages.
It was developed after `catkin_make` / `catkin_make_isolated` to build packages in parallel as well as provide significant usability improvements.
The tool supports building CMake packages and builds them in isolation as well as supports parallelizing the process across packages.

`catkin_tools` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.
- Plain CMake packages with a `package.xml` file.

### ament_tools

`ament_tools` is provided by a standalone Python 3 package used to build ROS 2 packages.
It was developed to bootstrap the ROS 2 project, is therefore only targeting Python 3, and works on Linux, MacOS and Windows.
In addition to CMake packages it also supports building Python packages and can infer meta information without requiring an explicit package manifest (which is e.g. used for the FastRTPS package).
The tool performs an "isolated" build like `catkin_make_isolated` and `catkin_tools` (one CMake invocation per package) and also parallelizes the build of packages which have no (recursive) dependencies on each other (like `catkin_tools`).
While it covers more build systems and platforms than `catkin_tools` it doesn't have any of `catkin_tools`s usability features like profiles, output handling, etc.

`ament_tools` supports building the following packages:

- ROS 2 `ament_cmake` package with a `package.xml` file (only format 2).
- Plain CMake package with a `package.xml` file.
- Plain CMake package without a manifest file (extracting the package name and dependencies from the CMake files).
- Python package with a `package.xml` file.
- Python package without a manifest file (extracting the package name and dependencies from the `setup.py` file).

### colcon

When the first draft of this article was written the conclusion was to not to spend any resources towards a universal build tool.
As a consequence the author of this article went ahead and developed [colcon](https://github.com/colcon/) as a personal project.
Therefore its feature set is closely aligned with the following requirements.

## Naming

The existing build tools in ROS are all named by the build system they are supporting.
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

In ROS 2 the concept of the *devel space* has intentionally been removed.
In the future it might be feasible to provide the concept of *symlinked installs* in ROS 1 to provide a similar benefit without the downsides.
However, this design document will assume that the *devel space* will remain in ROS 1.

#### Build ROS 2 workspaces

The tool needs to be able to build ROS 2 workspaces which can already be built using `ament_tools`.

#### Build Gazebo including dependencies

After cloning the repositories containing Gazebo and all its dependencies (excluding system packages) the tool needs to be able to build the set of packages.
Meta information not inferable from the sources can be provided externally without adding or modifying any files in the workspace.
After the build a single file can be sourced / invoked to setup the environment to use Gazebo (e.g. `GAZEBO_MODEL_PATH`).

### Development Environment Setup

Invoking a build system for a package implies also setting up environment variables before the process, e.g. the `CMAKE_PREFIX_PATH`.
It should be possible for developers to manually invoke the build system for one package.
The environment variable might be partially different from the environment variables necessary to use a package after it has been built.
To make that convenient the tool should provide an easy to use mechanism to setup the development environment necessary to manually invoke the build system.

### Beyond Building

Building packages is only one task the build tool can perform on a set of packages.
Additional tasks like e.g. running tests should also be covered by the build tool.
The build tool must provide these abstract tasks and then map them to the steps necessary for each supported build system.

### Software Criteria

The tool aims to support a variety of build systems, use cases, and platforms.
The above mentioned ones are mainly driven by the needs in the ROS ecosystem but the tool should also be usable outside the ROS ecosystem (e.g. for Gazebo).
Therefore it should be designed in a way which enables extending its functionality.

Assuming that the tool will be implemented in Python (since that is the case for all existing ROS build tools) the entry point mechanism provides a convenient way to make the software extensible.
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

When the first draft of this article was written neither of the existing build tools supported the superset of features described in this article.
There were multiple different paths possible to reach the goal of a universal build tool which fall into two categories:

- One approach is to incrementally evolve one of the existing tools to satisfy the described goals.
- Another approach would be to start "from scratch".

Since then the new project `colcon` has been developed which covers most of the enumerated requirements and represents the second category.

### Evolve catkin_make, catkin_make_isolated, or ament_tools

Since neither of these three build tools has the feature richness of `catkin_tools` it is considered strictly less useful to starting building upon one of these build tools.
Therefore neither of these are being considered as a foundation for a universal build tool.

### Evolve catkin_tools

Since `catkin_tools` is in many aspects the most complete ROS build tool it should be the one being evolved.
While `ament_tools` has a few features `catkin_tools` currently lacks (e.g. plain CMake support without a manifest, Windows support) the feature richness of `catkin_tools` makes it a better starting point.

### Start "from scratch" / colcon

Since the first draft of this article the `colcon` project has been developed with the goals and requirements of a universal build tool in mind.
In its current form it is already able to build ROS 1 workspaces, ROS 2 workspaces, as well as Gazebo including its ignition dependencies.
It uses Python 3.5+ and targets all platforms supported by ROS: Linux, macOS, and Windows.

Since it hasn't been used by many people yet more advanced features like cross compilation, `DESTDIR`, etc. hasn't been tested (and will therefore likely not work yet).

## Decision process

For the decision process only the following two options are being considering based on the rational described above:

A.  Use `catkin_tools` as a starting point
B.  Use `colcon` as a starting point

If this topic would have been addressed earlier some of the duplicate effort could have likely been avoided.
When the work towards a universal build tool was suspended over a year ago it was a conscious decision based on available resources.
Nevertheless move forward with a decision now will at least avoid further uncertainty and duplication.

Both of the considered options have unique and valuable features and there are good arguments to build our future development on either of the two tools.
Since both are written in Python either of the two tools could be "transformed" to cover the pros of the other one.
So the two important criteria for the decision are:

- the effort it takes to do (in the short term as well as in the long term) and
- the difference of the resulting code base after the "transformation" is completed.

### Immediate goals

A ROS 2 developer currently builds a steadily growing workspace with ROS 2 packages.
The same is happening in the monolithic Jenkins jobs on [ci.ros2.org](https://ci.ros2.org) (with the advantage to test changes across repositories easily).
Therefore features to easily filter the packages which need to be build are eagerly awaited to improve the development process.

For the last ROS 2 release *Ardent* the buildfarm [build.ros.org](http://build.ros2.org) only provides jobs to generate Debian packages.
Neither *devel* jobs or *pull request* jobs are available nor is it supported to build a local *prerelease*.
For the coming ROS 2 release *Bouncy* these job types should be available to support maintainers.

In ROS 2 *Bouncy* the universal build tool will be the only supported option and `ament_tools` will be archived.

#### Necessary work

For either option **A)** or **B)** the follow items would need to be addressed:

- The jobs and scripts on *ci.ros2.org* need to be updated to invoke the universal build tool instead of `ament_tools`.
- The `ros_buildfarm` package needs to be updated to invoke the universal build tool instead of `catkin_make_isolated`.
  The ROS 2 buildfarm would use this modification for the upcoming ROS 2 *Bouncy* release.
  The ROS 1 buildfarm could use the same modification in the future.

For option **A)** the follow items would need to be addressed:

- Support for setup files generated by `ament_cmake`.
- Support additional packages types: plain Python packages, CMake packages without a manifest.
- Support for Windows using `.bat` files.
- Support for the package manifest format version 3.

For option **B)** the follow items would need to be addressed:

- Address user feedback when the tool is being used by a broader audience.

### Future

The long term goal is that the universal build tool will be used in ROS 1, in ROS 2 as well as other non-ROS projects.
There is currently no time line when the tool will be used on the ROS 1 build or be recommended to ROS 1 users.
This solely depends on the resources available for ROS 1.

Beside that for both options there is follow up work beyond the immediate goals.
The following enumerates a few of them but is by no means exhaustive:

For option **A)** the follow items should be considered:

- Support for Python packages using a `setup.cfg` file.
- Support for `PowerShell` to work around length limitations for environment variable on Windows.
- Support to pass package specific argument.
- Remove support for the *devel space* concept in ROS 1.
- Update code base to Python 3.5+.
- Refactor code base to reduce coupling (e.g. separate [API](https://github.com/catkin/catkin_tools/blob/2cae17f8f32b0193384d2c7734afee1c60c4add2/catkin_tools/execution/controllers.py#L183-L205) for output handling).
- Additional functionality to build Gazebo including its dependencies.
- Whether or not to continue supporting the *devel space*.

For option **B)** the follow items should be considered:

- Support cross compilation.
- Support `DESTDIR`.
- Support a feature similar to the `profile` verb of `catkin_tools`.
- Support for a shared GNU Make job sever.
- Support for `GNUInstallDirs`
  - Not sure about the status of this, it would be in `colcon`'s generated shell files if anywhere.
  - Should have a test for this case.
- Test for, and fix if necessary, correct topological order with dependencies across workspaces.
  - See: [https://github.com/ros/catkin/pull/590](https://github.com/ros/catkin/pull/590)

## Summary and Decision

Based on the above information a decision has been made to pick `colcon` as the universal build tool.

The decision was made after considering the input of ROS 2 team members and some ROS 1 users.
The decision was not easy, as it was not unanimous, but the vast majority of input was either pro `colcon` or ambivalent.

To elaborate on the rational one significant advantage of `colcon` is that it is ready to be deployed for ROS 2 right now and it covers our current use cases.
Another argument leaning towards `colcon` is the expected little effort to provide devel / PR / prerelease jobs on build.ros2.org across all targeted platforms for the upcoming *Bouncy* release.
While some additional feature and usability options are still missing they can be added in the future whenever there is time and/or demand for them.

The necessary up front development effort for `catkin_tools` to achieve the goals described for  *Bouncy* would distract the ROS 2 team from spending their time on feature development and bug fixing of ROS 2 itself.

While the short term advantages are certainly a main reason for the decision in favor of `colcon` they are not the only ones.
The cleaner architecture, modularity and extensibility as well as Python 3.5 code base will be valuable long term benefits when developing this tool in the future.
The separation of the build tool name from the supported build systems as well as the separation from being a "ROS-only" tool will hopefully also help users to understand the difference and attract new users and potential contributors.

### Next steps

The following next steps will happen before the next ROS 2 release *Bouncy*.

- The instructions in the ROS 2 wiki to build from source will be updated to use `colcon` instead.
- The `ament_tools` repository will be archive, removed from the `ros2.repos` file, and won't be released into *Bouncy*.
- The ROS 2 buildfarm(s) will be updated to use `colcon` and provide devel / PR / prerelease jobs for the *Bouncy* release.

### Implications

The following items briefly enumerate what This means for ROS developers and users:

- **No CMake code** of any ROS 2 (or ROS 1) package **needs to be changed** for this transition.
- When building and testing ROS 2 the command `colcon build` / `colcon test` will be used instead of `ament build` / `ament test`.
  Please see the [documentation](http://colcon.readthedocs.io/en/latest/migration/ament_tools.html) how to map `ament_tools` command line arguments to `colcon` arguments.
- For ROS 1 nothing is changing at this point in time.
- In the future `colcon` will replace `catkin_make_isolated` and `catkin_make` as the recommended build tool for ROS 1.
  - `colcon` will not support the *devel space* and will require packages to have install rules
  - `catkin` will likely still support the *devel space*, though it might be removed at some point (that has not been decided yet)
  - Therefore, it is possible that the default build tool for ROS 1 may not support the *devel space*, though legacy tools will continue to support it.
  - Note that it is already the case that individual ROS 1 catkin packages may either not have installation rules but support the *devel space*, or they might have installation rules but not properly support the *devel space*.

### Outlook

- Since `colcon` can be used to build ROS 1 early adopters can try to use it to build ROS 1 from source.
  While there is documentation how to migrate from [catkin_make_isolated](http://colcon.readthedocs.io/en/latest/migration/catkin_make_isolated.html) and [catkin_tools](http://colcon.readthedocs.io/en/latest/migration/catkin_tools.html) `colcon` won't be the recommended build tool in ROS 1 for the foreseeable future.
- If a test buildfarm using `colcon` proofs to deliver the exact same results as the ROS 1 buildfarm using `catkin_make_isolated` it might be changed to use `colcon` in the future to benefit from the features `colcon` provides (like non-interleaved output per package when building in parallel, per package log files, etc.).
