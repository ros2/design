---
layout: default
title: Cross-compiling ROS 2 packages
abstract:
  This article is a design proposal for extending the build tools for ROS 2 to
  simplify cross-compiling ROS 2 packages for platforms currently not supported
  by ROS 2, or building on a host with different platforms than the target
  platform. That latter use case is particularly useful for speeding up build
  times by building the code on a powerful development workstation, and then
  deploying the binaries to a constrained resources hardware that uses the
  target platform, thus making the development cycle much faster.
author: >
  [Thomas Moulard](https://github.com/thomas-moulard),
  [Juan Rodriguez Hortala](https://github.com/juanrh)
published: false
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

The [cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation),
and the [steve/ros2_raspbian_tools repository](https://github.com/esteve/ros2_raspbian_tools)
contain instructions for cross-compiling ROS 2 to unsupported platforms like
ARM64 and ARM-HF, using some configuration files for CMake and Docker. However,
those instructions don't include how to build other ROS 2 packages, but just
for building a whole ROS 2 distribution. Ideally, we should be able to
cross-compile a ROS package for ROS 2 by launching a single `colcon build`
command. Finally, even though other projects like [steve/ros2_raspbian_tools](https://github.com/esteve/ros2_raspbian_tools),
[esteve/ros2_java](https://github.com/esteve/ros2_java) provide support
for additional platforms like Android and iOS, we would like to have an extensible
design that allows supporting new platforms with low effort.

## Proposed Approach

We propose continuing with the general approach of the cross-compilation
tutorial, based on building a sysroot for the target platform using QEMU and
Docker, and then using `cmake-toolchains` with a toolchain file pointing
`CMAKE_SYSROOT` to the sysroot location to compile with C and C++
cross-compilers.

On top of that we propose using a **colcon plugin `cc-build`** to wrap the
instructions on the cross-compilation tutorial into a single build command.
That plugin would define a new `colcon_core.verb` entry point pointing to a
class `CCBuildVerb` that uses the [Decorator pattern](https://en.wikipedia.org/wiki/Decorator_pattern)
to wrap the [`BuildVerb` class](https://github.com/colcon/colcon-core/blob/master/colcon_core/verb/build.py#L74)
used by `colcon build` with additional functionality to handle the sysroot and
toolchain file, and to inject additional `cmake-args` arguments. That would
allows us to run the single command below from a ROS 2 overlay workspace on a
developer workstation, for cross-compiling the packages in the workspace up to
a package `performance_test` for the ARM-HF platform.

```bash
colcon cc-build --arch armhf --os ubuntu_bionic \
  --packages-up-to performance_test # other `colcon build` arguments
```

That would should result on a new workspace directory `armhf-ubuntu_bionic`
with subdirectories `build`, `install`, and `log` directories, with the
ross-compiled binaries. All options for `colcon build` will be supported by
forwarding them to the `BuildVerb` class. That would be the only command
required for cross-compiling the packages in a workspace. Under the hood, the
plugin would do the following:

- The [development tools described in the cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation/#install-development-tools)
  (`qemu-user-static`, `g++-aarch64-linux-gnu`, ...) should be installed in the
  development workstation used for the build. The `cc-build` plugin will
  declare those system dependencies using a `stdeb.cfg` file like is done on
  [other colcon plugins](https://github.com/colcon/colcon-core/blob/master/stdeb.cfg).
- In order to build the sysroot, the plugin downloads a base ROS 2 Docker image
  for the target platform, and builds a workspace dependent sysroot image by
  doing a `COPY` to copy the contents of the workspace into the container, and
  running `rosdep` on the copy. That ensures no workspace system dependency is
  missing. Note we we can run `rosdep` in Docker for the ARM-HF platform thanks
  to [QEMU](https://www.qemu.org/). That image's file system is then exported by
  launching a container a running `docker container export` on it, to a path
  following a sensible convention, like a workspace-local `armhf-ubuntu_bionic/sysroot`
  directory, so it can be used as a sysroot during the build.
  - Those ROS 2 base images are variants of
    [sysroot/Dockerfile_ubuntu_arm](https://github.com/ros2/cross_compile/blob/master/sysroot/Dockerfile_ubuntu_arm)
  from ros2/cross_compile.
  - OSRF would publish those base ROS 2 images, with ROS 2 pre built from
    source. OSRF would also publish other base ROS 2 images that install
    `ros-$ROS_DISTRO-desktop` with Apt, instead of installing the system
    dependencies for ROS 2 with `rosdep install`.
- The plugin then sets the environment variable `SYSROOT` to the sysroot path,
  and other env vars required to configure the toolchain file [generic_linux.cmake](https://github.com/ros2/cross_compile/blob/master/cmake-toolchains/generic_linux.cmake)
  from ros2/cross_compile, and launches an instance of the `BuildVerb` class
  configured with `-DCMAKE_TOOLCHAIN_FILE` in `cmake-args` pointing to that
  toolchain file, to trigger a `colcon build`.

The `colcon cc-build` plugin will also support the following optional arguments:

- `--sysroot-base-image`: Specifies a Docker image that will be used as the
  base image for the workspace dependent sysroot image. This should be a string
  that is a valid argument for `docker image pull`, and that can be used in
  `FROM` Dockerfile statement. If it is not specified, the plugin will deduce
  the value of `--sysroot-base-image` from the `--arch` and `--os` arguments,
  for the architecture-OS combinations that have an image published for them on
  [osrf/ros2 on Docker Hub](https://hub.docker.com/r/osrf/ros2/).
- `--sysroot-image`: Specifies a Docker image for the workspace sysroot. The
  build will export the sysroot from that image, instead of creating a sysroot
  image from the workspace. No check is performed to ensure the image has all the
  dependencies required by the packages in the workspace.
- `--force-sysroot-build`: Boolean flag to force exporting the sysroot even if
  it already available in the workspace. This is useful for example when new
  system dependencies are added to a package in the workspace. This leads to
  rebuilding the dependent sysroot image after the `COPY` statements that copies
  the workspace into the image, including running `rosdep install` from scratch.
  This does not rebuild the base ROS 2 Docker image.
- `--sysroot-path`: Specifies a local path for an existing sysroot. This can be
  useful for example if different workspaces share the same set of system
  dependencies.
- `--use-base-image-sysroot`: Boolean flag, when true the base ROS 2 Docker
  image is used to build the sysroot, so the dependencies declared in packages in
  the current workspace are ignored. This is useful to speed up builds for
  packages without additional dependencies.

Support for other platforms can be added by creating corresponding ROS 2 Docker
images, and variants of the toolchain file if required.
We could include support for ARM-HF and ARM64 in the first release of the
plugin.

### Frequently asked questions

- _Are we building the sysroot for the workspace, and running `rosdep install`
  on each invocation of `colcon cc-build`?_ No, the Docker file system export is
  only performed once, if the sysroot is not already available. The argument
  `--force-sysroot-build` can be used to force building the sysroot if a sysroot
  file is already available, for example when new dependencies are added to some
  package in the workspace.
- _How can we run the resulting binaries?_ Packaging the resulting binaries is
  outside of the scope of this document. The cross-compiled binaries will be
  available in the workspace is a directory corresponding to the architecture-OS
  pair, e.g. `armhf-ubuntu_bionic`. We make the assumption that the user will
  copy the whole workspace to the remote device (including src/) and run `rosdep`
  on the target platform, to install the dependencies before running the binaries.

### Maintenance and testing

Regarding maintenance, in order to support cross-compilation for a new platform
we would need to:

- Prepare and publish a base ROS 2 Docker image for the new platform, both
  fetching the dependencies using the ROS 2 source using `rosdep install`, and
  from the pre-built binary if available for that platform.
- Adapt the toolchain file for the new platform, if required. For that the
  toolchains in [ruslo/polly](https://github.com/ruslo/polly) might be useful.
- Extend the `cc-build` plugin to include dependencies for the compiler for
  the new platform, and any modifications needed to use a suitable toolchain
  file for the platform.

We could then test cross-compile support for the new platform by building the
base  ROS 2 Docker image, and a custom sysroot image for some reference package
(e.g. [ApexAI/performance_test](https://github.com/ApexAI/performance_test/)),
and then launching a Docker container for the that sysroot image, running
`colcon test` using the cross-compiled binaries. The container would be
launched with a [bind mount](https://docs.docker.com/storage/bind-mounts/) of the
workspace so they have access to the compiled binaries.

In order to simplify the development of base ROS 2 Docker images for new
architecture-OS combinations, on a second iteration of this plugin we might add
a new verb `colcon cc-build-sysroot-base-image` that will 1) copy the required
QEMU files and ROS 2 source into a temporary file; 2) build a Docker image from
a specified Dockerfile, with access to the those resources; 3) optionally
publish the image into a Docker registry, using a suitable naming convention like
`${docker_repo_uri}:${os}_${arch}_${prebuilt?}_${rosdistro}`. Dockerfiles
compatible with this command, for each architecture-OS pair would be keep on the
`ros2/cross_compile` repository, so they can be used in CI/CI pipeline if needed.

### Advantages of the approach

The proposed design leads to a simple development experience, where cross
compilation is triggered by a one-liner command.
The design is extensible and new platforms can be supported easily.

For simple packages with additional dependencies, there is no need to build the
sysroot locally with Docker, and a readily available base Docker image can be
used. For packages with custom dependencies, the base sysroot can be extended
using `rosdep install` on a Docker image that uses QEMU through [`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc).

## Limitations and open questions

This setup is focused on cross-compiling for Linux based platforms. For example
we couldn't use this to cross-compile for Windows, because we cannot install
the Visual Studio compiler using Apt.
On the other hand, a Windows or Mac development workstation could use this
build setup, running the build commands inside a Docker container, although
that would imply some performance degradation due to the virtualization layer
that Docker employs on those platforms.

We should determine a process for building an publishing the base ROS 2 Docker
images for each platform. Ideally a nightly job would publish an image built
from source for the tip of master, but that might be not feasible depending on
the available resources.

## Alternative approaches

The [colcon bundle plugin](https://github.com/colcon/colcon-bundle) can be used
to package a ROS workspace together with all its dependencies into a tarball.
That could be used for cross-compiling a ROS 2 workspace by launching a Docker
container for the target platform, building and bundling the workspace, and
exporting the bundle, which can then be the deployed to the target hardware.
The main drawback of this approach is that using the default ARM compiler in
the Docker image significantly slows down the compilation, which might be a
burden for day-to-day development. By installing `qemu-user-static` in the
present proposal, we are not only installing the QEMU binaries to simulate an
ARM32/64 processor, but also registering [`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc)
hook in the local machine. That means that every time a program will start, if
it is for a platform that QEMU supports, the binary will automatically run
through QEMU. In practice, it lets you run ARM binaries on your X86 instance,
as it is done implicitly through Docker when building the base image. The
problem is that QEMU is ~10x slower than normal execution, but the present
proposal limits the QEMU usage to building the base image (which is
unavoidable), and uses compilers for the target platform running natively the
rest of the time.

The colcon bundle approach would require adding support for ROS 2 to colcon
bundle, that currently doesn't support Ament. That would provide a way to
distribute cross-compiled system dependencies, which is convenient because
`rosdep install` can take a lot of time on resource constrained hardware, and
requires an Internet connection. The presented approach and colcon bundle are
complementary approaches, and if colcon bundle eventually supports ROS 2, we
could use `colcon cc-build` for easy cross-compilation minimizing the usage
of Docker, followed by colcon bundle to generate a deployable artifact.

Regarding the workspace dependent sysroot image, we could avoid the Docker
`COPY` of the workspace by not building a workspace sysroot image, and instead
launching a container for the base ROS 2 image with a [bind mount](https://docs.docker.com/storage/bind-mounts/)
for the workspace. The container's entry point would launch `rosdep` on the
directory where the workspace is mounted, and we'd export the container's file
system is then exported with  `docker container export` as above. That avoids
a `COPY`, but a disadvantage is that then we wouldn't have a sysroot image for
our workspace, and we would have to run `rosdep` on the entry point of that
image each time we launch a container. That image would be useful for running
tests, as mentioned in the testing section above, and an eventual
`colcon cc-test` plugin could further simplify using that image.

Finally, compared with the original approach in the [cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation),
the proposed colcon plugin method basically organizes the commands that appear
in the tutorial, and that are used in [cross_compile/entry_point.sh](https://github.com/ros2/cross_compile/blob/master/entry_point.sh),
to allow cross-compiling a workspace with a one-liner command. So this is more
an evolution than an alternative proposal.
