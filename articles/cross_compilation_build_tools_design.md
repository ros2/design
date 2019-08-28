---
layout: default
title: Cross-compiling ROS 2 packages
abstract:
  This article is a design proposal for developing a ROS 2 tool that sets up and manages sysroot environments for cross-compilation with the objective of being simple and extensible.
  Extending support for new cross compilation configurations using colcon mixins is also proposed.
author: >
  [Thomas Moulard](https://github.com/thomas-moulard),
  [Juan Rodriguez Hortala](https://github.com/juanrh),
  [Anas Abou Allaban](https://github.com/piraka9011)
published: false
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

The [cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation) and the [esteve/ros2_raspbian_tools repository](https://github.com/esteve/ros2_raspbian_tools) contain instructions for cross-compiling ROS 2 to unsupported architectures, like ARM64 and ARM-HF, using toolchain files for CMake and Docker images.
Other projects like [esteve/ros2_objc](https://github.com/esteve/ros2_objc) and [esteve/ros2_java](https://github.com/esteve/ros2_java) provide support for additional platforms like Android and iOS.
Ideally, we should be able to cross-compile a ROS package for ROS 2 by launching a single command.
That command should also be implemented using an extensible design that allows supporting new platforms with low effort.
This design proposal is a push towards Tier-1 support of ARM-HF and ARM64 architectures.

## Proposed Approach

We propose continuing with the general approach of the cross-compilation tutorial.
This involves building a sysroot for the target platform, using QEMU and Docker, and then using [colcon mixins](https://github.com/colcon/colcon-mixin) to compile with C and C++ cross-compilers.

We propose adding a set of commands to wrap the instructions in the cross-compilation tutorial.
These commands would use a new workspace directory for the target platform, determined by a convention based on the:
- Target platform
- Operating system
- RMW implementation
- ROS distribution

Let us call this workspace directory the _cc-root_.
Under this convention, the platform implies the architecture, but generic platforms like `generic_armhf` would also be available.
An example _platform identifier_ would be `generic_armhf-ubuntu_bionic-fastrtps-crystal` or `turtlebot_armhf-ubuntu_bionic-fastrtps-dashing`.
To implement the proposal, we would add the following commands:

- A new `create-cc-sysroot` command that would use Docker to generate the sysroot for the target platform and store it in a `sysroot` subdirectory of the _cc-root_.
  This command would take arguments to specify the target platform, for example:

```bash
create-cc-sysroot --arch generic_armhf --os ubuntu_bionic \
                  --rmw fastrtps --rosdistro crystal
```

- A new colcon mixin for each known platform, which adds options to the colcon build task for using a sysroot generated with `create-cc-sysroot`, by using the same path conventions.
  For example, from a ROS 2 overlay workspace on a developer workstation, the following command would cross-compile the packages in the workspace up to a package `performance_test` for the platform specified.
  This would create `build`, `install`, and `log` subdirectories of the cc-root `generic_armhf-ubuntu_bionic-fastrtps-crystal` with the cross-compiled binaries.

```bash
colcon build --mixin cc-generic_armhf-ubuntu_bionic-fastrtps-crystal \
  --packages-up-to performance_test # any other other `colcon build` arguments
```

- On a second iteration of this initiative we would:
  - Maintain a docker image for cross compilation, that has both the cross compilation toolchain and the aforementioned colcon mixins installed.
  - Implement a new `cc-build` command that would ensure the sysroot is created using `create-cc-sysroot`, and then launch the cross compilation into a docker container using the corresponding Docker image.

Under the hood, the `create-cc-sysroot` command would do the following:

1. Downloads a base ROS 2 Docker image for the target platform.
1. Builds a workspace dependent sysroot image by using a Dockerfile that starts from that base image and:
   1. Runs `COPY` to get the contents of the workspace into the container.
   1. Uses `rosdep` to ensure no workspace system dependency is missing.
      Note we can run `rosdep` in Docker for architectures like ARM-HF thanks to [QEMU](https://www.qemu.org/).
1. Launches a container for that image and exports its file system into a `sysroot` subdirectory of the cc-root.
1. During a build, the colcon mixin sets the appropriate CMake arguments for cross compilation and/or point to a toolchain file (ex. [generic_linux.cmake](https://github.com/ros2/cross_compile/blob/master/cmake-toolchains/generic_linux.cmake)) using the `CMAKE_TOOLCHAIN_FILE` argument.

The ROS 2 base images are variants of [sysroot/Dockerfile_ubuntu_arm](https://github.com/ros2/cross_compile/blob/master/sysroot/Dockerfile_ubuntu_arm) from ros2/cross_compile which use the [Docker Official Images](https://hub.docker.com/_/ros).
OSRF would publish base ROS 2 images with ROS 2 prebuilt, that should be used for cross compiling workspaces with user packages.
OSRF would also publish other base ROS 2 images with the system setup and basic tools for building ROS 2 preinstalled, that should be used for building ROS 2 from source.

The `create-cc-sysroot` command will also support the following optional arguments:

- `--sysroot-base-image`: Specifies a Docker image that will be used as the base image for the workspace dependent sysroot image.
  This should be a string that is a valid argument for `docker image pull` and that can be used in the `FROM` Dockerfile statement.
  If it is not specified, the command will deduce the value of `--sysroot-base-image` from the target platform and pull from a published image on [Docker's Official ROS Images](https://hub.docker.com/_/ros), the [arm64v8 Dockerhub repo](https://hub.docker.com/u/arm64v8) or from the [arm32v7 Dockerhub repo](https://hub.docker.com/u/arm32v7).
- `--sysroot-image`: Specifies a Docker image for the workspace sysroot.
  The build will export the sysroot from that image, instead of creating a sysroot image from the workspace.
  No check is performed to ensure the image has all the dependencies required by the packages in the workspace.
- `--use-base-image-sysroot`: Boolean flag, when true the base ROS 2 Docker image is used to build the sysroot, so the dependencies declared in packages in the current workspace are ignored.
  This is useful to speed up builds for packages without additional dependencies.

Support for other platforms can be added by creating corresponding ROS 2 Docker images, and variants of the toolchain file if required.
We could include support for generic versions of ARM-HF and ARM64, on Ubuntu Bionic with ROS 2 crystal and fastrtps, in the first release of the commands.

### Frequently asked questions

- _How can we run the resulting binaries?_
  Packaging the resulting binaries is outside of the scope of this document.
  The cross-compiled binaries will be available in the workspace in the cc-root (e.g. `generic_armhf-ubuntu_bionic-fastrtps-crystal`).
  We make the assumption that the user will copy the whole workspace to the remote device (including `src`) and run `rosdep` on the target platform, to install the dependencies before running the binaries.

### Maintenance and testing

Regarding maintenance, in order to support cross-compilation for a new platform
we would need to:

- Prepare and publish a base ROS 2 Docker image for the new platform, both fetching the dependencies using the ROS 2 source using `rosdep install`, and from the pre-built binary if available for that platform.
- Adapt the toolchain file for the new platform, if required.
  For that the toolchains in [ruslo/polly](https://github.com/ruslo/polly) might be useful.
- Add a new colcon mixin to the `cross-compile` repository for the new platform and perform any modifications needed to use a suitable toolchain file.

We can then test cross-compile support for the new platform by:

1. Building the base ROS 2 Docker image.
1. Using `create-cc-sysroot` to build a custom sysroot image for some reference package, e.g. [ApexAI/performance_test](https://github.com/ApexAI/performance_test/), and export its file system into a sysroot.
1. Cross-compiling that package using the sysroot.
1. Launching a Docker container for that sysroot image and running `colcon test` using the cross-compiled binaries.
   The container would be launched with a [bind mount](https://docs.docker.com/storage/bind-mounts/) of the workspace so they have access to the compiled binaries.

In order to simplify the development of base ROS 2 Docker images for new architecture-OS combinations, on a future iteration of this initiative we might add a new command `build-sysroot-base-image` that would:

1. Copy the required QEMU files and ROS 2 source files into a temporary location.
1. Build a Docker image from a specified Dockerfile, with access to the those resources.
1. Optionally publish the image into a Docker registry, using a suitable naming convention that corresponds to the platform identifier.

Dockerfiles compatible with that command would be kept on the `ros2/cross_compile` repository, so they can be used in CI/CD pipelines if needed.

### Advantages of the approach

The proposed design leads to a simple development experience, where cross compilation is triggered by two simple commands, or a single `cc-build` command for the second iteration.
The design is extensible and new platforms can be supported easily.

For simple packages with additional dependencies, there is no need to build the sysroot locally with Docker, and a readily available base Docker image can be used.
For packages with custom dependencies, the base sysroot can be extended using `rosdep install` on a Docker image that uses QEMU through [`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc).

## Limitations and open questions

This setup is focused on cross-compiling for Linux based platforms.
For example we couldn't use this to cross-compile for Windows, because we cannot install the Visual Studio compiler using Apt.
On the other hand, a Windows or Mac development workstation could use this build setup, running the build commands inside a Docker container, although that would imply some performance degradation due to the virtualization layer that Docker employs on those platforms.

We should determine a process for building and publishing the base ROS 2 Docker images for each platform.
Ideally a nightly job would publish an image built from source for the tip of master, but that might be not feasible depending on the available resources.

## Alternative approaches

The [colcon bundle plugin](https://github.com/colcon/colcon-bundle) can be used to package a ROS workspace together with all its dependencies into a tarball.
That could be used for cross-compiling a ROS 2 workspace by launching a Docker container for the target platform, building and bundling the workspace, and exporting the bundle, which can then be deployed to the target hardware.
The main drawback of that approach is that using the default ARM compiler in the Docker image significantly slows down the compilation, which might be a burden for day-to-day development.
By installing `qemu-user-static` in the present proposal, we are not only installing the QEMU binaries to simulate an ARM32/64 processor, but also registering [`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc) hook in the local machine.
That means that every time a program will start, if it is for a platform that QEMU supports, the binary will automatically run through QEMU.
In practice, it lets you run ARM binaries on your X86 instance, as it is done implicitly through Docker when building the base image.
The problem is that QEMU is ~10x slower than normal execution, but the present proposal limits the QEMU usage to building the base image (which is unavoidable), and uses compilers for the target platform running natively the rest of the time.

The colcon bundle approach would require adding support for ROS 2 to colcon bundle, that currently doesn't support Ament.
That would provide a way to distribute cross-compiled system dependencies, which is convenient because `rosdep install` can take a lot of time on resource constrained hardware, and requires an Internet connection.
The presented approach and colcon bundle are complementary approaches, and if colcon bundle eventually supports ROS 2, we could use `cc-build` for easy cross-compilation minimizing the usage of Docker, followed by `colcon bundle` to generate a deployable artifact.

Regarding the workspace dependent sysroot image, we could avoid the Docker `COPY` of the workspace by not building a workspace sysroot image, and instead launching a container for the base ROS 2 image with a [bind mount](https://docs.docker.com/storage/bind-mounts/) for the workspace.
The container's entry point would launch `rosdep` on the directory where the workspace is mounted, and we'd export the container's file system is then exported with  `docker container export` as above.
That avoids a `COPY`, but a disadvantage is that then we wouldn't have a sysroot image for our workspace, and we would have to run `rosdep` on the entry point of that image each time we launch a container.
That image would be useful for running tests, as mentioned in the testing section above, and an eventual `cc-test` command could further simplify using that image.

Finally, compared with the original approach in the [cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation), the proposed commands basically organizes the commands that appear in the tutorial, and that are used in [cross_compile/entry_point.sh](https://github.com/ros2/cross_compile/blob/master/entry_point.sh), to allow cross-compiling a workspace with a one-liner command.
So this is more an evolution than an alternative proposal.
