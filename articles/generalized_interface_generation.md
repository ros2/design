---
layout: default
title: Generalized interface generation pipeline
permalink: articles/generalized_interface_generation.html
abstract:  Review of the current interface generation architecture and its drawbacks. Design proposal towards a multi-language and multi-build system architecture.
published: false
author: '[Michel Hidalgo](https://github.com/hidmic)'
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Scope

This article discusses a generalization of the `rosidl` interface generation pipeline to work with build system generators other than CMake and set the scene for better multi-language support in ROS.

## Review

### Overview

For a system architecture-level overview and rationale for the `rosidl` interface generation pipeline, see [here](https://github.com/ros2/ros_core_documentation/blob/e0b7e2bbd026aa466b23cfc5e070144c9114fe29/source/developer_overview.rst#type-specific-interfaces).

From a structural point-of-view, the `rosidl` interface generation pipeline is comprised of:

- **Generator packages**. These packages provide tooling to generate language-specific, in-memory representations of ROS interfaces. Common runtime (i.e. interface agnostic) functionality is conventionally but not necessarily split into a separate package (e.g. [`rosidl_runtime_c`](https://github.com/ros2/rosidl/tree/master/rosidl_runtime_c), [`rosidl_runtime_cpp`](https://github.com/ros2/rosidl/tree/master/rosidl_runtime_cpp)). It is not unusual for a generator to use another generator's output via language-specific binding machinery (e.g. [`rosidl_generator_py`](https://github.com/ros2/rosidl_python/blob/master/rosidl_generator_py/), which uses Python C bindings to wrap [`rosidl_generator_c`](https://github.com/ros2/rosidl/tree/master/rosidl_generator_c) representations). 
- **Type support packages**. These packages provide tooling to generate language-specific and often middleware-specific support code for middleware implementations to interact (e.g. on publish, on take) with in-memory representations of ROS interfaces. Type support functionality is encapsulated in `rosidl_message_type_support_t`, `rosidl_service_type_support_t`, and `rosidl_action_type_support_t` C structs in read-only global storage, one for each interface type. To support middleware runtime selection, it is not unusual to find language-specific type support packages that do not provide any functionality of their own but defer to middleware-specific type supports accordingly (e.g. [`rosidl_typesupport_c`](https://github.com/ros2/rosidl_typesupport/tree/master/rosidl_typesupport_c), [`rosidl_typesupport_cpp`](https://github.com/ros2/rosidl_typesupport/tree/master/rosidl_typesupport_c)).

Packages for interface definition translation (e.g. [`rosidl_adapter`](https://github.com/ros2/rosidl/tree/master/rosidl_adapter), [`rosidl_generator_dds_idl`](https://github.com/ros2/rosidl_dds/tree/master/rosidl_generator_dds_idl)), interface definition parsing (e.g. [`rosidl_parser`](https://github.com/ros2/rosidl/tree/master/rosidl_parser)), and `ament_cmake` integration (e.g. [`rosidl_cmake`](https://github.com/ros2/rosidl/tree/master/rosidl_cmake/rosidl_cmake)) complement and support generator and type support packages implementation.

Alongside generated source code, generator and type support packages are expected to provide the means to build it by adding to the `rosidl_generate_idl_interfaces`' [`ament` extension point](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#adding-to-extension-points). By tapping into this extension, downstream `ament_cmake` packages can delegate interface generation to all available packages in the workspace (or any of its underlays).

### Drawbacks

As it stands, the `rosidl` interface generation pipeline:

- **Is strongly coupled with CMake**, and in particular with `ament_cmake` machinery. There's no way for packages using different build systems or build system generators to generate their own interfaces (e.g. pure Python packages cannot generate interfaces nor extend the pipeline). There's no way for external projects using different build systems or build system generators to provide their own generator or type support packages.

- **Favors monolithic builds**. Since interface generation output cannot be controlled at the package level (i.e. all available extensions will be invoked), one and only one package can build and install all generated artifacts. It also implies that the set of artifacts that the package provides depends on the set of generator and type support packages present in the workspace at build-time. This results in package rebuilds to extend or restrict that support (e.g. adding support for a non-core language requires a rebuild of core interface packages), and loose package versioning (e.g. a change in a generator package can induce an API/ABI break in an interface package).

## Proposal

### Motivation

This proposal aims to mitigate or provide the means to mitigate all drawbacks in the current pipeline architecture.
As such, it does not fundamentally change the concepts that underpin it nor its outcome (e.g. public APIs).

### Goals

- Decouple tooling from all build systems and build system generators
- Standardize tooling to simplify user and developer experience
- Support tooling extensibility to encourage code share and reuse
- Enforce tooling versioning to ensure API/ABI stability and simplify deprecation cycles
- Allow tooling configuration to enable scoped builds

### Tooling updates

#### Source code generation

##### Description

Migrate generation logic to extensible, standardized tools that can compile interface definitions into language-specific type representations and middleware-specific type support code.
Support for new languages and middlewares can be added by external packages via a plugin system.

##### Specification

Interface type representation and type support generation CLIs are roughly equivalent but conceptually different, and thus kept separate. 
This allows them to deviate from each other in the future if need be.

*For interface type representations*
```sh
rosidl_generate ((-d|--output-directory) PATH)? ((-l|--language) IDENTIFIER(==VERSION)?)* ((-I|--include-path) PATH)* PATH+
```

*For interface type supports*
```sh
rosidl_typesupport_generate ((-d|--output-directory) PATH)? ((-t|--type-support) IDENTIFIER(==VERSION)?)* ((-I|--include-path) PATH)* PATH+
```

All positional arguments are paths to interface definition files.
If no output directory is specified, the current working directory is used.
If no type representation / type support generators are specified, all available generators available are used.
If a type representation / type support generator is specified but not found, the tool fails.
If a specific version of a type representation / type support generator is specified but not found, the tool fails.

##### Implementation considerations

Using Python allows for significant code reuse in the vast majority of existing generator and type support packages.
Additionally, [`setuptools` entrypoints](https://setuptools.readthedocs.io/en/latest/userguide/entry_point.html#advertising-behavior) can be leveraged as a plugin system.

#### Build system configuration

##### Description

Migrate build logic to extensible, standardized tools that can generate [common build specifications](#a-common-build-specification) for language-specific type representations and middleware-specific type support code.
Support for new languages and middlewares can be added by external packages via a plugin system.

##### Rationale

Source code generators output may vary significantly. 
It may be one source file or many source files in a hierarchy of directories.
To build these source files, a command or a script specific to the code generator may have to be executed.
Thus, in the most general case, build system integration is necessary.

However, integration with any given build system or build system generator couples a large portion of the interface pipeline to that set of tools, which then has to be ported over for projects that use other build systems or build system generators.
Programs that can bridge across build systems and/or build system generators do exist, but these are rare and often limited to a subset of the functionality that is shared.

By generating a build specification in a format designed to be simple yet general, an appropriate build system can be generated on consumption.

##### Specification

Interface type representation and type support build configuration CLIs are roughly equivalent but conceptually different, and thus kept separate.
This allows them to deviate from each other in the future if need be.

*For interface type representations*
```sh
rosidl_build_configure ((-o|--output-file) PATH)? ((-l|--language) IDENTIFIER(==VERSION)?)* ((-I|--include-path) PATH)* PATH+
```

*For interface type supports*
```sh
rosidl_typesupport_build_configure ((-o|--output-file) PATH)? ((-t|--type-support) IDENTIFIER(==VERSION)?)* ((-I|--include-path) PATH)* PATH+
```

All positional arguments are paths to interface definition files.
If no language / type support is specified, all type / type support generators available are used.
If no type / type support generator is found, the command fails.
If no output file path is provided, the generated build specification is sent to standard output.

##### Implementation considerations

All build logic in existing generator and type support packages is CMake code, and thus a port is necessary. 
Using Python would ensure tooling interoperability and their supporting libraries.
Additionally, [`setuptools` entrypoints](https://setuptools.readthedocs.io/en/latest/userguide/entry_point.html#advertising-behavior) can be leveraged as plugin system.

Since common build specifications are eminently declarative, most configuration must be resolved prior to generation e.g. for which OS to build.
This includes dependencies between generated outputs e.g. type support code may depend on type representation code to build, and thus a build specification for the former must ensure the presence of the latter by conditionally generating it if not present.

#### Meta build tool

##### Description

Develop an extensible, standardized tool that can translate [common build specifications](#a-common-build-specification) into an actual build system or build system generator code.
Support for new build systems and build system generators can be added by external packages via a plugin system.

##### Specification

```sh
ament_meta_build ((-o|--output-file) PATH)? ((-b|--build-system) IDENTIFIER)? PATH?
```

Its only positional argument is a path to a common build specification file.
If no common build specification file is given, it is read from standard input.
If no output file path is provided, generated build system sources are written to standard output.
If no build system is specified nor it can be deduced from output file extension, the tool fails.
If the specific build system is not supported, the tool fails.

### Build system integration

#### `ament_cmake` integration

It can be achieved by a thin layer that leverages [build system configuration](#build-system-configuration) and [meta build](#meta-build-tool) tooling to bootstrap an `ament_cmake` package i.e. to generate and include CMake code during the configuration stage.
This layer must bring in package dependencies as there is no build system-agnostic mechanism to do so.

## Appendix

### A. Common Build Specification

This specification is based on the [Common Package Specification](https://mwoehlke.github.io/cps/) format, a simple and relatively standard format that describes packages for downstream consumption in a build system and build tool agnostic way.

This [patch](TODO(hidmic): draft patch) extends the aforementioned format schema to describe component builds.

As an example, the following snippet describes a Python package `bar` that depends on a shared library `foo`,
whose sources are generated by a `foo-code` command.

```json
{
  "Components": {
    "foo-code": {
       "Type": "gen",
       "Command": ["/bin/touch", "foo.c", "foo.h"],
       "Output": ["foo.c", "foo.h"]
    },
    "foo": {
       "Type": "module",
       "Requires": [":foo-code"],
       "Sources": ["foo.c"],
       "Includes": ["foo.h"],
       "Link-Libraries": ["libpython-dev.so"]
    },
    "bar": {
      "Type": "pymodule",
      "Requires": [":foo"],
      "Sources": ["bar.py"]
    }
  }
}
```

Note that the format is declarative in nature.

### B. CLI Specification Syntax

The syntax used to specify command line interfaces in this article takes after [Python 3 regular expression syntax](https://docs.python.org/3/library/re.html#re-syntax). 

Upper case tokens are expressions aliases, namely:

- `IDENTIFIER` is a string of non-whitespace characters
- `VERSION` is a [semantic](https://semver.org/) version 
- `PATH` is a host file system path
