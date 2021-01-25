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

For a system architecture-level overview and rationale for the `rosidl` interface generation pipeline, see [here](https://github.com/ros2/ros_core_documentation/blob/e0b7e2bbd026aa466b23cfc5e070144c9114fe29/source/developer_overview.rst#type-specific-interfaces) (though bear in mind this documentation has not yet been updated to reflect the fact that `rosidl` generators consume `.idl` files nowadays, see [ros2/rosidl#298](https://github.com/ros2/rosidl/pull/298)).

From a structural point-of-view, the `rosidl` interface generation pipeline is comprised of:

- **Generator packages**. These packages provide tooling to generate language-specific, in-memory representations of ROS interfaces.
  Common runtime (i.e. interface agnostic) functionality is conventionally but not necessarily split into a separate package (e.g. [`rosidl_runtime_c`](https://github.com/ros2/rosidl/tree/master/rosidl_runtime_c), [`rosidl_runtime_cpp`](https://github.com/ros2/rosidl/tree/master/rosidl_runtime_cpp)). It is not unusual for a generator to use another generator's output via language-specific binding machinery (e.g. [`rosidl_generator_py`](https://github.com/ros2/rosidl_python/blob/master/rosidl_generator_py/), which uses Python C bindings to wrap [`rosidl_generator_c`](https://github.com/ros2/rosidl/tree/master/rosidl_generator_c) representations), although this dependency is not explicit: generator packages do depend on each other but it is assumed that their generated code will all be built within the same package so that build system dependencies can be established.
- **Type support packages**. These packages provide tooling to generate language-specific and often middleware-specific support code for middleware implementations to interact (e.g. on publish, on take) with in-memory representations of ROS interfaces.
  Type support functionality is encapsulated in `rosidl_message_type_support_t`, `rosidl_service_type_support_t`, and `rosidl_action_type_support_t` C structs in read-only global storage, one for each interface type.
  To support middleware runtime selection, it is not unusual to find language-specific type support packages that do not provide any functionality of their own but defer to middleware-specific type supports accordingly (e.g. [`rosidl_typesupport_c`](https://github.com/ros2/rosidl_typesupport/tree/master/rosidl_typesupport_c), [`rosidl_typesupport_cpp`](https://github.com/ros2/rosidl_typesupport/tree/master/rosidl_typesupport_c)).

Packages for interface definition translation (e.g. [`rosidl_adapter`](https://github.com/ros2/rosidl/tree/master/rosidl_adapter), [`rosidl_generator_dds_idl`](https://github.com/ros2/rosidl_dds/tree/master/rosidl_generator_dds_idl)), interface definition parsing (e.g. [`rosidl_parser`](https://github.com/ros2/rosidl/tree/master/rosidl_parser)), and `ament_cmake` integration (e.g. [`rosidl_cmake`](https://github.com/ros2/rosidl/tree/master/rosidl_cmake/rosidl_cmake)) complement and support generator and type support packages implementation.

Alongside generated source code, generator and type support packages are expected to provide the means to build it by adding to the `rosidl_generate_idl_interfaces`' [`ament` extension point](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#adding-to-extension-points).
By tapping into this extension, downstream `ament_cmake` packages can delegate build-system configuration to all available generator and type support packages in the workspace (or any of its underlays) during the configuration phase, so as to generate and then build source code during the build phase.
This is what constitutes a (roughly) 3-stage pipeline, from interface definition files to source code to artifacts:

![ROSIDL 3-stage pipeline](/img/generalized_interface_generation/rosidl_3_stage_pipeline.png)

Note that the order in which each `ament` extension runs is the order in which each package was imported into CMake, by virtue of how these work.
This order is further obfuscated by the discovery process that the [`rosidl_default_generators`](https://github.com/ros2/rosidl_defaults/tree/master/rosidl_default_generators) package performs.
Thus, for all practical purposes this order cannot be relied on, and build-system dependencies must be leveraged to establish dependencies across generators' output.

### Drawbacks

As it stands, the `rosidl` interface generation pipeline:

- **Is strongly coupled with CMake**, and in particular to `ament_cmake` machinery.
  There's no way for packages using different build systems or build system generators to generate their own interfaces (e.g. pure Python packages cannot generate interfaces nor extend the pipeline).
  There's no way for external projects using different build systems or build system generators to provide their own generator or type support packages.

- **Favors monolithic builds**.
  Since interface generation output cannot be controlled at the package level (i.e. all available extensions must and will be invoked), one and only one package can build and install all generated artifacts.
  It also implies that the set of artifacts that the package provides depends on the set of generator and type support packages present in the workspace at build-time.
  This results in package rebuilds to extend or restrict that support (e.g. adding support for a non-core language requires a rebuild of core interface packages), and loose package versioning (e.g. a change in a generator package can induce an API/ABI break in an interface package).

## Proposal

### Motivation

This proposal aims to mitigate or provide the means to mitigate all drawbacks in the current pipeline architecture.
As such, it does not fundamentally change the concepts that underpin it nor its outcome (e.g. public APIs).

### Goals

- Decouple tooling from all build systems (generators)
- Standardize tooling to simplify user and developer experience
- Support tooling extensibility to encourage code share and reuse
- Enforce tooling versioning to ensure API/ABI stability and simplify deprecation cycles
- Allow tooling configuration to enable scoped builds

### Overview

The first stage in the current pipeline is split in two, configuring a 4-stage pipeline.

![ROSIDL 4-stage pipeline](/img/generalized_interface_generation/rosidl_4_stage_pipeline.png)

In the first stage, a [common build specification](#a-common-build-specification) that outlines how type representation and type support code must be generated, built, and installed is generated by `rosidl_build_configure` (see below).
In the second stage, the build specification is translated into build-system's files by `ament_meta_build` (see below).
Both the first and second stages would normally occur during the build-system's __configuration_ phase.
In the third stage, code is actually generated by `rosidl_generate` (see below).
In the fourth stage, the build-system builds and install all artifacts.
Both the third and fourth stages would normally occur during the build-system's __build_ phase.

### Tooling updates

#### Source code generators

##### Description

Migrate generation logic to extensible, standardized tools that can compile interface definitions into language-specific type representation and middleware-specific type support code.

These tools take a set of interface definition files and generate type representation or type support source code.
Customarily, interface definition files are [IDL](https://www.omg.org/spec/IDL/About-IDL/) files.

Support for new languages and middlewares can be added by external packages via a system of _code generator_ plugins.
Each plugin version is expected to generate the same output for the same input over time.
Users can (but are not forced to) target specific plugin versions to prevent workspace (or underlay) updates from affecting tool output.

##### Specification

```sh
rosidl_generate ((-d|--output-directory) PATH)? ((-l|--language) IDENTIFIER(==VERSION)?)* ((-t|--type-support) IDENTIFIER(==VERSION)?)* ((-I|--include-path) PATH)* PATH+
```

All positional arguments are paths to interface definition files.
If no output directory is specified, the current working directory is used.
If no language nor type support generator is specified, all available generators are used.
If a language or a type support generator is specified but not found, the tool fails.
If a specific version of a language or type support generator is specified but not found, the tool fails.

##### Implementation considerations

Using Python allows for significant code reuse in the vast majority of existing generator and type support packages.
Additionally, [`setuptools` entrypoints](https://setuptools.readthedocs.io/en/latest/userguide/entry_point.html#advertising-behavior) can be leveraged as a plugin system.

#### Build specification generators

##### Description

Migrate build logic to extensible, standardized tools that can generate [common build specification](#a-common-build-specification) files for language-specific type representation and middleware-specific type support code.

These tools take a set of interface definition files and generate a [common build specification](#a-common-build-specification) files.
Customarily, interface definition files are [IDL](https://www.omg.org/spec/IDL/About-IDL/) or [ROS interface](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/) files.
A build specification must take care of any and all necessary interface definition conversions for the associated source code generator to work.

Support for new languages and middlewares can be added by external packages via a system of _build generator_ plugins.
Each plugin version is expected to generate the same output for the same input over time.
Users can (but are not forced to) target specific plugin versions to prevent workspace (or underlay) updates from affecting tool output.

##### Rationale

Source code generators output may vary significantly.
It may be one source file or many source files in a hierarchy of directories.
To build these source files, a command or a script specific to the code generator may have to be executed.
Thus, in the most general case, build system integration is necessary.

However, integration with any given build-system (generator) A couples a large portion of the interface pipeline to it.
If a different build-system (generator) B is to be used, users are forced to either:
* **Delegate from build-system (generator) B to build-system (generator) A**.
  This is challenging. Build-system (generator) A must be properly embedded in build-system (generator) B, pushing and pulling information across the gap for builds and installs to succeed, adapting invocations of build-system (generator) B to any constraints that build-system (generator) A may impose (e.g. Bazel performs sandboxed builds).
  There is a performance penalty in doing this.
* **Port from build-system (generator) A to build-system (generator) B**.
  This may be (and currently is) a non-trivial effort.
  Programs that can convert from one build-system (generator) to another do exist, but these are rare and often limited to a subset of the functionality that is shared between source and target.

By generating a build specification in a format designed to be simple yet general, an appropriate build system can be generated when needed.

##### Specification

```sh
rosidl_build_configure ((-o|--output-file) PATH)? ((-l|--language) IDENTIFIER(==VERSION)?)* ((-t|--type-support) IDENTIFIER(==VERSION)?)* ((-I|--include-path) PATH)* PATH+
```

All positional arguments are paths to interface definition files.
If no language nor type support generator is specified, all available generators are used.
If no language nor type support generator is found, the command fails.
If no output file path is provided, the common build specification is sent to standard output.

##### Implementation considerations

All build logic in existing generator and type support packages is CMake code, and thus a port is necessary.
Using Python would ensure tooling interoperability and their supporting libraries.
Additionally, [`setuptools` entrypoints](https://setuptools.readthedocs.io/en/latest/userguide/entry_point.html#advertising-behavior) can be leveraged as plugin system.

Common build specifications can only describe platform- and/or configuration-specific variations in how a component is built and distributed.
Any other conditionals must be resolved prior to generating an specification.
There are otherwise no requirements nor constraints as to whether and how a build is specified e.g. it may specify shared libraries as well as generic components on an arbitrary command (which could be a build tool).
Build specifications that use well-known component types are, however, more portable and yield better build-system integration.
A build-system adapter plugin in `ament_meta_build` cannot workaround a build specification that forces `cmake` to be called.

Common build specifications describe full packages.
In order to support running multiple generators at once, the tooling must merge build specifications and resolve all dependencies between these prior to generating an specification.
It is also recommended that equivalent components (e.g. `.idl`  files generated from `.msg` files) be de-duplicated.

#### Meta build tool

##### Description

Develop an extensible, standardized tool that can parse [common build specifications](#a-common-build-specification) and generate actual build system (generator) code.

Support for new build systems and build system generators can be added by external packages via a system of __build-system adapter_ plugins.
Each plugin version is expected to generate the same output for the same input over time.
Users can (but are not forced to) target specific plugin versions to prevent workspace (or underlay) updates from affecting tool output.

##### Specification

```sh
ament_meta_build ((-o|--output-file) PATH)? (--build-prefix PATH)? (--install-prefix PATH)? ((-b|--build-system) IDENTIFIER(==VERSION)?)? PATH?
```

Its only positional argument is a path to a common build specification file.
If no common build specification file is given, it is read from standard input.
If no output file path is provided, generated build system sources are written to standard output.
If no build prefix is specified, the default build prefix (if any) for the chosen build system is used.
If no install prefix is specified, the default install prefix (if any) for the chosen build system is used.
If no build system is specified nor it can be deduced from output file extension, the tool fails.
If the specific build system is not supported, the tool fails.

##### Implementation considerations

There are no requirements nor constraints as to whether and how any given build-system adapter plugin supports the common build specification schema.
A plugin may generate native build-system generator code to build some component types, delegate on other plugins (and thus to other build systems) to build other component types, and not support some component types at all.
The broader native support is, the lower will the build overhead (usually) be, and the more useful will an (open source) plugin be to the community.

### Build system integration

It can be achieved by adding thin layers that leverage [build system configuration](#build-system-configuration) and [meta build](#meta-build-tool) tooling to bootstrap an package builds.

For instance, an `ament_cmake` integration would generate and include CMake code during the configuration stage e.g.:

```cmake
# Generate C++ type representation build specification
execute_process(
  COMMAND rosidl_build_configure
     --language cpp
     -I path/to/interface/dependencies/
     interface0.idl interface1.idl
  OUTPUT_FILE build.spec)
# Translate build specification into CMake code
execute_process(
  COMMAND ament_meta_build -b cmake
  INPUT_FILE build.spec
  OUTPUT_FILE build.cmake)
# Use generate CMake code
include(build.cmake)
```

It is worth noting that these integration layers must bring in package dependencies on their own, as there is no build system-agnostic mechanism to do so.

### Migration

Migrating the current architecture is fairly straightforward.

- Each tightly coupled pair of plugins for `rosidl_generate` and `rosidl_build_configure` can be built on top of each existing generator or type support package.
- Generated build specifications can delegate to CMake while the `ament_meta_build` tooling is independently developed.
- New `ament_cmake` macros can be added to tap into the pipeline, while keeping existing ones (which may be deprecated over time, or re-implemented on top of the former). 

## Appendix

### A. Common Build Specification

The Common Build Specification is a mechanism for describing how to build and distribute a software package, such that any build tool can carry out the process. The draft specification can be found [here](https://github.com/hidmic/cps).

This specification is based on the [Common Package Specification](https://mwoehlke.github.io/cps/) format, a simple and relatively standard format that describes packages for downstream consumption in a build tool-agnostic way.
The CPS format was not designed to describe builds, and thus why a modification is needed.

As examples, see below the `.cbs` files that would result from running `rosidl_generator_c` and `rosidl_generator_py` on a `foo_msgs` package.


*foo_msgs_c.cbs*
```json
{
    "Name": "foo_msgs_c",
    "Description": "Build specification for ROS2 C foo_msgs interfaces",
    "License": "Apache License 2",
    "Version": "0.1.0",
    "Compat-Version": "0.8.0",
    "Platforms": {
        "GNU" : {
            "Cpp-Compiler": "gnu",
        },
        "Clang" : {
            "Cpp-Compiler": "clang",
        },
        "Windows": {
            "Kernel-Version": "windows"
        }
    },
    "Requires": {
        "rosidl_runtime_c": {},
        "rosidl_typesupport_interface": {},
        "action_msgs": {}
    },
    "Configurations": [ "Optimized", "Debug" ],
    "Default-Components": [ "c" ],
    "Components": {
        "idls": {
            "Type": "generic",
            "Assembly": {
                "Command": "rosidl_adapt -p foo_msgs $(sources)",
                "Sources": [
                    "msg/foo.msg",
                    "srv/bar.srv",
                    "action/baz.action"
                ],
                "Artifacts": [
                    "msg/foo.idl",
                    "srv/bar.idl",
                    "action/baz.idl"
                ]
            },
            "Distribution": {
                "Location": "@prefix@/share/foo_msgs"
            }
        },
        "c_code": {
            "Type": "generic",
            "Assembly": {
                "Requires": [ "action_msgs:idls", ":idls" ],
                "Command": "rosidl_generate --language c -p foo_msgs -I $(location action_msgs:idls) $(sources)",
                "Sources": [
                    "msg/foo.idl",
                    "srv/bar.idl",
                    "action/baz.idl"
                ],
                "Artifacts": [
                    "msg/foo.h",
                    "msg/detail/foo__functions.h",
                    "msg/detail/foo__struct.h",
                    "msg/detail/foo__type_support.h",
                    "msg/detail/foo__functions.c",
                    "srv/bar.h",
                    "srv/detail/bar__functions.h",
                    "srv/detail/bar__struct.h",
                    "srv/detail/bar__type_support.h",
                    "srv/detail/bar__functions.c",
                    "action/baz.h",
                    "action/detail/baz__functions.h",
                    "action/detail/baz__struct.h",
                    "action/detail/baz__type_support.h",
                    "action/detail/baz__functions.c",
                    "rosidl_generator_c_visibility_control.h"
                ]
            }
        },
        "c": {
            "Type": "dylib",
            "Assembly": {
                "Requires": [
                    ":c_code",
                    "rosidl_runtime_c",
                    "rosidl_typesupport_interface",
                    "action_msgs"
                ],
                "Sources": [
                    "msg/foo.h",
                    "msg/detail/foo__functions.h",
                    "msg/detail/foo__functions.c",
                    "msg/detail/foo__struct.h",
                    "msg/detail/foo__type_support.h",
                    "srv/bar.h",
                    "srv/detail/bar__functions.h",
                    "srv/detail/bar__functions.c",
                    "srv/detail/bar__struct.h",
                    "srv/detail/bar__type_support.h",
                    "action/baz.h",
                    "action/detail/baz__functions.h",
                    "action/detail/baz__functions.c"
                    "action/detail/baz__struct.h",
                    "action/detail/baz__type_support.h",
                    "rosidl_generator_c_visibility_control.h"
                ],
                "Configurations": {
                    "GNU": {
                        "Compiler-Features": [ "c11" ],
                        "Compiler-Flags": [ "-Wall", "-Wextra", "-Wpedantic" ],
                    },
                    "Clang": {
                        "Compiler-Features": [ "c11" ],
                        "Compiler-Flags": [ "-Wall", "-Wextra", "-Wpedantic" ],
                    },
                    "Windows": {
                        "Definitions": [ "ROSIDL_GENERATOR_C_BUILDING_DLL_foo_msgs" ],
                    }
                }
            },
            "Distribution": {
                "Name": "foo_msgs_c",
                "Requires": [
                    "rosidl_runtime_c",
                    "rosidl_typesupport_interface",
                    "action_msgs"
                ],
                "Include-Location": {
                    "@prefix@/include/foo_msgs": [
                        "msg/foo.h",
                        "msg/detail/foo__functions.h",
                        "msg/detail/foo__struct.h",
                        "msg/detail/foo__type_support.h",
                        "srv/bar.h",
                        "srv/detail/bar__functions.h",
                        "srv/detail/bar__struct.h",
                        "srv/detail/bar__type_support.h",
                        "action/baz.h",
                        "action/detail/baz__functions.h",
                        "action/detail/baz__struct.h",
                        "action/detail/baz__type_support.h",
                        "rosidl_generator_c_visibility_control.h"
                    ]
                },
                "Includes": [ "@prefix@/include" ]
            }
        }
    }
}
```

*foo_msgs_py.cbs*
```json
{
    "Name": "foo_msgs_py",
    "Description": "Build specification for ROS2 Python foo_msgs interfaces",
    "License": "Apache License 2",
    "Version": "0.1.0",
    "Compat-Version": "0.8.0",
    "Platforms": {
        "GNU" : {
            "Cpp-Compiler": "gnu",
        },
        "Clang" : {
            "Cpp-Compiler": "clang",
        },
        "Windows": {
            "Kernel-Version": "windows"
        }
    },
    "Requires": {
        // Base common dependencies
        "rmw": {},
        "rosidl_runtime_c": {},
        "rosidl_typesupport_c": {},
        "rosidl_typesupport_interface": {},
        "rosidl_typesupport_c": {},
        "rosidl_typesupport_fastrtps_c": {},
        "rosidl_typesupport_introspection_c": {},
        // CPython development dependencies
        "python-dev": {
            "Version": "3.5"
        },
        // A dependency on the package described above
        "foo_msgs_c": {},
        // Other foo_msgs generated packages
        "foo_msgs_typesupport_c": {},
        "foo_msgs_typesupport_fastrtps_c": {},
        "foo_msgs_typesupport_introspection_c": {},
        // Common action dependencies
        "action_msgs": {}
    },
    "Configurations": [ "Optimized", "Debug" ],
    "Default-Components": [ "py" ],
    "Components": {
        "idls": {
            // Generated IDL files from ROS legacy definitions
            "Type": "generic",
            "Assembly": {
                "Command": "rosidl_adapt -p foo_msgs $(sources)",
                "Sources": [
                    "msg/foo.msg",
                    "srv/bar.srv",
                    "action/baz.action"
                ],
                "Artifacts": [
                    "msg/foo.idl",
                    "srv/bar.idl",
                    "action/baz.idl"
                ]
            },
            "Distribution": {
                "Location": "@prefix@/share/foo_msgs"
            }
        },
        "py_code": {
            // Generated Python and C extension code
            "Type": "generic",
            "Assembly": {
                "Requires": [ "action_msgs:idls", ":idls" ],
                "Command": "rosidl_generate -l python -p foo_msgs -I $(location action_msgs:idls) $(sources)",
                "Sources": [
                    "msg/foo.idl",
                    "srv/bar.idl",
                    "action/baz.idl"
                ],
                "Artifacts": [
                    "__init__.py",
                    "msg/__init__.py",
                    "msg/_foo.py",
                    "msg/_foo_s.c",
                    "srv/__init__.py",
                    "srv/_bar.py",
                    "srv/_bar_s.c",
                    "action/__init__.py",
                    "action/_baz.py",
                    "action/_baz_s.c",
                    "_foo_msgs_s.ep.rosidl_typesupport_c.c",
                    "_foo_msgs_s.ep.rosidl_typesupport_fastrtps_c.c",
                    "_foo_msgs_s.ep.rosidl_typesupport_introspection_c.c",
                ]
            },
        },
        "py": {
            // Python module
            "Type": "pymodule",
            "Assembly": {
                "Requires": [ ":py_code" ],
                "Sources": [
                    "__init__.py",
                    "msg/__init__.py",
                    "msg/_foo.py",
                    "srv/__init__.py",
                    "srv/_bar.py",
                    "action/__init__.py",
                    "action/_baz.py",
                ]
            },
            "Distribution": {
                "Name": "foo_msgs"
            }
        },
        "cpython": {
            // Common CPython shared library to support extensions
            "Type": "dylib",
            "Assembly": {
                "Requires": [
                    ":py_code",
                    "foo_msgs_c",
                    "action_msgs:cpython",
                    "python-dev",
                ],
                "Sources": [
                    "msg/_foo_s.c",
                    "srv/_bar_s.c"
                    "action/_baz_s.c"
                ],
                // Use include directories from default component of foo_msgs_typesupport_c.
                "Include-Requires": [ "foo_msgs_typesupport_c" ],
                "Configurations": {
                    "Optimized": {
                        // Use Python interpreter to resolve numpy include path.
                        "System-Includes": [ "$(shell python -c \"import numpy; print(numpy.get_include())\")" ]
                    },
                    "Debug": {
                        // Use debug Python interpreter to resolve numpy include path.
                        "System-Includes": [ "$(shell pythond -c \"import numpy; print(numpy.get_include())\")" ]
                    }
                }
            },
            "Distribution": {
                // Install to default library location
                "Name": "foo_msgs_cpython"
            }
        },
        "pyext_typesupport_c": {
            // CPython extension to access C typesupport
            "Type": "cpyext",
            "Assembly": {
                "Requires": [
                    ":py_code",
                    ":cpython",
                    "foo_msgs_typesupport_c",
                    "rosidl_runtime_c",
                    "rosidl_typesupport_c",
                    "rosidl_typesupport_interface",
                    "action_msgs",
                ],
                "Configurations": {
                    "GNU": {
                        "Compiler-Flags": [ "-Wall", "-Wextra" ],
                    }
                    "Clang": {
                        "Compiler-Flags": [ "-Wall", "-Wextra" ],
                    }
                },
                "Sources": [ "_foo_msgs_s.ep.typesupport_c.c" ],
                // Use include directories from default component of foo_msgs_typesupport_c.
                "Include-Requires": [ "foo_msgs_typesupport_c" ]
            },
            "Distribution": {
                // Prefix should be set to the proper site-packages subdirectory
                "Location": "@python-prefix@/foo_msgs",
                // A proper Python extension suffix will be added for pyext components.
                "Name": "foo_msgs_s__typesupport_c"
            }
        },
        "pyext_typesupport_fastrtps_c": {
            // CPython extension to access Fast-RTPS C typesupport
            "Type": "cpyext",
            "Assembly": {
                "Requires": [
                    ":py",
                    ":cpython",
                    "foo_msgs_typesupport_fastrtps_c",
                    "rosidl_runtime_c",
                    "rosidl_typesupport_c",
                    "rosidl_typesupport_interface",
                    "action_msgs"
                ],
                "Configurations": {
                    "GNU": {
                        "Compiler-Flags": [ "-Wall", "-Wextra" ],
                    }
                    "Clang": {
                        "Compiler-Flags": [ "-Wall", "-Wextra" ],
                    }
                },
                "Sources": [ "_foo_msgs_s.ep.typesupport_fastrtps_c.c" ],
                "Include-Requires": [ "foo_msgs_typesupport_c" ]
            },
            "Distribution": {
                // Prefix should be set to the proper site-packages subdirectory
                "Location": "@python-prefix@/foo_msgs",
                // A proper Python extension suffix will be added for pyext components.
                "Name": "foo_msgs_s__typesupport_fastrtps_c"
            }
        },
        "pyext_typesupport_introspection_c": {
            // CPython extension to access introspection C typesupport
            "Type": "cpyext",
            "Assembly": {
                "Requires": [
                    ":py",
                    ":cpython",
                    "foo_msgs_typesupport_introspection_c"
                    "rosidl_runtime_c",
                    "rosidl_typesupport_c",
                    "rosidl_typesupport_interface",
                    "action_msgs",
                ],
                "Configurations": {
                    "GNU": {
                        "Compiler-Flags": [ "-Wall", "-Wextra" ],
                    }
                    "Clang": {
                        "Compiler-Flags": [ "-Wall", "-Wextra" ],
                    }
                },
                "Sources": [ "_foo_msgs_s.ep.typesupport_introspection_c.c" ],
                "Include-Requires": [ "foo_msgs_typesupport_c" ]
            },
            "Distribution": {
                // Prefix should be set to the proper site-packages subdirectory
                "Location": "@python-prefix@/foo_msgs",
                // A proper Python extension suffix will be added for pyext components
                "Name": "foo_msgs_s__typesupport_introspection_c"
            }
        }
    }
}
```

A few details worth noting:

- The explicit dependency on other `foo_msgs_*` generated packages, which the tooling should resolve if all are to be present on the same specification.
- The requirement on the `python-dev` package, which is a hint for the build system to build against CPython development libraries.
- The duplication of the `":idls"` component, which the tooling can (and should) de-duplicate.
- The overlap between build and source trees, which is a characteristic of CBS (for portability's sake) and that has to be dealt with by the build system (e.g. using precedence rules).

### B. CLI Specification Syntax

The syntax used to specify command line interfaces in this article takes after [Python 3 regular expression syntax](https://docs.python.org/3/library/re.html#re-syntax).

Upper case tokens are expressions aliases, namely:

- `IDENTIFIER` is a string of non-whitespace characters
- `VERSION` is a [semantic](https://semver.org/) version
- `PATH` is a host file system path
