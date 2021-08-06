---
layout: default
title: ROS Command Line Arguments
permalink: articles/ros_command_line_arguments.html
abstract:
  This article describes ROS 2 nodes command line arguments and their syntax.
author: '[Michel Hidalgo](https://github.com/hidmic)'
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Overview

As it was the case in ROS 1, ROS 2 nodes allow configuration via command line arguments to a certain degree.
In ROS 2, this interface had to become more complex to cope with a larger set of configuration options, an ambiguity in remapping rules and parameter assignment syntax (as a result of the leading underscore name convention for hidden resources), a one-to-many relationship between executables and nodes, to name a few.

Because of this, increasingly precise addressing mechanisms as well as leading double underscores (`__`) in some positional arguments, both natural extensions of existing ROS 1 command line features, are combined with ROS 2 specific command line flags.
Flags, in contrast with other custom syntax alternatives, are:

- Widely known and used.
- Explicit and less error prone.
- Suitable for tab completion.

Unfortunately, since these flags coexist with user-defined ones, additional guarding and extraction devices must be put in place -- one of the reasons why these were avoided entirely in ROS 1 command lines.

## Features

### Namespacing

To prevent ROS specific command line flags from colliding with user-defined ones, the former are scoped using the `--ros-args` flag and a trailing double dash token (`--`):

```sh
ros2 run some_package some_node [<user-defined-arg-0>...<user-defined-arg-N>] \
  --ros-args [<ros-specific-arg-0>...] -- [<user-defined-arg-N+1>...]
```

Note that `--ros-args --` i.e. an empty set is a valid invocation.

If no user defined arguments are provided after ROS specific arguments are, the double dash token (`--`) may be elided:

```sh
ros2 run some_package some_node [<user-defined-arg-0>...] --ros-args [<ros-specific-arg-0>...]
```

Note that a sole trailing `--ros-args` remains a valid invocation.

More than one set of ROS specific flags may appear in the same command line:

```sh
ros2 run some_package some_node --ros-args [<ros-specific-arg-0>...<ros-specific-arg-N>] -- \
  [<user-defined-arg-0>...] --ros-args [<ros-specific-arg-N+1>...]
```

This way, multiple sources, potentially unaware of each other, can append flags to the command line with no regard for previous sets.

### Capabilities

#### Summary

As a quick summary of ROS command line capabilities:

- For name remapping, use either `--remap from:=to` or `-r from:=to`.
- For single parameter assignment, use either `--param name:=value` or `-p name:=value` where value is in YAML format.
- For multiple parameter assignments, use `--params-file path/to/file.yaml` and a parameters YAML file.
- For setting logging (minimum) level, use `--log-level LEVEL_NAME`.
- For external logging configuration, use `--log-config-file path/to/file.config` and a log configuration file.
- For enabling/disabling logging:
  - to `rosout`, use `--enable-rosout-logs` or `--disable-rosout-logs`
  - to `stdout`, use `--enable-stdout-logs` or `--disable-stdout-logs`
  - to a external logging library, use `--enable-external-lib-logs` or `--disable-external-lib-logs`
- For enclave assignment, use either `--enclave value` or `-e value` where value is fully qualified enclave path.

For name remapping and parameter assignment, specific nodes can be targeted by prepending the option value with the node name followed by a colon `:`, as in `--remap my_node:from:=to` and `--param my_node:name:=value`.

#### Name remapping rules

Remapping rules may be introduced using the `--remap`/`-r` option.
This option takes a single `from:=to` remapping rule.

As an example, to remap from `foo` to `bar` for `some_ros_executable`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --remap foo:=bar
```

or its shorter equivalent:

```sh
ros2 run some_package some_ros_executable --ros-args -r foo:=bar
```

As is, this remapping rule applies to each and every node that `some_ros_executable` spawns unless explicitly ignored in code.
To limit it to `some_node`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args -r some_node:foo:=bar
```

#### Single parameter assignments

Parameter assignment may be achieved using the `--param`/`-p` option.
This option takes a single `name:=value` assignment statement, where `value` is in [YAML format](https://yaml.org/spec/) and thus YAML type inference rules apply.

As an example, to assign a string value `test` to a parameter `string_param` for `some_ros_executable`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --param string_param:=test
```

or its shorter equivalent:

```sh
ros2 run some_package some_ros_executable --ros-args -p string_param:=test
```

As is, this parameter assignment applies to each and every node that `some_ros_executable` spawns unless explicitly ignored in code.
To limit it to `some_node`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args -p some_node:string_param:=test
```

#### Multiple parameter assignments

Multiple parameter assignments can be performed at once using the `--params-file` option.
This option takes a [YAML](https://yaml.org/spec/) file with the following structure:

```yaml
node0_name:
  ros__parameters:
     param0_name: param0_value
     ...
     paramN_name: paramN_value
...
nodeM_name:
  ros__parameters:
     ...
```

Multiple nodes in a single executable can be targeted this way.
Note that YAML type inference rules for parameter values apply.

As an example, to assign a string value `foo` to a parameter `string_param` for `some_node` and a string value `bar` to that same parameter `string_param` but for `another_node` upon running `some_ros_executable` that contains both, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --params-file params_file.yaml
```

where `params_file.yaml` reads:

```yaml
some_node:
  ros__parameters:
     string_param: foo
another_node:
  ros__parameters:
     string_param: bar
```

#### Logging level assignments

Minimum logging level can be externally set either globally or per logger using the `--log-level` option.

As an example, to set a global logging level to `DEBUG` for `some_ros_executable`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --log-level DEBUG
```

Loggers can be set using the `--log-level` option as well:

```sh
ros2 run some_package some_ros_executable --ros-args --log-level talker1:=DEBUG --log-level talker2:=WARN --log-level rclcpp:=DEBUG
```

The minimum logging level of a specific logger will override the globally specified minimum logger level.
If a logging level is specified more than once in the passed command line arguments, the last one prevails.

See `rcutils` and `rcl` logging documentation for reference on existing logging levels.

#### External logging configuration

External logging may be configured using the `--log-config-file` option.
This option takes a single configuration file, whose format depends on the actual external logging library being used.

As an example, to pass `some_log.config` configuration file to `some_ros_executable`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --log-config-file some_log.config
```

#### Enabling and disabling logging

Logging to `rosout`, `stdout` and an external logging library can be independently enabled or disabled.

As an example, to disable logging to `rosout` and `stdout` but not to an external logging library for `some_ros_executable`, one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --disable-rosout-logs --disable-stdout-logs --enable-external-lib-logs
```

Logging is fully enabled by default, thus `--enable-*` options are usually redundant unless a `--disable-*` option found earlier in the command line is being overridden.

#### Enclave assignments

Enclave assignment may be achieved using the `--enclave`/`-e` option.
This option takes a single string `value` assignment statement, where `value` is a fully qualified enclave path used to locate the respective security artifacts within the configured keystore.

As an example, to assign an enclave path `/foo/bar` one may execute:

```sh
ros2 run some_package some_ros_executable --ros-args --enclave="/foo/bar"
```

or its shorter equivalent:

```sh
ros2 run some_package some_ros_executable --ros-args -e "/foo/bar"
```

As is, this enclave assignment applies to each and every Domain Participant that `some_ros_executable` spawns unless explicitly ignored in code or overridden via security environment variables.  

## Implementation

### Extraction

Command line argument extraction happens within `rcl`.
When an instance of the `--ros-args` flag is found in `argv`, until either a double dash token (`--`) is found or the end of the argument array is reached, all arguments that follow are taken as ROS specific arguments to be parsed as such.
Remaining arguments can still be accessed by the user via `rcl` API.

### Parsing

At the time of writing, most ROS specific arguments target and are thus parsed by `rcl`.
This is the case for name remapping rules or parameter assignments flags, to name a few.
However, to support ROS specific arguments that target upper ROS layers e.g. a ROS client library like `rclcpp`, arguments unknown to `rcl` are left unparsed but accessible by these layers, which in turn can continue parsing or eventually warn the user if unknown arguments remain.

## Alternative designs

Other, alternative designs were under discussion.

### Additional operators

Stop using the same `:=` operator for parameter assignments and name remapping rules and introduce additional operators e.g. `:=` for parameter assignment and `~=` for name remapping.
This keeps the command line verbosity at a minimum and avoids the need for flags, but is error prone.

### Full name addressing

Rely on full name addressing to disambiguate operator significance e.g. `rosparam://this:=that` would result in a `that` string value being assigned to parameter `this` while `rosremap://this:=that` would result in name `this` being remapped to name `that`.
Other URL schemes, specific to each interface type e.g. `rostopic` and `rosservice`, may also be used to further scope remapping rules.
This signficantly increases command line verbosity, but still avoids the need for flags.

### Prefixed option names

Remove the need for double dash tokens (`--`), conventionally used to signify the end of CLI options for a command, by adding the `--ros-` prefix to all ROS specific command line flags e.g. `--ros-remap`, `--ros-param`, etc.
In exchange, it makes argument extraction slightly more difficult as all options must be known ahead of time, whereas `--ros-args`-based namespacing can achieve the same with a couple rules.
It also increases command line verbosity.
