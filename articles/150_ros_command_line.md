---
layout: default
title: ROS Command Line
permalink: articles/ros_command_line.html
abstract:
  This article describes ROS 2 nodes command line capabilities and its syntax.
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
However, a larger set of configuration options, an ambiguity in remapping rules and parameter assignment syntax (as a result of the leading underscore name convention for hidden resources), a one-to-many relationship between executables and nodes, among others, add up complexity to an otherwise simple interface.

Increasingly precise addressing mechanisms, as well as leading double underscores (`__`) in positional arguments, solve part of the problem.
In ROS 2, however, command line options are introduced too as these are:

- Explicit and less error prone.
- Suitable for tab completion.

Unfortunately, additional devices must be put in place as these coexist with user-defined options -- part of the rationale behind ROS 1 command line design.

## Namespacing

To prevent ROS specific command line options from interacting with user-defined ones, the former are scoped using the `--ros-args` option and a trailing double dash token (`--`):

```sh
ros2 run some_package my_node [<user-defined-pre-arg0>...] \
  --ros-args <ros-specific-arg-0> [<ros-specific-arg-1>...] -- <user-defined-arg-1> [<user-defined-arg-2>...]
```

If no user defined arguments are provided after ROS specific arguments are, the double dash token (`--`) may be elided:

```sh
ros2 run some_package my_node [<user-defined-arg-0>...] --ros-args <ros-specific-arg-0> [<ros-specific-arg-1>...]
```

## Capabilities

### Summary

As a quick summary of ROS command line capabilities:

- For name remapping, use either `--remap from:=to` or `-r from:=to`.
- For parameter assignment, use either `--param name:=value` or `-p name:=value`.

### Name remapping rules

Remapping rules may be introduced using the `--remap`/`-r` option.
This option takes a single `from:=to` remapping rule.

As an example, to remap from `foo` to `bar` upon running `some_node`, one may execute:

```sh
ros2 run some_package some_node --ros-args --remap foo:=bar
```

or its shorter equivalent:

```sh
ros2 run some_package some_node --ros-args -r foo:=bar
```

### Parameter assignment statements

Parameter assignment may be achieved using the `--param`/`-p` option.
This option takes a single `name:=value` assignment statement, where `value` is in YAML format.

As an example, to assign an integer value of `2` to an `int_param` upon running `some_node`, one may execute:

```sh
ros2 run some_package some_node --ros-args --param int_param:=2
```

or its shorter equivalent:

```sh
ros2 run some_package some_node --ros-args -p int_param:=2
```

## Alternative designs

Other, alternative designs were under discussion.

### Additional operators

Stop using the same `:=` operator for parameter assignments and name remapping rules and introduce additional operators e.g. `:=` for parameter assignment and `~=` for name remapping. This keeps the command line verbosity at a minimum, but is error prone.

### Full name addressing

Rely on full name addressing to disambiguate operator significance e.g. `rosparam://this:=that` would mean to assign a `that` string value to parameter `this` while `rostopic://this:=that` would mean to remap topic `this` to `that`. This signficantly increases the command line verbosity.

### Prefixed option names

Remove the need for double dash tokens (`--`), conventionally used to signify the end of CLI options for a command, by adding the `--ros-` prefix to all ROS specific command line options e.g. `--ros-remap`, `--ros-param`, etc. This somewhat increases the command line verbosity.
