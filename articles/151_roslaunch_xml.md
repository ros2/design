---
layout: default
title: ROS 2 Launch XML Format
permalink: articles/roslaunch_xml.html
abstract:
  The XML format for declarative launch descriptions in the ROS 2 launch system.
author: '[Michel Hidalgo](https://github.com/hidmic)'
published: false
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}


# ROS 2 Launch XML Format v0.1.0

## Rationale

As an alternative to a programmatic approach to the ROS 2 launch system's API, a declarative description features a WYSIWYG approach, easier to read, audit and maintain.
This is the preferred approach for ROS 1 `roslaunch` launch files, thus some degree of familiarity is expected (and relied upon).

The body of such a description is mainly comprised of statically declared launch actions with a prescribed configuration.
To that, runtime value substitution is added in order to fullfill common dynamic (re)configuration needs like conditional execution, resource lookups, etc.
It is intended for these entities to map to those of the underlying implementation, reducing support to file parsing.

This article describes XML aiming to ease the bridge between ROS and ROS 2 launch files.
YAML is currently supported too, and other markup languages could be added.

## Static Description

### Schema Definition

```xml
{% include_relative specs/launch.0.1.0.xsd %}
```

### Tags Semantics

#### All Tags

Every tag's execution can be conditioned on a boolean predicate via `if` or `unless` attributes.

#### `<launch>` Tag

Root tag of any launch file.
There must only be one `<launch>` tag on a given launch file.

#### `<include>` Tag

##### Description

The `<include>` tag allows for bringing a launch file description into another, enabling re-use of hierarchical launch layouts.
The included launch file description has its own scope for launch configurations.
The included launch file description is not necessarily written in this format nor a declarative one (i.e. a programmatic description).

##### Examples

```xml
<include file="/opt/my_launch_file.py"/>
<include file="$(find-pkg my_pkg)/launch/some_launch_file.xml"/>
<include file="/opt/my_other_launch_file.xml">
  <arg name="some_argument" value="dummy_value"/>
</include>
```

#### `<group>` Tag

##### Description

The `<group>` tag allows for launch actions' grouping as well as optional launch file configuration scoping.

##### Examples

```xml
<group scoped="true">
  <node package="a_ros_package" name="dummy0" ns="my_ns" executable="dummy_node"/>
  <node package="a_ros_package" name="dummy1" executable="dummy_node"/>
</group>
```

#### `<let>` Tag

##### Description

The `<let>` tags allows for definition of scoped launch file configuration variables.

##### Examples

```
<let var="foo" value="$(env BAR)"/>
<let var="baz" value="false"/>
```

#### `<arg>` Tag

##### Description

The `<arg>` tag allows for launch file configuration via the command-line or when including it via an `<include>` tag.
Arguments are launch configuration variables just like the ones defined by `<let>` tags.
Arguments are limited to the scope of their definition and thus have to be explictly passed to included files if any.

##### Examples

```xml
<arg name="publish_frequency" default="10"/>
<arg name="output_path" description="Output path for some processing pipeline"/>
```

#### `<node>` Tag

##### Description

The `<node>` tag allows for executing a ROS node in the form of local OS process.

##### Examples

```xml
<node package="ros_demos" executable="publisher">
  <param name="publish_frequency" value="10"/>
  <remap from="generic_topic_name" to="my_topic"/>
</node>
```

#### `<param>` Tag

##### Description

The `<param>` tag allows for setting ROS parameters of a ROS node. These
can be scalar values or sequences of scalar values, defined directly or
either nested or brought from a YAML file to make a map.

##### Examples

```xml
<node package="my_pkg" executable="my_node">
  <param name="some_numeric_param" value="100.2"/>
  <param name="some_list_param" value="Some phrase,100.0,true" sep=","/>
  <param name="some_param_group">
    <param name="some_integer_param" value="10"/>
  </param>
  <param from="path/to/param/file.yml"/>
</node>
```

#### `<remap>` Tag

##### Description

The `<remap>` tag allows for ROS name remapping of a `<node>` instance.

##### Examples

```xml
<node package="my_pkg" executable="my_node">
  <remap from="chatter" to="/my_chatter"/>
  <remap from="*/stuff" to="private_\1/stuff"/>
</node>
```

#### `<env>` Tag

##### Description

The `<env>` tag allows for modifying an OS process environment.

##### Examples

```xml
<node package="my_pkg" executable="my_node">
  <env name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"/>
</node>

<executable cmd="ls" cwd="/var/log">
  <env name="LD_LIBRARY" value="/lib/some.so"/>
</executable>
```

## Dynamic Configuration

### Substitution Syntax

All substitutions are enclosed by `$(...)`.

### Built-in Substitutions

`$(find-pkg pkg-name)`
: Substituted by the path to package installation directory in the local filesystem.
  Forward and backwards slashes will be resolved to the local filesystem convention.
  Substitution will fail if the package cannot be found.

`$(find-exec exec-name)`
: Substituted by the path to the executable in the local filesystem.
  Lookups make use of the PATH environment variable.
  Forward and backwards slashes will be resolved to the local filesystem convention.
  Substitution will fail if the executable cannot be found.

`$(var name)`
: Substituted by the value of the launch configuration variable.
  Substitution will fail if the named argument does not exist.

`$(env env-var [default-value])`
: Substituted by the value of the given environment variable
  Substitution will fail if the variable is not set, unless default values are provided.

`$(dirname)`
: Substituted by the current launch file directory name.
  Substitution will always succeed.

#### User-defined Substitutions

TBD
