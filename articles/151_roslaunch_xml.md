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

The choice of XML over other markup languages aims to ease the bridge between ROS and ROS 2 launch files.
Widely available support for parsing XML in a myriad of languages and platforms also weighed (significantly) in the decision.

## Static Description

### Schema Definition

```xml
<?xml version="1.0"?>
<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema">
  <xs:annotation>
    <xs:documentation xml:lang="en">
      ROS 2 launch XML schema v0.1.0
    </xs:documentation>
  </xs:annotation>

  <xs:element name="launch" maxOccurs="1">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        The root element of a launch file.
      </xs:documentation>
    </xs:annotation>

    <xs:choice minOccurs="1" maxOccurs="unbounded">
      <xs:annotation>
        <xs:documentation xml:lang="en">
          A collection of actions to be launched in order of appearance, plus
          launch arguments for callers to provide, either through a tool or
          by inclusion.
        </xs:documentation>
      </xs:annotation>

      <xs:element name="arg">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Declares launch file-level arguments.
          </xs:documentation>
        </xs:annotation>

        <xs:complexType>
          <xs:attribute name="name" type="xs:string" use="required">
            <xs:annotation>
              <xs:documentation xml:lang="en">
                Name of the launch argument.
              </xs:documentation>
            </xs:annotation>
          </xs:attribute>
          <xs:attribute name="value" type="xs:string" use="optional">
            <xs:annotation>
              <xs:documentation xml:lang="en">
                Fixed value for the launch argument, disables overrides.
              </xs:documentation>
            </xs:annotation>
          </xs:attribute>
          <xs:attribute name="default" type="xs:string" use="optional">
            <xs:annotation>
              <xs:documentation xml:lang="en">
                Default value for the launch argument, used if non is provided.
              </xs:documentation>
            </xs:annotation>
          </xs:attribute>
          <xs:attribute name="description" type="xs:string" use="optional">
            <xs:annotation>
              <xs:documentation xml:lang="en">
                Brief description of the launch argument.
              </xs:documentation>
            </xs:annotation>
          </xs:attribute>
        </xs:complexType>
      </xs:element>
      <xs:element ref="let"/>
      <xs:element ref="node"/>
      <xs:element ref="executable"/>
      <xs:element ref="include"/>
      <xs:element ref="group"/>
    </xs:choice>
  </xs:element>

  <xs:element name="let">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Defines a launch configuration variable.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="var" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Name of the launch configuration variable.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="value" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Value for the launch configuration variable.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="include">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Includes another launch file, either descriptive or programmatic.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="file" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Path to the launch file to be included.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="ns" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A ROS namespace to scope included ROS entities launched by
            actions in the included launch file, if any.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="if" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition launch inclusion i.e. only do
            so if the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="unless" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition launch inclusion i.e. do
            so unless the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:sequence minOccurs="0" maxOccurs="unbounded">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Arguments for the included launch file, if any.
          </xs:documentation>
        </xs:annotation>

        <xs:element name="arg">
          <xs:annotation>
            <xs:documentation xml:lang="en">
              An included launch file argument provision.
            </xs:documentation>
          </xs:annotation>

          <xs:complexType>
            <xs:attribute name="name" type="xs:string" use="required">
              <xs:annotation>
                <xs:documentation xml:lang="en">
                  Name of the launch argument.
                </xs:documentation>
              </xs:annotation>
            </xs:attribute>
            <xs:attribute name="value" type="xs:string" use="required">
              <xs:annotation>
                <xs:documentation xml:lang="en">
                  Value for the launch argument.
                </xs:documentation>
              </xs:annotation>
            </xs:attribute>
          </xs:complexType>
        </xs:element>
      </xs:sequence>
    </xs:complexType>
  </xs:element>

  <xs:element name="group">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Groups and optionally scopes a set of actions.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:choice minOccurs="1" maxOccurs="unbounded">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            The actions that make up the group.
          </xs:documentation>
        </xs:annotation>
        <xs:element ref="executable"/>
        <xs:element ref="node"/>
        <xs:element ref="group"/>
        <xs:element ref="include"/>
        <xs:element ref="let"/>
      </xs:choice>
      <xs:attribute name="ns" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A ROS namespace to scope ROS entities launched by
            grouped actions.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="scoped" type="xs:boolean" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Whether the group is a scoping one launch configuration-wise or not.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="if" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition group launch i.e. only do
            so if the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="unless" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition group launch i.e. do
            so unless the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="env">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Modifies an executable process environment.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="name" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Name of the environment variable to be set.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="value" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Value to be set for the environment variable.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="if" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition environment modification i.e.
            only do so if the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="unless" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition environment modification i.e.
            do so unless the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="executable">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Executes an executable as a local OS process.
      </xs:documentation>
    </xs:annotation>
    <xs:complexType>
      <xs:choice minOccurs="0" maxOccurs="unbounded">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A collection of environment variable settings for the
            launched executable.
          </xs:documentation>
        </xs:annotation>

        <xs:element ref="env"/>
      </xs:choice>
      <xs:attribute name="cmd" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Path to the executable or a command-line if a
            shell is used to launch.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="cwd" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            The working directory for the launched process.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="name" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A name or label for the launched executable.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="args" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Additional 'command-line' arguments for the executable.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="shell" type="xs:boolean" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Whether to use a shell to launch or not.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="launch-prefix" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A prefix for the executable command-line if a shell is used to launch.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="output" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Output type for the launched executable.
          </xs:documentation>
        </xs:annotation>
        <xs:simpleType>
          <xs:restriction base="xs:string">
            <xs:enumeration value="log">
              <xs:annotation>
                <xs:documentation xml:lang="en">
                  Executable output goes to a log file.
                </xs:documentation>
              </xs:annotation>
            </xs:enumeration>
            <xs:enumeration value="screen">
              <xs:annotation>
                <xs:documentation xml:lang="en">
                  Executable output goes to stdout.
                </xs:documentation>
              </xs:annotation>
            </xs:enumeration>
          </xs:restriction>
        </xs:simpleType>
      </xs:attribute>
      <xs:attribute name="if" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition executable launch i.e. only do
            so if the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="unless" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition executable launch i.e. do so unless
            the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="param">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Sets a ROS parameter for the enclosing node to either
        a scalar value or a sequence of scalar values delimited
        by a known separator.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="name" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Name of the ROS parameter to be set.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="value" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Value or list of values to set the the ROS parameter with.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="sep" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Value separator when dealing with list of values.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="params">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Sets a sequence of ROS parameters for a node as read from a parameters
        file or as child elements, optionally namespacing them.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:choice minOccurs="0" maxOccurs="unbounded">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A collection of ROS parameters to be set.
          </xs:documentation>
        </xs:annotation>

        <xs:element ref="params"/>
        <xs:element ref="param"/>
      </xs:choice>
      <xs:attribute name="ns" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Namespace for the ROS parameters to be set.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="from" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Path to the parameters file to be loaded.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="remap">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Remaps ROS names for a node.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="from" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Name matching expression to look for replacement candidates.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="to" type="xs:string" use="required">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Name replacement expression to replace candidates found.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
    </xs:complexType>
  </xs:element>

  <xs:element name="node">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Executes a ROS node executable in a local OS process.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:choice minOccurs="0" maxOccurs="unbounded">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A collection of ROS parameters, ROS remappings and/or
            environment variable settings for the launched ROS node.
          </xs:documentation>
        </xs:annotation>

        <xs:element ref="env"/>
        <xs:element ref="param"/>
        <xs:element ref="params"/>
        <xs:element ref="remap"/>
      </xs:choice>
    </xs:complexType>
    <xs:attribute name="package" type="xs:string" use="required">
      <xs:annotation>
        <xs:documentation xml:lang="en">
          Name of the package where the node is to be found.
        </xs:documentation>
      </xs:annotation>
    </xs:attribute>
    <xs:attribute name="executable" type="xs:string" use="required">
      <xs:annotation>
        <xs:documentation xml:lang="en">
          Name of the node executable.
        </xs:documentation>
      </xs:annotation>
    </xs:attribute>
    <xs:attribute name="name" type="xs:string" use="optional">
      <xs:annotation>
        <xs:documentation xml:lang="en">
          A name for the launched ROS node.
        </xs:documentation>
      </xs:annotation>
    </xs:attribute>
      <xs:attribute name="args" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Additional 'command-line' arguments for the ROS node.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="ns" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A ROS namespace to scope the launched ROS node.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="launch-prefix" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A prefix for the ROS node command-line used for launch.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="output" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            Output type for the launched ROS node.
          </xs:documentation>
        </xs:annotation>
        <xs:simpleType>
          <xs:restriction base="xs:string">
            <xs:enumeration value="log">
              <xs:annotation>
                <xs:documentation xml:lang="en">
                  ROS node output goes to a log file.
                </xs:documentation>
              </xs:annotation>
            </xs:enumeration>
            <xs:enumeration value="screen">
              <xs:annotation>
                <xs:documentation xml:lang="en">
                  ROS node output goes to stdout.
                </xs:documentation>
              </xs:annotation>
            </xs:enumeration>
          </xs:restriction>
        </xs:simpleType>
      </xs:attribute>
      <xs:attribute name="if" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition ROS node launch i.e. only do
            so if the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
      <xs:attribute name="unless" type="xs:string" use="optional">
        <xs:annotation>
          <xs:documentation xml:lang="en">
            A predicate to condition ROS node launch i.e. do so unless
            the predicate evaluates to a truthy value.
          </xs:documentation>
        </xs:annotation>
      </xs:attribute>
  </xs:element>
</xs:schema>
```

### Tags Semantics

#### All Tags

Every tag's execution can be conditioned on a boolean predicate via `if` or `unless` attributes.
All action tags that can contain other action tags can scope ROS entities in a namespace via the `ns` attribute.

#### `<launch>` Tag

Root tag of any launch file.
There must only be one `<launch>` tag on a given launch file.

#### `<include>` Tag

##### Description

The `<include>` tag allows for bringing a launch file description into another, enabling re-use of hierarchical launch layouts.
The included launch file description has its own scope for launch configurations.
The included launch file description is not necessarily written in this format nor a declarative one (i.e. a programmatic description).
To avoid ROS name clashes, included nodes can be namespaced via the `ns` attribute.

##### Examples

```xml
<include file="/opt/my_launch_file.py" ns="/my_ns"/>
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
<group ns="dummy_group" scoped="true">
  <node package="a_ros_package" name="dummy0" executable="dummy_node"/>
  <node package="a_ros_package" name="dummy1" executable="dummy_node"/>
</group>
```

Namespaces for groups are usually relative to allow further namespacing by other launch files including the one the group is in.

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

The `<param>` tag allows for setting a ROS parameter of a ROS node.

##### Examples

```xml
<node package="my_pkg" executable="my_node">
  <param name="some_numeric_param" value="100.2"/>
  <param name="some_list_param" value="Some phrase,100.0,true" sep=","/>
</node>
```

#### `<params>` Tag

##### Description

The `<params>` tag allows to either bring ROS parameters from a YAML parameters file or to nest a `<param>` definitions under an appropriate name (i.e. to make a map of them).

##### Examples

```xml
<node package="my_pkg" executable="my_node">
  <params from="path/to/param/file.yml"/>
  <params ns="some_param_group">
    <param name="some_integer_param" value="10"/>
  </params>
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
