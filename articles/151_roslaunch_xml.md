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

  <xs:element name="launch">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        The root element of a launch file.
      </xs:documentation>
    </xs:annotation>

    <xs:choice maxOccurs="unbounded">
      <xs:element ref="arg"/>
      <xs:element ref="node"/>
      <xs:element ref="include"/>
      <xs:element ref="group"/>
    </xs:choice>
  </xs:element>

  <xs:element name="arg">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Declares launch file-level arguments.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="name" type="xs:string" use="required"/>
      <xs:attribute name="value" type="xs:string" use="optional"/>
      <xs:attribute name="default" type="xs:string" use="optional"/>
      <xs:attribute name="doc" type="xs:string" use="optional"/>
    </xs:complexType>
  </xs:element>

  <xs:element name="include">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Includes another launch file.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:attribute name="file" type="xs:string" use="required"/>
      <xs:attribute name="ns" type="xs:string" use="optional"/>
      <xs:attribute name="if" type="xs:string" use="optional"/>
      <xs:attribute name="unless" type="xs:string" use="optional"/>
      <xs:sequence maxOccurs="unbounded">
        <xs:element ref="arg"/>
      </xs:sequence>
    </xs:complexType>
  </xs:element>

  <xs:element name="group">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Groups (and optionally scopes) a set of actions.
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:choice maxOccurs="unbounded">
        <xs:element ref="node"/>
        <xs:element ref="env"/>
        <xs:element ref="include"/>
      </xs:choice>
      <xs:attribute name="ns" type="xs:string" use="optional"/>
      <xs:attribute name="scoped" type="xs:boolean" use="optional"/>
      <xs:attribute name="if" type="xs:string" use="optional"/>
      <xs:attribute name="unless" type="xs:string" use="optional"/>
    </xs:complexType>
  </xs:element>

  <xs:element name="node">
    <xs:annotation>
      <xs:documentation xml:lang="en">
        Executes a ROS node (as local OS process).
      </xs:documentation>
    </xs:annotation>

    <xs:complexType>
      <xs:choice maxOccurs="unbounded">
        <xs:element name="env">
          <xs:annotation>
            <xs:documentation xml:lang="en">
              Modifies a node's process environment.
            </xs:documentation>
          </xs:annotation>
          
          <xs:complexType>
            <xs:attribute name="name" type="xs:string" use="required"/>
            <xs:attribute name="value" type="xs:string" use="required"/>
            <xs:attribute name="if" type="xs:string" use="optional"/>
            <xs:attribute name="unless" type="xs:string" use="optional"/>
          </xs:complexType>
        </xs:element>
        
        <xs:element name="param">
          <xs:annotation>
            <xs:documentation xml:lang="en">
              Sets a ROS parameter for the enclosing node to either
              a scalar value or a sequence of scalar values.
              TODO(hidmic): Not ROS 1 compatible, revisit?
            </xs:documentation>
          </xs:annotation>

          <xs:complexType>
            <xs:simpleContent>
              <xs:extension base="xs:string">
                <xs:attribute name="name" type="xs:string" use="required"/>
                <xs:attribute name="sep" type="xs:string" use="optional"/>
              </xs:extension>
            </xs:simpleContent>
          </xs:complexType>
        </xs:element>
        <xs:element name="params">
          <xs:annotation>
            <xs:documentation xml:lang="en">
              Sets a sequence of ROS parameters for the enclosing node as read from
              a given YAML file or enclosed by this very same tag, optionally scoping
              them.
            </xs:documentation>
          </xs:annotation>

          <xs:complexType>
            <xs:choice maxOccurs="unbounded">
              <xs:element ref="params"/>
              <xs:element ref="param"/>
            </xs:choice>
            <xs:attribute name="name" type="xs:string" use="optional"/>
            <xs:attribute name="from" type="xs:string" use="optional"/>
          </xs:complexType>
        </xs:element>
        <xs:element name="remap">
          <xs:annotation>
            <xs:documentation xml:lang="en">
              Remaps a ROS name for the enclosing node.
            </xs:documentation>
          </xs:annotation>

          <xs:complexType>
            <xs:attribute name="from" type="xs:string" use="required"/>
            <xs:attribute name="to" type="xs:string" use="required"/>
          </xs:complexType>
        </xs:element>
      </xs:choice>
    </xs:complexType>
    <xs:attribute name="package" type="xs:string" use="required"/>
    <xs:attribute name="executable" type="xs:string" use="required"/>
    <xs:attribute name="name" type="xs:string" use="optional"/>
    <xs:attribute name="args" type="xs:string" use="optional"/>
    <xs:attribute name="ns" type="xs:string" use="optional"/>
    <xs:attribute name="launch-prefix" type="xs:string" use="optional"/>
    <xs:attribute name="output" use="optional">
      <xs:simpleType>
        <xs:restriction base="xs:string">
          <xs:enumeration value="log" />
          <xs:enumeration value="screen" />
        </xs:restriction>
      </xs:simpleType>
    </xs:attribute>
    <xs:attribute name="if" type="xs:string" use="optional"/>
    <xs:attribute name="unless" type="xs:string" use="optional"/>
  </xs:element>
</xs:schema> 
```

### Tags Semantics

#### All Tags

Every tag's execution can be conditioned on a boolean predicate via `if` or `unless` attributes.

#### <launch> Tag

Root tag of any launch file.

#### <include> Tag

The <include> tag allows for bringing a launch file description into another, enabling re-use of hierarchical launch layouts.
The included launch file description has its own scope for launch configurations.
To avoid ROS name clashes, included nodes can be namespaced via the `ns` attribute.

#### Examples

```xml
<include file="/opt/my_launch_file.xml" ns="/my_ns"/>
<include file="/opt/my_other_launch_file.xml">
  <arg name="some_argument" value="dummy_value"/>
</include>
```

#### <group> Tag

#### Description

The <group> tag allows for launch actions' grouping as well as optional launch file configuration scoping.

#### Examples

```xml
<group ns="/dummy_group" scoped="true">
  <node package="a_ros_package" name="dummy0" executable="dummy_node"/>
  <node package="a_ros_package" name="dummy1" executable="dummy_node"/>
</group>
```

#### <arg> Tag

##### Description

The <arg> tag allows for launch file configuration via the command-line or when including it via an <include> tag.
Arguments are limited to the scope of their definition and thus, have to be explictly passed to included files if any.

##### Examples

```xml
<arg name="publish_frequency" default="10"/>
<arg name="output_path" doc="Output path for some processing pipeline"/>
```

#### <node> Tag

##### Description

The <node> tag allows for executing a ROS node in the form of local OS process.

##### Examples

```xml
<node package="ros_demos" executable="publisher">
  <param name="publish_frequency">10</param>
  <remap from="generic_topic_name" to="my_topic"/>
</node>
```

#### <param> Tag

##### Description

The <param> tag allows for setting a ROS parameter of a ROS node.

##### Examples

```xml
<param name="some_numeric_param">100.0</param>
<param name="some_numeric_param" sep=",">Some phrase,100.0,true</param>
```

#### <params> Tag

##### Description

The <params> tag allows to either bring ROS parameters from a YAML parameters file or to nest a <param> definitions under an appropriate name (i.e. to make a map of them).

##### Examples

```xml
<params from="path/to/param/file.yml"/>
<params name="some_param_group">
  <param name="some_integer_param">10</param>
</params>
```

#### <remap> Tag

##### Description

The <remap> tag allows for ROS name remapping of a <node> instance.

##### Examples

```xml
<remap from="chatter" to="my_chatter"/>
```

#### <env> Tag

##### Description

The <env> tag allows for modifying a <node>'s OS process environment.

##### Examples

```xml
<env name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"/>
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

`$(arg name)`
: Substituted by the value of the named argument. 
  Substitution will fail if the named argument does not exist.

`$(env env-var)`
: Substituted by the value of the given environment variable.
  Substitution will fail if the variable is not set.

`$(dirname)`
: Substituted by the current launch file directory name.
  Substitution will always succeed.

#### User-defined Substitutions

TBD
