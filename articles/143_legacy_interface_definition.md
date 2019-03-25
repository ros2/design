---
layout: default
title: Legacy interface definition using .msg / .srv / .action files
permalink: articles/legacy_interface_definition.html
abstract:
  This article specifies the file format coming from ROS 1 describing the data structures exchanged by ROS components to interact with each other.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
categories: Interfaces
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Scope

This article specifies the legacy file format describing the data structures which are being used to exchange information between components.
The data structures are defined in a programming language agnostic way.
The format is based on the [<code>.msg</code> format definition](http://wiki.ros.org/msg#Message_Description_Specification) from ROS 1.

Below only the mapping to IDL types is described.
Please see the [Interface Definition and Language Mapping](idl_interface_definition.html) article for the mappings to programming language specific types and API.

## Overview

A data structure is defined by a set of *fields*.
The order of the fields is irrelevant.
Each field is described by a *type* and a *name*.

### Messages

A single data structure is called *message*.
Each message has a *name*.

### Services

For request / reply style communication the two exchanged data structures are related.
These pairs of data structures are called *services*.
Each service describes two messages, one for the request data structure, one for the reply data structure.

### Actions

For longer running request / reply style communication with feedback about the progress the exchanged data structures are related.
These triplets of data structures are called *actions*.
Each action describes three messages, one for the goal data structure, one for the result data structure, and one for the feedback data structure.

### Identifying data structures

Every data structure can be uniquely identified with three pieces of information:

1. **Package** - the name of the package containing the data structure definition.
2. **Subfolders** - the list of subfolders within the package where the data structure defintion can be found.
3. **Name** - the name of the data structure.

For example, a message with the name `Foo` in subfolder `msg` of the package `bar` has the unique identifier `bar/msg/Foo`.
Here, `/` is used as a separator, but in practice any delimeter could be used.

### Field types

The type of a field can be either a primitive type or another data structure.
Each of these can optionally be a dynamically or statically sized array.

#### Primitive field types

The following primitive types are defined:

- `bool`
- `byte`
- `char`
- `float32`, `float64`
- `int8`, `uint8`
- `int16`, `uint16`
- `int32`, `uint32`
- `int64`, `uint64`
- `string`

<div class="alert alert-info" markdown="1">
  While <code>byte</code> and <code>char</code> are deprecated in ROS 1 they are still part of this definition to ease migration.
</div>

<div class="alert alert-warning" markdown="1">
  In ROS 1 <code>string</code> does not specify any encoding and the transport is agnostic to it.
  This means commonly it can only contain ASCII.
  For explicit support of wide character strings please consider migrating to [.idl files](http://design.ros2.org/articles/idl_interface_definition.html) which defines explicit types for that.
</div>

#### Non-primitive field types

Beside the primitive types other messages can be referenced to describe the type of a "complex" field.
A complex field type is identified by a package and a message name.

#### Arrays with static size

A static array has exactly `N` elements of the specified type.
`N` must be greater than `0`.

#### Arrays with dynamic size

A dynamic array can have between `0` and `N` elements of the specified type.
`N` might not have an upper bound and may only be limited by the memory or other system specific limitations.

#### Upper boundaries

<div class="alert alert-info" markdown="1">
  This feature is not available in ROS 1.
</div>

The size of *strings* as well as *dynamic arrays* can be limited with an *upper boundary*.
This enables the preallocation of memory for data structures which use dynamically sized data.

### Default values

<div class="alert alert-info" markdown="1">
  This feature is not available in ROS 1.
</div>

A field can optionally specify a default value.
If no default value is specified a common default value is used:

- for `bool` it is `false`
- for *numeric* types it is a `0` value
- for `string` it is an *empty* string
- for *static size arrays* it is an array of N elements with its fields zero-initialized
- for *bounded size arrays* and *dynamic size arrays* it is an empty array `[]`

#### Array default values

A field of type `array` can optionally specify a default value.

- default values for an array must start with an opening square bracket (`[`) and end with a closing square bracket (`]`)
- each value within the array must be separated with a comma (`,`)
- all values in the array must be of the same type as the field
- there cannot be a comma `,` before the first value of the array
- a trailing comma after the last element of the array is ignored

Additional rule for `string` arrays:

- string arrays must contain only `string`s respecting the following rules:
  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)
  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> default values are currently not supported for <i>complex</i> fields
</div>

### Constants

Constants are defined by a *primitive type*, a *name* as well as a *fixed value*.

## Conventions

### Naming of messages and services

Each file contains a single message or service.
Message files use the extension `.msg`, service files use the extension `.srv`.

Both file names must use an upper camel case name and only consist of alphanumeric characters.

### Naming of fields

Field names must be lowercase alphanumeric characters with underscores for separating words.
They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

### Naming of constants

Constant names must be uppercase alphanumeric characters with underscores for separating words.
They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

## Syntax

The message and service definitions are text files.

### Comments

The character `#` starts a comment, which terminates at the end of the line on which it occurs.

### Message file format

A line can either contain a field definition or a constant definition.
While a single space is mandatory to separate tokens additional spaces can be inserted optionally between tokens.

#### Field definition

A field definition has the following structure:

    <type> <name> <optional_default_value>

#### Constant definition

A constant definition has the following structure:

    <type> <name>=<value>

#### Types

A `<type>` is defined by its *base type* and optional *array specifier*.

The *base type* can be one of the following:

- a primitive type from the list above: e.g. `int32`

- a string with an upper boundary: `string<=N` to limit the length of the string to `N` characters

- a complex type referencing another message:

  - an *absolute* reference of a message: e.g. `some_package/SomeMessage`
  - a *relative* reference of a message within the same package: e.g. `OtherMessage`

The *array specifier* can be one of the following:

- a static array is described by the suffix `[N]` where `N` is the fixed size of the array
- an unbounded dynamic array is described by the suffix `[]`
- a bounded dynamic array is described by the suffix `[<=N]`  where `N` is the maximum size of the array

#### Values

Depending on the type the following values are valid:

- `bool`:

  - `true`, `1`
  - `false`, `0`

- `byte`:

  - an opaque 8-bit quantity with a numerical value in the following interval `[0, 255]`

- `char`:

  - an unsigned integer value in the following interval `[0, 255]`

- `float32` and `float64`:

  - a decimal number using a dot (`.`) as the separator between the integer-part and fractional-part.

- `int8`, `int16`, `int32` and `int64`:

  - an integer value in the following interval `[- 2 ^ (N - 1), 2 ^ (N - 1) - 1]` where `N` is the number of bits behind `int`

- `uint8`, `uint16`, `uint32` and `uint64`:

  - an unsigned integer value in the following interval `[0, 2 ^ N - 1]` where `N` is the number of bits behind `uint`

- `string`:

  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)

  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped:

    - `string my_string "I heard \"Hello\""` is valid
    - `string my_string "I heard "Hello""` is **not** valid
    - `string my_string "I heard 'Hello'"` is valid
    - `string my_string 'I heard \'Hello\''` is valid
    - `string my_string 'I heard 'Hello''` is **not** valid
    - `string my_string 'I heard "Hello"'` is valid

### Service file format

A service file contains two message definitions which are separated by a line which only contains three dashes:

    ---

## Conversion to IDL
Code is generated for defined interfaces to be usable by different client libraries.
Interfaces described using the legacy format are first converted to [IDL](idl_interface_definition.html).
Code generation uses the generated file.

### Mapping to IDL types

| ROS type | IDL type           |
| -------- | ------------------ |
| bool     | boolean            |
| byte     | octet              |
| char     | uint8              |
| float32  | float              |
| float64  | double             |
| int8     | int8               |
| uint8    | uint8              |
| int16    | short              |
| uint16   | unsigned short     |
| int32    | long               |
| uint32   | unsigned long      |
| int64    | long long          |
| uint64   | unsigned long long |
| string   | string             |

<div class="alert alert-info" markdown="1">
  The mapping of <code>byte</code> uses a different type than in ROS 1 while still remaining an opaque 8-bit quantity.
  [Definition in ROS 1](http://wiki.ros.org/msg#Field_Types): deprecated alias for <code>int8</code>.
  Definition in IDL (7.4.1.4.4.2.6): an opaque 8-bit quantity.
</div>

<div class="alert alert-info" markdown="1">
  While the mapping of <code>char</code> is unintuitive it preserves compatibility with the [definition in ROS 1](http://wiki.ros.org/msg#Field_Types):
  "deprecated alias for <code>uint8</code>".
</div>

| ROS type                | IDL type         |
| ----------------------- | ---------------- |
| static array            | array            |
| unbounded dynamic array | sequence         |
| bounded dynamic array   | bounded sequence |
| bounded string          | bounded string   |
