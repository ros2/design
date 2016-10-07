---
layout: default
title: Interface definition
permalink: articles/interface_definition.html
abstract:
  This article specifies the file format describing the data structures exchanged by ROS 2 components to interact with each other.
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

This article specifies the file format describing the data structures which are being used to exchange information between components.
The data structures are defined in a programming language agnostic way.
Please see other articles for the mappings to programming language specific types and API.

## Overview

A data structure is defined by a set of *fields*.
The order of the fields is irrelevant.
Each field is described by a *type* and a *name*.

### Messages

A single data structure is called *message*.
Each message has a *name*.
Together with the name of the *package* a message can be uniquely identified.

### Services

For request / reply style communication the two exchanged data structures are related.
These pairs of data structures are called *services*
A service is identified by its *name* and the *package* it is in.
Each service describes two messages, one for the request data structure, one for the reply data structure.

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

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> consider <code>wchar</code>, <code>wstring</code>, <code>u16string</code>, <code>u32string</code>
</div>

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> <code>string</code> does not specify any encoding yet and the transport is agnostic to it, this means commonly it can only contain ASCII but all endpoints can also "agree" on using a specific encoding
</div>

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> consider removing <code>byte</code>, <code>char</code> after specifying the mapping to C++ and Python
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

The size of *strings* as well as *dynamic arrays* can be limited with an *upper boundary*.
This enables the preallocation of memory for data structures which use dynamically sized data.

### Default values

A field can optionally specify a default value.
If no default value is specified a common default value is used:

- for `bool` it is `false`
- for *numeric* types it is a `0` value
- for `string` it is an *empty* string

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> default values are currently not supported for <code>string array</code> fields and <i>complex</i> fields
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

  - an unsigned integer value in the following interval `[0, 255]`

- `char`:

  - an integer value in the following interval `[-128, 127]`

- `float32` and `float64`:

  - a decimal number using a dot (`.`) as the separator between the integer-part and fractional-part.

- `int8`, `int16`, `int32` and `int64`:

  - an integer value in the following interval `[- 2 ^ (N - 1), 2 ^ (N - 1) - 1]` where `N` is the number of bits behind `int`

- `uint8`, `uint16`, `uint32` and `uint64`:

  - an unsigned integer value in the following interval `[0, 2 ^ N - 1]` where `N` is the number of bits behind `uint`

- `string`:

  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)

### Service file format

A service file contains two message definitions which are separated by a line which only contains three dashes:

    ---
