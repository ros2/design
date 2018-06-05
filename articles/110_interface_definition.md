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

#### Naming Conflicts

When defining field names it has been recommended to avoid language keywords and other reserved symbols.
However since we cannot be guaranteed to know all conflicts of potential future languages or implementations we must have a way to systematically deal with these conflicts without disrupting existing code bases and implementations.
To that end we will require that language bindings provide a reproducable name mangling mapping that will prevent conflicts with reserved keywords or other conflicts.

##### Background

This has come up specifically as we're adding support for Windows for ROS2.
`winnt.h` defines several macros that conflict with existing enumerations.
It is also expected to happen as support for new languages are added.
We cannot know all the potential future keywords and restrictions from a language which is selected to add support for in the future so we must have a generic solution which will allow future languages to be added without disrupting the existing usages.

##### Language Generator Requirements

In the case that a new language is added and there is a detected conflicting name for a field, the language specific generator is expected to implement a basic deconfliction mangling of the symbol to avoid the conflict.

Each generator will define the mangling procedure and provide a list of symbols for which the procedure will be applied.
This list should be all known keywords or otherwise conflicting symbols.

##### Example Deconfliction Policy

A example of a simple name mangling policy would be to append a trailing underscore for conflicting symbols.
So that would mean that `FOO` if declared as a conflicting symbol would be used in generated code as `FOO_`.

Clearly this would collide if someone defined `class` and `class_`, so slightly more unique string is recommended for clarity and uniqueness.
The mangling should be designed so that accidental collisions between names and mangled names cannot happen.
One easy way to do this would be to a language specific valid character not valid in the standard field name specification.


##### Table of deconfliction

The language support package will provide a yaml file with a list of all the protected keywords and their mangled version in a dictionary as key and value respectively.

For example if the language had conflicts for `FOO`, `new`, and `delete` it's yaml file would like like this:

```
FOO: FOO_
new: new_
delete: delete_
```

This yaml file will be considered the authority for deconflicting and should be exported in a way that will allow downstream code generators to automatically apply the deconfliction policy.

##### Stability
Each language generator will provide a standard name deconfliction policy so that users can use the symbols in a stable manner.

This table will remain stable so that codebases can rely on using the mangled symbols.
Appending to the table may be necessary if the first pass did not catch all the conflicting keywords.

The removal of any protected keywords should be avoided if at all possible as it will require all code using that field or constant to be updated.

##### Legacy Support

There are a number of known conflicts with the existing ROS1 codebase.
In these few cases we will undefine the matching C/C++ preprocessor macros which are conflicting in the definition and define the old value.
This will be done only inside the declaration header and the definition will be restored before returning to the user's code.
If you are using the legacy defines it's required to use the namespaced reference instead of using a shorthand version as those are know to conflict.

For an easy migration path.
The mangled version of the constants will be made available in ROS Melodic by manually defining them in parallel so that code can be ported and use the same constant values while the codebase is spanning ROS1 and ROS2 implementations.

It is recommended to move to the deconflicted version immediately.

Constants known to be at issue:

* `NO_ERROR`
* `DELETE`
* `ERROR`

#### Naming Constants

Constant names must be uppercase alphanumeric characters with underscores for separating words except the first character which should be a lowercase k.
They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

To avoid conflicts generally all constants should be prefixed with a `k`.
This will avoid almost all conflicts in all languages.
This is designed to follow the Google Style Guide: [https://google.github.io/styleguide/cppguide.html#Constant_Names](https://google.github.io/styleguide/cppguide.html#Constant_Names)

This should be used for newly defined constants.
When migrating messages from ROS1 if there are existing constants in use the new version should be defined side by side with the old name to provide backwards compatibility for a full deprecation cycle.

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
