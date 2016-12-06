---
layout: default
title: Generated Python interfaces
permalink: articles/generated_interfaces_python.html
abstract:
  This article describes the generated Python code for ROS 2 interfaces.
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

This article specifies the generated Python code for ROS interface types defined in the [interface definition article](interface_definition.html).

## Namespacing

All code of a ROS package should be defined in a Python package with the same name.
To separate the generated code from other code within the package it is defined in a sub module:

- module for ROS messages: `<package_name>.msg`.
- module for ROS services: `<package_name>.srv`.

<div class="alert alert-warning" markdown="1">
  <b>NOTE:</b> The names are currently identical to the ones used in ROS 1.
  Therefore it is not possible to import both in a Python application.
</div>

## Generated files

Following the Python conventions the namespace hierarchy is mapped to a folder structure.
The package and module names and therefore the folder and file names use lowercase alphanumeric characters with underscores for separating words (following PEP 8).
Python files end with `.py`.

## Messages

For a message a Python `class` with the same name as the message is generated in the file `_<my_message_name>.py`.
The Python module `<package_name>.msg` / `<package_name>.srv` exports all message / service classes without the message module name to shorten import statements, e.g. `import <package_name>.msg.<MyMessageName`

### Types

#### Mapping of primitive types

| ROS type | Python type                  |
| -------- | ---------------------------- |
| bool     | builtins.bool                |
| byte     | builtins.bytes with length 1 |
| char     | builtins.str with length 1   |
| float32  | builtins.float               |
| float64  | builtins.float               |
| int8     | builtins.int                 |
| uint8    | builtins.int                 |
| int16    | builtins.int                 |
| uint16   | builtins.int                 |
| int32    | builtins.int                 |
| uint32   | builtins.int                 |
| int64    | builtins.int                 |
| uint64   | builtins.int                 |
| string   | builtins.str                 |

#### Mapping of arrays and bounded strings

| ROS type                | Python type   |
| ----------------------- | ------------- |
| static array            | builtins.list |
| unbounded dynamic array | builtins.list |
| bounded dynamic array   | builtins.list |
| bounded string          | builtins.str  |

### Properties

The class has same-named properties for every field of the message.
The setter of a property will ensure that ROS type constraints beyond the Python type are enforced.

### "Constants"

A constant is defined as a class variable with uppercase name.
The class variable is considered to be *read-only* which is ensured by overridding the setter or a magic method.

### Constructors

The *__init__* function initializes all members with their default value.

The class constructor supports only keyword arguments for all members.
The short reason for this is that if code would rely on positional arguments to construct data objects changing a message definition would break existing code in subtle ways.
Since this would discourage evolution of message definitions the data structures should be populated by setting the members separately, e.g. using the setter methods.

### Magic methods

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> decide which magic methods should be provided
</div>

## Services

For a service a `class` with the same name is generated.

The class contains only two `types`:

- `Request` which is the type of the request part of the service
- `Response` which is the type of the request part of the service

The generated code is split across multiple files the same way as message are.

### Request and response messages

For the request and response parts of a service separate messages are being generated.
These messages are named after the service and have either a `_Request` or `_Response` suffix.
They are are still defined in the `srv` sub namespace.
