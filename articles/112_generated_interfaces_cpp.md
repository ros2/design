---
layout: default
title: Generated C++ interfaces
permalink: articles/generated_interfaces_cpp.html
abstract:
  This article describes the generated C++ code for ROS 2 interfaces.
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

This article specifies the generated C++ code for ROS interface types defined in the [interface definition article](interface_definition.html).

## Namespacing

All code of a ROS package should be defined in a namespace named after the package.
To separate the generated code from other code within the package it is defined in a sub namespace:

- namespace for ROS messages: `<package_name>::msg`.
- namespace for ROS services: `<package_name>::srv`.

<div class="alert alert-info" markdown="1">
  <b>NOTE:</b> Using the additional sub namespace ensures that the symbols are different and don't overlap with the ROS 1 symbols.
  That allows to include both in a single compilation unit like the <code>ros1_bridge</code>.
</div>

## Generated files

Following the C++ style guide of ROS 2 the namespace hierarchy is mapped to a folder structure.
The filenames use lowercase alphanumeric characters with underscores for separating words and end with either `.hpp` or `.cpp`.

## Messages

For a message a templated `struct` with the same name followed by an underscore is generated.
The single template argument is the allocator for the data structure.

For ease of use there is a `typedef` with the same name as the message which uses a default allocator (e.g. `std::allocator`).

For each message two files are being generated:

- `<my_message_name>.hpp` currently only includes `<my_message_name>__struct.hpp`
- `<my_message_name>__struct.hpp` containing the definition of the struct

This allows to add additional files besides the one with the suffix `__struct` to provide additional functionality.
For each additional functionality it can be decided to include it from the first header file.

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> specify content of <code>&lt;my_message_name&gt;__traits.hpp</code> file
</div>

### Types

#### Mapping of primitive types

| ROS type | C++ type    |
| -------- | ----------- |
| bool     | bool        |
| byte     | uint8_t     |
| char     | char        |
| float32  | float       |
| float64  | double      |
| int8     | int8_t      |
| uint8    | uint8_t     |
| int16    | int16       |
| uint16   | uint16      |
| int32    | int32       |
| uint32   | uint32      |
| int64    | int64       |
| uint64   | uint64_t    |
| string   | std::string |

#### Mapping of arrays and bounded strings

| ROS type                | C++ type           |
| ----------------------- | ------------------ |
| static array            | std::array<T, N>   |
| unbounded dynamic array | std::vector<T>     |
| bounded dynamic array   | custom_class<T, N> |
| bounded string          | std::string        |

### Members

The struct has same-named public member variables for every field of the message.
For each field a `typedef` is created which is named after the member with a leading underscore and a trailing `_type`.

### Constants

Numeric constants are defined as `enums` within the struct.

All other constants are declared as `static const` members in the struct and their values are defined outside of the struct.

### Constructors

The *default constructor* initializes all members with their default value.
Optionally the constructor can be invoked with an allocator.

The struct has no constructor with positional arguments for the members.
The short reason for this is that if code would rely on positional arguments to construct data objects changing a message definition would break existing code in subtle ways.
Since this would discourage evolution of message definitions the data structures should be populated by setting the members separately, e.g. using the setter methods.

### Setters

For each field a *setter* method is generated to enable [method chaining](https://isocpp.org/wiki/faq/ctors#named-parameter-idiom).
They are named after the fields with a leading `set__`.
The setter methods have a single argument to pass the value for the member variable.
Each setter method returns the struct itself.

### Operators

The comparison operators `==` and `!=` perform the comparison on a per member basis.

### Pointer types

The struct contains `typedefs` for the four common pointer types `plain pointer`, `std::shared_ptr`, `std::unique_ptr`, `std::weak_ptr`.
For each pointer type there a non-const and a const `typedef`:

- `RawPtr` and `ConstRawPtr`
- `SharedPtr` and `ConstSharedPtr`
- `UniquePtr` and `ConstUniquePtr`
- `WeakPtr` and `ConstWeakPtr`

For similarity to ROS 1 the `typedefs` `Ptr` and `ConstPtr` still exist but are deprecated.
In contrast to ROS 1 they use `std::shared_ptr` instead of Boost.

## Services

For a service a `struct` with the same name followed by an underscore is generated.

The struct contains only two `typedefs`:

- `Request` which is the type of the request part of the service
- `Response` which is the type of the request part of the service

The generated code is split across multiple files the same way as message are.

### Request and response messages

For the request and response parts of a service separate messages are being generated.
These messages are named after the service and have either a `_Request` or `_Response` suffix.
They are are still defined in the `srv` sub namespace.
