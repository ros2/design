---
layout: default
title: Mapping between ROS interface types and DDS IDL types
permalink: articles/mapping_dds_types.html
abstract:
  This article specifies the mapping between the ROS interface types and the DDS types.
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

This article specifies the mapping between ROS interface types defined in the [interface definition article](interface_definition.html) and DDS types defined in the [Interface Definition Language](http://www.omg.org/spec/IDL35/).

## Mapping of primitive types

| ROS type | DDS type           |
| -------- | ------------------ |
| bool     | boolean            |
| byte     | octet              |
| char     | char               |
| float32  | float              |
| float64  | double             |
| int8     | octet              |
| uint8    | octet              |
| int16    | short              |
| uint16   | unsigned short     |
| int32    | long               |
| uint32   | unsigned long      |
| int64    | long long          |
| uint64   | unsigned long long |
| string   | string             |

## Mapping of arrays and bounded strings

| ROS type                | DDS type       |
| ----------------------- | -------------- |
| static array            | T\[N\]         |
| unbounded dynamic array | sequence<T>    |
| bounded dynamic array   | sequence<T, N> |
| bounded string          | string<N>      |
