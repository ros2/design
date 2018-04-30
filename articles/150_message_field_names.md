---
layout: default
title: Message Field Name Conflicts
permalink: articles/message_field_conflicts.html
abstract:
  As we move to support more platforms there are we are running into predefined fields and constants that conflict with languages or implementation keywords and macros.
  This defines how to deal with existing and potential future conflicts.
published: true
author: '[Tully Foote](https://github.com/tfoote)'
categories: Interfaces
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Scope

This article specifies how we will deal with conflicting fields and constants for different languages.
In the use of the IDL (TODO (tfoote) reference) there will inevitably be conflicting symbols, keywords, or definitions.

## Background

This has come up specifically as we're adding support for Windows for ROS2.
`winnt.h` defines several macros that conflict with existing enumerations. 
It is also expected to happen as support for new languages are added.

## New constants

To avoid conflicts generally all constants should be prefixed with a `k`. This will avoid almost all conflicts.

Google Style Guide: https://google.github.io/styleguide/cppguide.html#Constant_Names

TODO(tfoote) Reference ROS2 style guide

## Language Generator Requirements

In the case that a new language is added and there is a detected existing symbol, the language specific generator is expected to implement a basic deconfliction mangling of the symbol to avoid the conflict.

Each generator will define the mangling procedure and provide a list of symbols for which the procedure will be applied.
This list should be all known keywords or otherwise conflicting symbols.

### Example Deconfliction Policy

A example of a simple name mangling policy would be to append a trailing underscore for conflicting symbols.
So that would mean that `FOO` if declared as a conflicting symbol would be used in generated code as `FOO_`.

A slightly more unique string is recommended for clarity.

### Stability
Each language generator will provide a standard name deconfliction policy so that users can use the symbols in a stable manner.
If a change is needed the generated code should provide a full cycle of backwards compatibility to support changes.

This table will remain stable so that codebases can rely on using the mangled symbols.
Appending to the table may be necessary if the first pass did not catch all the conflicting keywords.

The removal of any protected keywords should be avoided if at all possible as it will require all code using that field or constant to be updated.

### Table of deconfliction

The language support package will provide a yaml file with a list of all the protected keywords and their mangled version in a dictionary as key and value respectively.

For example if the language had conflicts for `FOO`, `new`, and `delete` it's yaml file would like like this:

```
FOO: FOO_
new: new_
delete: delete_
```

This yaml file will be considered the authority for deconflicting and should be exported in a way that will allow downstream code generators to automatically apply the deconfliction policy.

## Legacy Support

There are a number of known conflicts with the existing ROS1 codebase.
In these few cases we will undefine the matching C/C++ preprocessor macros which are conflicting in the definition and define the old value.
This will be done only inside the declaration header and the definition will be restored before returning to the user's code.
If you are using the legacy defines it's required to use the namespaced reference instead of using a shorthand version as those are know to conflict.

These values will also provide the new version with the deconfliction mangling applied.
It is recommended to move to the deconflicted version immediately.

The backwards compatibility will be maintained for one year after Bouncy.

* `NO_ERROR`
* `DELETE`
* `ERROR`
