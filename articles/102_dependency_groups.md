---
layout: default
title: Dependency groups
permalink: articles/dependency_groups.html
abstract:
  This article describes the concept of dependency groups, their use cases, how they can described in the package manifest, and how they can be mapped into binary packages.
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
published: true
categories: Overview
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Preface

In the ROS ecosystem a ROS package provides information about its dependencies in the manifest.
The latest format is specified in the [REP 140](http://www.ros.org/reps/rep-0140.html).
It supports different kind of dependencies (`build`, `run`, `test`, `doc`, etc.).
But each dependency is mandatory and needs to satisfied on its own.

This article describes the concept of "dependency groups" which could be used for different purposes.
In terms of terminology a dependency group is just a set of dependencies.
For such a group a different semantic can then be applied (instead of just satisfy each dependency on its own).

## Use Cases

The following subsections will describe two different use cases how dependencies could be used.

### (1) Need at least one "Provider"

A package might want to declare that at least one dependency from a group needs to be present.
Compared to "normal" dependencies it is not requires that all dependencies of that group need to be present to satisfy the dependency group.

An example for this is the `rmw_implementation` package.
Currently it depends on all RMW implementation packages in order to ensure that it may only be build after all RMW implementation packages have been built.
The goal would be that at least one RMW implementation needs to be present in order to build or use the `rmw_implementation` package.
It would be acceptable if more or even all RMW implementations are available but that is not required.

### (2) All packages of a specific "type"

Another use case is that a dependency group doesn't need to be explicitly defined by all dependency names.
Instead a dependency group is defined by an identifier and the expectation is that all packages which have that identifier in their manifest should be available.

An example for this is the `ros1_bridge` package.
Currently it depends on a manually selected set of message packages.
The goal would be that it can declare that it depends on *all* packages which provide messages.

## Top Down

First, we look at the capabilities of binary packages if and how these use cases can be satisfied since we don't have much influence on these capabilities and have to use the features available.
Additionally we consider the needs for a from-source build.
Since ROS provides its own build tools it should be possible to adjust the logic to whatever is necessary to support these features.
Last, we define the information we need to additionally provide in the package manifest and how to integrate them into the existing structure.

### Binary Packages

For know this article focuses on Debian packages.
It would be great if someone could add the similar information for other platforms to ensure the concept is solid and works across mutliple platforms.

#### Binary Packages

...

### Building from Source

...

### Information in the Manifest

To provide the information for the build tool as well as the packaging process the following information need to be provided by the manifest:

* Use case (1)

  * In a specific package a set of dependencies need to be associated to a dependency group.
  * The semantic of the group needs to be defined (in this case "at-least-one-from-the-group").

* Use case (2)

  * A package declares a dependency on a group identifier.
  * A set of packages declares that group identifier.

#### Possible Format Extensions

...

## Proposed Solution

### Extensions for the Package Manifest

...

### Build Tool

For use case (1) the build tool only needs to treat all dependencies of the set as "normal" dependencies.
When they are available within the workspace they should be processed before and being made available to the package declaring the dependency group.

For use case (2) the build tool needs to extract the identifiers from each package and consider them when detemining the topological order.

### Packaging process

The packaging tool needs to map the dependencies groups and identifiers to the platform specific files as described in the above section.
