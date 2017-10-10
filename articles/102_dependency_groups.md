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
It supports different kind of dependencies (`build`, `exec`, `test`, `doc`, etc.).
But each dependency is mandatory and needs to satisfied on its own.

This article describes the concept of "dependency groups" which could be used for different purposes.
In terms of terminology a dependency group is a set of dependencies identified by the name of the group.
For such a group a different semantic can then be applied (instead of just satisfy each dependency on its own).

## Use Cases

The following subsections will describe two different use cases how dependencies could be used.

### (1) Need at least one "Provider"

A package might want to declare that at least one dependency from a group needs to be present (called `satisfy-at-least-one`).
Compared to "normal" dependencies it is not requires that all dependencies of that group need to be present to satisfy the dependency group.

An example for this is the `rmw_implementation` package.
Currently it depends on all RMW implementation packages in order to ensure that it may only be build after all RMW implementation packages have been built.
The goal would be that at least one RMW implementation needs to be present in order to build or use the `rmw_implementation` package.
It would be acceptable if more or even all RMW implementations are available but that is not required.

In case multiple package declare to be part of that dependency group the `rmw_implementation` package might want to provide an order list of preferred names which should be considered in that order.

### (2) All packages of a specific "type"

Another use case is that a dependency group doesn't need to be explicitly defined by all dependency names (called `satisfy-all`).
Instead a dependency group is defined by an identifier and the expectation is that all packages which declare that they are being part of that group should be available.

An example for this is the `ros1_bridge` package.
Currently it depends on a manually selected set of message packages.
The goal would be that it can declare that it depends on *all* packages which provide messages.

Since the `ros1_bridge` manifest only contains the name of the group dependency in doesn't control which packages declare that they are part of that group.
It might want to exclude specific packages to be considered even though they are in the group.

## Processes

The following two processes need to be update to consider dependency groups.

### Build tool

The build tool needs to consider the declared dependency groups when computing the topological order.
Since it operates on a set packages in a workspace it has all manifests for all of these packages available.
Independent of the semantic of the dependency group (satisfy-all vs. satisfy-at-least-one) the build tool can simply expand a dependency group to all packages of the set.

### Release tool

Since the release tool `bloom` operates on a single package it can't resolve a dependency group without further information.
It will need all package manifests available from `rosdistro` to resolve a dependency group to a set of packages.

For the `satisfy-all` semantic it can simply generate dependencies on all packages from the group.
For the `satisfy-at-least-one` semantic the release tool could either use a platform specific mechanism like `provides` in Debian control files or simply expand the group to all packages (same as `satisfy-all`).
This design document doesn't aim to specify what the "best" behavior for the release tool is since the decision is likely platform dependent.

Since the release tool expands the dependency groups when being invoked packages which join the group after that release won't be used until the package defining the group dependency is being re-released.

### rosinstall_generator

The `rosinstall_generator` already utilized all manifests from the `rosdistro` to decide which packages / repositories should be fetched.
This design document doesn't aim to specify what the "best" behavior for this tool is.
It could either match the behavior of the build tool or of the release tool or even something different.

## Information in the Manifest

To provide the necessary metadata for the build and release tools the following information must be provided by the manifest:

* (A) A specific package declares a group dependency and uses a name to identify the group.
  The dependency can be of any type specified in [REP 140](http://www.ros.org/reps/rep-0140.html): e.g. `build`, `exec`, `test`, `doc`.

* (B) The semantic of the group dependency needs to be defined (in case (1) `satisfy-at-least-one`, in case (2) `satify-all`).

* (C) A list of preferred / excluded packages can be enumerated for each dependency group.

* (D) Any package can declare that it is part of that group.

### Possible Format Extensions

The group dependency (A) could be specified in two ways:

* (A.1) Using the existing `*_depend` tags and adding an attribute to the tag to make it a group dependency.
* (A.2) Defining new `*_group_depend` tags which specifically identify group dependencies.

Considering the existing API in `catkin_pkg` which exposes the different dependencies as members of the `Package` class (A.1) would require existing code to distinguish the type of the dependency (group vs. no-group).
On the other hand (A.2) will simply add additional members which doesn't require existing code to change until it wants to support handling group dependencies.

The semantic of the group dependency (B) can be modeled as an attribute of the group dependency tag.

Each preferred / excluded package (C) name can be listed in child-tags of the dependency group to keep the information atomic.

The group membership (D) could be specified in two ways:

* (D.1) Adding a new tag to the `export` section: e.g. `member_of_group`.
* (D.2) Defining a new tag in the `package` section: e.g. `member_of_group`.

#### Example for use case (1)

A package declaring a group dependency:

```
<package>
  <name>rmw_implementation</name>
  <group_depend satisfy="at-least-one">i_am_a_rmw_implementation</group_depend>
</package>
```

A package declaring membership of a group:

```
<package>
  <name>rmw_fastrtps_cpp</name>
  <member_of_group>i_am_a_rmw_implementation</member_of_group>
</package>
```

#### Example for use case (2)

A package declaring a group dependency:

```
<package>
  <name>ros1_bridge</name>
  <group_depend satisfy="all">i_am_a_msg_pkg</group_depend>
</package>
```

A package declaring membership of a group:

```
<package>
  <name>std_msgs</name>
  <export>
    <member_of_group>i_am_a_msg_pkg</member_of_group>
  </export>
</package>
```

## Extensions for the Package Manifest

The modified schema for the manifest will be specified in [REP 149](https://github.com/ros-infrastructure/rep/pull/138).
