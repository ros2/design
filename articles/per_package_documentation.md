---
layout: default
title: Per-Package Documentation
permalink: articles/per_package_documentation.html
abstract: This article describes the requirements and design of ROS 2’s per-package documentation system.
author: Marya Belanger
published: true
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

ROS 2 is lacking a process for the conglomeration of package documentation.
Discoverability for package documentation (and documentation in general) is one of the most frequent user complaints.

With a standardized package documentation process that encourages package maintainers to robustly document their packages, and presents the results alongside ROS 2's general documentation, issues of discoverability and consistency will be addressed.

In general, our vision for the system is:

- Package maintainers document their packages in their repositories following some guidelines or templates we recommend
- Package documentation from the repositories is built on our documentation site by our buildfarm in an automatic process with no extra input from the maintainers
- Package documentation is indexed alongside ROS 2's generic content on our docs site

## Context

The per-package documentation plan is an extension of the general ROS 2 documentation project.
That project, in the implementation phase now, includes some keys points that affect the context of the per-package documentation.

- All documentation will be hosted on `docs.ros.org`
- All documentation will be versioned by ROS 2 distribution names
- The URL structure will be `docs.ros.org/<lang>/<distro>/<page>` for generic documentation and `docs.ros.org/<lang>/<distro>/p/<package_name>/<page>` for package documentation
  - For more context on where documentation will fall under this URL structure, see [this diagram](https://docs.google.com/drawings/d/1KxzDrcSZzwGgudk-kEXGWnoRuAtc1ffl_KBeIu-V60Y/edit?usp=sharing).

## Requirements

The "Primary requirements" are those that must be in place for the system to be functional and achieve its purpose.
The "Secondary requirements" are important features and functionality that are not necessary to roll out the first stage of implementation, but should be carried out following the system's roll out.

### Primary requirements

**All package documentation must be available with the rest of the ROS 2 documentation under a single domain**

Package documentation will be treated the same as the generic ROS 2 documentation.
Its presence as part of the docs site should be made well-known from the overview as well as any entry point into the docs site, and be intuitive from the organization and layout of the site.

**Package documentation must have an easily navigable and intuitive URL structure**

Every package's "home page" should be reachable with minimal effort by the scheme `docs.ros.org/p/<package_name>/`, which will redirect to default language and version, `docs.ros.org/<lang>/<distro>/p/<package_name>/`  

**Package documentation must be maintainable in its repository without going through a third party.**

Maintainers and contributors will only have to work on their package's documentation within its repository.
The details of building and hosting will not be a concern of package maintainers.

Despite being hosted alongside the generic documentation, working on a package's documentation will not require any work on the repositories of the generic documentation (currently `ros2/ros2_documentation`) or the site repository (currently `ros-infrastructure/rosindex`).

**Support C++ and Python API docs generation**

The system will automatically extract API docs for these languages from the source code in package repositories and build the output on the docs site.

**Allow package documentation to be versioned per ROS 2 distribution**

The docs site and buildfarm will allow documentation for the latest version of a package corresponding to each ROS 2 distribution the docs site supports versioning for.
This means no versioning for several versions of a package per ROS 2 distribution.
The package repository can still maintain it's own docs for previous versions, they just won't be hosted on our site.

**The buildfarm automatically builds package documentation**

Changes to the documentation in a package repository shouldn't require the maintainer to trigger a build or anything like that.
The changes should be automatically updated to the site.

**Automatically generate content for a package so it's listed on docs.ros.org even if the package maintainer does not explicitly set up anything in the package repository**

Maintainers can explicitly opt out of being listed, but otherwise by default some package info will be listed for every available package.

**Support cross-referencing between packages**

A stable linking process will be in place to cross-reference between package docs despite their originating from separate repositories.

**Maintainers should not have to avoid certain file names because of possible collisions with auto-generated content on the system's side**

When writing package documentation, maintainers should not have to concern themselves with auto-generated content, like the current `/changelogs` and `/symbols` directories in the current `docs.ros.org` API docs structure for ROS 1.

### Secondary requirements

**Ability to include free form documentation, like tutorials, design docs, etc., alongside generated API docs on docs.ros.org**

**Provide a standardized interface for generating an index page/landing page for packages, consistent across ROS 2**

**Standardized method for generating package documentation locally, including build error alerts**

**Make the system extensible for more languages**

**Switching between versions (distributions) while viewing a package's documentation**

**Table of contents on docs.ros.org/p/ listing all packages with documentation**

**Support for building documentation for more than one version of a package per ROS 2 distribution**


## Design

*forthcoming*
