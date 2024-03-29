---
layout: default
title: ROS 2 Documentation System
permalink: articles/ros_documentation_system.html
abstract:
  This article describes the proposed system for doing documentation for ROS 2.
author: '[William Woodall](https://github.com/wjwwood)'
date_written: 2015-01
last_modified: 2016-02
published: false
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}

Date Written: {{ page.date_written }}

Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

This document is meant to capture the results of discussions about the way we (the ROS 2 developers) would like documentation to work in ROS 2 and polish those results into a proposal.
It is likely that this document should be refined and made into a REP once a suitable design is settled on.

## Glossary

- __Documentation Engine__: Any tool which takes in information and produces documentation for the end user, e.g. Sphinx, Doxygen, a custom documentation script, etc.
- __Documentation Job__: A series of actions taken to provide documentation prerequisites, do documentation, validate the result, and finally transfer the result to a hosting solution, i.e. what to do on the build farm.
- __Documentation Process__: A well defined process by which any package can be introspected and then have all of its documentation generated by running the correct documentation engines with the correct settings, i.e. how to build documentation for any package.
- __Documentation System__: The cumulation of all components of documentation including: describing what to document, the documentation job, the documentation process, the documentation inputs and outputs for every package, and supporting tools.

## Documentation System Overview

Probably the principal difference between this proposed design and the existing ROS 1 system is the belief that general documentation about a package should be captured in the package as version controlled plain text files.

### Wiki Versus Static Documentation

In this case "general" documentation is what you would typically find on the ROS 1 wiki and consists of things like detailed descriptions of packages, links to external references, images and diagrams, descriptions of nodes, launch files, and other common components of ROS packages.
This is to distinguish these things from other kinds of documentation like Code API documentation, which is often written along side the code as comments, extracted automatically, and so therefore traditionally kept in plain text formats and version controlled with the code already.
This "general" documentation could also encompass other types of documentation or memorandum like tutorials, API reviews, and discussions, but these types of documentation could also be kept in a separate system like a wiki.

In ROS 1, a conscious decision was made to push as much of the documentation into the ROS 1 wiki as possible and in ROS 2 we would like to diverge from that and suggest that more documentation should be version controlled with the code.
The main reason, as far as we can gather, for this decision in ROS 1 was to make it as easy as possible for non developers to contribute changes to the documentation.
It is still the case for outsiders that changing documentation in a source repository is more difficult than documentation which is located in a wiki.
However, it seems that times have changed in this respect, and that the gap between editing a wiki page and getting a change in a source repository has narrowed.
This is mostly due to improvements in the open source contribution practices on services like GitHub and BitBucket and the proliferation of both the Pull Request pattern and the "Edit in GitHub/BitBucket" buttons on generated documentation pages.

In exchange for moving some types of documentation into the source repository, we believe that maintaining changes in documentation across different ROS versions and different package versions will become easier.
Additionally, the systems for doing code documentation will allow for more robust cross referencing to and from the code API documentation.
Finally, by doing this kind of documentation in code, it brings the ROS community's recommended practices closer to common practice for modern open source software projects, where the last few years have seen a migration from wiki's to statically generated documentation which comes from plain text files in the version control system.

### Risks of Static Documentation

There are some risks associated with using static documentation over something more dynamic like a wiki.
As mentioned before the barrier to entry to contributions may be too high for some users, especially on documentation which does not need review for each change.
Another potential issue is in documenting packages which would like to straddle multiple ROS distributions.
This happens with code too, where a package would ideally have one line of versions and one development branch for several similar ROS distributions.
Doing so cuts down on the number branches and amount of forward and back porting required which reduces the overall developer load.
By doing more documentation in source it may become harder to maintain these ROS agnostic development branches.
This essentially becomes a trade-off in how easy it is to document packages with large or small changes between ROS distributions.

Another risk of this proposal is that it may discourage usage of a wiki in scenarios where a wiki would be appropriate.
The intention of this proposal is not to suggest that wiki's should not be used, but instead means it will be up to the developer to decide what kinds of documentation are best suited for difference documentation options.
For example, documentation which bridges multiple packages is not well suited to being contained in one package's documentation and therefore may work better in a wiki style system.
However, by encouraging developers to make use of static documentation generation tools, there will be less need for heavy infrastructure, mirroring the documentation will be easier, and documentation will be more tightly coupled to the code.

### Other Goals of the Documentation System

One desired goal of the documentation system is to generate a standard landing page for all packages which are released or documented.
Similar to how the [PackageHeader](http://wiki.ros.org/WikiMacros#PackageHeader) wiki macro works in ROS 1, this page would summarize the items common to all packages.
These items would consist of information extracted from the package manifest, the package's README file, and other external resources like the build farm and the ROS distribution file.
This page can also contain dynamically generated information about the package which transcends one version of the package or one ROS distribution, e.g. it could list all of the ROS distributions in which the package has been released.
This page would also be able to automatically collect and link to resources in the packages.
For example, it could link to documentation for messages or external services like wiki's and other kinds of external documentation.
Ideally this page would be easy to produce locally as part of the normal documentation of the package and is not something special that is only done on the ROS infrastructure.
Generating it locally would allow the developer to preview all of the content for their package before publishing it to the publicly hosted infrastructure.
To support this goal, there would need to be a documentation engine which is run by every ROS package regardless of any other general documentation a developer wishes to create.

Another goal of the system is to support general documentation of ROS packages in source repositories rather than in a separately version controlled system like a wiki.
To achieve this the system would need to supply the developers with ROS specific documentation tools.
These kinds of tools would do things like cross reference to ROS messages automatically or make the information from the package manifest accessible in the documentation engine.
This kind of tooling would need to be provided for each documentation engine which we expect users to use for general documentation, which would be a burden to support.
Therefore, it would make sense to have a recommended documentation engine for general documentation of packages, limiting the need for tools in many documentation engines.

There are some other tasks which every package _may_ need to perform based on its contents.
Typically this is related to documenting special parts of the package, things like messages, launch files, or node parameter descriptions.
For messages (include services and actions), no extra action may be necessary since there is a build time generation step at which point documentation could also be generated.
However, there is no such step in the normal build process where generating documentation for things launch files is appropriate so packages which contain these things will need to generate documentation using some documentation engine, regardless of any other general documentation the developer creates.
Even if the developer does not reference their own launch files or messages, another package may wish to, so the documentation for these things should always be created.

## Default Documentation Engine

Based on the above goals, it seems clear that one or more default documentation engines will be required.
The above goals mostly apply to ROS packages, but not necessarily any package our tool chain can build, i.e. third party CMake, Python, or autotools packages.
The distinction is that our tool chain should not automatically assume a default engine, but rather the qualities that make a package a "ROS" package, i.e. depending on a ROS documentation package, should determine whether or not the default documentation engine is exercised by the tool chain.
This allows the tool chain to be agnostic to these additional items which are common to all ROS packages, but not necessarily all packages.

While it would be convenient to have one default documentation engine, it may be necessary to have several for either technical reasons or for modularity.
For simplicity, this article will describe a system with a single documentation system, with the understanding that this default documentation engine may have inputs from previous processes or that it might aggregate several different tools as part of its execution.

So for ROS packages this default engine would always be run and would be responsible for generating the package's landing page and for generating documentation for any special items like launch files and other common items which can be automatically documented.
If the developer does not provide any configurations for this default documentation engine then a boiler plate configuration which generates a landing page would be created for them automatically.
This would allow all packages to have some minimal documentation with zero configuration by the developer.
The developer would be able to generate this boiler plate documentation and configuration if they wanted to modify it, for instance to add something to the default landing page (much like you would on the ROS 1 wiki below the PackageHeader macro).
However, the system should do some minimal check to ensure that the landing page looks and feels consistent with other packages, e.g. ensure the equivalent to the "PackageHeader" is on the landing page and that the ROS theme is being used or extended.

When selecting a default engine, some things should be considered:

- The engine should be easy to extend (in order to provide the aforementioned tooling to the developer).
- The engine's syntax should be easy to learn (on the level of the current ROS 1 wiki's syntax).
- The engine must be design for general purpose documenting (not just automatic code documenting).
- The engine should have a cross referencing system to help maintain links between packages.

Based on that, feasible options would include Sphinx, Doxygen, or a custom Python script that just expands an ``empy`` template.

### Sphinx as a Default Documentation Engine

Sphinx is a compelling solution since it can be easily extended with Python and has a cross referencing system.
It uses the ReStructured Text (rst) standard for its markup language, which is fairly easy to learn, but can be cumbersome in some scenarios.
The benefit of rst and Sphinx is that its "directive" system provides a flexible way to extend the markup language without relying on an external template system.
Philosophically, Sphinx is oriented around creating documentation rather than extracting documentation from code.

### Doxygen as a Default Documentation Engine

Doxygen would also be a good choice since it has a good cross reference system and can parse markdown files for general purpose documentation.
Markdown is arguably simpler and easier to read than rst, but it also is less flexible since it does not have a built in system for extension.
Doxygen has manually extended Markdown to include cross referencing, but more complicated tasks would need to be done using a template engine in conjunction with Markdown, similar to how Jekyll works.
Doxygen excels at extracting documentation from code, especially C++ and Java, and has a lot of its features geared toward describing and automatically organizing code documentation.
Its extension system is arguably less approachable than Sphinx's since they are done in C++ and scripting of Doxygen commands is limited to argument aliasing.

### Custom Tool as a Default Documentation Engine

A custom tool as the default documentation engine provides perhaps the most opportunity for flexibility and leanness, but also will require the most effort to develop.
A custom tool would need to either not support cross referencing to other systems or amongst itself or a cross referencing system would need to be developed, neither of which seem like a good option.
It would be leaner than either Sphinx or Doxygen since it would have less features and would not try to document or extract code API's.

### Using the Default Documentation Engine

Prioritizing for the least development time and the most features, Sphinx seems like a reasonable choice for the default documentation engine.
However, suggesting that Sphinx be the default documentation engine is not a complete proposal.
Several details remain which will affect the requirements of the system and how the developers of packages will interact with it.

TODO: explain the issue with having Sphinx as the default: should the developer reuse it for documentation; should the developer run their own Sphinx instead; how to integrate them and make them both cross reference-able?

If Sphinx is the default documentation engine, one issue to sort out is whether or not the user should reuse this default run of Sphinx or not.
The user could integrate their own Sphinx documentation into the default Sphinx run if the system allowed it.
On the other hand the user could put all of their Sphinx documentation into a second run of Sphinx.

The advantage to the later option is that the fact that the default documentation is generated by Sphinx becomes an implementation detail.
However, in this case there are technical details to address, like how to integrate cross referencing the default documentation run and the user's Sphinx job.
In Sphinx, a cross reference is done by calling out to another project by name.
For example, to reference the function `foo_func` in the `foo_pkg` package the developer would do something like this: ```... :py:func:`foo_pkg:foo_func` ...```.
When there is a single run of Sphinx per project then the name used for the mapping would simply be the package's name.
However, when there are multiple runs of Sphinx then different names would be required.
One possibility is that the default run is always something like `<package_name>-default`, so that referencing a section in the landing page would be something like: ```... :ref:`foo_pkg-default:SectionName` ...```.
It may be possible compose a single cross referencing from multiple runs of Sphinx into a single mapping to make it appear as if multiple Sphinx projects were actually one, but this would need to deal with collisions of references, something that is handled by Sphinx normally.

<div class="alert alert-warning" markdown="1">
TODO: see if merging Sphinx object-inventories is possible.
</div>

This same issue comes up when considering if the developer should be able to run multiple instances of a particular documentation engine, for example should they be able to run Doxygen multiple times and if so, how is that organized such that other packages can cross reference both runs of Doxygen?

## Standard Documentation Inputs and Outputs

Each documentation engine has its own inputs and outputs, so to support more than one and arbitrary documentation engines in the future, the documentation system need to have a flexible way to export and import documentation information between packages.
All engines have some configurations which are required to do documentation.
For many of the configurations the default values are fine, but can be tuned to present a different behavior or look and feel.
The documentation system will want to provide some default values for these configurations.

Additionally most documentation engines have a method for configuring where to find external projects' documentation for use in cross referencing.
This information is both an input and an output for documentation of packages.
The cross referencing information could be useful only for other packages using the same documentation engine or it could be useful to other packages in ways that the current package cannot not anticipate.
For example, a C++ only package may export Doxygen XML files which is used by another package using Sphinx to do cross referencing.

### Documentation Output

Because the content of this exported and imported information is specific to the documentation engines that are being used, the transfer of information should be flexible.
One idea is that every package can generate a file which describes the sets of documentation output it generates, assigned each a name and source documentation engine.
For example, this might look something like this in the general form:

{% highlight yaml %}
package_name: foo
documentation_artifacts:
  sphinx:
    foo-default:
      output_dir: '/'
    foo:
      output_dir: '/sphinx/'
  doxygen:
    foo:
      output_dir: '/doxygen/'
      xml_output_dir: '/doxygen_xml/'
{% endhighlight %}

This example is extremely declarative, but the developer would not need to explicitly fill this file out, but instead it could be dynamically generated by the documentation process and tooling.
In the above example, the output is categorized hierarchically by documentation engine, then a named run of the engine, and then the details of that run.
The details of the run can be a documentation engine specific opaque set of settings.

### Documentation Input

Additional information, e.g. a base path, could be paired with the above example output and given as input to another package.
With a base path and this example output, other packages should be able to successfully cross reference the `foo` package if their documentation engine supports this.
So in addition to the default configurations, the documentation inputs for a package would be a series of base paths and "documentation artifact manifests" for each of the package's build, run, and doc dependencies.

## The Documentation Job

There are two processes which need to be detailed in order to successfully document different kinds of packages.
The process with the largest scope has been described in this article as the documentation job.
The documentation job is the process of preparing the operating system for documentation a particular package, invoking the documentation process, and then dealing with the results.
The rough outline of the process is:

- Build-like dependencies are installed.
- The package is built, unless the user opts-out.
- Run-like and doc dependencies are installed.
- The package is documented using the documentation process.
- The documentation which has been installed is packaged or moved to hosting.

<div class="alert alert-warning" markdown="1">
TODO: Fill in details about the documentation job
</div>

## The Documentation Process

The documentation process consists of the series of actions which must be preformed in order to generate documentation for a single package.
The process starts with an environment that has all of the dependencies for the package resolved and in which the package has already been built.
This process constitutes what happens for the "doc target" of the build tools.
The build tools will perform these steps in the documentation process:

- Create a list of requested and implicit runs of documentation engines.
- For each run, the appropriate default configurations and inputs are compiled.
- For each run, documentation engine specific execution occurs.
- For each run, the generated documentation output is cataloged and copied to the install destination.
- The cataloged output meta data is compiled into a single documentation manifest for the package and installed.

The implicit runs would encompass the default documentation engine.

<div class="alert alert-warning" markdown="1">
TODO: Fill in details about the documentation process
</div>

## User Stories

This section will look at the documentation system from the point of view of the users, in this case package developers and package consumers, by constructing some common and uncommon scenarios in order to gain a better understanding of the requirements.

### Discovering Packages Through Documentation

Scenario:

> A "package consumer", someone who is looking for packages to use in their ROS system, hears about a package that sounds interesting to them, e.g. ``wall_e_plant_detection``.
> She searches for ``wall_e_plant_detection`` with her favorite search engine and finds a page which summarizes or references the package.
> After a while she realizes that she can go to ``packages.ros2.org/wall_e_plant_detection`` get redirected to the latest version of the package for the current ROS distribution, which might end up going to something more specific like ``http://packages.ros2.org/en/jade/latest/wall_e_plant_detection/index.html``.
> The page summarizes or references all the information about the package that can be compiled by the documentation system, e.g. Code documentation, Tutorials, wiki, jobs on the farm, etc.
> This page has also been customized by the developer to include a longer description and more content like custom links to external resources.

This ROS package landing page would be similar to the landing pages for Ubuntu packages, e.g. ``http://packages.ubuntu.com/trusty/libboost-all-dev``.
From here the user can discover general and API documentation, tutorials, other versions, dependencies, dependents, provided resources, etc., all for this package.

``packages.ros2.org`` is just an example.
Based on the implementation of the infrastructure for this site, something more like ``docs.ros2.org`` may make more sense.
Even ``wiki.ros2.org``, if it is implemented more as a wiki with static pages embedded throughout.

One question is whether or not the package developer should be able to customize this page at all.
Ideally the developer could hook into this process and add more stuff than what is in the `package.xml`'s description.
This could be accomplished by either having the ability to import something like a `README.rst` file or the ability for the developer to edit the contents of the landing page directly as a file in their repository.

### Documentation for a Released Package

Scenario:

> After providing zero extra documentation or configuration a package developer releases their package to take advantage of the binary packaging provided for released packages.
> After the release has been processed by the build farm, the package will have a discoverable landing page with as much information extracted from the package as is possible.

This is a slight improvement on the ROS 1 system, because in ROS 1 the package developer would have to additionally go to the wiki and create a package with the `PackageHeader` macro in it in order to gain the same effect.

### Documentation for an Unreleased Package

Scenario:

> A package developer would like to make their package discoverable by others and have their documentation hosted, but has not taken the time to do a proper release of the package.
> She submits a pull request to the ROS distribution file to add a "doc" entry for her package(s).
> This doc entry points to repository which contains her package(s) on her main development branch.
> Later the build farm, triggered either by a commit or a periodic timer, cloned her repository and ran the documentation job on the package(s).
> Documentation was generated based on her package's configurations and the result is uploaded to the doc servers for others to find.

The scenario shows that users should be able to take advantage of as much of the documentation system as possible without releasing their package.

### Maintaining Multiple Versions of Documentation

Scenario:

> A package developer adds to the public API in a backwards compatible way during a ROS release, e.g. between versions ``1.1.0`` and ``1.2.0`` for the Jade ROS distribution.
> Because of this change, the package developer would like to make the documentation for both versions of the package available to their users.
> So in addition to specifying a branch of their VCS repository that should be documented periodically using a doc entry, she explicitly specifies version ``0.1.0`` to be generated and kept around.
> So now there are three versions of the documentation for Jade, her latest branch ``jade-devel``, version ``0.1.0``, and the most recent released version ``0.2.0``.
> When users browse to her package's documentation they will see three options for versions of documentation for Jade, ``latest``, ``0.1.0``, and ``0.2.0``.

Currently the ROS 1 infrastructure only allows for one version of documentation per package per ROS distribution.
This works for most cases, but a more flexible option for packages which change during the life time of distributions would be to have multiple released versions as well as a floating "latest" version if desired.
To do this would require capturing which VCS tags should be documented, where the tags map to older versions of the software.
This would be in addition to the information we already capture in the "doc" entry which covers a tag or branch to document and the "release" entry which tells us the latest version of the package currently released but not previous versions.
This idea is borrowed from the way the [https://readthedocs.org/](https://readthedocs.org/) on-line documentation service operates.

## Documentation Requirements

The functional requirements were arrived at by building on the experience of the ROS 1 documentation system and practices, while keeping in mind some non-functional requirements.

### Non-Functional Requirements

Listed in no particular order, the documentation system should be...

- _scalable_ by ensuring that a default, minimal output for each package is generated with zero configuration.
- _usable_ by reusing information in the package manifest to minimize duplication of information throughout.
- _extensible_ by ensuring that many documentation engines can be supported and that special elements can be automatically documented, e.g. ROS messages, launch files, node parameters, etc.
- _standard compliant and conventional_ by reusing tools which already exist where possible.
- _accessible and consistent_ by providing common configuration items in order to reduce effort for developers and to improve consistency across packages.
- _configurable_ by giving the developer access to each documentation engine's configuration, as not to hinder their ability to customize the output.
- _maintainable_ by reusing specific documentation engine's configuration mechanism where possible.
- _efficient_ by allowing the developer to opt-out of steps which are not required for their package.
- _safe_ by identifying undesirable results and notifying the developer.
- _discoverable_ by having easy to read and understand URL schemes and by facilitating the storage of and switching between multiple versions of the documentation.
- _stable_ by having conventions about documentation layout, warning when they are violated, and enforcing them where possible.

These no-functional requirements, or qualities, of the system guided the following proposed functional requirements and system design.

### Functional Requirements

The above non-function requirements have suggestions at some of the more concrete function requirements.
Listed in no particular order, the documentation system will...

- support at least Sphinx and Doxygen.
- document each package, one at a time, in a topological order which respects ``<doc_depend>`` type dependencies.
- build each package before running documentation engines.
- allow developers to skip building their package before running documentation engines on it.
- run Sphinx by default.
- ___not___ allow the developer to skip running Sphinx on their package.
- provide a default Sphinx configuration for packages which do not provide one.
- provide a tool to generate the default Sphinx configuration, allowing the developer to modify it.
- validate the Sphinx settings and warn the developer in the case that a setting might cause a problem with the output.
- provide a mechanism in Sphinx to access and reuse information stored in the package manifest file.
- provide a mechanism in Sphinx to generate the "package page header" which summarizes the package and provides a consistent landing page for all packages (similar to the package header in the ROS 1 wiki).
- enforce that the index page for Sphinx follows a certain format which starts with the "package page header".
- enforce that reserved names are not used as pages or folders in the Sphinx layout, e.g. ``msgs`` which will contain ROS message documentation.
- run Doxygen by default.
- allow the developer to skip running Doxygen on their package.
- provide default Doxygen configuration for packages which do not provide them.
- provide a tool to generate the default Doxygen configuration, allowing the developer to modify them.
