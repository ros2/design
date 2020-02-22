---
layout: default
title: ROS 2 Security Contexts
permalink: articles/ros2_security_contexts.html
abstract:
  This article specifies the integration between security and contexts.
author:  >
  [Ruffin White](https://github.com/ruffsl),
  [Mikael Arguedas](https://github.com/mikaelarguedas)

published: false
categories: Security
---

{:toc}


# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


TODO: Some concise overview introduction here

## Concepts

Before detailing the SROS 2 integration of the contexts, the following concepts are introduced.

### Participant

Participant is the object representing a single entity on the network, in the case of DDS the ``Participant`` is a DDS DomainParticipant and the object to which a set of access control and security identity apply.

### Namespaces

Namespaces are a fundamental design pattern in ROS and are widely used to organize and differentiate many types of resources as to be uniquely identifiable; i.e. for topics, services, actions, and node names.
As such, the concept of namespaceing is well know and understood by current users, as well as strongly supported with the existing tooling.
Namespaces are often configurable at runtime via command line arguments or statically/programmatically via launch file declarations.

Previously, the Fully Qualified Name (FQN) of a node was used directly by a selected security directory lookup strategy to load the necessary key material.
However, with the advent of contexts, such a direct mapping of FQN to security artifacts may no longer suffice.

### Contexts

With the advent of ROS 2, multiple nodes may now be composed into one process for improved performance.
Previously however, each node would retain it's one to one mapping to a separate middleware ``Participant``.
Given the non-negligible overhead incurred of multiple ``Participant``s per process, a concept of contexts was introduced.
Contexts permit a many-to-one mapping of nodes to ``Participant`` by grouping many nodes per context, and one ``Participant`` per context.

Based on the DDS Security specification v1.1, a ``Participant`` can only utilise a single security identity; consequently the access control permissions applicable to every node mapped to a given context must be consolidated and combined into a single set of security artifacts.
As such, additional tooling and extensions to SROS 2 are necessary to support this new paradigm.


## Keystore

With the additional structure of contexts, it’s perhaps best to take the opportunity to restructure the keystore layout as well.
Rather than a flat directory of namespaced node security directories, we can push all such security directories down into a designated `contexts` sub-folder.
Similarly, private and public keystore materials can also be pushed down into their own respective sub-folders within the root keystore directory.
This is reminiscent of the pattern used earlier Keymint [1].


```
$ tree keystore/
keystore
├── contexts
│   └── ...
│       └── ...
├── private
│   ├── ca.csr.pem
│   └── ca.key.pem
└── public
    ├── ca.cert.pem
    ├── identity_ca.cert.pem
    └── permissions_ca.cert.pem
```


### ``public``

The ``public`` directory contains anything permissable as public, such as public certificates for the identity or permissions certificate authorities.
As such, this can be given read access to all executables.
Note that in the default case, both the `identity_ca` and `permissions_ca` are the same CA certificate.

### ``private``

The ``private`` directory contains anything permissable as private, such as private key material for aforementioned certificate authorities.
This directory should be redacted before deploying the keystore onto the target device/robot.

### ``contexts``

The ``contexts`` directory contains the security artifacts associated with individual contexts, and thus node directories are no longer relevant.
Similar to node directories however, the `contexts` folder may still recursively nest sub-paths for organizing separate contexts.


## Runtime

TODO: Some transition paragraph here about ros launch

### Unqualified context path

For nodes with unqualified context paths, the context directory will subsequently default to the root level context.

``` xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener"/>
</launch>
```

```
$ tree contexts/
contexts/
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem
└── permissions.p7s
```

### Pushed unqualified context path

For nodes with unqualified context paths pushed by a namespace, the context directory will subsequently be pushed to the relative sub-folder.

``` xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener"/>
  </group>
</launch>
```

```
$ tree --dirsfirst contexts/
contexts/
├── foo
│   ├── cert.pem
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   └── permissions.p7s
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem
└── permissions.p7s
```

### Relatively pushed qualified context path

For nodes with qualified context paths pushed by a namespace, the qualified context directory will subsequently be pushed to the relative sub-folder.

``` xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" context="bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst contexts/
contexts/
└── foo
    └── bar
        ├── cert.pem
        ├── governance.p7s
        ├── identity_ca.cert.pem
        ├── key.pem
        ├── permissions_ca.cert.pem
        └── permissions.p7s
```

### Fully qualified context path

For nodes with absolute context paths, namespaces do not subsequently push the relative sub-folder.

``` xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" context="/bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst contexts/
contexts/
└── bar
    ├── cert.pem
    ├── governance.p7s
    ├── identity_ca.cert.pem
    ├── key.pem
    ├── permissions_ca.cert.pem
    └── permissions.p7s
```


## Alternatives

### Context path orthogonal to namespace

An alternative to reusing namespaces to hint the context path could be to completely disassociate the two entirely, treating the context path as it's own unique identifier.
However, having to book keep both identifier spaces simulations may introduce to many degrees of freedom that a human could groc or easily introspect via tooling.

#### `<push_ros_namespace namespace="..." context="foo"/>`

TODO: Describe added `context` attribute to `push_ros_namespace` element.
Keeps pushing contexts close/readable to pushing of namespaces.

#### `<push_ros_context context="foo"/>`

TODO: Describe added `push_ros_context` element.
Keeps pushing context path independent/flexable from namespaces.


## Concerns

### Multiple namespaces per context

For circumstances where users may compose multiple nodes of dissimilar namespaces into a single context, the user must subsequently specify a common fully qualified context path for each node to compose, as the varying different namespaces would not push to a common context.
For circumstances where the context path is orthogonal to node namespace, the use of fully qualifying all relevant nodes is could be tedious, but could perhaps could still be parametrized via the use of `<var/>`, and `<arg/>` substitution and expansion.


### Modeling permissions of nodes in a process v.s. permission of the middleware ``Participant``

Before the use of contexts, multiple nodes composed into a single process where each mapped to a separate ``Participant``.
Each ``Participant`` subsequently load a security identity and access control credential prevalent to its' respective node.
However, all nodes in that process share the same memory space and can thus access data from other nodes.
There is a mismatch between the middleware credentials/permissions loaded and the resources accessible within the process.

By using contexts, all nodes in a context share the same security identity and access control credentials.
This inevitably means that code compiled to node ``foo`` can access credentials/permissions only trusted to node ``bar``.
This consequence of composition could unintendedly subvert the minimal spanning policy as architected by the policy designer or measured/generated via ROS 2 tooling/IDL.

With the introduction of contexts, it becomes possible to describe the union of access control permission by defining a collection of SROS 2 policy profiles as element within a specific context.
This would allow for formal analysis tooling [2] to check for potential violations in information flow control given the composing of nodes at runtime.
If a process contains a single context, this reconciles the permissions of a ``Participant`` and the ones of the process.

However, should multiple contexts be used per process, then such security guaranties are again lost because both contexts will share the same memory space.
Thus it should be asked whether if multiple contexts per process should even be supported.


In summary, the distinction here is that before, the composition of multiple permissions could not be conveyed to the tooling.
Whether nodes could gain the permission of others in the same process space is not the hinge point of note; it's the fact that such side effects could not be formally modeled or accounted for by the designer.
It will now be possible with contexts, however allowing for multiple contexts per process will reintroduce and exacerbates the same modeling inaccuracies.

### Composable launchfile includes

A particular challenge in using launchfiles with security contexts is that of keeping the include hierarchy composable.
An inherit tradeoff between simplicity and configurability can arise when writing launchfiles for downstream use.
Authors can selectively choose what attributes to expose as input arguments, while users may implicitly override provided defaults.

In case of contexts, it is not inherently clear what best practices either package authors or users should employ to retain a composable and intuitive launchfile structure. E.g:
Should authors parametrize context paths for each node as input arguments?
Should users push namespaces of included launchfiles to separate contexts?
Should the setting of security environment variables be discouraged from within launchfiles, limiting the use of simple static analysis of launchfiles combined with Node IDL for procedural context generation?

### Composable nodes in container

Given that containers can be dynamic, where nodes can be added or removed at runtime, there is perhaps some question as to how containers should integrate with secure contexts.
The most straightforward approach is perhaps only specifying the context wherever the container is first defined/launched, applying to the container participant, thus to all nodes/components inside that container.
This should be further deliberated when eventually extending the launch API for containers.

### Migration for RMW implementations

As it may take time before all RMW implementations implement the new system of contexts, a defined fallback behavior should still be designated.
For such implementations, the context security directory determined by RCL should be loaded for the participant as per the priority of context setting specified in the "ROS 2 DDS-Security integration" design doc.
This primarily desists the use of including the node name in the default lookup path, consequently getting users in the habit of creating separate contexts for separate processes, or explicitly specifying unique context names via launchfiles.

## References

1. [Procedurally Provisioned Access Control for Robotic Systems](https://doi.org/10.1109/IROS.2018.8594462)

``` bibtex
@inproceedings{White2018,
	title     = {Procedurally Provisioned Access Control for Robotic Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Christensen, Henrik and Cortesi, Agostino},
	year      = 2018,
	booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	doi       = {10.1109/IROS.2018.8594462},
	issn      = {2153-0866},
	url       = {https://arxiv.org/pdf/1810.08125.pdf}}
```


2. [Network Reconnaissance and Vulnerability Excavation of Secure DDS Systems](https://doi.org/10.1109/EuroSPW.2019.00013)

``` bibtex
@inproceedings{White2019,
	title     = {Network Reconnaissance and Vulnerability Excavation of Secure DDS Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Jiang, Chenxu and Ou, Xinyue and Yang, Zhiyue and Cortesi, Agostino and Christensen, Henrik},
	year      = 2019,
	booktitle = {2019 IEEE European Symposium on Security and Privacy Workshops (EuroS PW)},
	doi       = {10.1109/EuroSPW.2019.00013},
	pages     = {57-66},
	url       = {https://arxiv.org/abs/1908.05310.pdf}}
```
