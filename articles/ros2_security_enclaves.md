---
layout: default
title: ROS 2 Security Enclaves
permalink: articles/ros2_security_enclaves.html
abstract:
  This article specifies the integration of security enclaves.
author:  >
  [Ruffin White](https://github.com/ruffsl),
  [Mikael Arguedas](https://github.com/mikaelarguedas)

published: true
categories: Security
---

{:toc}


# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

This design document formalizes the integration of ROS 2 with security enclaves.
In summary, all processes must load a common enclave that contains the runtime security artifacts unique to that enclave, yet each process may not necessarily have a unique enclave.
Multiple enclaves can be encapsulated in a single security policy to accurately model the information flow control.
Users can tune the fidelity of such models by controlling at what scope enclaves are applied at deployment.
E.g. one unique enclave per OS process, or per OS user, or per device/robot, or per swarm, etc.
The rest of this document details how enclaves can be organized and used by convention.

## Concepts

Before detailing the SROS 2 integration of the enclaves, the following concepts are introduced.

### Participant

Participant is the object representing a single entity on the network.
In the case of DDS, the ``Participant`` is a DDS DomainParticipant, which has both access control permissions and a security identity.

### Namespaces

Namespaces are a fundamental design pattern in ROS and are widely used to organize and differentiate many types of resources as to be uniquely identifiable; i.e. for topics, services, actions, and node names.
As such, the concept of namespacing is well known and understood by current users, as well as strongly supported by the existing tooling.
Namespaces are often configurable at runtime via command line arguments or statically/programmatically via launch file declarations.

Previously, the Fully Qualified Name (FQN) of a node was used directly by a selected security directory lookup strategy to load the necessary key material.
However, now that Participants map to contexts and not nodes, such a direct mapping of node FQN to security artifacts is no longer appropriate.

### Contexts

With the advent of ROS 2, multiple nodes may now be composed into one process for improved performance.
Previously however, each node would retain it's one to one mapping to a separate middleware ``Participant``.
Given the non-negligible overhead incurred of multiple ``Participant``s per process, a change was introduced to map a single ``Participant`` to a context, and allow for multiple nodes to share that context.

Based on the DDS Security specification v1.1, a ``Participant`` can only utilise a single security identity; consequently the access control permissions applicable to every node mapped to a given context must be consolidated and combined into a single set of security artifacts, or enclave.
As such, additional tooling and extensions to SROS 2 are necessary to support this new paradigm.

## Keystore

With the addition of contexts, it’s perhaps best to take the opportunity to restructure the keystore layout as well.
Rather than a flat directory of namespaced node security directories, we can push all such security directories down into a designated `enclaves` sub-folder.
Similarly, private and public keystore materials can also be pushed down into their own respective sub-folders within the root keystore directory.
This is reminiscent of the pattern used earlier in Keymint [1].


```
$ tree keystore/
keystore
├── enclaves
│   └── ...
│       └── ...
├── private
│   ├── ca.csr.pem
│   └── ca.key.pem
└── public
    ├── ca.cert.pem
    ├── identity_ca.cert.pem -> ca.cert.pem
    └── permissions_ca.cert.pem -> ca.cert.pem
```


### ``public``

The ``public`` directory contains anything permissible as public, such as public certificates for the identity or permissions certificate authorities.
As such, this can be given read access to all executables.
Note that in the default case, both the `identity_ca` and `permissions_ca` points to the same CA certificate.

### ``private``

The ``private`` directory contains anything permissable as private, such as private key material for aforementioned certificate authorities.
This directory should be redacted before deploying the keystore onto the target device/robot.

### ``enclaves``

The ``enclaves`` directory contains the security artifacts associated with individual enclaves, and thus node directories are no longer relevant.
Similar to node directories however, the `enclaves` folder may still recursively nest sub-paths for organizing separate enclaves.
The `ROS_SECURITY_ROOT_DIRECTORY` environment variable should by convention point to this directory.

## Integration

With the introduction of contexts into rcl, instead of relying on node namespaces to lookup security artifacts from the keystore, the enclave path is now completely disassociated from node namespaces, instead serving as its own unique resource identifier.
Although having to book keep both identifier spaces simultaneously may introduce more degrees of freedom, it should still be possible to organize enclaves within the keystore to mimic node namespace hierarchy for transparent traceability of allocated permissions.

## Future Work

Introspective tooling and launchfile interfaces should be updated to help lessen the complexity introduced with the migration to contexts and enclaves.

### Runtime

Given the normative case where a enclave within a policy may be specific to a single node/container process, the namespace the node is remapped to will inevitably affect the required security permissions necessary within the enclave.
To highlight this interdependency, and to help avoid enclave path collisions, a hierarchy borrowing namespaces is appropriate. 
By convention, ros2launch could be used to prefix relative enclave paths for single process node or containers using the namespace in scope, to enable a convention of composable launch files with adjustable and parameterized enclave paths.
Given the runtime command argument for specifying the fully qualified enclave path, ros2launch would accordingly resolve relative enclave paths for executables, as defined by launch attributes.

### Unqualified enclave path

For single process nodes with unqualified enclave paths, the enclave directory will subsequently default to the root level enclave.

``` xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener"/>
</launch>
```

```
$ tree enclaves/
enclaves/
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem -> ../public/identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem -> ../public/permissions_ca.cert.pem
└── permissions.p7s
```

### Pushed unqualified enclave path

For single process nodes with unqualified enclave paths pushed by a namespace, the enclave directory will subsequently be pushed to the relative sub-folder.

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
$ tree --dirsfirst enclaves/
enclaves/
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

> Symbolic links suppressed for readability

### Relatively pushed qualified enclave path

For single process nodes with qualified enclave paths pushed by a namespace, the qualified enclave directory will subsequently be pushed to the relative sub-folder.

``` xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" enclave="bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
└── foo
    └── bar
        ├── cert.pem
        ├── governance.p7s
        ├── identity_ca.cert.pem
        ├── key.pem
        ├── permissions_ca.cert.pem
        └── permissions.p7s
```

### Fully qualified enclave path

For single process nodes with absolute enclave paths, namespaces do not subsequently push the relative sub-folder.

``` xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" enclave="/bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
└── bar
    ├── cert.pem
    ├── governance.p7s
    ├── identity_ca.cert.pem
    ├── key.pem
    ├── permissions_ca.cert.pem
    └── permissions.p7s
```


## Alternatives

#### `<push_ros_namespace namespace="..." enclave="foo"/>`

One such approach could be done by adding a `enclave` attribute to `push_ros_namespace` element.
This also keeps the pushing of enclaves close/readable to pushing of namespaces.

#### `<push_ros_enclave enclave="foo"/>`

Another alternative approach could be to add an entirely new `push_ros_enclave` element.
This could ensure the pushing of enclave path independent/flexable from namespaces.

## Concerns

### Multiple namespaces per context

For circumstances where users may compose multiple nodes of dissimilar namespaces into a single context, the user must still subsequently specify a common enclave path that is applicable for all nodes composed.
For circumstances where the enclave path is orthogonal to node namespace, the use of fully qualifying all relevant enclave paths could be tedious, but could perhaps could still be parametrized via the use of `<var/>`, and `<arg/>` substitution and expansion.

### Modeling permissions of nodes in a process v.s. permission of the middleware ``Participant``

Before the use of contexts, multiple nodes composed into a single process where each mapped to a separate ``Participant``.
Each ``Participant`` subsequently load a security identity and access control credential prevalent to its' respective node.
However, all nodes in that process share the same memory space and can thus access data from other nodes.
There is a mismatch between the middleware credentials/permissions loaded and the resources accessible within the process.

By using enclaves, all nodes in a context share the same security identity and access control credentials.
This inevitably means that code compiled to node ``foo`` can access credentials/permissions only trusted to node ``bar``.
This consequence of composition could unintendedly subvert the minimal spanning policy as architected by the policy designer or measured/generated via ROS 2 tooling/IDL.

With the introduction of enclaves, it becomes possible to describe the union of access control permission by defining a collection of SROS 2 policy profiles as element within a specific enclave.
This would allow for formal analysis tooling [2] to check for potential violations in information flow control given the composing of nodes at runtime.
If a process loads a single enclave, this reconciles the permissions of a ``Participant`` and the ones of the process.

However, should multiple enclaves be loaded per process, then such security guaranties are again lost because of shared same memory space.
Thus it should be asked whether if multiple enclaves per process should even be supported.

In summary, the distinction here is that before, the composition of multiple node permissions could not be conveyed to the tooling.
Whether nodes could gain the permission of others in the same process space is not the hinge point of note; it's the fact that such side effects could not be formally modeled or accounted for by the designer.
It will now be possible with enclaves, however allowing for multiple contexts per process that load separate enclaves would reintroduce and exacerbate the same modeling inaccuracies.

### Composable launchfile includes

A particular challenge in using launchfiles with security enclaves is that of keeping the include hierarchy composable.
An inherit tradeoff between simplicity and configurability can arise when writing launchfiles for downstream use.
Authors can selectively choose what attributes to expose as input arguments, while users may implicitly override provided defaults.

In case of enclaves, it is not inherently clear what best practices either package authors or users should employ to retain a composable and intuitive launchfile structure. E.g:
Should authors parametrize enclave paths for each node as input arguments?
Should users push namespaces of included launchfiles to unique enclaves?

To be sure though, the setting of security environment variables from within launchfiles should be discouraged, as this would restrict the use of static analysis of launchfiles combined with Node IDL for procedural policy generation.

### Composable nodes in container

Given that containers can be dynamic, where nodes can be added or removed at runtime, there is perhaps some question as to how containers should integrate with secure enclaves.
In ros2launch, the namespace in scope at the container's instantiation could be used to resolve the container's specified relative enclave path, thus to all nodes/components inside that container.
This should be further deliberated when eventually extending the launch API for containers.

### Migration for RMW implementations

As it may take time before all RMW implementations implement the new system of contexts, a defined fallback behavior should still be designated.
For such implementations, the enclave security directory determined by RCL should be loaded for the participant as specified in the "ROS 2 DDS-Security integration" design doc.
This primarily desists the use of including the node name in the default lookup path, consequently getting users in the habit of creating separate enclaves for separate processes, or explicitly specifying unique enclave paths via launchfiles.

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
