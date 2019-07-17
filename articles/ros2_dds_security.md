---
layout: default
title: ROS 2 DDS-Security integration
permalink: articles/ros2_dds_security.html
abstract: >
  Robotics is full of experimentation: evaluating different hardware and software, pushing ahead with what works, and culling what doesn't.
  ROS was designed to be flexible to enable this experimentation; to allow existing components to easily be combined with new ones or swapped with others.
  In ROS 1, this flexibility was valued above all else, at the cost of security.
  By virtue of being designed on top of DDS, ROS 2 is able to retain that flexibility while obtaining the ability to be secured by properly utilizing the DDS-Security specification.
  This article describes how ROS 2 integrates with DDS-Security.
author:  >
  [Kyle Fazzari](https://github.com/kyrofa)

published: true
categories: Security
---

{:toc}


# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


# DDS-Security overview

The [DDS-Security specification][dds_security] expands upon the [DDS specification][dds], adding security enhancements by defining a Service Plugin Interface (SPI) architecture, a set of builtin implementations of the SPIs, and the security model enforced by the SPIs. Specifically, there are five SPIs defined:

- **Authentication**: Verify the identity of a given domain participant.
- **Access control**: Enforce restrictions on the DDS-related operations that can be performed by an authenticated domain participant.
- **Cryptographic**: Handle all required encryption, signing, and hashing operations.
- **Logging**: Provide the ability to audit DDS-Security-related events.
- **Data tagging**: Provide the ability to add tags to data samples.

ROS 2's security features currently utilize only the first three.
This is due to the fact that neither **Logging** nor **Data Tagging** are required in order to be compliant with the [DDS-Security spec][dds_security] (see section 2.5), and thus not all DDS implementations support them.
Let's delve a little further into those first three plugins.


## Authentication

The **Authentication** plugin (see section 8.3 of the [DDS-Security spec][dds_security]) is central to the entire SPI architecture, as it provides the concept of a confirmed identity without which further enforcement would be impossible (e.g. it would be awfully hard to make sure a given ROS node could only access specific topics if it was impossible to securely determine which node it was).

The SPI architecture allows for a number of potential authentication schemes, but ROS 2 uses the builtin authentication plugin (called "DDS:Auth:PKI-DH", see section 9.3 of the [DDS-Security spec][dds_security]), which uses the proven Public Key Infrastructure (PKI).
It requires a public and private key per domain participant, as well as an x.509 certificate that binds the participant's public key to a specific name.
Each x.509 certificate must be signed by (or have a signature chain to) a specific Certificate Authority (CA) that the plugin is configured to trust.


## Access control

The **Access control** plugin (see section 8.4 of the [DDS-Security spec][dds_security]) deals with defining and enforcing restrictions on the DDS-related capabilities of a given domain participant.
For example, it allows a user to restrict a particular participant to a specific DDS domain, or only allow the participant to read from or write to specific DDS topics, etc.

Again the SPI architecture allows for some flexibility in how the plugins accomplish this task, but ROS 2 uses the builtin access control plugin (called "DDS:Access:Permission", see section 9.4 of the [DDS-Security spec][dds_security]), which again uses PKI.
It requires two files per domain participant:

- **Governance** file: A signed XML document specifying how the domain should be secured.
- **Permissions** file: A signed XML document containing the permissions of the domain participant, bound to the name of the participant as defined by the authentication plugin (which is done via an x.509 cert, as we just discussed).

Both of these files must be signed by a CA which the plugin is configured to trust.
This may be the same CA that the **Authentication** plugin trusts, but that isn't required.


## Cryptographic

The **Cryptographic** plugin (see section 8.5 of the [DDS-Security spec][dds_security]) is where all the cryptography-related operations are handled: encryption, decryption, signing, hashing, etc.
Both the **Authentication** and **Access control** plugins utilize the capabilities of the **Cryptographic** plugin in order to verify signatures, etc.
This is also where the functionality to encrypt DDS topic communication resides.

ROS 2 uses the builtin cryptographic plugin (called "DDS:Crypto:AES-GCM-GMAC", see section 9.5 of the [DDS-Security spec][dds_security]), which provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter Mode (AES-GCM).


# DDS-Security integration with ROS 2: SROS 2

Now that we have established some shared understanding of how security is supported in DDS, let's discuss how that support is exposed in ROS 2.
By default, none of the security features of DDS are enabled in ROS 2.
The set of features and tools in ROS 2 that are used to enable them are collectively named "Secure ROS 2" (SROS 2).


## Features in the ROS client library (RCL)

Most of the user-facing runtime support for SROS 2 is contained within the [ROS Client Library](https://github.com/ros2/rcl).
Once its requirements are satisfied it takes care of configuring the middleware support for each supported DDS implementation.
RCL includes the following features for SROS 2:

- Support for security files for each domain participant.
- Support for both permissive and strict enforcement of security.
- Support for a master "on/off" switch for all SROS 2 features.

Let's discuss each of these in turn.


### Security files for each domain participant

As stated earlier, the DDS-Security plugins require a set of security files (e.g. keys, governance and permissions files, etc.) per domain participant.
Domain participants map to a specific instance of a node in ROS 2, so each node requires a set of these files.
RCL supports being pointed at a directory containing security files in two different ways:

- Directory tree of all security files.
- Manual specification.

Let's delve further into these.


#### Directory tree of all security files

RCL supports finding security files in one directory that is the root of a directory structure corresponding to the fully-qualified names of every node instance (i.e. namespace + node name).
For example, for the `/front/camera` node, the directory structure would look like:

    <root>
    └── front
        └── camera
            ├── cert.pem
            ├── key.pem
            ├── ...

To be clear: this directory structure needs to reflect the state of the running system.
In other words, it does not contain a set of files per node on disk, but per node instance _in the ROS graph_.

The set of files expected within each node instance directory are:

- **identity_ca.cert.pem**: The x.509 certificate of the CA trusted by the **Authentication** plugin (the "Identity" CA).
- **cert.pem**: The x.509 certificate of this node instance (signed by the Identity CA).
- **key.pem**: The private key of this node instance.
- **permissions_ca.cert.pem**: The x.509 certificate of the CA trusted by the **Access control** plugin (the "Permissions" CA).
- **governance.p7s**: The XML document that specifies to the **Access control** plugin how the domain should be secured  (signed by the Permissions CA).
- **permissions.p7s**: The XML document that specifies the permissions of this particular node instance to the **Access control** plugin (also signed by the Permissions CA).

This can be specified by setting the `$ROS_SECURITY_ROOT_DIRECTORY` environment variable to point to the root of the directory tree.


##### Support security files lookup methods

If using the directory tree approach to organize security files, RCL supports two different methods for looking up a given node instance's security files in the tree:

- **Exact**: Only load security files from a directory exactly matching the fully-qualified name of the node instance.
For example, given a node named "baz_123" within the "/foo/bar/" namespace, only load security files from `<root>/foo/bar/baz_123/`.
This is the default behavior.
- **Prefix**: Attempt to load the most specific set of security files, but if they can't be found, check for security files under a less-specific node name.
For example, given a node named "baz_123" within the "/foo/bar/" namespace, load security files from `<root>/foo/bar/baz_123/`.
However, if that directory doesn't exist, find the most specific (i.e. longest) node name that _does_ have security files within that namespace (e.g. `<root>/foo/bar/baz_12/`, or `<root>/foo/bar/baz/`, etc.).
Note that it will not search higher in the namespace hierarchy.

The desired lookup method can be specified by setting the `$ROS_SECURITY_LOOKUP_TYPE` environment variable to "MATCH_EXACT" (case-sensitive) for the **Exact** method, or "MATCH_PREFIX" (case-sensitive) for the **Prefix** method.


#### Manual specification

RCL supports specifying the path to a directory containing the set of security files for the exact node instance that needs to be launched.
The set of files expected within that directory are the same as outlined in the "Directory tree of all security files" section above for individual node instance directories.

This can be specified by setting the `$ROS_SECURITY_NODE_DIRECTORY` environment variable to point to the directory containing the security files.
Note that this setting takes precedence over `$ROS_SECURITY_ROOT_DIRECTORY`.


### Support for both permissive and strict enforcement of security

Nodes with the security features enabled will not communicate with nodes that don't, but what should RCL do if one tries to launch a node that has no discernable keys/permissions/etc.? It has two options:

- **Permissive mode**: Try to find security files, and if they can't be found, launch the node without enabling any security features.
This is the default behavior.
- **Strict mode**: Try to find security files, and if they can't be found, fail to run the node.

The type of mode desired can be specified by setting the `$ROS_SECURITY_STRATEGY` environment variable to "Enforce" (case-sensitive) for strict mode, and anything else for permissive mode.


### Support for a master "on/off" switch for all SROS 2 features

In addition to the supported features just discussed, RCL also supports a master shutoff for security features for easy experimentation.
If it's turned off (the default), none of the above security features will be enabled.

In order to enable SROS 2, set the `$ROS_SECURITY_ENABLE` environment variable to "true" (case-sensitive).
To disable, set to any other value.


## Features in the SROS 2 CLI

Configuring a ROS 2 system to be secure in RCL involves a lot of new technology (PKI, DDS governance and permissions files and their syntax, etc.).
If a user is comfortable with these technologies, the above information should be all that's necessary to properly lock things down.
However, the [SROS 2 CLI](https://github.com/ros2/sros2) includes a number of commands to help those who don't want to set it all up themselves.
This section is not meant to be a tutorial, but a reference of capabilities that are available.


    $ ros2 security create_keystore <keystore path>

This creates the initial directory structure that will be the `$ROS_SECURITY_ROOT_DIRECTORY`, and also creates a self-signed CA and uses it to sign a governance file that will encrypt all DDS traffic.


    $ ros2 security create_key <keystore path> <fully-qualified node instance name>

This creates a new identity for the node, generating a keypair and signing its x.509 certificate using the CA created in `create_keystore`.
It also creates an initial permissions document (that allows everything by default) and signs it with the same CA.
It then copies in the governance file created in `create_keystore` as well as the x.509 certificate for the CA to act as both the identity CA as well as the permissions CA.


    $ ros2 security generate_policy <policy file>

This takes the current state of the ROS graph (i.e. all the nodes currently running, along with their topic publications/subscriptions, services, etc.) and turns it into an [SROS 2 policy file](/articles/sros2_policy_files.html).


    $ ros2 security create_permission \
        <keystore path> \
        <fully-qualified node instance name> \
        <policy file>

This takes an SROS 2 policy file and uses it to generate (and sign) the DDS permissions document for the given node instance.


    $ ros2 security generate_artifacts \
        -k <keystore path> \
        -n <space-separated list of fully-qualified node instance names> \
        -p <space-separated list of policy files>

This is essentially a combination of `create_keystore`, `create_key` and `create_permission`.
It takes a set of fully-qualified node instance names as well as a set of policy files and generates the keystore (if it doesn't already exist) as well as the entire set of keys and permissions necessary for them to run securely.


[dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF
[dds]: https://www.omg.org/spec/DDS/1.4/PDF
