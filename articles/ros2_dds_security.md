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

The [DDS-Security specification][dds_security] expands upon the [DDS specification][dds], adding security enhancements by defining a Service Plugin Interface (SPI) architecture, a set of builtin implementations of the SPIs, and the security model enforced by the SPIs.
Specifically, there are five SPIs defined:

- **Authentication**: Verify the identity of a given domain participant.
- **Access control**: Enforce restrictions on the DDS-related operations that can be performed by an authenticated domain participant.
- **Cryptographic**: Handle all required encryption, signing, and hashing operations.
- **Logging**: Provide the ability to audit DDS-Security-related events.
- **Data tagging**: Provide the ability to add tags to data samples.

ROS 2's security features currently utilize only the first three.
This is due to the fact that neither **Logging** nor **Data Tagging** are required in order to be compliant with the [DDS-Security spec][dds_security] (see section 2.3), and thus not all DDS implementations support them.
Let's delve a little further into those first three plugins.


## Authentication

The **Authentication** plugin (see section 8.3 of the [DDS-Security spec][dds_security]) is central to the entire SPI architecture, as it provides the concept of a confirmed identity without which further enforcement would be impossible (e.g. it would be awfully hard to make sure a given ROS context could only access specific topics if it was impossible to securely determine which context it was).

The SPI architecture allows for a number of potential authentication schemes, but ROS 2 uses the builtin authentication plugin (called "DDS:Auth:PKI-DH", see section 9.3 of the [DDS-Security spec][dds_security]), which uses the proven Public Key Infrastructure (PKI).
It requires a public and private key per domain participant, as well as an x.509 certificate that binds the participant's public key to a specific name.
Each x.509 certificate must be signed by (or have a signature chain to) a specific Certificate Authority (CA) that the plugin is configured to trust.

The rationale for using the builtin plugin as opposed to anything else is twofold:

1. It's the only approach described in detail by the spec.
2. It's mandatory for all compliant DDS implementations to interoperably support it (see section 2.3 of the [DDS-Security spec][dds_security]), which makes the ROS 2 security features work across vendors with minimal effort.


## Access control

The **Access control** plugin (see section 8.4 of the [DDS-Security spec][dds_security]) deals with defining and enforcing restrictions on the DDS-related capabilities of a given domain participant.
For example, it allows a user to restrict a particular participant to a specific DDS domain, or only allow the participant to read from or write to specific DDS topics, etc.

Again the SPI architecture allows for some flexibility in how the plugins accomplish this task, but ROS 2 uses the builtin access control plugin (called "DDS:Access:Permission", see section 9.4 of the [DDS-Security spec][dds_security]), which again uses PKI.
It requires two files per domain participant:

- **Governance** file: A signed XML document specifying how the domain should be secured.
- **Permissions** file: A signed XML document containing the permissions of the domain participant, bound to the name of the participant as defined by the authentication plugin (which is done via an x.509 cert, as we just discussed).

Both of these files must be signed by a CA which the plugin is configured to trust.
This may be the same CA that the **Authentication** plugin trusts, but that isn't required.

The rationale for using the builtin plugin as opposed to anything else is the same as the **Authentication** plugin:

1. It's the only approach described in detail by the spec.
2. It's mandatory for all compliant DDS implementations to interoperably support it (see section 2.3 of the [DDS-Security spec][dds_security]), which makes the ROS 2 security features work across vendors with minimal effort.


## Cryptographic

The **Cryptographic** plugin (see section 8.5 of the [DDS-Security spec][dds_security]) is where all the cryptography-related operations are handled: encryption, decryption, signing, hashing, etc.
Both the **Authentication** and **Access control** plugins utilize the capabilities of the **Cryptographic** plugin in order to verify signatures, etc.
This is also where the functionality to encrypt DDS topic communication resides.

While the SPI architecture again allows for a number of possibilities, ROS 2 uses the builtin cryptographic plugin (called "DDS:Crypto:AES-GCM-GMAC", see section 9.5 of the [DDS-Security spec][dds_security]), which provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter Mode (AES-GCM).

The rationale for using the builtin plugin as opposed to anything else is the same as the other plugins:

1. It's the only approach described in detail by the spec.
2. It's mandatory for all compliant DDS implementations to interoperably support it (see section 2.3 of the [DDS-Security spec][dds_security]), which makes the ROS 2 security features work across vendors with minimal effort.


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
Domain participants map to a specific instance of a context in ROS 2, so each context requires a set of these files.
RCL supports being pointed at a directory containing security files in two different ways:

- Directory tree of all security files.
- Manual specification.

Let's delve further into these.


#### Directory tree of all security files

RCL supports finding security files in one directory that is the root of a directory structure corresponding to the fully-qualified names of every context (e.g. namespace + context name).
For example, for the `/front/camera` context, the directory structure would look like:

    <root>
    └── contexts
        └── front
            └── camera
                ├── cert.pem
                ├── key.pem
                ├── ...

To be clear: this directory structure needs to reflect the state of the running system.
In other words, it does not contain a set of files per context on disk, but per context instance _in the ROS graph_.

The set of files expected within each context instance directory are:

- **identity_ca.cert.pem**: The x.509 certificate of the CA trusted by the **Authentication** plugin (the "Identity" CA).
- **cert.pem**: The x.509 certificate of this context instance (signed by the Identity CA).
- **key.pem**: The private key of this context instance.
- **permissions_ca.cert.pem**: The x.509 certificate of the CA trusted by the **Access control** plugin (the "Permissions" CA).
- **governance.p7s**: The XML document that specifies to the **Access control** plugin how the domain should be secured  (signed by the Permissions CA).
- **permissions.p7s**: The XML document that specifies the permissions of this particular context instance to the **Access control** plugin (also signed by the Permissions CA).

This can be specified by setting the `$ROS_SECURITY_ROOT_DIRECTORY` environment variable to point to the root of the directory tree.


##### Support security files lookup methods

If using the directory tree approach to organize security files, RCL supports two different methods for looking up a given context instance's security files in the tree:

- **Exact**: Only load security files from a directory exactly matching the fully-qualified name of the context.
For example, given a context "baz_123" within the "/foo/bar/" namespace, only load security files from `<root>/contexts/foo/bar/baz_123/`.
This is the default behavior.
- **Prefix**: Attempt to load the most specific set of security files, but if they can't be found, check for security files under a less-specific context path.
For example, given a context "baz_123" within the "/foo/bar/" namespace, load security files from `<root>/contexts/foo/bar/baz_123/`.
However, if that directory doesn't exist, find the most specific (i.e. longest) context path that _does_ have security files within that namespace (e.g. `<root>/contexts/foo/bar/baz_12/`, or `<root>/contexts/foo/bar/baz/`, etc.).
Note that it will not search higher in the namespace hierarchy.

The desired lookup method can be specified by setting the `$ROS_SECURITY_LOOKUP_TYPE` environment variable to "MATCH_EXACT" (case-sensitive) for the **Exact** method, or "MATCH_PREFIX" (case-sensitive) for the **Prefix** method.


#### Manual specification

RCL supports specifying the path to a directory containing the set of security files for the exact context instance that needs to be launched.
The set of files expected within that directory are the same as outlined in the "Directory tree of all security files" section above for individual context instance directories.

This can be specified by setting the `$ROS_SECURITY_CONTEXT_DIRECTORY` environment variable to point to the directory containing the security files.
Note that this setting takes precedence over `$ROS_SECURITY_ROOT_DIRECTORY`.


### Support for both permissive and strict enforcement of security

Contexts with the security features enabled will not communicate with contexts that don't, but what should RCL do if one tries to launch a context that has no discernable keys/permissions/etc.? It has two options:

- **Permissive mode**: Try to find security files, and if they can't be found, launch the context without enabling any security features.
This is the default behavior.
- **Strict mode**: Try to find security files, and if they can't be found, fail to run the context.

The type of mode desired can be specified by setting the `$ROS_SECURITY_STRATEGY` environment variable to "Enforce" (case-sensitive) for strict mode, and anything else for permissive mode.


### Support for a master "on/off" switch for all SROS 2 features

In addition to the supported features just discussed, RCL also supports a master shutoff for security features for easy experimentation.
If it's turned off (the default), none of the above security features will be enabled.

In order to enable SROS 2, set the `$ROS_SECURITY_ENABLE` environment variable to "true" (case-sensitive).
To disable, set to any other value.


## Features in the SROS 2 CLI

Configuring a ROS 2 system to be secure in RCL involves a lot of new technology (PKI, DDS governance and permissions files and their syntax, etc.).
If a user is comfortable with these technologies, the above information should be all that's necessary to properly lock things down.
However, the [SROS 2 CLI](https://github.com/ros2/sros2) should include a tool `ros2 security` to help those who don't want to set it all up themselves, including the following capabilities:

- Create Identity and Permissions CA.
- Create directory tree containing all security files.
- Create a new identity for a given context instance, generating a keypair and signing its x.509 certificate using the Identity CA.
- Create a governance file that will encrypt all DDS traffic by default.
- Support specifying context instance permissions [in familiar ROS terms](/articles/ros2_access_control_policies.html) which are then automatically converted into low-level DDS permissions.
- Support automatically discovering required permissions from a running ROS system.


[dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF
[dds]: https://www.omg.org/spec/DDS/1.4/PDF
