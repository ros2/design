---
layout: default
title: ROS 2 Access Control Policies
permalink: articles/ros2_access_control_policies.html
abstract:
  This article specifies the policy format used for access control when securing ROS subsystem.
author:  >
  [Ruffin White](https://github.com/ruffsl),
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


[SROS 2](/articles/ros2_dds_security.html) introduces several security properties, including encryption, authentication, and authorization.
Authorization is obtained by combining the first two properties with a model for access control.
Such models are often referred to as access control policies.
A policy serves as a high-level abstraction of privileges associated with attributes that may then be transpiled into low-level permissions for individual identities, such as specific ROS nodes within a secure DDS network.

## Concepts

Before detailing the SROS 2 policy design of the [access control](https://en.wikipedia.org/wiki/Computer_access_control), by which the system constrains the ability of a subject to access an object, it is important to establish a few concepts useful in formalizing the design approach in terms of security.
In this context, a subject may be thought of as a participant on a distributed data-bus (e.g. a ROS node in the computation graph), whereas an object may be an instance of a particular subsystem (e.g. a ROS topic), and access is defined as the capability to act upon that object (e.g. publish or subscribe).

### Mandatory Access Control

[Mandatory Access Control](https://en.wikipedia.org/wiki/Mandatory_access_control) (MAC) refers to allowing access to an object if and only if rules exist that allow a given subject to access the resource; the term mandatory denotes this requirement that a subject’s access to an object must always be explicitly provisioned.
Most importantly, contrary to discretionary access control (DAC), such policies are enforced by a set of authorization rules that cannot be overridden or modified by the subject either accidentally or intentionally.
This could also be referred to as “deny by default”.

### Principle of Least Privilege

[Principle of Least Privilege](https://en.wikipedia.org/wiki/Principle_of_least_privilege) (PoLP) requires that in a particular abstraction layer, every subject must be able to access only the resources necessary for its legitimate purpose.
This is also known as the principle of minimal privilege or the principle of least authority.
Applying PoLP not only bolsters system security, but can also ease deployment and help improve system stability.

### Privilege Separation

[Privilege Separation](https://en.wikipedia.org/wiki/Privilege_separation) requires that a subject’s access be divided into parts which are limited to the specific privileges they require in order to perform a specific task.
This is used to mitigate the potential damage of a security vulnerability.
Systems unable to comply with this requirement may consequently fail to satisfy PoLP as well.

### Separation of Concerns

[Separation of Concerns](https://en.wikipedia.org/wiki/Separation_of_concerns) (SoC) is a design principle for separating a system into distinct sections, so that each section addresses a separate concern.
Separate concerns in this case may be how encryption is governed in a system versus how authorization is given to subjects.

## Criteria

Design criteria for SROS 2 policies and for selecting the [Extensible Markup Language](https://en.wikipedia.org/wiki/XML) (XML) are discussed here.

### Validation

Prior to interpreting any user configuration input, such as an access control policy, [data validation](https://en.wikipedia.org/wiki/Data_validation) should be applied to ensure inputs are compliant and correctly formatted.
Incorrect inputs can affect the soundness of most programs or tools, yet guarding against general malformations may itself require meticulous validation logic.
Formalizing the description of the data using a precise schema allows for separate programs to assert inputs are compliant without replicating validation logic across implementations.
In XML, this is achieved using [XSD](https://en.wikipedia.org/wiki/XML_schema); allowing the policy markup to be defined by an extendable standard definition rather than a canonical implementation.

### Transformation

For usability and generalizability, access control policies can be expressed using domain specific abstractions, such as ROS based subjects and objects.
However such abstractions may translate into different representations when applied to lower level transports and policy enforcement points.
Formalizing this [data transformation](https://en.wikipedia.org/wiki/Data_transformation) using a transformation language allows for separate programs to transpile without replicating conversion logic across implementations.
In XML, this is achieved using [XSLT](https://en.wikipedia.org/wiki/XML_transformation_language); allowing the policy markup to be easily transpiled for various transports by simply swapping or extending transforms.

### Composition

When formulating an access control policy, many subjects may share fundamental privileges for basic access.
To avoid unnecessary repetition that could exacerbate human errors or other discrepancies, policies should possess sufficient [expressive power](https://en.wikipedia.org/wiki/Expressive_power_(computer_science)) to remain [DRY](https://en.wikipedia.org/wiki/Don%27t_repeat_yourself).
In XML, this is achieved using [XInclude](https://en.wikipedia.org/wiki/XInclude); allowing the policy markup to easily include or substitute external reference to particular profiles and permissions that repeat across separate policies or profiles.

## Schema

The SROS 2 policy schema is defined with XML.
The elements and attributes that make up a policy are described below.

``` xml
{% include_relative ros2_access_control_policies/policy.xsd %}
```

### `<policy>` Tag

Root tag of the policy file.
There must be only one `<policy>` tag per policy file.

Attributes:
- **version**: declared version of schema version in use
  - Allows for advancing future revisions of the schema

### `<contexts>` Tag

Encapsulates a sequence of unique contexts.
This method of nesting sequences allows for additional tags to be extended to the `<policy>` root.

### `<context>` Tag

Encapsulates a collection of profiles.
This is specific to a context as determined by associative attributes.

Attributes:
- **path**: Fully qualified context path

Given that multiple nodes can be composed into a single process, a context is used to contain the collection of profiles of all respective nodes.
A context may therefor be considered the union of contained profiles.
Note that the union of profiles within a context will result in denied privileges of any profile to supersede all allowed privileges for every profile.
See section `<profile>` Tag for more info on MAC is applied. 

### `<profiles>` Tag

Encapsulates a sequence of unique profiles.
This method of nesting sequences allows for additional tags to be extended to the `<context>` root.

### `<profile>` Tag

Encapsulates a collection of subject privileges.
This is specific to a unique node instance as determined by associative attributes.

Attributes:
- **ns**: Namespace of the node
- **node**: Name of the node

In accordance with MAC, privileges must be explicitly qualified as allowed.
Additionally, as with many other MAC languages, while composed privileges may overlap, any particular denied privilege will suppress any similarly applicable allowed privileges.
That is to say the priority of denied privileges conservatively supersedes allowed privileges, avoiding potential lapses in PoLP.
This method of flatting privileges enables users to provision general access to a larger set of objects, while simultaneously revoking access to a smaller subset of sensitive objects.
Although recursion of qualifiers is subsequently prevented, transformations are subsequently simplified, preventing potential for unintended access.

#### Privileges

Privileges are defined as configuration of rules and permissions for object access.
As objects can be categorized by subsystem type, rules and respective permission are identically structured.
Given an average profile is likely to reference more unique objects with multiple permissions than number of rules, the subsequent hierarchy of rules/permissions/objects is chosen to minimize verbosity.

| rule types | permissions        |
|------------|--------------------|
| actions    | call, execute      |
| services   | reply, request     |
| topics     | publish, subscribe |

Each subsystem is associated to a given rule type, while permissions are expressed as attributes in their respective respective rule tags.
Uniqueness or ordering of rules in this sequence is not required, as this is accounted for by transformation templates.
In fact, a profile may contain an empty set of privileges; particularly useful when a node may require no subsystem permissions, but must still be provisioned an identity nonetheless for discovery purposes in DDS.

Each rule includes a sequence of objects that the permissions apply.
For some secure transports, such as [Secure DDS](https://www.omg.org/spec/DDS-SECURITY/About-DDS-SECURITY), matching expressions may also be used to expand the scope further using globbing patterns, specifically those supported by [fnmatch](https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html) as specified in the POSIX standard.
However, caution should be taken when using expression matching, as discussed further in the concerns section.

Basic fnmatch-style patterns are supported:

| Pattern     | Meaning                               |
|-------------|---------------------------------------|
| *           | Matches everything                    |
| ?           | Matches any single character          |
| [sequence]  | Matches any character in sequence     |
| [!sequence] | Matches any character not in sequence |

### `<topics>` Tag

A group of `<topic>` tags with the specified permissions.

Attributes:
- **publish**: Whether or not publication on this set of topics is allowed
  - i.e. whether the node can be a topic publisher
  - Valid values are "ALLOW" or "DENY"
- **subscribe**: Whether or not subscription on this set of topics is allowed
  - i.e. whether the node can be a topic subscriber
  - Valid values are "ALLOW" or "DENY"


### `<services>` Tag

A group of `<service>` tags with the specified permissions.

Attributes:
- **request**: Whether or not requesting the service is allowed
  - i.e. whether the node can be a service client
  - Valid values are "ALLOW" or "DENY"
- **reply**: Whether or not replying to service requests is allowed
  - i.e. whether the node can be a service server
  - Valid values are "ALLOW" or "DENY"


### `<actions>` Tag

A group of `<action>` tags with the specified permissions.

Attributes:
- **call**: Whether or not calling the action is allowed
  - i.e. whether the node can be an action client
  - Valid values are "ALLOW" or "DENY"
- **execute**: Whether or not executing the action is allowed
  - i.e. whether the node can be an action server
  - Valid values are "ALLOW" or "DENY"

## Templating

To transpile SROS 2 policies into security artifacts for a targeted access controlled transport, XSLT templates can be used to perform this level of document conversion.
This may include any number of optimisations or adjustments specific for the target transport.
For example, the pipeline stages for targeting Secure DDS is as follows:

1. An XML document with a root of the SROS 2 policy is specified
2. The document is fully expanded using XInclude to reference external elements
2. The expanded document is then validated with the equivalent schema version
2. At this point the document tree may or may not be pruned to a particular profile
2. The valid document is then transpiled using the transform template
2. For each profile, a matching DDS grant is appended into the permission document
   - privileges and namespaces are remapped into a DDS centric representations
   - privileges with matching attributes are conjoined to reduce payload size
   - duplicate objects in the same privilege are pruned to reduce payload size
   - privileges are sored deny first, abiding the priority of qualifiers when using DDS
   - objects are also sorted alphabetically to improve readability and change diffs

## Alternatives

This section lists concerns about the proposed design and alternatives that were considered, including different [Markup Languages](https://en.wikipedia.org/wiki/Markup_language) and policy formats.

### YAML

[YAML](https://en.wikipedia.org/wiki/YAML), a recursive acronym for “YAML Ain't Markup Language”, was originally adopted for specifying access control policies in the first version of  SROS [1].
Although the policy model used in [SROS 1](http://wiki.ros.org/SROS/Concepts/PolicyDissemination) was semantically equivalent, the YAML format lent it being quite verbose due to repetition of permissions per namespaced resource given the lack of clear element attributes.
For SROS 2 we decided switched away from YAML to XML for many of the reasons weighed in the following pros and cons:

- Pros
    - Human Readable: Minimal Line Noise
        - YAML has very minimal syntax and is targeted for human editable configuration files, making it simple to read and write manually.
    - Data Model: Intuitive Interpretation
        - YAML has very simple data model forming tree structure using key-value pair dictionaries and lists making it quite approachable.
- Cons
    - Parsability: Implicit Type Casting
        - Given YAML is a data-serialization language, it may attempt to type cast where possible.
However this does not always have the desired effect and may lead to unintended behaviors.
Parsing of booleans v.s. strings are notable example of ambiguity.

    - Interpreters: Validation and Transformation
        - Although YAML is supported for many programming languages, YAML itself provides no schema to enforce document structure.
        - Validation must be repeated for each interpreter implementation, rendering it non-agnostic to the programing language used.
        - Similarly, transformations for transpiling policies into transport security artifacts is less generalizable across implementations.
    - Composability: Reuse of Profiles
        - Although YAML supports a degree of composability via Anchors, Aliases and Extensions, allowing documents to be more DRY, these do not extend to separate files or external resources.
    - Expressiveness: Succinct Representation
        - Given YAML’s inherit data model, it’s expressive power is quite limited, necessitating either verbose file structures, or unintuitive options to achieve similar access control configurations.

### Custom

As an alternative to choosing an existing markup format, it would be possible to define our own formal language for expressing access control permissions for ROS 2 using a custom file syntax.
An example of a MAC based policy language would include that which is used in [AppArmor](https://gitlab.com/apparmor/apparmor/wikis/home).
Although affording the flexibility to succinctly express profile permission while minimizing general syntactic overhead,  this approach was not pursued for many of the reasons weighed in the following pros and cons:

- Pros
    - Expressiveness: Succinct Representation
        - Complete control of syntax and interpretation, allowing for domain specific optimizations for SROS policy representation.
- Cons
    - Interpreters: Validation and Transformation
        - Specification and implementation for parsing and interpreting a custom policy format would be considered undertaking.
        - Validation must be repeated for each interpreter implementation, rendering it non-agnostic to the programing language used.
        - Similarly, transformations for transpiling policies into transport security artifacts is less generalizable across implementations.
    - Correctness: Policy Remaining Sound
        - Maintaining and synchronizing parsing support across multiple programing language could affect policy soundness.

### ComArmor

[ComArmor](https://github.com/ComArmor/comarmor) [2], a predecessor and inspiration for the existing XML ROS 2 policy format now used in SROS 2, is itself inspired from AppArmor’s policy language.
ComArmor facilitates composition through the use of a nested tree structure of policy/profile/permission primitives.
As with AppArmor, it also supports nesting of profiles, i.e. importation of child profiles into that of a parent profile.
While this greatly extends the flexibility given the nesting of imported sun-profile hierarchies, it also adds complexity to the transpiling process when converting policies to security transport artifacts.

In an effort to strike a balance between simplicity and flexibility, a flat sequence of single level profiles was opted for SROS 2 instead, allowing the policy format to serve as both a grounded intermediate representation for higher level policy languages and tools to build upon, such as [XACML](https://en.wikipedia.org/wiki/XACML) or [Keymint](https://github.com/keymint/keymint_tools), while remaining succinctly expressive of ROS concepts for general use.

## Concerns

### Separation of Privileges

ROS 2 subsystems such as topics, services, actions, and parameters must eventually map to transport layer interfaces, such as DDS topic, that can sufficiently enforce the desired access control policy in order to secure the ROS application layer.
However, any quirks between mapping of subsystems and separation of privileges can degridate security.

As an example, if granting access to all topics and services starting with `/foo` additionally grants access to all actions starting with `/foo`, this would be a week example of privilege separation.
Such can be exacerbated when using globbing expressions that include matching patterns, such as with `fnmatch`, leading to innocuous and sound policies being inaccurately applied to the underlying transport security.
While such privilege separation in [remains week between ROS 2 and DDS](https://github.com/ros2/design/pull/203), perhaps it is wise to discourage the use of expression matching for general use in permissions.

### Separation of Concerns

Middleware transports, such as DDS, offer a myriad of features and options, such as those for QoS as well as security.
Drawing a boundary between many of them when deciding what to expose from a configuration standpoint can be tricky.
Still, among the objective for ROS 2 includes remaining as agnostic of transport as reasonable.

Although the SROS 2 policy format was intentionally structured to mimic that of Secure DDS’s permission.xml format, care should be taken when adding extensions to surface non-functional security properties tangential to ROS 2 data flow, such as governance on encryption or discovery.

Yet, if the intended purpose of SROS 2 policy becomes that of an intermediate representation across transports, and is subsequently auto generated from higher level tooling/representations, or composability is adequate preserved, then perhaps this concern is of lesser priority.

### Composability

ROS 2 allows for the remapping of many namespaced subsystems at runtime, such as when reusing launch files to orchestrate larger applications.
While it is perhaps unreasonable to expect this dynamic flexibility from staticky provisioned permissions without allocating such capabilities prior, it should be made possible to infer the necessary capabilities from composed launch files and similar orchestrations.

Static analysis of such remapping in conjunction with the context of the nominal requirements of respective nodes could be used to auto generate the new satisfactory policies.
However, inferring such context from the source code could be equated to the halting problem.
Thus, it stands to reason nodes could instead provide a manifest or IDL defining these nominal requirements so that permission may as easily be remapped, at least at design time.

## References

1. [SROS1: Using and Developing Secure ROS1 Systems](https://doi.org/10.1007/978-3-319-91590-6_11)

``` bibtex
@inbook{White2019,
	title     = {SROS1: Using and Developing Secure ROS1 Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Christensen, Henrik and Cortesi, Agostino},
	year      = 2019,
	booktitle = {Robot Operating System (ROS): The Complete Reference (Volume 3)},
	doi       = {10.1007/978-3-319-91590-6_11},
	isbn      = {978-3-319-91590-6},
	url       = {https://doi.org/10.1007/978-3-319-91590-6_11}}
```

2. [Procedurally Provisioned Access Control for Robotic Systems](https://doi.org/10.1109/IROS.2018.8594462)

``` bibtex
@inproceedings{White2018,
	title     = {Procedurally Provisioned Access Control for Robotic Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Christensen, Henrik and Cortesi, Agostino},
	year      = 2018,
	booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	doi       = {10.1109/IROS.2018.8594462},
	issn      = {2153-0866},
	url       = {https://doi.org/10.1109/IROS.2018.8594462}}
```
