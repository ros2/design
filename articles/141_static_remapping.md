---
layout: default
title: Remapping Names
permalink: articles/static_remapping.html
abstract:
  Topics, parameters, and services are identified by [Names](http://wiki.ros.org/Names). Names are hard coded in ROS nodes, but they can be changed at runtime through remapping. Without remapping every instance of a node would require changes in code. This article describes the requirements, rationale, and mechanisms for remapping names in ROS 2.
author: '[Shane Loretz](https://github.com/sloretz)'
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Why remap names

Remapping names allows reusing the same node executable in different parts of the system.
A robot that has multiple sensors of the same type could launch multiple instances of the same node with outputs remapped to different topics.

```
            +-------------+
 (Lidar 1)--> node inst 1 +-->/head_scan
            +-------------+

            +-------------+
 (Lidar 2)--> node inst 2 +-->/base_scan
            +-------------+
```

## Structure of a Name

The complete definition of a name is [here](http://design.ros2.org/articles/topic_and_service_names.html).
It should be read before reading this article.

### Quick Summary

If a name begins with `/` it is called a **Fully Qualified Name** (FQN) otherwise it is called a **relative name**.
The strings between slashes are called **tokens**.
Names are conceptually divided into two pieces: **namespace** and **basename**.
The basename is the last token in a name.
The namespace is everything prior to the basename.

### Example names

- `/foo`
- `/foo/bar`
- `~/foo/bar`
- `{node}/bar`
- `bar`

## Structure of a Remapping Rule

**Remapping rules** are the instructions describing how a node should change the names it uses.
Remapping rules have two parts.
The first part is used to determine if the rule applies to a name.
The second part is the replacement for a matched name.
The act of replacing one name with another is **remapping**.

## ROS 2 Remapping Use cases

These use cases are being considered for remapping in ROS 2:

- Remap One Node in a Process
- Change a Namespace
- Change a Basename
- Change a Token
- Pre-FQN Remapping
- Exact FQN Replacement
- Exact Relative Name Replacement
- Remap via Command Line
- Change the Default Namespace
- Change the Node Name
- Remap Topic and Service Names Separately

### Remap One Node in a Process

This is the ability to apply remap rules to one node in a process without affecting the other nodes.
Because processes in ROS 2 can contain multiple nodes, it is possible multiple nodes in a process may use the same name for different purposes.
A user may want to change a name used in one node without affecting the rest.

### Change a Namespace

Nodes are said to be in a namespace or have a **default namespace**.
This namespace gets prepended to all relative names used by the node.
This use case is the ability to change the namespace of multiple names with one rule.

A popular ROS 1 package [actionlib](http://wiki.ros.org/actionlib) creates 5 topics with the same namespace.
In ROS 1 remapping an actionlib client or server means creating 5 remapping rules.
In ROS 2 just one rule could remap them all.

*Example:*

- Node provides an actionlib server `move_head` and checks a parameter called `move_head`
- A rule remaps namespace `move_head` to `move_head_check_collision`
- All 5 actionlib topics are remapped to `/move_head_check_collision`, but the parameter name remains unchanged

### Change a Basename

This is the ability to change the basename of multiple names with one rule.
It's possible a user may want to change multiple instances of a basename to another token.

*Example:*

- A node uses names `/scan/head/scan`, `/base/scan`
- A user wants the node to subscribe to the same data after some processing
- The user remaps basename `scan` to `scan_filtered`
- The final topics are `/scan/head/scan_filtered`, `/base/scan_filtered`

### Change a Token

This is the ability to change a token in multiple names regardless of where it appears.
It is possible a token is used throughout an interface, but is undesirable to the end user.
This means it should be possible to make a rule that replaces all uses of this token.

*Example:*

- A company sells a generic mobile robot base with a ROS 2 driver
- The driver uses lots of names with the company's name in it: `UmbrellaCorp`
- Another company incorporates the base into their product, and their customers want a ROS 2 interface
- The second company doesn't want their interface to contain `UmbrellaCorp`, so they remap the token to `mobile_base` when launching the driver

### Pre-FQN Remapping

This is the ability to match a name by how it is used in code.
Doing so requires matching prior to FQN expansion.
This could be useful when two different names expand to the same FQN.

*Example:*

- A node uses two names `cat` and `/ns/cat`
- The node is run in namespace `/ns/`, so the FQN of both names is `/ns/cat`
- A rule remaps Pre-FQN expansion `cat` to `lion`
- The final names are `/ns/lion` and `/ns/cat`

### Exact FQN Replacement

This is the ability to replace a name by exactly matching it.
This is part of the behavior of ROS 1 remapping, so it has proven useful and including it will ease the transition to ROS 2.

*Example:*

- A node uses names `/ns/bar` and `/ns/barista`
- A rule is created to remap `/ns/bar` to `/ns/foo`
- The final names are `/ns/foo` and `/ns/barista`

### Exact Relative Name Replacement

This allows a user to remap a relative name to another name.
It works by first expanding the relative name and then doing FQN replacement.
This is also part of ROS 1 remapping.

*Example:*

- A node is in namespace `/ns/` and uses name `bar`
- A rule is created to remap `bar` to `foo`
- The node's use of `bar` is expanded to `/ns/bar`
- Both sides of the remap rule are expanded to `/ns/bar` and `/ns/foo`
- Because the FQN match, the final name is `/ns/foo`

### Remap via Command Line

A user can supply node specific remapping arguments via the command line.
Because a process can contain multiple nodes, there must be a way to uniquely identify a node in a process.
This is a feature of ROS 1 remapping.

*Example:*

- Node is an executable at `/bin/my_node`
- User wants to change `/cat` to `/dog`
- User types `/bin/my_node /cat:=/dog`

### Change the Default Namespace

The default namespace is the one in which relative names get expanded to.
This should be changeable without affecting FQN.
ROS 1 has this feature using either the environment variable `ROS_NAMESPACE` or the argument `__ns`.

*Example:*

- Node is in namespace `/ns` and uses relative name `bar`
- User changes the default namespace to `/foo`
- The final name is `/foo/bar`

### Change the Node Name

The node name is used in log messages and to create private names.
ROS 1 has this feature using the argument `__name`.

*Example:*

- Node is named `camera_driver` and uses a private name `camera_info`
- User changes the node name to `left_camera_driver`
- The final name is `/ns/left_camera_driver/camera_info` and log messages use `left_camera_driver`

### Remap Topic and Service Names Separately

This is the ability to create a rule that will remap only topics or only services.

*Example:*

- Node subscribes to a topic `/map` and offers a service `/map`
- User changes the topic name to `/map_stream`
- The node is subscribed to topic `/map_stream` and offers a service `/map`

## Remapping Names in ROS 1

Remapping is a feature that also exists in ROS 1.
In ROS 1 remapping works by passing in [arguments](http://wiki.ros.org/Remapping%20Arguments) to each node.
Client libraries also have APIs in code to pass remapping rules when the node is initialized.
A remap rule consists of two names: one that should be replaced with another.

ROS 1 remapping works on **Fully Qualified Names** (FQN).
Both sides of a rule are [expanded to FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#a377ff8fede7b95398fd5d1c5cd49246b).
Before a name is remapped it is also [expanded to FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#ab2eebaf734abfbdccb4122f8e24f547f).
The name is remapped to the right side only if it exactly matches the left side of a rule.

## Remapping rule syntax

This is a proposal for the ROS 2 remapping rule syntax.
It attempts to be the same as ROS 1 syntax when possible.

Use cases supported by this syntax:

- Remap One Node in a Process
- Change a Namespace
- Change a Basename
- Exact FQN Replacement
- Exact Relative Name Replacement
- Remap via Command Line
- Change the Default Namespace
- Change the Node Name
- Remap Topic and Service Names Separately

Not supported:

- Change a Token
- Pre-FQN Remapping

### How the Syntax Works

The structure of a remapping rule is `match:=replacement`.
`match` tests if a name should be remapped.
`replacement` says what the new name will be.
`:=` behaves the same as it does in ROS 1.

Example rules are:

- `foo:=bar`
- `/foo/bar:=fiz/buzz`
- `nodename:~/foo:=foo`
- `/ns1/nodename:foo:=bar`
- `**/foo:=\1/bar`

#### Match Part of a Rule

The match part of a rule uses these operators:

- `*` matches a single token
- `**` matches zero or more tokens delimited by slashes
- `rosservice://` prefixed to the match makes the rule apply to only services
- `rostopic://` prefixed to the match makes the rule apply to only topics
- `nodename:` prefixed to the match makes it apply only to a node with that name, which could be a FQN

The operators `*` and `**` are similar to the globbing behavior in bash.
`**` behaves similar to its use in bash>=4.0 with the globstar option set.

The URL schemes `rosservice://` and `rostopic://` may only be given to topic or service name rules.
They may not be prefixed to a node name or namespace replacement rule (`__name`, `__node`, or `__ns`).
If both a node name prefix and URL scheme are given, the node name prefix must come first.

`*`, and `**` match whole tokens only.
`*bar` looks like it would match `foobar`, but that would mean matching a partial token.
To avoid confusion they are required to be separated from tokens, substitutions, and each other by a `/`.
For example `*/bar` `**/*` `~/*` are allowed, but `*bar` `***` `~*` are invalid.

Matching works on FQN only.
When a name is to be tested the substitution operators (`~` and `{}`) in the name and in the rule are replaced with the content they stand for.
Then the name is expanded to a FQN.
If the match part of a rule does not begin with `/`, `*`, or `**` it is prefixed with `/namespace/` to make it a FQN.
Finally the name is compared against the match part of the rule.
If the name matches it is remapped.

#### Replacement Part of a rule

These special operators are unique to the replacement part of a rule:

- `\1` - `\9` are replaced with the matched content of a `*` or `**`

The syntax for `\1` through `\9` was taken from backreferences in POSIX BRE.
However, parenthesis are not used; the wild cards always capture.

These references are required to be separated from tokens by a `/`.
When this creates a name with `//` one slash is automatically deleted.
For example `**/bar:=/bar/\1` matches the name `/foo/bar` with `**` capturing `/foo`, but the new name is `/bar/foo`.

The replacement part of a rule may not have a URL scheme.
This is to avoid a mismatch between the scheme type of the match side and of the replacement side.

The substitution operators (`~` and `{}`) are replaced first.
Afterwards the reference operators are replaced with the matched content.
Then if the replacment name does not begin with `/` it is automatically prefixed with the node's default namespace to make it a FQN.
Finally the name is replaced with the replacement.
For example, `/bar/*:=\1/bar` matches the name `/bar/foo` use by a node with default namespace `/ns` with `*` capturing `foo` and replacement name `/ns/foo/bar`.

#### Special Rule for Changing the Default Namespace

The string `__ns` can be given on the match part of a rule to signal a change of the default namespace.
On the match side `__ns` must be used by itself or with a `nodename:` prefix.
The replacement side of a rule must have a FQN which will become the new default namespace.

#### Special Rule for Changing the Node Name

The strings `__name` or  `__node` can be given on the match part of a rule to signal a change of the node's name.
On the match side it may be used by itself or with a `nodename:` prefix.
The replacement must be a single token which will become the node's new name.

#### Order of Applying Remapping Rules

Remapping rules are applied in the following order:

1. Namespace remapping
1. Node name remapping
1. All other rules

Within each category, the rules are applied in the order in which the user gave them.

**Example of topic/service remapping order:**

- a node uses a name `/foo/bar`
- a user gives the node a rule `/*/*:=/asdf` and then `/foo/bar:=fizzbuzz`
- the final name is `/asdf` because the name did not match the second rule after being remapped by the first rule

**Example of node/namespace remapping order:**

- A node has name `talker`
- A user gives the rules `talker:__ns:=/my_namespace` then `/talker:__node:=foo`
- The final node name is the default `talker` because the namespace remap is applied before the node name remap

**Example of a default and node specific namespace remap:**

- A node has name `talker`
- A user gives the rules `talker:__ns:=/foo` then `__ns:=/bar`
- talker's namespace is `/foo` because that rule was given first

### Applications of the syntax

The following sections explain how the syntax enables the use cases above.

#### Supporting: Remap One Node in a Process

Remapping a node in a process requires a way to uniquely identify a node.
Assuming the node's name is unique in a process, a rule can be prefixed with the name of the target node and a `:`.
If the node name is not prefixed, the rule will be applied to all nodes in the process.

*Example:*

- Multiple nodes in a process use the name `scan`
- A rule is given to one node `node1:scan:=scan_filtered`
- Only the node named `node1` uses the rule

#### Supporting: Change a Namespace

There are two cases: changing part of a namespace, and changing the entire namespace.
The first case requires a wildcard to match the rest of a namespace.
The second requires a wildcard to match the basename at the end.

*Example of partial namespace replacement:*

- Node uses names `/foo`, `/foo/bar`, `/foo/bar/baz`
- Node given rule `/foo/**:=/fizz/\1`
- Resulting names `/foo`, `/fizz/bar`, `/fizz/bar/baz`

*Example of full namespace replacement:*

- Node uses names `/foo/bar/baz`, `/foo/bar/fee/biz`
- Node given rule `/foo/bar/*:=/bar/foo/\1`
- Resulting names `/bar/foo/baz`, `/foo/bar/fee/biz`

#### Supporting: Change a Basename

Changing a basename requires a wildcard which matches the entire namespace.
The wildcard `**` is useful because it matches every possible namespace when combined with a slash.

*Example:*

- Node uses names `/foo`, `/buz/foo`, `/biz/buz/foo`
- Node given rule `**/foo:=\1/bar`
- Resulting names `/bar`, `/buz/bar`, `/biz/buz/bar`

#### Supporting: Exact FQN Replacement

Exact FQN replacement requires no wildcards.
This syntax is identical to ROS 1.

*Example rules:*

- `/foo/bar:=/fiz/buz`
- `/foo:=/foo/bar`

#### Supporting: Exact Relative Name Replacement

Exact relative replacement also requires no wildcards.
It means relative names are first expanded to FQN, and then processed as during exact FQN replacement.
This syntax is identical to ROS 1.

*Example rules:*

- `foo:=/foo/bar` in namespace `/ns` is identical to `/ns/foo:=/foo/bar`
- `foo:=bar` in namespace `/ns` is identical to `/ns/foo:=/ns/bar`
- `/foo/bar:=foo` in namespace `/ns` is identical to `/foo/bar:=/ns/foo`

#### Supporting: Remap via Command Line

The syntax here can be passed to a node via the command line.
The syntax has been chosen to not conflict with special shell characters in bash.
For example in bash, the character `*` only has special behavior if it is surrounded by whitespace, but remap rules don't contain whitespace.
This character may still be difficult on other shells, like zsh.

#### Supporting: Change the Default Namespace

This isn't really a remapping rule, but the syntax is similar.
In ROS 1 the argument `__ns:=` could change the default namespace.
Here the syntax is the same, and additionally it can be prefixed with a node's name.
The replacement side must have a FQN with no special operators.
All relative names are expanded to the new namespace before any remapping rules are applied to them.

*Examples:*

- `__ns:=/new/namespace`
- `node1:__ns:=/node1s/new/namespace`

#### Supporting: Change the Node Name

This also isn't a true remapping rule, but the syntax is similar.
In ROS 1 the argument `__name:=` could change the node's name.
Here the syntax is the same, and additionally it can be prefixed with a node's current name.
The argument `__node:=` has the same effect.
The replacement side must have a single token.
Log messages use the new name immediately.
All private names are expanded to the new name before any remapping rules are applied to them.

*Examples:*

- `__name:=left_camera_driver`
- `__node:=left_camera_driver`
- `camera_driver:__name:=left_camera_driver`

#### Supporting: Remap Topic and Service Names Separately
Specifying a URL scheme on the match side of the rule makes it exclusive to one type of name.
If no URL scheme is given then the rule applies to both topics and services.

*Examples:*

- `rosservice:///foo/bar:=/bar/foo`
- `rostopic://foo/bar:=bar/foo`
- `nodename:rosservice://~/left:=~/right`

#### Not Supporting: Change a Token

The syntax can't change all uses of a token with one rule.
Supporting this use case with a single rule is not a priority.

*Workaround using two rules*

- First rule remaps token used in namespace `**/foobar/**:=\1/fizzbuz/\2`
- Second rule remaps token used as basename `**/foobar:=\1/fizzbuz`

#### Not Supporting: Pre-FQN Remapping

The syntax doesn't have a way to specify that a rule should be applied Prior to FQN expansion.
There is no workaround.

### Fnmatch Syntax

A syntax like fnmatch is being considered.
The character for the wild card `*` was chosen to match fnmatch.
Because remapping needs to capture text to use during replacement, the C function `fnmatch()` cannot be used as the implementation.
The extra wildcards `?` and `[]` don't appear to enable more uses cases above.
Fnmatch syntax may or may not match text with slashes depending on the option `FNM_PATHNAME`.

## Static Versus Dynamic Remapping

Static remapping is giving a node remapping rules at the time it is launched.
Dynamic remapping is the ability to remap a name while a node is running.
It may be useful for a developer who has started a node and wants to connect it to a different source.
Because the user will see the name after it has been remapped by static rules, dynamic rules should be applied after static ones.
This way the new rule matches against the name the user sees with introspection tools rather than the original name used in code.
