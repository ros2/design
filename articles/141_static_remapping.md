---
layout: default
title: Static Remapping
abstract:
  This article describes the requirements, rationale, and mechanisms for remapping names in ROS2.
author: '[Shane Loretz](https://github.com/sloretz)'
published: false
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Remapping Names
Topics, parameters, and services are identified by [Names](http://wiki.ros.org/Names).
Names are hard coded in ROS nodes.
Remapping is what allows names to be changed at runtime.
Without remapping every instance of a node would require code changes to use the right names.
**Remapping rules** are the instructions given to a node to change the names it uses.

### Structure of a Name
Names are hierarchical strings delineated by `/`.
If they begin with `/` they are said to be **global** or **Fully Qualified Names** (FQN).
Otherwise they are said to be **relative**.
Relative names are resolved to FQN prior to being used.

**Example names:**

- `/foo`
- `/foo/bar`
- `foo/bar/baz`
- `foo/bar`
- `bar`

Each part of a name between a slash is a **token**.
The examples above use the tokens `foo`, `bar` and `baz`.
FQN are conceptually divided into two pieces: **namespace** and **basename**.
The basename is the last token in a name.
The namespace is all tokens and slashes prior to the basename in a FQN.

**Example namespaces:**

- `/`
- `/foo/`
- `/foo/bar/baz/`

**Example basenames:**

- `bar`
- `foo`

Nodes are said to be in a **namespace**.
This namespace becomes a prefix to all relative names used by the node.
Relative names are expanded to FQN by prepending the node's namespace to them.


**Example:**

- Node is in namespace `/fiz/buz`
- Node uses relative name `foo/bar`
- The FQN is `/fiz/buz/foo/bar`


### ROS 2 Use Cases
Remapping in ROS 2 will likely support the following use cases:


#### A user can run multiple instances of a node using the same executable in different data paths
This implies remapping rules are supplied to an instance of a node at runtime, and that remapping rules are specific to an instance of a node.

**Rationale:**

- A node may be a driver for a type of sensor
- A robot that has multiple sensors of the same type could launch multiple instances of the same node outputting to different topics
```
            +-------------+
 (Lidar 1)--> node inst 1 +-->/head_scan
            +-------------+

            +-------------+
 (Lidar 2)--> node inst 2 +-->/base_scan
            +-------------+
```


#### A user can remap a namespace without affecting the basename
**Rationale:**
A popular ROS 1 package [actionlib](http://wiki.ros.org/actionlib) creates 5 topics with the same namespace.
Remapping an actionlib client or server means creating 5 remapping rules.
One rule could remap the namespace itself.

- Node provides an actionlib server `move_head` and checks a parameter called `move_head`
- A rule remaps namespace `move_head` to `move_head_check_collision`
- All 5 actionlib topics are remapped to `/move_head_check_collision`, but the parameter name remains unchanged


#### A user can remap a basename without affecting the namespace
**Rationale:**
The token used for a basename may also be in the namespace of another name the user does not want to change.

- A node uses names `/scan/head/scan`, `/scan/base/scan`, `/scan/arm/scan`
- A user wants the node to subscribe to the same data after some processing
- The user remaps basename `scan` to `scan_filtered`
- The final topics are `/scan/head/scan_filtered`, `/scan/base/scan_filtered`, `/scan/arm/scan_filtered`


#### A user can remap a token regardless of where it appears in a name
**Rationale:**
A token may be used in many names.
It might require many remap rules to replace a token if the token is undesirable.

- A company sells a generic mobile robot base with a ROS 2 driver
- The driver uses lots of names with the company's name in it: `UmbrellaCorp`
- Another company incorporates the base into their product, and their customers want a ROS 2 interface
- They don't want their interface to contain `UmbrellaCorp`, so they remap the token to `mobile_base` when they launch the nodes for the base


#### A user can remap just a relative name used in code
This implies remapping prior to FQN expansion

**Rationale:**

- A node uses two names `cat` and `/ns/cat`
- The node is run in namespace `/ns/`, so the FQN of both names is `/ns/cat`
- A rule remaps just `cat` to `lion`
- The final names are `/ns/cat` and `/ns/lion`


#### A user can remap a FQN to another name
**Rationale:**
This is part of the behavior of ROS 1 remapping, so including it will ease the transition to ROS 2.

- A node uses names `/ns/bar` and `/ns/barista`
- A rule is created to remap just `/ns/bar` to `/ns/foo`
- The final names are `/ns/foo` and `/ns/barista`


#### A user can remap a relative name to another name
**Rationale:**
This is part of the behavior of ROS 1 remapping.

- A node is in namespace `/ns/` and uses name `bar`
- A rule is created to remap `bar` to `foo`
- The final name is `/ns/foo`


#### A user can supply node specific remapping arguments via the command line
This implies there is a way to uniquely identify a node in a process.

**Rationale:**
This is part of the behavior of ROS 1 remapping.


#### A user can put a node into a specific namespace
This implies there is a way to uniquely identify a node in a process.

**Rationale:**
ROS 1 has this feature using the environment variable `ROS_NAMESPACE` and the argument `__ns`

- A system has multiple robots
- A users wants the robots to not interfere with each other, so each robot is put into its own namespace
- *TODO Global topics like /tf and /tf_static must be individually remapped in ROS 1. What to do about them?*


#### A user can remap a name they see on a node after it has launched
This implies remapping rules are applied in sequence.
This also implies there is a way to supply remapping rules to a node besides the command line (services?).

**Rationale:**
A developer may want to make a running node subscribe to a different source of data to see how it behaves.
It is less work for them to change the name they see rather than digging through the source code to find the original name.


### Structure of a Remapping Rule
Remapping rules have two parts:

1. a part used to determine if the rule applies to a name
2. a replacement for the names that are matched


### Remapping Names in ROS 1
Remapping is a feature that also exists in ROS 1.
In ROS 1 remapping works by passing in [arguments](http://wiki.ros.org/Remapping%20Arguments) to each node.
Client implementations also have APIs in code to pass remapping rules when the node is initialized.
A remap rule consists of two names: one that should be replaced with another.

ROS 1 remapping works on **Fully Qualified Names** (FQN).
Both sides of a rule are [expanded to FQN](http://docs.ros.org/api/roscpp/html/namespaceros_1_1names.html#a377ff8fede7b95398fd5d1c5cd49246b).
Before a name is remapped it is also [expanded to FQN](http://docs.ros.org/api/roscpp/html/namespaceros_1_1names.html#ab2eebaf734abfbdccb4122f8e24f547f).
The name is remapped to the right side only if it exactly matches the left side of a rule.


### Thoughts on ROS 2 implementation
`rmw` is likely ignorant of remapping.
The APIs offered by client implementations should remap names automatically to prevent creation of names that are impossible to remap.


### Remapping rule command line syntax
In ROS 1 remapping is done primarily through command line arguments, commonly via roslaunch.
It may not be possible to use the same syntax for ROS 2 because multiple nodes can be inside of the same process, and it is desired that remap rules are node specific and not process specific.
This syntax assumes the node's name is guaranteed to be unique in a process.
It does not support pre-FQN expansion remapping.

- `%`
 - During matching it is a wildcard that matches a single token
 - It may appear at most once on the matching side of a rule
 - During replacement it is replaced with the matched token

- `%%`
 - During matching it is a wildcard that matches zero or more tokens with at least one slash
 - It may appear at most once on the matching side of a rule
 - During replacement it is replaced with the matched slashes and tokens

- `:=` divides the two parts of a remapping rule: matching and replacement

- `~` expands to `/namespace/nodename`

- `nodename|` prefixed to a rule makes it apply only to a node with that unique name
 - If not specified, the rule applies to all nodes in the process

**Examples:**

- rule `dog:=cat` node in namespace `/ns/`
 - FQN `/ns/dog` gets remapped to `/ns/cat`

- rule `/dog:=cat` node in namespace `/ns/`
 - FQN `/dog` gets remapped to `/ns/cat`

- rule `%:=cat` node in namespace `/ns/`
 - FQN `/ns/dog` gets remapped to `/ns/cat`

- rule `/%:=cat` node in namespace `/ns/`
 - FQN `/dog` gets remapped to `/ns/cat`

- rule `/red%%:=/blu%%` node in any namespace
 - FQN `/red/cat` gets remapped to `/blu/cat`
 - FQN `/red/big/cat` gets remapped to `/blu/big/cat`

- rule `%%cat:=/big/cat`
 - FWN `/cat` gets remapped to `/big/cat`
 - FQN `/small/cat` gets remapped to `/big/cat`
 - FQN `/really/small/cat` gets remapped to `/big/cat`

- rule `node1|/dog:=/cat` with two nodes named `node1` and node2`
 - node1 using name `/dog` gets remapped to `/cat`
 - node2 using name `/dog` is unchanged

