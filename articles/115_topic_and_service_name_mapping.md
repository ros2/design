---
layout: default
title: Topic and Service name mapping to DDS
permalink: articles/topic_and_service_names.html
abstract:
  This article describes the proposed mapping between ROS topic and service names to DDS topic and service names.
author: '[William Woodall](https://github.com/wjwwood)'
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Context

Before proposing constraints for ROS 2 topic and service names and a mapping to underlying DDS topics, this article will first summarize the existing guidelines and restrictions for ROS 1 and DDS.

### ROS 1 Names

In ROS 1, topic and service name guidelines are covered all under the umbrella of "ROS Names" and have these restrictions:

*From [http://wiki.ros.org/Names](http://wiki.ros.org/Names):*

```
A valid name has the following characteristics:

* First character is an alpha character ([a-z|A-Z]), tilde (~) or forward slash (/)
* Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_), or forward slashes (/)

Exception: base names (described below) cannot have forward slashes (/) or tildes (~) in them.
```

### DDS Topic Names

In DDS, topic names are restricted by these restrictions:

*From DDS 1.4 specification, or the [RTI documentation](http://community.rti.com/rti-doc/510/ndds/doc/html/api_cpp/group__DDSQueryAndFilterSyntaxModule.html):*

```
TOPICNAME - A topic name is an identifier for a topic, and is defined as any series of characters 'a', ..., 'z', 'A', ..., 'Z', '0', ..., '9', '_' but may not start with a digit.
```

*Note:* that the DDS specification has a known typo, where it says `-` are allowed, but the RTI documentation correctly lists `_` as allowed.

Additionally, DDS has a hard limit on topic names of 256 characters, so an additional goal is to minimize the number of extra characters used when mapping from ROS to DDS names.

## ROS 2 Topic and Service Name Constraints

In this section an outline of the proposed constrains for ROS 2 topic and service names will be enumerated along with rationales where appropriate.

For convenience here is a summary of all rules for topic and service names in ROS 2:

- may contain alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`), tildes (`~`), or forward slashes (`/`)
- must not start with a numeric character (`[0-9]`)
- must not contain any number of repeated underscores (`_`)
- must not end with an underscore (`_`)
- must not have an underscore (`_`) proceeded by a forward slash (`/`), i.e. `_/`
- must not end with a forward slash (`/`)
- must not contain any number of repeated forward slashes (`/`)
- must not have tilde (`~`) adjacent to anything other than forward slashes (`/`)
- may start with or end with a tilde (`~`)

Additionally, topic and service names can be represented in the [Uniform Resource Locator (URL)](https://en.wikipedia.org/wiki/Uniform_Resource_Locator) format to further disambiguate the resource name.
A topic name may be preceded by a `rostopic://` scheme prefix, and a service name may be preceded by a `rosservice://` scheme prefix.
For example, the absolute name `/foo` could also be represented as a topic with `rostopic:///foo` or as a service with `rosservice:///foo`.
Note the triple forward slash (`/`), which is a similar style as the `file://` scheme.
A relative name `foo/bar` could would be represented (as a topic) with `rostopic://foo/bar`.

### ROS 2 Name Examples

For example, these are valid names:

| `foo`       | `abc123`            | `_foo`               | `Foo` | `BAR`       |
| `~`         | `foo/bar`           | `~/foo`              | `/~`  | `foo/~/bar` |
| `foo/_/bar` | `rosservice:///foo` | `rostopic://foo/bar` |       |             |

But these are not valid names:

| `123abc`   | `123`  | `__foo`  | `foo__bar` | `foo bar`  |
| `foo__`    | ` `    | `foo_`   | `foo//bar` | `foo/`     |
| `foo_/bar` | `~foo` | `foo~`   | `foo~/bar` | `foo/~bar` |
| `/_/bar`   | `_`    | `_/_bar` |            |            |

### Namespaces

Topic and service names:

- may be split into tokens using a forward slash (`/`) as a delimiter
- must not end with a forward slash (`/`)

The last token is the topic or service name, and any preceding tokens comprise the namespace of the topic or service.

For example, the topic name `/foo/bar/baz` comprises of a topic or service named `baz` in the `/foo/bar` namespace.
In another example, the name `/foo` does not use splitting into tokens, so it comprises of a topic or service named `foo` in the `/` namespace (the root namespace).

Topic and service names:

- may be specified as absolute or relative
- must start with a forward slash (`/`) if they are absolute

An absolute name begins with a forward slash (`/`) and does not respect namespaces, i.e. it can be considered "global".
A relative name does not begin with a forward slash (`/`) and does respect the namespace of the node which created them.

Relative names are appended to the namespace of the node which creates them.
For example, if the node is put into a namespace `/ping/pong` and the topic or service name is `foo/bar`, then the absolute name will become `/ping/pong/foo/bar`.
However, if the topic or service name is `/foo/bar`, then the absolute name will stay just `/foo/bar`, ignoring the node's namespace.

### Name Tokens

Name token's are the strings between the namespace delimiters, e.g. the tokens of the topic or service `/foo/bar/baz` are `foo`, `bar`, and `baz`.

Topic and service name tokens:

- must not be empty, e.g. the name `//bar` is not allowed

  - rational: it removes the chance for accidental `//` from concatenation; also if the implementation chooses to use a trailing underscore (`_`) then it prevents `foo_/_bar` and `foo//bar` from being ambiguous

- may use alphanumeric characters (`[0-9|a-z|A-Z]`), or an underscore (`_`)

- must not start with numeric characters (`[0-9]`)

- must not end with a single underscore (`_`)

  - rational: if tokens are allowed to end with and start with `_` then `foo_/bar` is indistinguishable from `foo/_bar` if the `/` is replaced with `__`, i.e. both result in `foo___bar`

- must not be a single underscore (`_`)

  - rational: this is a special case of "must not end with an underscore"

- must not have two or more underscores (`__`) repeated anywhere

  - rational: this provides the implementation with a safe to use delimiter

- may be a single tilde character (`~`)

### Private Namespace Substitution Character

The special single character token `~` will be replaced with a namespace snippet that is a concatenation of the namespace for the node and the node name.
For example, for a node `node1` in a namespace `/foo` would result in `~` being replaced with `/foo/node1`.
For another example, a node `node1` in a namespace `foo/bar` would result in `~` being replaced with `foo/bar/node1`.
It may be used in any token position.
Any resulting empty namespaces which result from expanding the `~` will be collapsed.
For example, when `~` expands to `/foo/node1` and the name `ping/~/pong` is given, it will expand to `ping/foo/node1/pong` and not to `ping//foo/node1/pong`.
Here is a table with some example expansions:

| **Input Name** | Node: `my_node` NS: none | Node: `my_node` NS: `/foo`     |
|----------------|--------------------------|--------------------------------|
| `ping`         | *`/ping`*                | *`/foo/ping`*                  |
| `/ping`        | *`/ping`*                | *`/ping`*                      |
| `~`            | *`/my_node`*             | *`/foo/foo/my_node`*           |
| `~/ping`       | *`/my_node/ping`*        | *`/foo/foo/my_node/ping`*      |
| `/~`           | *`/my_node`*             | *`/foo/my_node`*               |
| `/~/ping`      | *`/my_node/ping`*        | *`/foo/my_node/ping`*          |
| `ping/~/pong`  | *`/ping/my_node/pong`*   | *`/foo/ping/foo/my_node/pong`* |
| `/ping/~/pong` | *`/ping/my_node/pong`*   | *`/ping/foo/my_node/pong`*     |

### Hidden Topic or Service Names

Any topic or service name that contains any tokens (either namespaces or a topic or service name) which starts with an underscore (`_`) will be considered hidden and tools may not show them unless explicitly asked.

## Mapping of ROS 2 Topic and Service Names to DDS Topics

The ROS topic and service name constraints allow more characters than the DDS topic names because ROS allows for the forward slash (`/`) and the tilde (`~`).
Since ROS 2 topic and service names are expanded to absolute names before being used, there is always a leading forward slash (`/`) and any tildes (`~`) will have been expanded.
Additionally any URL related syntax is removed, e.g. the `rostopic://` prefix.
Therefore the only forward slash has to be substituted when converting to DDS topic names.

### ROS Specific Name Prefix

In order to differentiate ROS topics easily, all DDS topic names created by ROS shall be prefixed with `rX_`, where `X` is a single character which indicates what subsystem of ROS to which the topic belongs.
For example, a plain topic called `/foo` would translate to a DDS topic named `rt_foo`.

For systems where Services are implemented with topics (like with OpenSplice), a different subsystem character can be used: `rq_` for the request topic and `rr_` for the response topic.
On systems where the implementation is handled for us by DDS (like with Connext), we use `rs_` as the common prefix.

Here is a non-exhaustive list of prefixes:

| ROS Subsystem        | Prefix |
|----------------------|--------|
| ROS Topics           | rt_    |
| ROS Service Request  | rq_    |
| ROS Service Response | rr_    |
| ROS Service          | rs_    |
| ROS Parameter        | rp_    |
| ROS Action           | ra_    |

### Substitution of the Namespace Delimiter

The namespace delimiter in ROS 2 topic and service names, a forward slash (`/`), will be replaced with double underscores (`__`).
The leading forward slash (`/`) is not replaced with double underscores (`__`), and is replaced instead by the ROS specific name prefix.

### ROS to DDS Name Conversion Examples

Here are some examples of translations between ROS topic names and DDS topic names:

| ROS Name          | Subsystem | DDS Name              |
|-------------------|-----------|-----------------------|
| `/foo`            | Topic     | `rt_foo`              |
| `/foo/bar`        | Topic     | `rt_foo__bar`         |
| `/_foo/bar`       | Topic     | `rt__foo__bar`        |
| `/foo/_bar`       | Topic     | `rt_foo___bar`        |
| `/_foo/_bar`      | Topic     | `rt__foo___bar`       |
| `/_foo/_bar/_baz` | Topic     | `rt__foo___bar___baz` |

## Concerns/Alternatives

This section lists concerns about the proposed design and alternatives that were considered.

### Alternative ROS to DDS Mapping

There was some discussion of alternatives and concerns with respect to the ROS -> DDS translation.

#### Capital Letter Substitution Alternative

Since the forward slash (`/`) is replaced with double underscores (`__`) when translating to DDS from ROS topic names, two characters out of the 256 character limit are lost with each additional namespace.
One proposed alternative was to add a constraint that ROS topic names could not use capital letters, and then capital letters could be used as the stand in for the forward slashes (`/`).

Trade-offs:

- Uses one less character per namespace and makes it easier to calculate the maximum length of a ROS topic or service name.
- Prevents users from using capital letters in their names, which is constraining and might be a problem for backwards compatibility with ROS 1 topics and services.

Rational:

Preventing users from using capital letters was too constraining for the added benefit.

#### Prefix with Double Underscores

This alternative differs only in that it uses two underscores in the prefix, i.e. `rt__` rather than `rt_`.

Trade-offs:

- More consistent with replacement of other forward slashes (`/`)
- Uses one more character

Rational:

Slight preference given to the shorter alternative.

#### Limited Prefixes Alternative

This alternative would:

- not prefix topics
- optionally prefix other kinds of "implementation detail topics"

Trade-offs:

- it would be easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - however, the types would need to match as well
  - in the current proposal the ROS topic `image` would become `rt_image`, so DDS topics would need to follow the ROS topic conversion scheme to interoperate with ROS components

- it would be hard to distinguish ROS created DDS topics and normal DDS topics

- services would still need to be differentiated

  - e.g. service `/foo` would need to make two topics, something like `foo_Request` and `foo_Reply`

Rational:

Slight preference was given to easily categorizing ROS created topics with DDS created topics over easily connecting to existing DDS topics.
Connecting to DDS topics could be achieved by having an option when subscribing or publishing to "alias" to an implementation topic name, e.g. something like `sub->alias_to_explicit_topic('dds_topic')`.
Also, the workaround for separating ROS created topics from other DDS topics was considered to be more complicated than suggested solution for allowing users to specify specific DDS topic names for their publishers and subscriptions.

#### Suffix Alternative

This alternative would:

- No prefix

- The prefix is restructured to be a suffix, i.e. `rX_<topic>` -> `<topic>_rX_`

  - this would be unique to user defined names because they cannot have a trailing underscore (`_`)

Trade-offs:

- more difficult to distinguish ROS created DDS topics from normal or built-in DDS topics when listing them using DDS tools because they are not sorted by a prefix

- if the service name is suffixed again by the DDS vendor (like in Connext's implementation of Request-Reply) then it would be potentially difficult to differentiate from a user's topic name

  - e.g. service `/foo` might become topics: `foo_s_Requster` and `foo_s_Replier` and the user could create a topic called `/foo_s_Requster` too.
  - this also applies to any other similar transformations that an rmw implementation might make to the topic

Rational:

This alternative was not selected over the prefix solution because of a lack of advantages over the prefix solution.
Also, it typically took one more character to express (`rt_` versus `_rt_`) and the potential issues with ambiguity when the DDS implementation handles Request-Reply (added suffixes).

#### Limited Suffix Alternative

This alternative is the same as the "Suffix Alternative" except:

- Topics would not have a suffix or prefix at all

Trade-offs:

- same trade-offs as the "Suffix Alternative"

- but also easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - the types would need to match
  - in the current proposal the ROS topic `image` would become `rt_image`, so DDS topics would need to follow our naming scheme to interoperate with ROS components

Rational:

While this alternative provided the benefits of both the suffix and limited prefix alternatives, the rational for the limited prefix alternative still applies here.
