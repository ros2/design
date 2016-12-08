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

Before proposing constraints for ROS 2 topic and service names and a mapping to underlying DDS topics, this article will first summarize the existing guidelines and restrictions for both ROS 1 and DDS.

### ROS 1 Names

In ROS 1, topic and service name guidelines are all covered under the umbrella of "ROS Names" and have these restrictions:

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

- must not be empty
- may contain alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`), or forward slashes (`/`)
- may use balanced curly braces (`{}`) for substitutions
- may start with a tilde (`~`), the private namespace substitution character
- must not start with a numeric character (`[0-9]`)
- must not contain any number of repeated underscores (`_`)
- must not end with an underscore (`_`)
- must not have an underscore (`_`) followed by a forward slash (`/`), i.e. `_/`
- must not end with a forward slash (`/`)
- must not contain any number of repeated forward slashes (`/`)
- must separate a tilde (`~`) from the rest of the name with a forward slash (`/`), i.e. `~/foo` not `~foo`
- must have balanced curly braces (`{}`) when used, i.e. `{sub}/foo` but not `{sub/foo` nor `/foo}`

The content of substitutions, i.e. the string in side of balanced curly braces (`{}`), follow very similar rules to names.
The content of substitutions:

- must not be empty

- may contain alphanumeric characters (`[0-9|a-z|A-Z]`) and underscores (`_`)

- must not start with a numeric character (`[0-9]`)

- are unlike topic and service names in that they:

  - may start with or end with underscores (`_`)
  - may contain repeated underscores, e.g. `__`

### Fully Qualified Names

The topic and service name rules allow for some convenience syntax, which in some cases requires additional context to expand to the fully qualified name and then to the DDS equivalent name.
For example, as outlined in further detail in the sections that follow, names may be relative (e.g. `foo` versus the absolute `/foo`), they may contain the private namespace substitution character (`~`), or arbitrary substitutions which are within the curly braces (`{}`) syntax.
With context, each of these features can be expanded to some simple string to form the fully qualified name.
Fully qualified names have these additional restrictions:

- must start with a forward slash (`/`), i.e. they must be absolute
- must not contain tilde (`~`) or curly braces (`{}`)

Note that expanded substitutions must result in a valid name.
For example, a name `foo_{bar}` is not valid if `bar` is expanded as `_baz`, because that would result in a fully qualified name of `foo__baz` which contains repeated underscores (`_`).

### Uniform Resource Locators (URLs)

Additionally, topic and service names can be represented in the [Uniform Resource Locator (URL)](https://en.wikipedia.org/wiki/Uniform_Resource_Locator) format to further disambiguate the resource name.
A topic name may be preceded by a `rostopic://` scheme prefix, and a service name may be preceded by a `rosservice://` scheme prefix.
For example, the absolute name `/foo` could also be represented as a topic with `rostopic:///foo` or as a service with `rosservice:///foo`.
Note the triple forward slash (`/`), which is a similar style to the `file://` scheme.
A relative name `foo/bar` could would be represented (as a topic) with `rostopic://foo/bar`.

### ROS 2 Name Examples

For example, these are valid names:

| `foo`       | `abc123`            | `_foo`               | `Foo`        | `BAR`            |
| `~`         | `foo/bar`           | `~/foo`              | `{foo}_bar`  | `foo/{ping}/bar` |
| `foo/_bar`  | `rosservice:///foo` | `rostopic://foo/bar` |              |                  |

But these are not valid names:

| `123abc`    | `123`  | `__foo`  | `foo__bar` | `foo bar`   |
| `foo__`     | ` `    | `foo_`   | `foo//bar` | `foo/`      |
| `foo_/bar`  | `~foo` | `foo~`   | `foo~/bar` | `foo/~bar`  |
| `/_/bar`    | `_`    | `_/_bar` | `/~`       | `foo/~/bar` |
| `foo/_/bar` |        |          |            |             |

These are some valid fully qualified names:

| `/foo`     | `/bar/baz` | `rostopic:///ping` | `/_private/thing` | `/public_namespace/_private/thing` |

### Namespaces

Topic and service names:

- may be split into tokens using a forward slash (`/`) as a delimiter

  - see the "Name Tokens" section for more details on tokens

- must not end with a forward slash (`/`)

The last token is the topic or service basename, and any preceding tokens make up the namespace of the topic or service.

For example, the topic name `/foo/bar/baz` is composed of a topic or service with the basename `baz` in the `/foo/bar` namespace.
In another example, the name `/foo` splits into one token, such that it is composed of a topic or service with the basename `foo` in the `/` namespace (the root namespace).

Topic and service names:

- may be specified as absolute or relative
- must start with a forward slash (`/`) if they are absolute

An absolute name begins with a forward slash (`/`) and does not respect namespaces, i.e. it can be considered "global".
A relative name does not begin with a forward slash (`/`) and does respect the namespace of the node which created them.

Relative names are appended to the namespace of the node which creates them.
For example, if the node is put into a namespace `/ping/pong` and the topic or service name is `foo/bar`, then the absolute name will become `/ping/pong/foo/bar`.
However, if the topic or service name is `/foo/bar`, then the absolute name will stay just `/foo/bar`, ignoring the node's namespace.

### Name Tokens

"Name tokens" are the strings between the namespace delimiters, e.g. the tokens of the topic or service `/foo/bar/baz` are `foo`, `bar`, and `baz`.

Topic and service name tokens:

- must not be empty, e.g. the name `//bar` is not allowed

  - rationale: it removes the chance for accidental `//` from concatenation and thereore the need to collapse `//` to `/`

- may use alphanumeric characters (`[0-9|a-z|A-Z]`), single underscores (`_`), and/or balanced curly braces (`{}`)

- must not start with numeric characters (`[0-9]`)

- must not end with a single underscore (`_`)

  - rationale: if tokens are allowed to end with and start with `_` then `foo_/bar` is indistinguishable from `foo/_bar` if the `/` is replaced with `__` (as proposed in the "Substitution of the Namespace Delimiter" section), i.e. both result in `foo___bar`

- must not be a single underscore (`_`)

  - rationale: this is a special case of "must not end with an underscore"

- must not have two or more underscores (`__`) repeated anywhere

  - rationale: this provides the implementation with a safe to use delimiter

- may be a single tilde character (`~`)

### Private Namespace Substitution Character

The special single character token `~` will be replaced with a namespace snippet that is a concatenation of the namespace for the node and the node name.
For example, a node `node1` in a namespace `/foo` would result in `~` being replaced with `/foo/node1`.
As another example, a node `node1` in a namespace `foo/bar` would result in `~` being replaced with `foo/bar/node1`.
It must be used at the beginning of a non-fully qualified name, if at all.
Here is a table with some example expansions:

| **Input Name** | Node: `my_node` NS: none | Node: `my_node` NS: `/my_ns`   |
|----------------|--------------------------|--------------------------------|
| `ping`         | *`/ping`*                | *`/my_ns/ping`*                |
| `/ping`        | *`/ping`*                | *`/ping`*                      |
| `~`            | *`/my_node`*             | *`/my_ns/my_node`*             |
| `~/ping`       | *`/my_node/ping`*        | *`/my_ns/my_node/ping`*        |

Note the private namespace substitution character makes the name absolute, and therefore the namespace is not added a second time.

### Substitutions

The bracket syntax (`{substitution_name}`) may be used in non-fully qualified names to substitute useful contextual information into the name.
The set of substitution keys (names) are not set in this document, but some reasonable examples might be: `{node}` expands to the current node's name or `{ns}` expands to the current node's namespace.

Substitutions are expanded before the private namespace substitution character is expanded.
Therefore a substitution may contain the private namespace substitution character, i.e. `~`.
For example, given the name `{private}foo`, if there is a substitution called `{private}` that expands to `~/_`, the current node name is `my_node`, and the current node's namespace is `my_ns`, then the fully qualified name would be `/my_ns/my_node/_foo`.

### Hidden Topic or Service Names

Any topic or service name that contains any tokens (either namespaces or a topic or service name) that start with an underscore (`_`) will be considered hidden and tools may not show them unless explicitly asked.

## Mapping of ROS 2 Topic and Service Names to DDS Topics

The ROS topic and service name constraints allow more types of characters than the DDS topic names because ROS additionally allows the forward slash (`/`), the tilde (`~`), and the balanced curly braces (`{}`).
These must be substituted or otherwise removed during the process of mapping to DDS topic names.
Since ROS 2 topic and service names are expanded to fully qualified names, any balanced bracket (`{}`) substitutions and tildes (`~`) will have been expanded.
Additionally any URL related syntax, e.g. the `rostopic://` prefix, will be removed once parsed.
Therefore only forward slashes have to be substituted when converting to DDS topic names.

### Substitution of the Namespace Delimiter

The namespace delimiter in ROS 2 topic and service names, a forward slash (`/`), will be replaced with double underscores (`__`).
Note that as fully qualified ROS 2 topic and service names are absolute, there is always a leading forward slash (`/`).

### ROS Specific Name Prefix

In order to differentiate ROS topics easily, all DDS topic names created by ROS shall be prefixed with `rX`, where `X` is a single character that indicates to which subsystem of ROS the topic belongs.
For example, a plain topic called `/foo` would translate to a DDS topic named `rt__foo`, which is the result of concatenating the prefix `rt` for being a ROS topic, with `__` for the leading `/`, and the topic name `foo`.

For systems where Services are implemented with topics (like with OpenSplice), a different subsystem character can be used: `rq` for the request topic and `rr` for the response topic.
On systems where the implementation is handled for us by DDS (like with Connext), we use `rs` as the common prefix.

Here is a non-exhaustive list of prefixes:

| ROS Subsystem        | Prefix |
|----------------------|--------|
| ROS Topics           | rt     |
| ROS Service Request  | rq     |
| ROS Service Response | rr     |
| ROS Service          | rs     |
| ROS Parameter        | rp     |
| ROS Action           | ra     |

While all planned prefixes consist of two characters, i.e. `rX`, anything proceeding the first namespace separator, i.e. `__`, can be considered part of the prefix.
The standard reserves the right to use up to 8 characters for the prefix in case additional prefix space is needed in the future.

### Communicating with Non-ROS Topics

Since all ROS topics are prefixed when being converted to DDS topic names, it makes it impossible to subscribe to existing DDS topics which do not follow the same naming pattern.
For example, if an existing DDS program is publishing on the `image` topic (and is using the DDS equivalent to the ROS message type) then a ROS program could not subscribe to it because of the name mangling.
Therefore to allow ROS programs to interoperate with "native" DDS topic names the API should provide a way to skip the ROS specific prefixing.

### ROS to DDS Name Conversion Examples

Here are some examples of translations between ROS topic names and the corresponding DDS topic names:

| ROS Name          | Subsystem | DDS Name               |
|-------------------|-----------|------------------------|
| `/foo`            | Topic     | `rt__foo`              |
| `/foo/bar`        | Topic     | `rt__foo__bar`         |
| `/_foo/bar`       | Topic     | `rt___foo__bar`        |
| `/foo/_bar`       | Topic     | `rt__foo___bar`        |
| `/_foo/_bar`      | Topic     | `rt___foo___bar`       |
| `/_foo/_bar/_baz` | Topic     | `rt___foo___bar___baz` |

### ROS Topic and Service Name Length Limit

Because DDS topic names must be limited to 255 characters, the length of a ROS topic or service name is also limited in length.
In the case of a topic, the length is governed by the following algorithm:

`C + N + P <= 255`

Where `P` is `8`, the maximum possible length of the ROS specific prefix, `C` is the number of characters in the topic name, and `N` is the number of name tokens in the topic name.
Note that this algorithm must be applied on a fully qualified name, i.e. after expanding all substitutions and the private namespace substitution character (`~`), after removing any URL related syntax (e.g. without the `rostopic://` prefix).

Services are governed by the same algorithm, but in some implementations may require additional characters to be subtracted from the limit depending on how the request and response topics are created by the middleware.
In the specific case of RTI Connext's Request-Reply implementation, they append the `Request` and `Reply` strings to the topic names.
Therefore, it would be safest to assume the Service name limit to be less 8 more characters.

## Compare and Contrast with ROS 1

In order to support a mapping to the more restrictive DDS topic name rules, these rules are in some ways more constraining than the rules for ROS 1.
Other changes have been proposed for convenience or to remove a point of confusion that existed in ROS 1.
In ROS 2, topic and service names differ from ROS 1 in that they:

- must separate the tilde (`~`) from the rest of the name with a forward slash (`/`)

  - This is done to avoid inconsistency with how `~foo` works in filesystem paths versus when used in a ROS name.

- may contain substitutions which are delimited with balanced curly braces (`{}`)

  - This is a more generic extension of the idea behind the tilde (`~`).

- have length limits

  - This is driven by the topic name length limit of DDS.

- may be indicated as "hidden" by using a leading underscore (`_`) in one of the namespaces

  - This is used to hide common but infrequently introspected topics and services.

## Concerns/Alternatives

This section lists concerns about the proposed design and alternatives that were considered.

### Alternative Name Rules and Concerns About Name Rules

There were some suggested but then rejected alternatives to the rules for topic and service names.

#### More Versatile Private Namespace Substitution Character

Currently the `~` private namespace substitution character may only be used at the beginning of the name, but it was also suggested that it could be placed anywhere within the name and could be substituted in place.

This was rejected because it was complicated to explain and did not behave the same as the `~` when used in filesystem paths on Unix machines.
Also, it was difficult to justify its existence because all suggested use cases were quite contrived.
It also behaved differently from how it worked in ROS 1, which was yet another negative for this alternative.

#### Alternative Substitution Syntax

There were some alternative syntaxes proposed for substitutions in the names before the plain balanced curly braces syntax (`{}`) was selected:

- `%{sub}`
- `${sub}`
- `$sub`

The most serious alternatives considered were the "bash-like" syntax of `${sub}` and `$sub`.
The `$sub` style syntax has the downside of being difficult to process and making it impossible to express some kinds of concatenation.
The `${sub}` was a strong candidate, but ultimately was rejected because it would collide with use in shell scripts.
For example, you can imagine a shell script that runs a node and remaps a topic name would contain these substitutions, but they would need to be escaped to prevent bash itself from trying to expand them.
The `{}` syntax avoids this problem but is also easy to parse.

The `{}` syntax will collide with Python String substitution, but since that is an explicit action (unlike shell substitution which will implicitly always occur) it's less of an issue.

#### Concerns About Substitutions

This document does not prescribe what substitutions should be supported by implementations.
This was done to avoid blocking progress on this document on the need to agree on the set of required substitutions.
However, this is just delaying the issue.
For the substitutions to be useful, then all implementations that process topic and service names need to support them.

This compromise was made so that when more work is done on substitutions, it would hopefully not require changes to the name syntax, but instead revolve around what substitutions to support and whether or not that support is optional.

### Alternative ROS to DDS Mapping

There was some discussion of alternatives and concerns with respect to the ROS -> DDS translation.

#### Capital Letter Substitution Alternative

Since the forward slash (`/`) is replaced with double underscores (`__`) when translating to DDS from ROS topic names, two characters out of the 256 character limit are lost with each additional namespace.
One proposed alternative was to add a constraint that ROS topic names could not use capital letters, and then capital letters could be used as the stand in for the forward slashes (`/`).

Trade-offs:

- Uses one fewer character per namespace and makes it easier to calculate the maximum length of a ROS topic or service name.
- Prevents users from using capital letters in their names, which is constraining and might be a problem for backwards compatibility with ROS 1 topics and services.

Rationale:

Preventing users from using capital letters was too constraining for the added benefit.

#### ROS Prefix with Single Underscore

This alternative differs only in that it uses a single underscore in the prefix, i.e. `rt_` rather than `rt__` (`rt` + the leading `/`).

Trade-offs:

- Uses one fewer character
- Less consistent with replacement of other forward slashes (`/`)

Rationale:

Slight preference given to the more consistent alternative.

#### Limited Prefixes Alternative

This alternative would:

- not prefix topics
- optionally prefix other kinds of "implementation detail topics"

Trade-offs:

- it would be easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - however, the types would need to match as well
  - in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow the ROS topic conversion scheme to interoperate with ROS components

- it would be hard to distinguish ROS created DDS topics and normal DDS topics

- services would still need to be differentiated

  - e.g. service `/foo` would need to make two topics, something like `foo_Request` and `foo_Reply`

Rationale:

Slight preference was given to easily categorizing ROS created topics with DDS created topics over easily connecting to existing DDS topics.
Connecting to DDS topics could be achieved by having an option when subscribing or publishing to "alias" to an implementation topic name, e.g. something like `sub->alias_to_explicit_topic('dds_topic')`.
Also, the workaround for separating ROS created topics from other DDS topics was considered to be more complicated than the suggested solution of allowing users to specify specific DDS topic names for their publishers and subscriptions.

#### Suffix Alternative

This alternative would:

- not prefix topics

- restructure prefixes to instead be suffixes, i.e. `rX<topic>` -> `<topic>_rX_`

  - this would be unique to user defined names because they cannot have a trailing underscore (`_`)

Trade-offs:

- more difficult to distinguish ROS created DDS topics from normal or built-in DDS topics when listing them using DDS tools because they are not sorted by a ROS specific prefix

- if the service name is suffixed again by the DDS vendor (like in Connext's implementation of Request-Reply) then it would be potentially difficult to differentiate from a user's topic name

  - e.g. service `/foo` might become topics: `foo_s_Request` and `foo_s_Reply` and the user could create a topic called `/foo_s_Request` too.
  - this also applies to any other similar transformations that an RMW implementation might make to the topic

Rationale:

This alternative was not selected over the prefix solution because of a lack of advantages over the prefix solution.
Also, it typically took one more character to express (`rt` versus `_rt_`; unless you also drop the implicit first namespace `/` then it's `rt__` versus `_rt_`) and the potential issues with ambiguity when the DDS implementation handles Request-Reply (added suffixes).

#### Limited Suffix Alternative

This alternative is the same as the "Suffix Alternative" except:

- Topics would not have a suffix or prefix at all

Trade-offs:

- same trade-offs as the "Suffix Alternative"

- but also easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - the types would need to match
  - in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow our naming scheme to interoperate with ROS components

Rationale:

While this alternative provided the benefits of both the suffix and limited prefix alternatives, the rationale for the limited prefix alternative still applies here.
