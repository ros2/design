---
layout: default
title: Resource Names
permalink: articles/resource_names.html
abstract:
  This article describes an specification proposal for resource addressing in ROS, as well as its implementation in a DDS setting.
author: '[William Woodall](https://github.com/wjwwood) [Michel Hidalgo](https://github.com/hidmic)'
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

### Concepts

#### Content-oriented resources

Content-oriented resources are not bound to any particular node in a given system.
ROS topics, which may be freely published or subscribed to by multiples ROS nodes, fall under this classification.

Note that in ROS 1, every resource i.e. topics, services, actions and parameters, is a content-oriented resource.

#### Host-oriented resources

Host-oriented resources are provided by a node in a given system and thus are bound to it.
Multiple resources with the same name and type may by provided by different nodes.
Therefore, the provider node name is necessary to unambiguously address a host-oriented resource.
In the preceding, it is assumed that node name uniqueness has been ensured by means not prescribed in this document.

ROS services, actions and parameters fall under this classification.

### Restrictions

Before proposing constraints for ROS resource names and a mapping to underlying DDS concepts, this article will first summarize the existing guidelines and restrictions for both ROS 1 and DDS.

#### ROS 1 Names

In ROS 1, topic and service name restrictions are all covered under the umbrella of "ROS Names":

*From [http://wiki.ros.org/Names](http://wiki.ros.org/Names):*

```
A valid name has the following characteristics:

* First character is an alpha character ([a-z|A-Z]), tilde (~) or forward slash (/)
* Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_), or forward slashes (/)

Exception: base names (described below) cannot have forward slashes (/) or tildes (~) in them.
```

#### DDS Topic Names

In DDS, topic names restrictions include:

*From DDS 1.4 specification, or [RTI documentation](http://community.rti.com/rti-doc/510/ndds/doc/html/api_cpp/group__DDSQueryAndFilterSyntaxModule.html):*

```
TOPICNAME - A topic name is an identifier for a topic, and is defined as any series of characters 'a', ..., 'z', 'A', ..., 'Z', '0', ..., '9', '_' but may not start with a digit.
```

*Note:* the DDS specification has a known typo, where it says `-` are allowed, but the RTI documentation correctly lists `_` as allowed.

Additionally, DDS - or more precisely the underlaying RTPS protocol - has a hard limit on topic names of 256 characters, so an additional goal is to minimize the number of extra characters used when mapping from ROS to DDS names.
See The Real-time Publish-Subscribe Protocol (RTPS) DDS Interoperability Wire Protocol Specification, Table 9.12 for more details.

## Uniform Resource Locators (URLs)

Resources are addressed by means of the [Uniform Resource Locators (URL)](https://en.wikipedia.org/wiki/Uniform_Resource_Locator) format.

### Syntax Components

#### Provider Names

For host-oriented resources i.e. services, actions and parameters, the provider name refers to the node that is exposing said resources.

##### Definition

A provider name is made up of one or more tokens, delimited by periods (`.`) i.e. using URL hostname style syntax.

Each token:

- must not be empty.
- may use alphanumeric characters (`[0-9|a-z|A-Z]`) and/or underscores (`_`) only.
- must not start with numeric characters (`[0-9]`).

As a consequence, a provider name:

- must not contain any number of repeated periods (`.`), e.g. the name `foo..bar` is not allowed.
  - rationale: it removes the chance for accidental `..` from concatenation and therefore the need to collapse `..` to `.`.
  
Additionally, a provider name must not end nor start with a period (`.`).

The last token in a provider name is its base name and any preceding tokens make up its namespace.
For example, a `foo.bar.baz` provider name is made up of `foo`, `bar` and `baz` tokens, and refers to a node in the `foo.bar` namespace whose base name is `baz`.

#### Resource Names

All resources, whether content-oriented or host-oriented, have a name.

##### Definition

A resource name is made up of one or more tokens, delimited by forward slashes (`/`) i.e. using URL path style syntax.

Each token:

- must not be empty.
- may use alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`), and/or balanced curly braces (`{}`).
- must not start with numeric characters (`[0-9]`).
- may be a single tilde character (`~`).

As a consequence, a resource name:

- must not contain any number of repeated forward slashes (`/`), e.g. the name `//bar` is not allowed.
  - rationale: it removes the chance for accidental `//` from concatenation and therefore the need to collapse `//` to `/`.

Additionally, a resource name must not end with a forward slash (`/`), but may start with one (see [Locator Resolution](#locator-resolution) section).

The last token in a resource name is its base name, and any preceding tokens make up its namespace.
For example, a `/foo/bar/baz` resource name is made up of `foo`, `bar` and `baz` tokens, and refers to a resource in the `/foo/bar` namespace whose base name is `baz`.

#### Resource Type

All resources have a well known type. 

##### Definition

For each ROS resource type, a URL scheme (or protocol identifier) is defined as follows:

| Resource       | Scheme     |
|----------------|------------|
| ROS Topics     | rostopic   |
| ROS Services   | rosservice |
| ROS Actions    | rosaction  |
| ROS Parameters | rosparam   |

Thus, a topic name may be preceded by a `rostopic://` scheme prefix, and a service name may be preceded by a `rosservice://` scheme prefix.
For example, the resource name `/foo` may unambiguously refer to a topic with `rostopic:///foo` or to a service with `rosservice:///foo`.
Note the triple forward slash (`/`), which is a similar style to the `file://` scheme.

### Locator Resolution

#### Rationale

For ease of use, resource locators allow for a number of context dependent resolution mechanisms.

Additionally, both resource type and provider name components may be left unspecified, which may or may not lead to ambiguity when addressing resources in a system.
Tools and libraries may error in such circumstances, recompose missing components based on well known assumptions or simply leverage that ambiguity in application specific ways.
Some examples of this include but are not limited to:

- remapping names for multiple resources of different types that are named alike if no resource type is specified e.g. `foo:=bar` may result in name remapping for matching topics, services and actions.
- presume (and enforce) that upon a given operation on a resource, said resource is of the expected type even if not resource type is specified e.g. if subscribing to `foo/chatter` it is assumed and expected that said resource name refers to a topic.
- operating on a subset of all matching resources if no resource type and/or no provider name (if it applies) are specified e.g. calling an `add_two_ints` service may result in the first service found being called or all of them being called and the first response received returned.

#### Regular Resource Names (RRNs)

Regular resource names allow for a number of context dependent resolution mechanisms.

*Note*: this applies to resource names and not to provider names, which are always fully qualified.

##### Relative names resolution

Content-oriented resource names may be absolute or relative.

An absolute name begins with a forward slash (`/`) and it's not affected by the addressing node's namespace.
It can be regarded as a _global_ name, or a name at the _root_ namespace.

A relative name does not begin with a forward slash (`/`) and it is resolved _relative_ to the addressing node's namespace.
Periods (`.`) in the addressing node's namespace are replaced by forward slashes (`/`) and a leading forward slash (`/`) is added during resolution.

For example, if a `ping.pong.ball` node addresses (e.g. subscribes to) a `foo/bar` topic, then the topic name will be resolved to `/ping/pong/foo/bar`.
However, for the `/foo/bar` topic the topic name will stay `/foo/bar`, ignoring the node's namespace.

##### Private namespace substitution character 

The special single character token `~` is substituted by the addressing node's name.
Periods (`.`) in the addressing node's name are replaced by forward slashes (`/`) and a leading forward slash (`/`) is added during substitution.

It must be used at the beginning of a non-fully qualified name, if at all.

Here is a table with some example expansions:

| **Name** | **Node** : `my_node` | **Node** : `my_ns.my_node` |
|----------|----------------------|----------------------------|
| `ping`   | *`/ping`*            | *`/my_ns/ping`*            |
| `/ping`  | *`/ping`*            | *`/ping`*                  |
| `~`      | *`/my_node`*         | *`/my_ns/my_node`*         |
| `~/ping` | *`/my_node/ping`*    | *`/my_ns/my_node/ping`*    |

Note that the private namespace substitution character makes the name absolute, and therefore the namespace is not added a second time.

##### Substitutions

The bracket syntax (`{substitution_name}`) may be used in resource names to substitute useful contextual information into the them.
The set of substitution keys (names) are not set in this document, but some reasonable examples might be `{node}`, expanding to the addressing node's name, or `{ns}`, expanding to the addressing node's namespace.

Substitutions must result in a valid name and comply with the name constraints.
An example of an invalid substitution would be `{sub}/foo` and replace `{sub}` with a numeric value, which thus leads to a (invalid) resource name starting with a numeric character.

Substitutions are expanded after the private namespace substitution character is expanded.
Therefore a substitution may not contain the private namespace substitution character, i.e. `~`.
For example, given the name `{private}foo`, a substitution called `{private}` which expands to `~/_` results in an error because the `~/_` will end up in the expanded name as `/my_ns/~/_foo`, which is invalid.

Substitutions are expanded in a single pass, so substitutions should not expand to contain substitutions themselves.
For example, given the name `/foo/{bar_baz}` where `{bar_baz}` expands to `{bar}/baz` and where `{bar}` in turn expands to `bar`, you will get `/foo/{bar}/baz` as the final result, which is invalid, and not `/foo/bar/baz` as you might expect.

Substitutions are also not allowed to be nested, i.e. substitutions may not contain other substitutions in their names.
This is implicitly enforced by the rules above that say substitution names may only contain alphanumerics and underscores (`_`).
For example, given the name `{% raw %}/foo/{{bar}_baz}{% endraw %}` would result in an error because `{` and `}` are not allowed in a substitution names and the substitution name `{bar}_baz` does contain them.

#### Fully Qualified Resource Names (FQRNs)

Regular resource names resolution rules allow for convenient syntax, which in some cases requires additional context to expand to a fully qualified name that can be mapped to e.g. a DDS topic name.
For example, as outlined in further detail in the preceding sections, resource names may be relative (e.g. `foo` versus the absolute `/foo`), they may contain the private namespace substitution character (`~`), or arbitrary substitutions in curly braces (`{}`) syntax.
With enough context, each of these features can be expanded to some simple string to form the fully qualified name.

Fully qualified names have these additional restrictions:

- must start with a forward slash (`/`), i.e. they must be absolute.
- must not contain tilde (`~`) nor balanced curly braces (`{}`).

### Resource Qualification

#### Hidden nodes

If a (resource provider) node name contains any tokens that start with an underscore (`_`), the node it refers to will be considered hidden and tools may not show them unless explicitly asked to.

#### Hidden resources

If a resource name contains any tokens that start with an underscore (`_`), the resource it refers to will be considered hidden and tools may not show them unless explicitly asked to.

### Examples

For example, these are valid names:

| `foo`      | `abc123`   | `_foo`  | `Foo`               | `BAR`                |
| `~`        | `foo/bar`  | `~/foo` | `{foo}_bar`         | `foo/{ping}/bar`     |
| `foo/_bar` | `foo_/bar` | `foo_`  | `rosservice:///foo` | `rostopic://foo/bar` |
| `foo/_bar` | `foo_/bar` | `foo_`  | `rosservice:///foo` | `rostopic://foo/bar` |

But these are not valid names:

| `123abc`    | `123`  | `foo bar`  | ` `        | `foo//bar` |
| `/~`        | `~foo` | `foo~`     | `foo~/bar` | `foo/~bar` |
| `foo/~/bar` | `foo/` | `foo__bar` |            |            |

These are some valid fully qualified names:

| `/foo`                    | `rosparam://another.node/bool_param` | `rostopic:///ping`                 | `/_private/thing`                  |
| `rosaction://a.node/ping` | `/bar/baz`                           | `/public_namespace/_private/thing` | `rosservice://_private.node/reset` |

### Summary

For convenience, here is a summary of all the rules that apply to URLs in ROS.

Provider names in a URL:

- may contain alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`) and periods (`.`).
- must not start with a numeric character (`[0-9]`).
- must not end nor start with a period (`.`).
- must not contain any number of repeated periods (`.`).
- must not contain any number of repeated underscores (`_`).

Resource names in a URL:

- must not be empty.
- may contain alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`), and forward slashes (`/`).
- may use balanced curly braces (`{}`) in the resource name for substitutions.
- may start with a tilde (`~`), the private namespace substitution character.
- must not start with a numeric character (`[0-9]`).
- must not end with a forward slash (`/`).
- must not contain any number of repeated forward slashes (`/`).
- must not contain any number of repeated underscores (`_`).
- must separate a tilde (`~`) from the rest of the name with a forward slash (`/`), i.e. `~/foo` not `~foo`.
- must have balanced curly braces (`{}`) when used, i.e. `{sub}/foo` but not `{sub/foo` nor `/foo}`.

The content of substitutions, i.e. the string in side of balanced curly braces (`{}`), follow very similar rules to names.
The content of substitutions:

- must not be empty.
- may contain alphanumeric characters (`[0-9|a-z|A-Z]`) and underscores (`_`).
- must not start with a numeric character (`[0-9]`).

## Mapping Resource Names to DDS Concepts

### Naming Constraints 

#### Allowed characters

ROS URL syntax allows for more characters than e.g. DDS topic names allow: the forward slash (`/`), the tilde (`~`), the balanced curly braces (`{}`), the period (`.`) and the semicolon (`:`).
These must be substituted or otherwise removed during the process of converting a ROS URL to e.g. a DDS topic name.

After ROS resource names are expanded to fully qualified names, any balanced bracket (`{}`) substitutions and tildes (`~`) will have been expanded.
Periods (`.`) in provider names must be transformed into another suitable character e.g. the forward slash `/`, as it is done when resolving relative resource names.
Type schemes must also be removed after parsing and before mapping can take place.

*Note*: Previously, forward slashes (`/`) were disallowed in DDS topic names but now the restriction has been lifted (see [issue](https://issues.omg.org/issues/lists/dds-rtf5#issue-42236) on omg.org) and therefore ROS topic names are first prefixed with a ROS Specific Namespace prefix (described below) and then mapped directly to DDS topic names.

#### Name Length Limit

The length of a DDS topic name must not exceed 256 characters. Therefore the length of a ROS topic name, including the namespace hierarchy, the base name of the topic and any ros specific prefixes must not exceed 256 characters since this is mapped directly as DDS topic.

##### Considerations for RTI Connext

While testing our implementation with Connext, we encountered some additional limitations when it comes to the topic length.
That is, for example the length of a service name has tighter limits than the length of a topic.
In the RTI Connext implementation, service names are suffixed with the GUID value of the DDS participant for the service response topic.
Additionally, a content filtered topic (max length 256 characters) is created which is mapped from the suffixed service name.
Therefore when linking against `rmw_connext_c` or `rmw_connext_cpp`, service names cannot be longer than 185 characters including the namespace hierarchy and any ros specific prefixes.

### ROS Specific Namespace Prefix

In order to differentiate ROS resources easily in a DDS ecosystem, all DDS resources like topics created by ROS shall have their names automatically prefixed with a namespace like `/rX`, where `X` is a single character that indicates to which subsystem of ROS the resource belongs.
For example, a plain topic called `/foo` would translate to a DDS topic called `rt/foo`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the root namespace `/` and has a base name `foo`.
As another example, a topic called `/left/image_raw` would translate to a DDS topic `rt/left/image_raw`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the namespace `/left` and has a base name `image_raw`.

For systems where services are implemented with topics (e.g. OpenSplice), a different subsystem character can be used: `rq` for the request topic and `rr` for the response topic.
On systems where services are handled explicitly implemented, we consider a separate prefix, e.g. `rs`.

Here is a non-exhaustive list of prefixes:

| ROS Subsystem        | Prefix |
|----------------------|--------|
| ROS Topic            | rt     |
| ROS Service          | rs     |
| ROS Service Request  | rq     |
| ROS Service Response | rr     |

While all planned prefixes consist of two characters, i.e. `rX`, anything proceeding the first namespace separator, i.e. `/`, can be considered part of the prefix.
The standard reserves the right to use up to 8 characters for the prefix in case additional prefix space is needed in the future.

### Examples

Here are some examples of how a fully qualified ROS name would be broken down into DDS concepts:

| ROS Topic Name                  | DDS Topic                         |
|---------------------------------|-----------------------------------|
| `/foo`                          | `rt/foo`                          |
| `rostopic:///foo/bar`           | `rt/foo/bar`                      |
| `/robot1/camera_left/image_raw` | `rt/robot1/camera_left/image_raw` |

### Addressing Non-ROS DDS Resources

Since all ROS URLs are transformed and prefixed when being converted to DDS resource names, it makes it impossible to address existing DDS resources which do not follow the same naming pattern.
For example, if an existing DDS program is publishing on the `image` topic (and is using the DDS equivalent to the ROS message type) then a ROS program would not be able to subscribe to it because of the name mangling produced by the implicit ROS specific namespace.
Therefore, to allow ROS programs to interoperate with "native" DDS topic names, the API should provide a way to skip the ROS specific prefixing.

There is an option in the API, a boolean `avoid_ros_namespace_convention` in the `qos_profile` which can be set to `false` to use ROS namespace prefixes and to `true` to not use them.

For example:

| ROS Name           | avoid\_ros\_namespace_conventions | DDS Topic  |
|--------------------|-----------------------------------|------------|
| `rostopic://image` | `false`                           | `rt/image` |
| `rostopic://image` | `true`                            | `image`    |

#### Alternative (Idea)

Note that the alternative below is not part of the proposal, but only possible solutions to the issue of communicating with "native" DDS topics.
Another option would be to have some markup in the scheme name, for example:

| ROS Name                              | DDS Topic           |
|---------------------------------------|---------------------|
| `rostopic://image`                    | `rt/image`          |
| `rostopic+exact://image`              | `image`             |
| `rostopic+exact://camera_left/image`  | `camera_left/image` |
| `rostopic+exact:///camera_left/image` | `camera_left/image` |

## Comparison with ROS 1

In order to support a mapping to the - slightly more - restrictive DDS topic name rules, these rules are in some ways more constraining than the rules for ROS 1.
Other changes have been proposed for convenience or to remove a point of confusion that existed in ROS 1.
In ROS 2, resource names differ from ROS 1 in that they:

- must separate the tilde (`~`) from the rest of the name with a forward slash (`/`).

  - This is done to avoid inconsistency with how `~foo` works in filesystem paths versus when used in a name.

- may contain substitutions which are delimited with balanced curly braces (`{}`)

  - This is a more generic extension of the idea behind the tilde (`~`).

- have length limits.

  - This is driven by the name length limits in DDS.

- may be indicated as "hidden" by using a leading underscore (`_`) in one of the namespaces.

  - This is used to hide common but infrequently introspected resources like topics and services.

## Discussion

This section lists concerns about the proposed design and alternatives that were considered.

### Alternative Name Rules and Concerns About Name Rules

There were some suggested but then rejected alternatives to the rules for ROS resource names.

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
For the substitutions to be useful, then all implementations that process resource names need to support them.

This compromise was made so that when more work is done on substitutions, it would hopefully not require changes to the name syntax, but instead revolve around what substitutions to support and whether or not that support is optional.

### Alternative ROS to DDS Mappings

There was some discussion of alternatives and concerns with respect to the ROS -> DDS translation.

#### Alternative using DDS Partitions

Previously the usage of forward slashes (`/`) was disallowed in DDS topic names and hence a strategy was proposed which used DDS partitions to address the forward slashes (`/`) which are present in ROS resource names. The main idea was to separate the ROS resource name into "namespace" and "base name", and then place the namespace, stripped of leading and trailing forward slashes (`/`), into a single DDS partition entry and the remaining base name into the DDS topic name.
This addressed the issue because ROS resource base names will not contain any forward slashes (`/`) by definition and so there are no longer any disallowed characters in the DDS topic name.
The DDS partition would contain the ROS resource namespace, including any forward slashes (`/`) that made up the namespace and were not at the beginning or the end of the namespace.
That is acceptable because DDS partition names are allowed to contain forward slashes (`/`) unlike the DDS topic names previously, but now DDS topic names allow forward slashes (`/`).

DDS partitions are implemented as an array of strings within the `DDS::Publisher` and `DDS::Subscriber` QoS settings and have no hierarchy or order. 
Each entry in the partition array is directly combined with the DDS topic and they are not sequentially combined.
If a publisher has two partition entries, e.g. `foo` and `bar` with a base name of `baz`, this would be equivalent to having two different publishers on these topics: `/foo/baz` and `/bar/baz`.
Therefore this proposal used only one of the strings in the partitions array to hold the entire ROS resource's namespace.

You can read more about partitions in RTI's documentation:

- [PARTITION_QosPolicy](https://community.rti.com/static/documentation/connext-dds/5.2.3/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/PARTITION_QosPolicy.htm)

Trade-offs (in comparison to using the whole ROS name along with the namespaces):

- Splitting the ROS resource name into "namespace" and "base name", and placing the complete namespace into a field designed for another purpose seemed incorrect.
- In general partitions are recommended to be used as a spare, but using partitions for all ROS resource names suggested otherwise.
- Major concern was reported in this [issue](https://github.com/ros2/rmw_connext/issues/234), where having two topics with same base name, although different namespace and different types caused problem. For example: topicA is `/camera/data` of type `Image` and topicB is `/imu/data` of type `Imu`. The base names for both topicA and topicB is `data`, generated errors as described in the [issue](https://github.com/ros2/rmw_connext/issues/234).
- Newer standards such as [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE) might not have partitions at all.
- Using the complete ROS name in the later strategy will cause a tighter length limit on base name because the DDS topic name would contain ROS prefix, namespace along with the base name which should not exceed DDS topic name limitation which is 256 characters.

Rationale:
- With the decision from the DDS vendors to allow forward slashes (`/`) in DDS topic names, using the complete ROS name seemed simple and more intuitive than using partitions.

#### Alternative Substitute the Namespace Delimiter

A previous proposal was to substitute the namespace delimiter, i.e. the forward slash (`/`), with something that is allowed in DDS topic names, and then only use the DDS topic name to represent the full ROS resource name.
For example in the simplest case, a topic name `/foo/bar/baz` might become `__foo__bar__baz`, where the forward slash (`/`) has been replaced with a double underscore (`__`) and double underscores (`__`) were not allowed in ROS resource names.

Trade-offs (in comparison to the use of DDS partitions):

- Has a tighter length limit, since it limited by just the DDS topic name and does not benefit from part of the ROS topic name going into the DDS partition.
- The replacement for the forward slash (`/`) had to be more than one character since all usable characters were already in allowed in both the ROS names and DDS topic names, so each namespace further reduced the topic length limit.
- Implementation requires string replacement and validation afterwards, which is moderately complicated.

Rationale:

The DDS partition proposal is preferred over this alternative because it allows for longer total ROS names and because it is simpler to implement, i.e. splitting the string into base name and namespace is simpler than replacing the forward slashes (`/`) with double underscores (`__`) and then redoing length limit testing afterwards.

#### Capital Letter Substitution Alternative

This is another alternative that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".
Since the forward slash (`/`) is replaced with double underscores (`__`) when translating to DDS from ROS topic names, two characters out of the 256 character limit are lost with each additional namespace.
One proposed alternative was to add a constraint that ROS topic names could not use capital letters, and then capital letters could be used as the stand in for the forward slashes (`/`).

Trade-offs:

- Uses one fewer character per namespace and makes it easier to calculate the maximum length of a ROS topic or service name.
- Prevents users from using capital letters in their names, which is constraining and might be a problem for backwards compatibility with ROS 1 topics and services.

Rationale:

Preventing users from using capital letters was too constraining for the added benefit.

#### ROS Prefix with Single Underscore

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".
This alternative differs only in that it uses a single underscore in the prefix, i.e. `rt_` rather than `rt__` (`rt` + the leading `/`).

Trade-offs:

- Uses one fewer character
- Less consistent with replacement of other forward slashes (`/`)

Rationale:

Slight preference given to the more consistent alternative.

#### Limited Prefixes Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

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

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

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

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

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
