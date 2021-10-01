---
layout: default
title: ROS 2 Launch Static Descriptions - Implementation Considerations
permalink: articles/roslaunch_frontend.html
abstract:
  The launch system in ROS 2 aims to support extension of static descriptions, so as to easily allow both exposing new features of the underlying implementation, which may or may not be extensible itself, and introducing new markup languages. This document discusses several approaches for implementations to follow.
author: '[Michel Hidalgo](https://github.com/hidmic) [William Woodall](https://github.com/wjwwood)'
date_written: 2019-09
last_modified: 2020-07
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}

Date Written: {{ page.date_written }}

Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

## Context

Static launch descriptions are an integral part to ROS 2 launch system, and the natural path to transition from predominant ROS 1 `roslaunch` XML description.
This document describes parsing and integration approaches of different front ends i.e. different markup languages, with a focus on extensibility and scalability.

## Proposed approaches

In the following, different approaches to parsing launch descriptions are described.
This list is by no means exhaustive.

It's worth noting some things they all have in common:

- All of them attempt to solve the problem in a general and extensible way to help the system scale with its community.

- All of them require a way to establish associations between entities, solved using unique reference id (usually, a human-readable name but that's not a requisite).

- All of them associate a markup language with a given substitution syntax.

- All of them supply a general, interpolating substitution to deal with embedded substitutions e.g. "rooted/$(subst ...)".

- All of them need some form of instantiation and/or parsing procedure registry for parsers to lookup.
  How that registry is populated and provided to the parser may vary.
  For instance, in Python class decorators may populate a global dict or even its import mechanism may be used if suitable, while in C++ convenience macros may expand into demangled registration hooks that can later be looked up by a dynamic linker (assuming launch entities libraries are shared libraries).

### Forward Description Mapping (FDM)

In FDM, the parser relies on a schema and well-known rules to map a static description (markup) to implementation specific instances (objects).
The parser instantiates each launch entity by parsing and collecting the instantiations of the launch entities that make up the former description.

#### Description Markup

Some description samples in different markup languages are provided below:

*XML*
```xml
<launch version="x.1.0">
  <action name="some-action" type="actions.ExecuteProcess">
    <arg name="cmd" value="$(substitutions.FindExecutable name=my-process)"/>
    <arg name="env">
      <pair name="LD_LIBRARY_PATH"
            value="/opt/dir:$(substitutions.EnvironmentVariable name=LD_LIBRARY_PATH)"/>
    </arg>
    <arg name="prefix" value="$(substitutions.EnvironmentVariable name=LAUNCH_PREFIX)"/>
  </action>
  <on_event type="events.ProcessExit" target="some-action">
    <action type="actions.LogInfo">
      <arg name="message" value="I'm done"/>
    </action>
  </on_event>
</launch>
```

*YAML*
```yml
launch:
  version: x.1.0
  actions:
    - type: actions.ExecuteProcess
      name: some-action
      args:
        cmd: $(substitutions.FindExecutable name=my-process)
        env:
          LD_LIBRARY_PATH: "/opt/dir:$(substitutions.EnvironmentVariable name=LD_LIBRARY_PATH)"
        prefix: $(substitutions.EnvironmentVariable name=LAUNCH_PREFIX)
  event_handlers:
    - type: events.ProcessExit
      target: some-action
      actions:
        - type: actions.LogInfo
          args:
            message: "I'm done"
```

#### Advantages & Disadvantages

*+* Straightforward to implement.

*+* Launch implementations are completely unaware of the existence of the static description formats and their parsing process (to the extent that type agnostic instantiation mechanisms are available).

*-* Statically typed launch system implementations may require variant objects to deal with actions.

*-* Static descriptions are geared towards easing parsing, making them more uniform like a serialization format but also less user friendly.

*-* Care must be exercised to avoid coupling static descriptions with a given implementation.

### Forward Description Mapping plus Markup Helpers (FDM+)

A variation on FDM that allows launch entities to supply markup language specific helpers to do their own parsing.
The parser may thus delegate entire description sections to these helpers, which may or may not delegate back to the parser (e.g. for nested arbitrary launch entities).

#### Description Markup

Some description samples in different markup languages are provided below:

*XML*
```xml
<launch version="x.1.0">
  <executable name="some-action" cmd="$(find-exec my-process)" prefix="$(env LAUNCH_PREFIX)">
     <env name="LD_LIBRARY_PATH" value="/opt/dir:$(env LD_LIBRARY_PATH)"/>
     <on_exit>
        <log message="I'm done"/>
     </on_exit>
  </executable>
</launch>
```

*YAML*
```yml
launch:
  version: x.1.0
  children:
    - executable:
        cmd: $(find-exec my-process)
        env:
          LD_LIBRARY_PATH: "/opt/dir:$(env LD_LIBRARY_PATH)"
        prefix: $(env LAUNCH_PREFIX)
        on_exit:
          - log: "I'm done"
```

#### Parsing Hooks

Some code samples in different programming languages are provided below:

*Python*
```python
@launch_markup.xml.handle_tag('process')
def process_tag_helper(xml_element, parser):
    ...
    return launch.actions.ExecuteProcess(...)

@launch_markup.handle_subst('env')
def env_subst_helper(args, parser):
    ...
    return launch.substitutions.EnvironmentVariable(parser.parse(args[0]))
```

*C++*
```c++
std::unique_ptr<launch::actions::ExecuteProcess>
process_tag_helper(const XmlElement & xml_element, const launch_markup::xml::Parser & parser) {
    // ...
    return std::make_unique<launch::actions::ExecuteProcess>(/* ... */);
};

LAUNCH_XML_MARKUP_HANDLE_TAG("process", process_tag_helper);

std::unique_ptr<launch::substitutions::EnvironmentVariable>
env_subst_helper(const std::vector<std::string> & args, const launch_markup::Parser & parser) {
    // ...
    return std::make_unique<launch::substitutions::EnvironmentVariable>(parser.parse(args[0]));
}

LAUNCH_MARKUP_HANDLE_SUBST("env", env_subst_helper);
```

#### Advantanges & Disadvantages

*+* Straightforward to implement.

*-* Launch system implementations are aware of the parsing process, being completely involved with it if sugars are to be provided.

*+* Allows leveraging the strengths of each markup language.

*-* Opens the door to big differences in the representation of launch entities across different front end implementations, and even within a given one by allowing the users to introduce multiple custom representations for the same concept (e.g. a list of numbers).

*-* Care must be exercised to avoid coupling static descriptions with a given implementation.

### Abstract Description Parsing (ADP)

In ADP, the parser provides an abstract interface to the static description and delegates parsing and instantiation to hooks registered by the implementation.
The parser does not attempt any form of description inference, traversing the description through of the provided hooks.

#### Description Markup

Some description samples in different markup languages are provided below:

*XML*
```xml
<launch version="x.1.0">
  <executable name="some-action" cmd="$(find-exec my-process)" prefix="$(env LAUNCH_PREFIX)">
     <env name="LD_LIBRARY_PATH" value="/opt/dir:$(env LD_LIBRARY_PATH)"/>
     <on_exit>
      <log message="I'm done"/>
    </on_exit>
  </executable>
</launch>
```

*YAML*
```yaml
launch:
  version: x.1.0
  children:
    - executable:
        cmd: $(find-exec my-process)
        env:
          LD_LIBRARY_PATH: "/opt/dir:$(env LD_LIBRARY_PATH)"
        prefix: $(env LAUNCH_PREFIX)
        on_exit:
          - log:
              message: 'I'm done'
```

#### Abstract Description

To be able to abstract away launch descriptions written in conceptually different markup languages, the abstraction relies on the assumption that all launch system implementations are built as object hierarchies.
If that holds, then ultimately all domain specific schemas and formats will just be different mappings of said hierarchy.
Therefore, a hierarchical, object-oriented representation of the markup description can be built.

Define an `Entity` object that has:

- a namespaced type;
- optionally a name, unique among the others;
- optionally a parent entity (i.e. unless it's the root entity);
- optionally one or more named attributes, whose values can be entities, ordered sequences of them or neither e.g. scalar values.

Some sample definitions in different programming languages are provided below:

*Python*
```python
class Entity:

    @property
    def type_name(self):
        pass

    @property
    def parent(self):
        pass

    @property
    def children(self):
        pass

    def get_attr(
      self,
      name,
      *,
      data_type=str,
      optional=False
    ):
        pass
```

*C++*
```c++
namespace parsing {
class Entity {

  const std::string & type() const;

  const Entity & parent() const;

  const std::vector<Entity> & children() const;

  template<typename T>
  const T & get(const std::string& name) const { ... }

  const Entity & get(const std::string& name) const { ... }

  template<typename T>
  bool has(const std::string & name) const { ... }

  bool has(const std::string & name) const { ... }
};
}  // namespace parsing
```

It is up to each front end implementation to choose how to map these concepts to the markup language.
For instance, one could map both of the following descriptions:

*XML*
```xml
<node name="my-node" pkg="demos" exec="talker">
  <param name="a" value="100."/>
  <param name="b" value="stuff"/>
</node>
```

*YAML*
```yaml
- node:
    name: my-node
    pkg: demos
    exec: talker
    param:
      - name: a
        value: 100.
      - name: b
        value: stuff
```

such that their associated parsing entity `e` exposes its data as follows:

*Python*
```python
e.type_name == 'node'
e.get_attr(name) == 'my-node'
e.get_attr('pkg', optional=True) != None
params = e.get_attr('param', data_type=List[Entity])
params[0].get_attr('name') == 'a'
params[1].get_attr('name') == 'b'
```

*C++*
```c++
e.type() == "node"
e.get<std::string>("name") == "my-node"
e.has<std::string>("pkg") == true
auto params = e.get<std::vector<parsing::Entity>>("param")
params[0].get<std::string>("name") == "a"
params[1].get<std::string>("name") == "b"
```

Inherent ambiguities will arise from the mapping described above, e.g. nested tags in an XML description may be understood either as children (as it'd be the case for a grouping/scoping action) or attributes of the enclosing tag associated entity (as it's the case in the example above).
It is up to the parsing procedures to disambiguate them.

#### Parsing Delegation

Each launch entity that is to be statically described must provide a parsing procedure.

*Python*
```python
@launch.frontend.expose_action('some-action')
class SomeAction(launch.Action):

    @classmethod
    def parse(
        cls,
        entity: launch.frontend.Entity,
        parser: launch.frontend.Parser
    ) -> SomeAction:
        return cls(
            scoped=parser.parse_substitution(entity.get_attr(scoped)), entities=[
                parser.parse_action(child) for child in entity.children
            ]
        )

@launch.frontend.expose_substitution('some-subst')
class SomeSubstitution(launch.Substitution):

    @classmethod
    def parse(
        cls,
        data: Iterable[SomeSubstitutionsType]
    ) -> SomeSubstitution:
        return cls(*data)
```

*C++*
```c++
class SomeAction : public launch::Action {
  static std::unique_ptr<SomeAction>
  parse(const parsing::Entity & e, const parsing::Parser & parser) {
    std::vector<launch::DescriptionEntity> entities;
    for (const auto & child : e.children()) {
      entities.push_back(parser.parse(child));
    }
    return std::make_unique<SomeAction>(
      parser.parse(e.get<parsing::Value>("scoped")), entities
    );
  }
};

LAUNCH_EXPOSE_ACTION("some-action", SomeAction);

class SomeSubstitution : public launch::Substitution {
  static std::unique_ptr<SomeSubstitution>
  parse(const parsing::Value & v, const parsing::Parser & parser) {
    return std::make_unique<SomeSubstitution>(v.as<std::vector<std::string>>());
  }
};

LAUNCH_EXPOSE_SUBSTITUTION("some-subst", SomeSubst);
```

As can be seen above, procedures inspect the description through the given parsing entity, delegating further parsing to the parser recursively.
To deal with substitutions, and variant values in general, the concept of a 'value' is introduced.
Note that a value _may be_ an entity, but it isn't necessarily one.

#### Procedure Provisioning

##### Manual Provisioning

In the simplest case, the user may explicitly provide their own parsing procedure for each launch entity.

##### Automatic Derivation

If accurate type information is somehow available (e.g.: type annotations in constructor), reflection mechanisms can aid derivation of a parsing procedure with no user intervention.

#### Advantages & Disadvantages

The abstraction layer allows.

*-* Launch system implementations are aware of the parsing process.

*+* The static description abstraction effectively decouples launch frontends and backends, allowing for completely independent development and full feature availability at zero cost.

*-* No markup language specific sugars are possible.
    REVISIT(hidmic): IMHO explicitly disallowing this is a good thing, it makes for more homogeneus descriptions and avoids proliferation of multiple representation of the same concepts (e.g. a list of strings).

*+* The transfer function nature of the parsing procedure precludes the need for a rooted object type hierarchy in statically typed launch system implementations.

*-* Automatic parsing provisioning requires accurate type information, which may not be trivial to gather in some implementations.

*-* Harder to implement.
