---
layout: default
title: ROS 2.0 ROSbags
abstract:
  Research and ideas on how to realize an efficient implementation for rosbags in the ROS2.0 ecosystem
published: false
author: '[Karsten Knese](https://github.com/karsten1987)'
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


## Motivation

One of the most crucial and essential components of ROS1 is its persistent data recording mechanism, called [rosbags](http://wiki.ros.org/rosbag).
It has been proven to be a core and essential component of ROS1 as the capability to record and replay system data of all types has become crucial to data analysis and debugging purposes.
Given various use cases for this data logging, rosbags can contain either very simple data such as a trajectory of a robotics end effector up to highly complex data such as autonomous vehicle with multiple redundant high resolution sensors.
Their file size thus can range from only a few kilobytes with just a few messages up to multiple terabytes and millions of message instances stored.
In order to maintain high performance when logging, rosbags avoid the serialization and de-serialization of the recorded messages.

When designing ROS2, this functionality of a high performance data recording has to be present as well.
As there are fundamental differences between ROS1 and ROS2, the technical challenges shall be examined within this article.

### Rosbags in ROS1

The implementation of rosbags in ROS1 are next to high level tools such as [rqt_bag](http://wiki.ros.org/rqt_bag), split in two fundamental packages.
The [rosbag_storage](http://wiki.ros.org/rosbag_storage) package abstracts the writing and reading of ROS messages from the ros client libraries and therefore decouples the storage from the ROS interface and implements its own storage format described in [here](http://wiki.ros.org/Bags/Format/2.0).
The current version 2.0 of this format has features such as compression and a second-level index for gathering information about the content of the bag file.

### Use cases in ROS2

Within ROS2, the implementation of rosbags in brings new use cases with it that shall be addressed.
The new ROS2 API architecture introduces a [generic RMW interface](http://design.ros2.org/articles/ros_middleware_interface.html), which allows various data formats from different middlewares, such as CDR from DDS.

Even though DDS is the default middleware of ROS2, the implementation of rosbags in ROS2 requires to be flexible enough to record and replay other data formats than CDR, such as protobuf or ZeroMQ.

## Implementation of rosbags in ROS2

In the next section describes the overall proposal for the implementation of rosbags in ROS2.
Similar to how it is realized in ROS1, the idea is to split the architecture into multiple independent packages.

From bottom-up, we define three layers of abstraction: rosbag storage API, rosbag API, rosbag command line interface.
The rosbag storage API is ROS2 agnostic in a sense that it only reads and writes abstract binary messages, which are well defined given some meta information.
The second layer, the rosbag API, is responsible for fetching ROS2 messages, serialize and sufficiently subscribe them for the storage API to write them.
Inversely, it receives the binary representation from the storage API and converts them into fully defined ROS2 messages.
Finally, the command line tool gives a user friendly entry point for steering the recording and replaying rosbags.

The following image depicts this architecture.

![rosbag2 system diagram](https://i.imgur.com/frhfnuN.png)

In the following, we are going to define these in detail and highlight their main purpose and technical requirements.

## rosbag storage API

We define data storage as the underlying method of storing the incoming ros messages in a binary format.
The data storage is responsible for persistently save ros messages with enough meta information to restore its full context when loading.
While its main purpose will be to store ros messages, the abstraction layer shall be ROS2 agnostic and really only deal with binary data and generic meta information.
This allows to manually insert custom data such as [pcap](https://en.wikipedia.org/wiki/Pcap) recordings, if suitable meta information is available.

### Data format

We define the data format as the format in which ros messages are represented in binary form.
The format has to be uniquely described and to be linked to the binary representation in the data storage such to be able to interpret its binary representation and thus restore the ros message.

As introduced in the motivation section, multiple representations can be supported in ROS2.
Therefore, the requirements for the rosbag storage API is that it can handle multiple data formats.

### Data storage

We introduce a separate layer of API for interfacing the with underlying data storage.
In order to write a message, an abstract representation of a message has to be given as input.
Such a representation comprises the binary blob for the actual messages and a key-value pair of meta data.
The data format description is part of this meta data.

We decided to chose a third party technology for this and move away from the custom rosbag storage in ROS1.
After looking into existing solutions, we decided to implement an extensible plugin architecture for this API layer, which enables to connect various data storage solutions and therefore gives the user a chance to optimize their data storage according to their use cases.

We chose SQLite as the default solutions for this implementation given that is extensively tested, actively maintained and a broad user group.
// TODO(karsten1987): Benchmark and qualified evaluation results why we chose sqlite

However, this plugin architecture becomes already a necessity when thinking about the integration or compatibility with existing ROS1 rosbags.
As there is a huge amount of data stored in ROS1 rosbags, the rosbag tool for ROS2 has to be able to easily cope with legacy rosbags.
This can be done by having a ROS1 plugin, which can read existing ROS1 messages and bridge them into a ROS2 message format, e.g. with the [static bridge](https://github.com/ros2/ros1_bridge).
Note that this plugin most likely will be implemented as read-only, meaning it is solely possible to read existing ROS1 messages and populate them in the ROS2 ecosystem, however the other way around will not be supported.

For each supported data storage (e.g. SQLite or ROSbag format2) these key-value pairs have to be interpreted accordingly to correctly store the message.

### Query format

// TODO(karsten1987): Describe how we can fetch/query data from the data storage


## rosbag API

The rosbag API describes the necessary interface to read and write ros messages into the bag file.
It is responsible for storing incoming messages with the correct data format in the data storage.
Similarly, the API must be restoring the ros message with its given data format from its binary representation in the data storage.

The rosbag API has a strong dependency on the rosidl typesupport packages of ROS2, generically described [here](http://design.ros2.org/articles/interface_definition.html)
The main purpose of this package is to issue queries towards the storage API, receive binary data and the respective meta information and convert the message to its appropriate ROS2 message type.

// TODO(karsten1987): Describe how tools can use this API


## rosbag CLI

The rosbag API shall be integrated as part of a new verb for the existing [ros2cli](https://github.com/ros2/ros2cli).


## Alternatives

### Relaxed or dismissed requirements
We need a data storage format which allows to sufficiently store and replay transmitted data with the least possible cost overhead.
There are a few requirements for writing and reading to such a data storage format:

#### Scalability
Nowadays, robotic systems can comprise a large number of sensors publishing data in parallel.
This can easily lead to a significantly large amount of data over time.
The chosen format has thus be able to scale up to a huge file size (> 1 TB).

#### Parallel I/O
Processing time increases with slow file I/O.
In order to provide efficient data processing, a parallel read and write to the file from multiple processes should be available.
This would allow multiple processes (e.g. one per sensor) directly write to a commonly shared bag file without having a single recoding instance subscribing to all topics..

#### Compression
When the file size becomes larger or disk space is only limited, it should be possible to compress the bag file.
Compression can either be happening during write time or in a post-processing step.

#### Random access
It must be possible to grant random read access to the file and extract specific individual messages.
Random access further means that it should be possible to extract the n-th message of one topic without having to scroll through all messages in the same chunk.

#### Range access
It further should be possible to only replay/read a section of the record, specified by a range of time.
It should be possible to access a range of messages in terms of timestamps from `tx` to `tx+n`.

#### Variable Chunk sizes
The chunk sizes must be configurable to fit various large message types in order to guarantee best performance for various large message types.
It should further be possible to configure the condition on when to write such a chunk permanently to disk (e.g. in a given time interval or when chunk size is reached).

#### Backwards compatible with ROS1
A general requirement is to be backwards compatible with existing ROS1 bags.
This compatibility can either be via a conversion script, which permanently converts ROS1 bags into ROS2 bags or a bridge API which allows to manually open existing ROS1 bags and publish them into the ROS2 system.


### Dismissed data storage formats

In the following, we are iterating over a couple of data formats, which may be suitable for the underlying ROSbag implementation.
We hereby iterate over existing third party software as well as examining of maintaining a self-made format.


#### HDF5

One very popular framework for storing scientific data is [HDF5](https://support.hdfgroup.org/HDF5/).
It basically has all the necessary requirements listed above such as random access, parallelism and compression.
It is further designed for highly complex data with an extensive amount of data.
HDF5 is open source and its source can be freely obtained from [bitbucket](https://bitbucket.hdfgroup.org/projects/HDFFV/repos/hdf5/browse) and is under a [permissive license](https://bitbucket.hdfgroup.org/projects/HDFFV/repos/hdf5/browse/COPYING) of the HDF group.
Multiple language wrapper or bindings are available, namely C/C++, python, fortran or java.

##### Pros
- Open Source and standard specification for file format
- Fulfills the requirements given
- Multi language support
- Large community of users

##### Cons
- Depending on a slowly developing standard
- The table dimensions of each chunk have to be of fixed size, known at startup.

There is a popular [blog post](http://cyrille.rossant.net/moving-away-hdf5/) by Cyrille Rossant, which gives a short introduction, but also discusses some controversy with HDF5.


#### Existing ROS1 format

Alternatively, the existing ROS1 format can be continued to be used. The format description can be found [here](http://wiki.ros.org/Bags/Format/2.0).

##### Pros
- Already ROS specific and evaluated for ROS messages, existing code could be reused

##### Cons
- No random access


#### SQLite

A third alternative which provides capabilities for data logging, and is used as such, is [SQLite](https://www.sqlite.org/about.html)
Despite other relational database systems, it doesn't require any external SQL Server, but is self-contained.
It is also open source, [extensively tested](https://www.sqlite.org/testing.html) and known to have a large [community](https://www.sqlite.org/support.html).
The Gazebo team created a nicely composed [comparison](https://osrfoundation.atlassian.net/wiki/spaces/GAZ/pages/99844178/SQLite3+Proposal) between SQLite and the existing rosbag format.

##### Pros
- Table dimensions do not have to be known at startup and can be flexibly extended.
- Ability to query the tables with classical relational SQL syntax.

##### Cons
- tbd


## API requirements

TODO: Describe idea of having a plugin-like API where a methods for (de-)serialization and storage file format can be chosen dynamically. This allows an optimal rosbag configuration per use-case. For example could it be possible to use CDR and sqlite when data is written sequentially and thus writing speed matters. Other possibilities could be using deserialized JSON and MongoDB when writing speed doesn't matter too much, rather than indexing and introspecting.

## API for recording and replaying

The most important requirement for rosbags is being capable to record all available topics with minimal deserializing overhead.
We shall therefore implement a set of functions in the rmw layer, which allows users to take messages raw, meaning in a serialized form object to the underlying middleware.
In the case of DDS, such a raw message shall correspond to the CDR data being sent over the wire.
Simultaneously, there shall be an API to convert a ROS2.0 message into its binary representation, such as CDR, in order to record data which is not being sent over the wire but created manually.

The same requirements are set for publishing stored data, where already serialized data shall be transmitted over the wire without the need of serializing.
Analog to taking a message in its raw format, we shall implement a rmw function which allows publishing a raw message on a topic.
In order to read a serialized message from a rosbag, we shall have a function which converts a serialized binary representation in its corresponding ROS2.0 message.

Given the requirements above, we propose the following rmw API:

```c
rmw_ret_t
rmw_take_raw(const rmw_subscription_t * subscription, rmw_message_raw_t * raw_message, bool * taken);

rmw_ret_t
rmw_to_raw_message(const void * ros_message, const rosidl_message_type_support_t * type_support, rmw_message_raw_t * raw_message);

rmw_ret_t
rmw_publish_raw(const rmw_publisher_t * publisher, const rmw_message_raw_t * raw_message);

rmw_ret_t
rmw_from_raw_message(const void * rmw_message_raw_t, const rosidl_message_type_support_t * type_support, void * ros_message);
```

The in the code snippet mentioned raw message shall be defined as follows:

```c
typedef struct rmw_message_raw_t
{
  unsigned int encoding_identifier;
  unsigned int length;
  char * raw_data;
} rmw_message_raw_t;
```

The `encoding_identifier` here indicates which encoding is used in this raw message, e.g. CDR in the case of DDS.
The `raw_data` field shall contain all message data needed to extract a ROS message given a respective type support, which contains all necessary information on how to concert the raw data into its corresponding ros message type.
An example for CDR data in case of DDS:

The ROS message string
```
std_msgs::msg::String msg;
msg.data = "hello world 42";
```
translates into a rmw_message_raw_t
```
length: 24
data (in hex): 0x00 0x01 0x00 0x00 0x0f 0x00 0x00 0x00 0x68 0x65 0x6c 0x6c 0x6f 0x20 0x77 0x6f 0x72 0x6c 0x64 0x20 0x34 0x32 0x00 0x00
```
