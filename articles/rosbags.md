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

ROSbags have been proven to be a core and essential component of ROS1.
The capability to record and replay robotic data of all types became crucial to data analysis and debugging.
This functionality has to be available in ROS2.0 as well.
The technical challenges shall be examined within this article.
We shall the discuss the requirements on such a tool, the technical challenges and changes to be made to the current ROS2.0 system.


## Requirements for storage format

To be chosen from section "Relaxed or dismissed requirements" in the "Alternatives" section


## Proposal for data storage format

To be chosen from section "Dismissed data storage formats" in the "Alternatives" section


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
