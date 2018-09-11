---
layout: default
title: WideStrings
abstract:
  Some ROS2 users would like to use multi-byte characters for topic data, suitable for supporting multi-byte languages.
  This article lays out the problem, and explores potential solutions to this problem.
author: '[Chris Lalancette](https://github.com/clalancette)'
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

ROS2 currently supports the concept of strings, which are sequences of single-byte characters.
When working with ROS 2.0 topic data, some users would like to use multi-byte characters (e.g. UTF-8) for their topic data.
This article explores the problem and the possible solutions.
Note that this article specifically does not talk about binary data, as that is already handled by using uint8_t arrays.
It also does not talk about using multi-byte characters for the topic *names*, as this is disallowed by the DDS specification.

## Multi-byte character background

Before delving into ROS 2.0 specifics, some time should be spent discussing multi-byte characters in general.
What is meant by multi-byte characters?
The original ASCII character set only provided for printable characters using binary sequences 0 to 127.
Several extensions were made to this to use 128-255, but that approach only extended the character set to be able to cover a few more languages.
Several other encodings were created as the need arose, and it became clear by the 1990's that a worldwide standard was required.
Thus, Unicode was created to be a single specification that could cover all of the languages of the world.
The early versions of Unicode actually failed to live up to this expectation, and some of those early versions of Unicode are still baked into widely used software (Microsoft Windows, Java, etc).
Later on, the Unicode standard evolved to truly be able to cover the languages of the entire world, and this is what should generally be used for new designs.
As a side benefit, the newer versions of Unicode (UTF-8) are backwards compatible with ASCII.
There is a lot of history here, and things are much more complicated then the brief introduction above.
For more information, the following links provide introductory material:

* [http://kunststube.net/encoding/](http://kunststube.net/encoding/)
* [http://stackoverflow.com/questions/4588302/why-isnt-wchar-t-widely-used-in-code-for-linux-related-platforms](http://stackoverflow.com/questions/4588302/why-isnt-wchar-t-widely-used-in-code-for-linux-related-platforms)
* [http://www.diveintopython3.net/strings.html](http://www.diveintopython3.net/strings.html)
* [http://stackoverflow.com/questions/402283/stdwstring-vs-stdstring](http://stackoverflow.com/questions/402283/stdwstring-vs-stdstring)

The rest of this document is generally going to assume Unicode, unless stated otherwise.

## Introducing Wide Strings
To support multi-byte characters, ROS 2.0 will introduce the concept of a wide string.
There are a few basic questions to answer on how a wide string will be integrated into ROS 2.0:

* What is the size impact of the wide string?
* What is the encoding of the wide string?
* What does the API look like to a user of ROS 2.0?

Each of these questions will be examined in more detail below.

### What is the size impact of the wide string?

There are 2 parts to this question.
One is how the wide string is mapped onto the underlying DDS implementation, which defines how large messages will be on-the-wire (and on disk).
The other part of the question deals with the runtime impact (in terms of speed and code size) of having to deal with wide strings.

**Mapping onto the DDS implementation**

There are a couple of different ways that a wide string could be mapped onto a DDS implementation.
The first way it could be mapped is onto a uint8_t byte array.
While this would work, it has the downside that the byte array would be "opaque", and any introspection tools into DDS wouldn't see anything but a stream of bytes.
The second way it could be mapped is onto the "wchar/wstring" type defined by the DDS specification.
The DDS specification itself doesn't define a size for wchar, but the X-Types specification (for DDS introspection) does say that a wchar should be mapped onto a Char32.
Thus, all known encodings would fit into a wchar, and DDS introspection tools could look into wstrings.
Thus, the current design of wide strings map onto wchar DDS types.

**Runtime impact of wide string**

Dealing with wide strings puts more strain on the software of a system, both in terms of speed and in terms of code size.
This is felt most acutely when talking about small microcontrollers, where both flash size (code space) and performance (processor speed) are at a premium.
Some of the common encodings (including UTF-8) are defined as being variable width, meaning that a character can take 1, 2, or 4 characters to represent.
These encoding also typically have the ability to combine one character with the next in non-trivial ways.
As one of the goals of ROS 2.0 is to support small, constrained systems, dealing with wide strings may be out of the question.
Thus, the current ROS 2.0 design for wide strings makes them optional, defines them as a separate type from regular strings, and recommends against using them for common messages.

### What is the encoding of the wide string?

There are two main ways that ROS 2.0 can provide for the encoding of a wide string.
Either ROS 2.0 can define exactly what the encoding will be for all strings, or it can allow the user to specify the encoding when handing it to ROS 2.0.
There are pros and cons to each approach.

**ROS 2.0 defines the encoding**

Pros:

* All strings known to be in the same encoding.

Cons:

* If the user-level code uses a different encoding, the user is responsible for doing a conversion to the ROS 2.0 defined encoding.
  This can involve some runtime cost.

**User defines the encoding**

Pros:

* No conversions necessary for user-to-ROS 2 API.

Cons:

* The user must always tell ROS 2.0 what encoding the data is in.
* The encoding needs to be transmitted on the wire to the other side.
* It is not clear how ROS 2.0 will transmit the encoding.
  Fixed list?
  Which encodings go into the list?

While a user defined encoding looks somewhat attractive at first, the downsides to it make it difficult to determine exactly which encodings are in use.
The downsides also mean that any program that wants to parse ROS 2 messages (or bag files) may potentially have to deal with many different encodings.
Additionally, Unicode (and in particular, UTF-8) are very commonplace now, while other encodings are becoming more esoteric.
Even earlier Unicode (UTF-16 and UTF-32) encodings are essentially deprecated in favor of UTF-8.
For these reasons, the current design of the ROS 2.0 wide string is to use a UTF-8 encoding for all strings.

### What does the API look like to a user of ROS 2.0?

#### Python 3

Python 3 splits byte arrays from strings pretty cleanly.
Thus, in Python 3, a string is a sequence of characters, where each character can take 1 or more bytes.
Byte arrays are sequences of bytes.
Thus, the ROS 2.0 API for dealing with wide strings (such as the publish API) will take just a string type in, and encode it as utf-8.
The ROS 2.0 API for dealing with regular strings will take a string type in, and encode it as ASCII.

**Example**

```
import sys
from time import sleep

import rclpy

from rclpy.qos import qos_profile_default

from std_msgs.msg import WString

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args)

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(WString, 'chatter', qos_profile_default)

    msg = String()

    i = 1
    while True:
        msg.data = 'Hello World: {0}'.format(i)
        i += 1
        print('Publishing: "{0}"'.format(msg.data))
        chatter_pub.publish(msg)
        # TODO(wjwwood): need to spin_some or spin_once with timeout
        sleep(1)


if __name__ == '__main__':
    main()
```

Notice that this code looks almost identical to the same demo for String.
That's because in python, the characters in the string and the byte representation are cleanly separated.
Thus, the API looks exactly the same as it would for the String type.

#### C++

C++ doesn't have good built-in support for wide strings.
Using wchar_t is generally not a good option, as wchar_t has different sizes on different platforms (2 bytes on Windows, 4 bytes on Linux).
Thus, ROS 2.0 will define a new type to deal with wide strings.
The APIs that deal with wide strings will expect this new type to be passed in.

**Example**

```
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/w_string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("talker");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  auto chatter_pub = node->create_publisher<std_msgs::msg::WString>("chatter", custom_qos_profile);

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<std_msgs::msg::WString>();
  auto i = 1;

  while (rclcpp::ok()) {
    utf8_string hello("Hello World: " + std::to_string(i++));
    msg->data = hello;
    std::cout << "Publishing: '" << msg->data.cpp_str() << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
```

Here, ROS 2 has implemented a new type called "utf8_string" (this is still to be implemented, and the name may change).
The publisher takes in types of utf8_string, and generates the correct data on the wire.

### Bounded wide strings

A note on bounded wide strings.
ROS 2.0 allows for "bounded" strings and wide strings.
These are strings that can be no longer than their bounded number.
In the case of strings, the width of a character == the width of a byte in most cases, so the number of bytes in a bounded string is generally the bound (plus 1 for the NULL-termination).
Wide strings are a little bit different.
When dealing with bounded wide strings, the bound is on the number of characters, not bytes.
Thus the maximum number of bytes the bounded wide string can be may be, and most likely will be, larger than the number of characters.

## Summary

To summarize, the current ROS 2.0 design for wide strings:

* Maps wide characters onto the DDS type wchar.
  This in turn maps wide strings on the DDS type wstring.
* Allows them to be optionally supported by an implementation.
* Defines wide strings as a separate type from strings.
* Recommends against using wide strings for common messages.
* Defines the encoding for all wide string to be UTF-8.
* For python the APIs that deal with wstrings will take a regular str type in.
* For C++, ROS 2.0 will define a new type to deal with wstrings.
