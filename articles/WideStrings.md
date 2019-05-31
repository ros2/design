---
layout: default
title: Unicode Support
permalink: articles/wide_strings.html
abstract:
  This article describes how ROS 2 will support sending multi-byte character data using the [Unicode](https://en.wikipedia.org/wiki/Unicode) standard.
  It also describes how such data will be sent over the ROS 1 bridge.
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

Some users would like to send text data in languages that cannot be represented by ASCII characters.
Currently ROS 1 only supports ASCII data in the string field but [allows users to populate it with UTF-8](http://wiki.ros.org/msg).

Note that topic names cannot use multi-byte characters as they are disallowed by the DDS specification.
See the [topic and service name](/articles/topic_and_service_names.html) design document for more information.

The following links have more information about multi-byte characters and the history of character encodings.

* [http://kunststube.net/encoding/](http://kunststube.net/encoding/)
* [http://stackoverflow.com/questions/4588302/why-isnt-wchar-t-widely-used-in-code-for-linux-related-platforms](http://stackoverflow.com/questions/4588302/why-isnt-wchar-t-widely-used-in-code-for-linux-related-platforms)
* [http://www.diveintopython3.net/strings.html](http://www.diveintopython3.net/strings.html)
* [http://stackoverflow.com/questions/402283/stdwstring-vs-stdstring](http://stackoverflow.com/questions/402283/stdwstring-vs-stdstring)
* [https://utf8everywhere.org/](http://utf8everywhere.org/)


## Unicode Characters in Strings
Two goals for ROS 2 strings are to be compatible with ROS 1 strings, and compatible with the DDS wire format.
ROS 1 says string fields are to contain ASCII encoded data, but allows UTF-8.
DDS-XTYPES mandates UTF-8 be used as the encoding of IDL type `string`.
To be compatibile with both, in ROS 2 the content of a `string` is expected to be UTF-8.

## Wide Strings
ROS 2 messages will have a new [primitive field type](/articles/interface_definition.html) `wstring`.
This purpose is to allow ROS 2 nodes to comminicate with non-ROS DDS entities using an IDL containing a `wstring` field.
The encoding of data in this type should be UTF-16 to match DDS-XTYPES 1.2.
Since both UTF-8 and UTF-16 can encode the same code points, new ROS 2 messages should prefer `string` over `wstring`.

## Encodings are Required but not Guaranteed to be Enforced
`string` and `wstring` are required to be UTF-8 and UTF-16, but the requirement may not be enforced.
Since ROS 2 is targeting resource constrained systems, it is left to the rmw implementation to choose whether to enforce the encoding.
Further, since many users will write code to check that a string contains valid data, checking again in lower layers may not be necessary in some cases.

If a `string` or `wstring` field is populated with the wrong encoding then the behavior is undefined.
It is possible the rmw implementation may allow invalid strings to be passed through to subscribers.
Each subscriber is responsible for detecting invalid strings and deciding how to handle them.
For example, subscribers like `ros2 topic echo` may echo the bytes in hexadecimal.

The IDL specification forbids `string` from containing `NULL` values.
To be compatible, a ROS message `string` field must not contain zero bytes, and a `wstring` field must not contain zero words.
This restriction will be enforced.

## Unicode Strings Across ROS 1 Bridge
Since ROS 1 and 2 both allow `string` to be UTF-8, the ROS 1 bridge will pass values unmodified between them.
If a message with a string field fails to serialize because the content is not legal UTF-8 then the default behavior will be to drop the entire message.
Other strategies like replacing invalid bytes could unintentionally change the meaning, so they will be opt-in if available at all.

If a ROS 2 message has a field of type `wstring` then the bridge will attempt to convert it from UTF-16 to UTF-8.
The resulting UTF-8 encoded string will be published as a `string` type.
If the conversion fails then the bridge will by default not publish the message.

## Size of a Wide String

Both UTF-8 and UTF-16 are variable width encodings.
To minimize the amount of memory used, the `string` and `wstring` types are to be stored in client libraries according to the smallest possible code point.
This means `string` must be specified as a sequence of bytes, and `wstring` is to be specified as a sequence of words.

Some DDS implementations currently use 32bit types to store wide strings values.
This may be due to DDS-XTYPES 1.1 section 7.3.1.5 specifying `wchar` as a 32bit value.
However this changes in DDS-XTYPES 1.2 section 7.3.1.4 to be a 16bit value.
It is expected that most DDS implementations will switch to 16bit character storage in the future.
ROS 2 will aim to be compatible with DDS-XTYPES 1.2 and use 16bit storage for wide characters.
Generated code for ROS 2 messages will automatically handle the conversion when a message is serialized or deserialized.

### Bounded wide strings

Message definitions may restrict the maximum size of a string.
These are referred to as bounded strings.
Their purpose is to restrict the amount of memory used, so the bounds must be specified as units of memory.
If a `string` field is bounded then the size is given in bytes.
Similarly the size of a bounded `wstring` is to be specified in words.
It is the responsibility of whoever populates a bounded `string` or `wstring` to make sure it contains whole code points only.
Partial code points are indistinuguishable from invalid code points, so a bounded string whose last code point is incomplete is not guaranteed to be published.

## Runtime impact of wide string

Dealing with wide strings puts more strain on the software of a system, both in terms of speed and code size.
UTF-8 and UTF-16 are both variable width encodings, meaning a code point can take 1 to 4 bytes depending on the encoding.
It may take multiple code points to represent a single user perceived character.
One of the goals of ROS 2 is to support microcontrollers that are constrained by both code size and processor speed.
Some wide string operations like splitting a string on a user perceived character may not be possible on these devices.

However, whole string equality checking is the same whether using wide strings or not.
Further splitting a UTF-8 string on an ASCII character is identical to splitting an ASCII character on an ASCII string.
If code on a microcontroller must do string manipluation then it could assert that a `string` only contains ASCII data by ceasing to proces a string when it encounters a byte greater than 127.

## What does the API look like to a user of ROS 2?

### Python 3

In Python the `str` type will be used for both strings and wide strings.
Bytes of a known encoding should be converted to a `str` using [bytes.decode](https://docs.python.org/3/library/stdtypes.html#bytes.decode) before being assigned to a field.

**Example**

```python
import sys
from time import sleep

import rclpy
from rclpy.qos import qos_profile_default
from std_msgs.msg import WString


if __name__ == '__main__':
    rclpy.init(sys.argv)

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(WString, 'chatter', qos_profile_default)

    msg = WString()
    msg.data = 'Hello World' + bytes([0x00, 0x00, 0x26, 0x3A]).decode('utf-32-be')
    print('Publishing: "{0}"'.format(msg.data))
    chatter_pub.publish(msg)
    node.destroy_node()
    rclpy.shutdown()
```

### C++

In c++ wsing `wchar_t` has different sizes on different platforms (2 bytes on Windows, 4 bytes on Linux).
Instead ROS 2 will use `char16_t` for characters of wide strings, and `std::u16string` for wide strings themselves.

**Example**

```
#include <codecvt>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/wstring.hpp"

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
  std::wstring_convert<std::codecvt<char16_t, char, std::mbstate_t>, char16_t> convert_to_u16;

  while (rclcpp::ok()) {
    std::u16string hello(u"Hello World: " + convert_to_u16.from_bytes(std::to_string(i++)));
    msg->data = hello;
    std::cout << "Publishing: '" << msg->data.cpp_str() << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
```
