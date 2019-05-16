---
layout: default
title: ROS 2 Security Testing
permalink: articles/ros2_sectest.html
abstract:
  This document describes a proposed package for testing
  security concerns specific to ROS 2.
author: >
  [Thomas Moulard](https://github.com/thomas-moulard),
  [Ryan Newell](https://github.com/ryanewel)

published: true
categories: Security
---

{:toc}

{::options parse_block_html="true" /}
<style>
table {
    table-layout: fixed;
    width: 100%;
}

th {
    width: 11em;
}
</style>

# {{ page.title }}

## Intent Of This Document
The intent of this document is to:

1. Describe the design of a new ROS 2 packages called `ROS2-SecTest`. This package will gather code which can be used to "attack" ROS 2 applications.
1. Explain the motivation behind proposing this new package and how this work is a follow-up on the ROS 2 threat model.

We are contributing this document to the community to get buy-in from the community, raise visibility and hopefully create a single place for ROS 2 users to collaborate.

## Objective of ROS2-SecTest

We have previously published a document describing a [ROS 2 threat model][threat_model]. This threat model is composed of attacks which are, as of today, theoretical. To continue raising the bar on ROS 2 security, we propose to:

1. Attempt to implement the attacks described in the document,
2. Propose and implement mitigations for the successful attacks.

The order in which threats will be considered will be determined by the threat vulnerability assessment score (DREAD) as well as how hard / how much time would be required to implement the attack. We will target “low-hanging fruits” which can lead to critical vulnerabilities.

## Vulnerability Evaluation Process

Assessing and mitigating a vulnerability is a 4-step process. Its output is an attack proof-of-concept, a mitigation and a report (see appendix).

1. **Attack PoC** - a proof of concept for the attack is implemented and contributed back to a Git repository listing all potential attacks.
2. **Attack Validation** - the attack implemented in 1. is used to disrupt, take control or attack in any way a robot. The target platform is unimportant. For commodity reasons, we suggest to use TurtleBot3 as a reference platform for this step. Note: Not all attacks require validation on a robot. A conscious effort should be made during this step to avoid over-relying on any particular robot specific characteristic. If this is the case, those assumptions should be made explicit in the vulnerability report. This attack validation needs to be easily reproducible by the community, but does not need to be automated (i.e. it is OK to ask the user to check manually whether the robot is moving differently due to the attack for instance).
3. **Mitigation PoC** - the mitigation, from a practical standpoint, means an improvement (PR) contributed back to any ROS 2 package.
4. **Mitigation Validation** - Once the PR in 3. is merged, the step 2. should be repeated. At this point, the attack code should **not** be able to disrupt or impact the robot in any way. if the mitigation is partial (requires particular setup or configuration or does not cover all cases), this should be explicitly called out in the vulnerability report.

## Vulnerability Evaluation Process Scope

### ROS 2 specific VS Generic Vulnerabilities

Unlike the ALIAS Robotic Vulnerability Disclosure Program [RVDP][RVDP], this security assessment program focuses on vulnerabilities *directly related to ROS 2*.

For instance, the following issues are in scope:

* Leaking ROS 2 topic data,
* Triggering ROS 2 services or actions without being unauthorized,
* Abusing node API to disrupt the robot behavior.

However, vulnerabilities unrelated to ROS 2 are not in scope:

* Checking that the robot SSH server cannot be accessed using “easy to guess” credentials such as user = root, password = admin for instance.
* Validating that the robot NTP server or kernel is up-to-date and is not vulnerable to attacks.


The rationale is that any arbitrary software package may be bundled with a particular robot. Trying to evaluate all possible vulnerabilities can be done using a generic vulnerability scanner or framework. We should not try to re-invent generic tools already developed by the security community.

On the opposite, we need to address ROS 2 specific concerns as limited efforts have been done to document and mitigate those issues.

There may be edge cases. Those should be discussed on a case-by-case basis.

### Local VS Remote vulnerabilities

Both local and remote vulnerabilities *are in scope*.

A *remote* vulnerability is defined as a vulnerability which can be exploited from any remote computer which can communicate with the ROS 2 application running on the robot (usually, any host which can communicate with the robot using DDS).

A *local* vulnerability is defined as a vulnerability which can only be exploited from a robot computer.

While remote vulnerabilities are more critical than local vulnerabilities, we make the assumption that, in some cases, attacker will have access to a shell on the robot and that ROS 2 need to cope with those situations in the best way possible.

### Relationship to SROS

Attacks mitigated by SROS *are in-scope*.

SROS helps to prevent some attacks (unauthorized actor should not be able to read or write topic data. However, SROS may be disabled either accidentally, by an attacker or due to technical limitations (SROS overhead is unacceptable, etc.).

As a consequence, we make the assumption that it is helpful to implement attacks for vulnerabilities which could be mitigated by SROS.


## How to implement and validate an attack

### Overview

An attack PoC is a ROS 2 node which, when launched, can disrupt the activity of a robot in any way, including: recording private information, preventing nodes from behaving correctly, corrupting information, etc.
![RunnerNode](/img/ros2_sectest/runner_node.png)

Those ROS 2 nodes are loaded and run through the Runner class. This runner class is in charge of spinning (`rclcpp::spin()`) and is occupying the main thread.

#### Example: CPU attack & Mitigation

For instance, a CPU abuse attack is a ROS 2 node creating one or more threads which consume an excessive amount of CPU.

An attack PoC is simply a ROS 2 node using a lot of CPU (local attack).

The validation step can be based on the TB3. We can use simple demo app making the robot run in circle.
Without the attack node: the robot behaves properly.
With the attack node running on the robot: the robot does not follow the expected trajectory.

A possible mitigation for this attack is to extend roslaunch to run nodes as other users.
For instance, we could create N user accounts on the robot and run each node (including the attack node) using a different user account. With the appropriate cgroups settings, we should be able to prevent having one node consuming all robot resources.

Once the mitigation is implemented, the validation step should be executed again. This time, the robot should run in circle, even if the attack node is running.

_Note_: it is OK to assume that the attack node will end up running as a non-privileged account. If all ROS nodes are using non-privileged account and the “malicious node” is injected in the robot by abusing / attacking an already existing node, the attacked node and malicious node will run as the same non-privileged account. Therefore, the assumption we're making are reasonable. It is OK to highlight in this mitigation that if the malicious node can run as a non-privileged account, this mitigation won't work.

## ROS 2 Attack Framework Package Design Overview

### Overview

This package aims at:

1. factoring out all the boilerplate code required to build attacks,
2. provide a method to implement attacks so that all attacks end up implemented similarly,

### Architecture

Each attack will be implemented on top of the [composition API][composition] and [managed node API][composition]. This offload from the attack code the following features:

* Attack start/stop
* Running N attacks sequentially or all together.


As described in the managed node design doc, managed nodes have multiple states:

* *Unconfigured* - do nothing
* *Inactive* - do nothing
* *Active* - attack is _running_
* *Finalized* - do nothing

The rationale is that we don't actually want to disrupt the robot more than necessary. Having a clear API to start/stop the attack is important to keep the robot usable after an attack. Attacks should do their best effort to “revert” their impact when the attack stops. This step is not strictly necessary but can helpful to debug mitigations for instance.

### Testing Plan

For obvious reasons, running the exploit through tests is dangerous. To ensure that the code is working properly, a null / no-op attack will be implemented.

### Project Ownership & Governance

We would like to ensure this project becomes a part of the ROS 2 code base / ros2 organization on GitHub. This is to ensure a high level of visibility to this project (and avoid duplication of efforts in the community).

## Selected Attacks to be implemented

To start, we chose to implement three “easy” attacks:

1. CPU abuse
2. Memory abuse
3. Disk abuse

Those three attacks need to run on the robot (local attacks). They have been chosen because they are quick to implement and exhibit issues linked to the lack of sandboxing / isolation between ROS 2 nodes.


## Example of Attacks

The threat model is the source of truth, but we could also build the following exploits easily:

* (remote / SROS off) Record data from topics
* (remote / SROS off) Write data from topics
* (remote) Topic DDoS
* (local / SROS on) Time manipulation (node changes the system time, SROS fails because the deployed certificates are not valid for this point of time (e.g. if the SROS certificate indicates valid after May 2015, set the computer clock to 1970 to prevent any communication from happening).





[threat_model]: http://design.ros2.org/articles/ros2_threat_model.html
[RVDP]: https://github.com/aliasrobotics/RVDP
[composition]: https://index.ros.org/doc/ros2/Tutorials/Composition/
[node_lifecycle]: http://design.ros2.org/articles/node_lifecycle.html
