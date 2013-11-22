design
======

Design documentation for ROS 2.0 effort

to be captured
==============

From Vincent's French Robotics Conference:

```
Here you go:
- good stuff: docs/tutorials, easy to use/install, good for education
- bad: catkin (need was understood but the main complaint was that it is harder to teach student), lack of multicast, lack of pluggable message transport, contribution to the core were hard/rejected (a pluggable ros_comm was submitted to Dirk but that was a year ago, during the Willow/catki/GitHub move crisis),
- fear of perennity: if I switch to ROS now, will it work in a few years ?
- how does the industry react ? (Mentioned a few companies, ros industrial)
- good: .msg files are well defined and can be reused in other frameworks (e.g. Genom3 http://homepages.laas.fr/mallet/soft/architecture/genom3)
- bad: actionlib is clunky (not sure exactly why) so they reimplemented their own
- Python ROS comm is slower and not robust to packet loss.
- good: relatively plug and play. A student can get started in a few days. Many drivers
- bad: a lot of stuff on the wiki that needs to be deprecated removed
- no high level decision framework (smash but good basically) but that was just to show that it was not A solution to high level robotics. Just something complementary
- need of clarity/ roadmap to see how contributions can happen

So basically the usual :)
```
