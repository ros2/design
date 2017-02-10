### Remapping Names
#### ROS 1
In ROS 1 [Names](http://wiki.ros.org/Names) can be remapped to other names by passing [remapping arguments](http://wiki.ros.org/Remapping%20Arguments) via the command line.
`roscpp` additionally allows remaps to be passed via Init as strings in STL containers.
A ROS1 remap consists of two names: one that should be replaced with another.

In ROS 1 remapping works on **Fully Qualified Names** (FQN).
When a name is to be checked for remappings, it is first expanded to a FQN.
Both parts of the remapping rule are expanded to a FQN.
Then a name is changed to the remap's second name only if it exactly matches the first name.

#### ROS 2
In ROS2 remapping will be more powerful.
See the requirements section below for examples.
There are more types of remappings that will be supported:

- Namespace replacement

 - Replace all or part of a namespace with another namespace
 
- Basename replacement

 - Replace all of the basename with another basename
 
- Arbitrary substring replacement

 - Replace any substring with another string
 
- Exact replacement (This is ROS 1)

 - Replace a name with another name exactly

ROS 2 may support remapping rules being applied either before or after the names are expanded to FQN.
ROS 1 only supports the latter.

#### Pre-FQN expansion remapping
Say the source code for a node use two names `cat` and `/ns/cat`.
If the node runs in namespace `ns`, the FQN of both topics will be identical: `/ns/cat`.
No rule could remap these names individually once they have been expanded.
A rule applied Pre-FQN expansion could remap just one of them.

#### Requirements for remapping capabilities
##### 1. Namespace Replacement
These are rules that only apply to the namespace part of the name.

**A user can remap the first namespace token without replacing all namespace tokens with the same name.**

*Example*
Node uses names `/big/cat`, `/big/red/dog`, `/red/big/cat`.
It must be possible to make a rule to that replaces "big" with "small" such that the final names are `/small/cat`, `/small/red/dog`, `/red/big/cat`

**A user can replace all matching namespace tokens without changing the basename.**

*Example*
Node uses names `/big/cat`, `/red/big/dog`, `/dog/big`.
It must be possible to make a rule to that replaces "big" with "small" such that the final names are `/small/cat`, `/red/small/dog`, `/dog/big`

**A user can replace multiple matching namespace tokens only if whole namespaces are matched.**

*Example*
Node uses names `/big/red/dog`, `/very_big/red/dog`.
It must be possible to make a rule to that replaces "big/red" with "small" such that the final names are `/small/dog`, `/very_big/red/dog`

##### 2. Basename Replacement
**A user can replace a full basename token with another basename token**

*Example*
Node uses names `/big/cat`, `/big/cat/paws`.
It must be possible to make a rule to that replaces "cat" with "dog" such that the final names are `/big/dog`, `/big/cat/paws`

##### 3. Exact Replacement
**A user can replace a full name with another name**

*Example*
Node uses names `/big/cat`, `/big/cat/paws`.
It must be possible to make a rule to that replaces "/big/cat" with "/bear" such that the final names are `/bear`, `/big/cat/paws`

##### 4. Arbitrary Substring Replacement
**A user can replace any substring with another string**

*Examples*
Node uses names `/big/red/dog`, `/very_big/red/dog`.
It must be possible to make a rule to that replaces "big/red" with "small" such that the final names are `/small/dog`, `/very_small/dog`

Node uses names `/big/dog`, `/dog/big`.
It must be possible to make a rule to that replaces "dog" with "cat" such that the final names are `/big/cat`, `/cat/big`

Node uses names `/big/red/dog`.
It must be possible to make a rule to that replaces "/" with "_" such that the final name is `/_big_red_dog`

#### Static and Dynamic Remapping
Static remapping is the ability to provide a node with remap rules prior to launching it.
A node stores this remapping rules for the duration of it's life.
All names have remapping rules applied before they are used.
This means the DDS interface, and maybe all of `rmw`, is ignorant of remapping.

Dynamic remapping is the ability to remap a name after it has already been used.
For a neat user experience, Dynamic remapping requires rules to be changed.
A user should be able to remap the name they see rather than the original name that was used.
This suggests remapping rules have an order in which they should be applied (chronological?).

#### Remapping syntax
In ROS 1 remapping is done primarily through command line arguments, commonly via roslaunch.
It may not be possible to use the same syntax for ROS 2 because multiple nodes can be inside of the same process, and it is desired that remap rules are node specific and not process specific.

