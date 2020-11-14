---
layout: default
title: ROS2 on IBM Cloud Kubernetes
abstract:
  This article describes how to get ROS2 running on IBM Cloud Kubernetes using Docker files. It first gives a brief overview of docker images and how they work locally and then explores IBM Cloud and how the user can push their containers on it. 
  Afterwards, a short description of how the user can use their own custom packages for ROS2 from github on IBM Cloud is provided.
  A walkthrough of how to create a cluster and utilize Kubernetes on IBM Cloud is provided and finally the Docker image is deployed on the cluster to run ROS2.
author: '[Mahmoud Nasr](https://github.com/mm-nasr)'
published: false
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## ROS2 on IBM Cloud

In this tutorial, we show how you can easily integrate and run ROS2 on IBM Cloud with your custom packages.

ROS2 is the new generation of ROS which gives more control over multi-robot formations, more secure connections, and an overall more structured robotics environment. With the advancements of cloud computing, cloud robotics are becoming more important in today's age. In this tutorial, we will go through a short introduction on running ROS2 on IBM Cloud. By the end of the tutorial, you will be able to create your own packages in ROS2 and deploy them to the cloud using docker files. 

The following instructions assume you're using Linux and have been tested with Ubuntu 18.04 (Bionic Beaver).

### Step 1: Setting up your system
Before we go into how the exact process works, lets first make sure all the required software is properly installed. We'll point you towards the appropriate sources to set up your system and only highlight the details that pertain to our use-case.

#### a) Docker files?

Docker files are a form of containers that can run separate from your system, this way, you can set-up potentially hundreds of different projects without affecting one another. You can even set-up different versions of Linux on one machine, without the need for virtual machine. 
Docker files have an advantage of saving space and only utilizing your system resources when running. 
In addition, dockers are versatile and transferable. They contain all the required pre-requisites to run separately, meaning that you can easily use a docker file for a specific system or service without any cubersome steps!

Excited yet? Let's start off by installing docker to your system by following the following [link](https://docs.docker.com/get-docker/).
From the tutorial, you should have done some sanity checks to make sure docker is properly set-up. Just in case, however, let's run the following command once again that uses the hello-world docker image:
```
$ sudo docker run hello-world
```

You should obtain the following output:
```
Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

#### b) ROS2 Image

ROS [announced](https://discourse.ros.org/t/announcing-official-docker-images-for-ros2/7381/2) image containers for several ROS distributions in January 2019.
More detailed instructions on the use of ROS2 docker images can be found [here](https://hub.docker.com/_/ros/).

Let's skip through that and get to real-deal right away; creating a local ROS2 docker. We'll create our own Dockerfile (instead of using a ready Image) since we'll need this method for deployment on IBM Cloud. First, we create a new directory which will hold our Dockerfile and any other files we need later on and navigate to it. 
Using your favorite $EDITOR of choice, open a new file named *Dockerfile* (make sure the file naming is correct):

```
$ mkdir ~/ros2_docker

$ cd ~/ros2_docker

$ $EDITOR Dockerfile
```

Insert the following in the *Dockerfile*, and save it (also found [here](https://github.com/mm-nasr/ros2_ibmcloud/blob/main/dockers/ros2_basic/Dockerfile)):

```
FROM ros:foxy

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/* && mkdir /ros2_home

WORKDIR /ros2_home

# launch ros package
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
```

* __FROM__: creates a layer from the ros:foxy Docker image
* __RUN__: builds your container by installing vim into it and creating a directory called /ros2_home
* __WORKDIR__: informs the container where the working directory should be for it

Of course, you are free to change the ROS distribution (_foxy_ is used here) or change the directory name.
The above docker file sets up ROS-foxy and installs the demo nodes for C++ and Python. Then it launches a file which runs a talker and a listener node. 
We will see it in action in just a few, but they act very similar to the publisher-subscriber example found in the [ROS wiki](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

Now, we are ready to build the docker image to run ROS2 in it (yes, it is THAT easy!). 

__Note__: if you have errors due to insufficient privileges or _permission denied_, try running the command with _sudo_ privileges:

```
$ docker build .

# You will see a bunch of lines that execute the docker file instructions followed by:
Successfully built 0dc6ce7cb487
```

_0dc6ce7cb487_ will most probably be different for you, so keep note of it and copy it somewhere for reference. 
You can always go back and check the docker images you have on your system using:

```
$ sudo docker ps -as
```

Now, run the docker file using:

```
$ docker run -it 0dc6ce7cb487
[INFO] [launch]: All log files can be found below /root/.ros/log/2020-10-28-02-41-45-177546-0b5d9ed123be-1
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [28]
[INFO] [listener-2]: process started with pid [30]
[talker-1] [INFO] [1603852907.249886590] [talker]: Publishing: 'Hello World: 1'
[listener-2] [INFO] [1603852907.250964490] [listener]: I heard: [Hello World: 1]
[talker-1] [INFO] [1603852908.249786312] [talker]: Publishing: 'Hello World: 2'
[listener-2] [INFO] [1603852908.250453386] [listener]: I heard: [Hello World: 2]
[talker-1] [INFO] [1603852909.249882257] [talker]: Publishing: 'Hello World: 3'
[listener-2] [INFO] [1603852909.250536089] [listener]: I heard: [Hello World: 3]
[talker-1] [INFO] [1603852910.249845718] [talker]: Publishing: 'Hello World: 4'
[listener-2] [INFO] [1603852910.250509355] [listener]: I heard: [Hello World: 4]
[talker-1] [INFO] [1603852911.249506058] [talker]: Publishing: 'Hello World: 5'
[listener-2] [INFO] [1603852911.250152324] [listener]: I heard: [Hello World: 5]
[talker-1] [INFO] [1603852912.249556670] [talker]: Publishing: 'Hello World: 6'
[listener-2] [INFO] [1603852912.250212678] [listener]: I heard: [Hello World: 6]
```

If it works correctly, you should see something similar to what is shown above. 
As can be seen, there are two ROS nodes (a publisher and a subscriber) running and their output is provided to us through ROS INFO.

## Step 2: Running the image on IBM Cloud

The following steps assume you have an IBM cloud account and have ibmcloud CLI installed. 
If not, please check this [link](https://cloud.ibm.com/docs/cli/reference/ibmcloud/download_cli.html#install_use) out to get that done first.

We also need to make sure that the CLI plug-in for the IBM Cloud Container Registry is installed by running the command 

```
$ ibmcloud plugin install container-registry
```

Afterwards, login to your ibmcloud account through the terminal:

```
$ ibmcloud login --sso
```

From here, let's create a container registry name-space. Make sure you use a unique name that is also descriptive as to what it is. Here, I used _ros2nasr_.


```
$ ibmcloud cr namespace-add ros2nasr
``` 

IBM cloud has a lot of shortcuts that would help us get our container onto the cloud right away. 
The command below builds the container and tags it with the name __ros2foxy__ and the version of __1__. 
Make sure you use the correct registry name you created and you are free to change the container name as you wish. 
The __.__ at the end indicates that the _Dockerfile_ is in the current directory (and it is important), if not, change it to point to the directory containing the Dockerfile.

```
$ ibmcloud cr build --tag registry.bluemix.net/ros2nasr/ros2foxy:1 .
```

You can now make sure that the container has been pushed to the registry you created by running the following command

```
$ ibmcloud cr image-list
Listing images...

REPOSITORY               TAG   DIGEST         NAMESPACE   CREATED         SIZE     SECURITY STATUS
us.icr.io/ros2nasr/ros2foxy   1     031be29301e6   ros2nasr    36 seconds ago   120 MB   No Issues

OK
```

Next, it is important to log-in to your registry to run the docker image. 
Again, if you face a _permission denied_ error, perform the command with sudo previliges. Afterwards, run your docker file as shown below.

```
$ ibmcloud cr login
Logging in to 'registry.ng.bluemix.net'...
Logged in to 'registry.ng.bluemix.net'.
Logging in to 'us.icr.io'...
Logged in to 'us.icr.io'.

OK

$ docker run -v -it registry.ng.bluemix.net/ros2nasr/ros2foxy:1
```

Where _ros2nasr_ is the name of the registry you created and _ros2foxy:1_ is the tag of the docker container and the version as explained previously.

You should now see your docker file running and providing similar output to that you saw when you ran it locally on your machine.



### Step 3: Using Custom ROS2 Packages

So now we have the full pipeline working, from creating the Dockerfile, all the way to deploying it and seeing it work on IBM Cloud. But, what if we want to use a custom set of packages we (or someone else) created?

Well that all has to do with how you set-up your Dockerfile. Lets use the example provided by ROS2 [here](https://hub.docker.com/_/ros/). Create a new directory with a new Dockerfile (or overwrite the existing one) and add the following in it (or download the file [here](https://github.com/mm-nasr/ros2_ibmcloud/blob/main/dockers/git_pkgs_docker/Dockerfile))

```
ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  ros2/demos: \n\
    type: git \n\
    url: https://github.com/ros2/demos.git \n\
    version: ${ROS_DISTRO} \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/ros2/demos/demo_nodes_cpp \
        src/ros2/demos/demo_nodes_py \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        demo_nodes_cpp \
        demo_nodes_py \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
```

Going through the lines shown, we can see how we can add custom packages from github in 4 steps:

1. Create an overlay with custom packages cloned from Github:

```
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  ros2/demos: \n\
    type: git \n\
    url: https://github.com/ros2/demos.git \n\
    version: ${ROS_DISTRO} \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos
```

2. Install package dependencies using rosdep

```
# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/ros2/demos/demo_nodes_cpp \
        src/ros2/demos/demo_nodes_py \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

```

3. Build the packages _you need_

```
# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        demo_nodes_cpp \
        demo_nodes_py \
      --mixin $OVERLAY_MIXINS
```

4. Running the launch file

```
# run launch file
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
```

Likewise, we can change the packages used, install their dependencies, and then run them. 


__Back to IBM Cloud__

With this Dockerfile, we can follow the same steps we did before to deploy it on IBM Cloud. Since we already have our registry created, and we're logged in to IBM Cloud, we directly build our new Dockerfile. Notice how I kept the tag the same but changed the version, this way I can update the docker image created previously. (You are free to create a completely new one if you want)

```
$ ibmcloud cr build --tag registry.bluemix.net/ros2nasr/ros2foxy:2 .
```

Then, make sure you are logged in to the registry and run the new docker image:

```
$ ibmcloud cr login
Logging in to 'registry.ng.bluemix.net'...
Logged in to 'registry.ng.bluemix.net'.
Logging in to 'us.icr.io'...
Logged in to 'us.icr.io'.

OK

$ docker run -v -it registry.ng.bluemix.net/ros2nasr/ros2foxy:2
```

You should see, again, the same output. However, this time we did it through custom packages from github, which allows us to utilize our personally created packages for ROS2 on IBM Cloud.


#### Extra: Deleting Docker Images

As you may find yourself in need of deleting a specific docker image(s) from IBM Cloud, this is how you should go about it!

1. List all the images you have and find all the ones that share the _IMAGE_ name corresponding to _registry.ng.bluemix.net/ros2nasr/ros2foxy:2_ (in my case). Then delete them using their _NAMES_

```
$ docker rm your_docker_NAMES
```

2. Delete the docker image from IBM Cloud using its _IMAGE_ name

```
$ docker rmi registry.ng.bluemix.net/ros2nasr/ros2foxy:2
```


### Step 4: Kubernetes

#### a) Creating the Cluster

Create a cluster using the Console. The instructions are found [here](https://cloud.ibm.com/docs/containers?topic=containers-clusters#clusters_ui). The settings used are detailed below. These are merely suggestions and can be changed if you need to. However, make sure you understand the implications of your choices:

1. Plan: _Standard_

2. Orchestration Service: _Kubernetes v1.18.10_

3. Infrastructure: _Classic_

4. Location:
  
  * Resource group: _Default_

  * Geography: _North America_ (you are free to change this)

  * Availability: _Single zone_ (you are free to change this but make sure you understand the impact of your choices by checking the IBM Cloud documentation.)

  * Worker Zone: _Toronto 01_ (choose the location that is physically closest to you)

5. Worker Pool:

  * Virtual - shared, Ubuntu 18

  * Memory: 16 GB

  * Worker nodes per zone: _1_ 

6. Master service endpoint: _Both private & public endpoints_

7. Resource details (Totally flexible):

  * Cluster name: _mycluster-tor01-rosibm_

  * Tags: _version:1_


After you create your cluster, you will be redirected to a page which details how you can set up the CLI tools and access your cluster. Please follow these instructions (or check the instructions [here](https://github.com/mm-nasr/ros2_ibmcloud/Kubernetes-Cluster-Set-Up.md))and wait for the progress bar to show that the worker nodes you created are ready by indicating _Normal_ next to the cluster name. You can also reach this screen from the IBM Cloud Console inside the Kubernetes. 


#### b) Deploying your Docker Image _Finally!_

1. Create a deployment configuration yaml file named _ros2-deployment.yaml_ using your favorite $EDITOR and insert the following in it:

```
apiVersion: apps/v1
kind: Deployment
metadata:
  name: <deployment>
spec:
  replicas: <number_of_replicas>
  selector:
    matchLabels:
      app: <app_name>
  template:
    metadata:
      labels:
        app: <app_name>
    spec:
      containers:
      - name: <app_name>
        image: <region>.icr.io/<namespace>/<image>:<tag>
```

You should replace the tags shown between _"<" ">"_ as described [here](https://cloud.ibm.com/docs/containers?topic=containers-images#namespace). The file in my case would look something like this:

```
apiVersion: apps/v1
kind: Deployment
metadata:
  name: ros2-deployment
spec:
  replicas: 1
  selector:
    matchLabels:
      app: ros2-ibmcloud
  template:
    metadata:
      labels:
        app: ros2-ibmcloud
    spec:
      containers:
      - name: ros2-ibmcloud
        image: us.icr.io/ros2nasr/ros2foxy:2
```

Deploy the file using the following command

```
$ kubectl apply -f ros2-deployment.yaml
deployment.apps/ros2-deployment created
```

Now your docker image is fully deployed on your cluster!


### Step 5: Using CLI for your Docker Image

1. Navigate to your cluster through the IBM Cloud console Kubernetes.

2. Click on _Kubernetes dashboard_ on the top right corner of the page.

You should now be able to see a full list of all the different parameters of your cluster as well as its CPU and Memory Usage.

3. Navigate to _Pods_ and click on your deployment.

4. On the top right corner, click on _Exec into pod_ 

Now you are inside your docker image! You can source your workspace (if needed) and run ROS2! For example:

```
root@ros2-deployment-xxxxxxxx:/opt/ros/overlay_ws# . install/setup.sh
root@ros2-deployment-xxxxxxxx:/opt/ros/overlay_ws# ros2 launch demo_nodes_cpp talker_listener.launch.py
```

The output is shown [here](https://github.com/mm-nasr/ros2_ibmcloud/tree/main/images/kubernetes_ros2_run.png).

### Final Remarks

At this point, you are capable of creating your own docker image using ROS2 packages on github. It is also possible, with little changes to utilize local ROS2 packages as well. This could be the topic of another article. However, you are encouraged to check out the following [Dockerfile](https://github.com/mm-nasr/ros2_ibmcloud/tree/main/dockers/local_pkgs_docker) which uses a local copy of the demos repository. Similarly, you can use your own local package. 
