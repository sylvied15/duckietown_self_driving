# Template: template-ros

This template provides a boilerplate repository
for developing ROS-based software in Duckietown.

**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).


## How to use it

### 1. Fork this repository

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Build the code
In a terminal, use the following command to build the code :

dts devel build -f


### 4. Run on your duckiebot
In a terminal, use the following command to run the main code (self driving) :

dts devel run -R !ROBOT_NAME -L panel-reading2
