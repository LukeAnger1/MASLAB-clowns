# ROS2 FizzBuzz Tutorial

## Overview
In this tutorial you will create a version of the classic FizzBuzz programming exercise using ROS2 and Python 3. 

This tutorial is meant to be interactive, so the code on this page will not work by itself. You will have to make some changes and complete the parts labeled `TODO`. If you run into problems or get stuck, a completed version of the code is available at https://github.com/MASLAB/ROS2_FizzBuzz. If you find errors in this document you can also submit bug reports on that repository.

### You will
1. Create a ROS2 workspace
1. Create a package in your workspace
1. Make a Number Publisher Node
1. Create your own ROS2 message type
1. Make a FizzBuzz Node
1. Create a Launch File to run both of these nodes
1. Examine the node structure with rqt

### Prerequisites
If you haven't already, you will need to install ROS2. Here is a link for instructions on how to install ROS2: https://github.com/MASLAB/ros2-setup.

To use ROS2 make sure that your ROS2 installation's setup file is sourced as part of your terminal's pre-run script. This is already set up if you used the automatic setup procedure. Or you can manually `source` the setup file in your current terminal.

```shell
source /opt/ros/jazzy/setup.bash
```

> [!TIP]
> To copy/paste in Ubuntu terminal, use `Ctrl` + `Shift` + `C`/`V`

You can also permanently add this line to your bash profile such that it is ran everytime you open your terminal using this command:
``` shell
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## Creating a Workspace
To start working with ROS2 you will need a workspace. To create one you can run the following command. It will make a workspace in your home (`~`) directory called `ros2_ws` and a `src` directory inside the workspace where your packages will go. For more information on setting up a workspace see this tutorial: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html.

``` shell
mkdir -p ~/ros2_ws/src
```

## Creating a Package
For more information regarding ROS2 packages, see this tutorial: https://docs.ros.org/en/jazzy/Tutorials/Creating-Your-First-ROS2-Package.html.  

### Making the `fizzbuzz` Package
ROS2 supports packages written in Python, c++, or combination of both. More information about different package setup is available in the tutorial link above. In this tutorial, you are going to make a Python package.

The first step is to create a new package in your workspace. Navigate to the src directory in your `ros2_ws` workspace and create a Python ROS package named `fizzbuzz`.

``` shell
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python fizzbuzz
```

Now you will have a directory named `fizzbuzz` inside your `src` directory that contains the following files.

```shell
$ tree ~/ros2_ws/src/fizzbuzz
.
├── fizzbuzz
│   └── __init__.py
├── package.xml
├── resource
│   └── fizzbuzz
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

3 directories, 8 files

```

### Compiling the `fizzbuzz` Package
Now that we have a package, we can compile it to see if we created the package and workspace correctly. Navigate to the top of your workspace and run `colcon build`.

```shell
cd ~/ros2_ws
colcon build
```

If everything worked the output from `colcon build` will look something like this below and you will have 3 new directories in the root of your workspace named `build`, `log`, and `install`.

```shell
Starting >>> fizzbuzz
Finished <<< fizzbuzz [2.59s]          

Summary: 1 package finished [2.98s]
```

> [!IMPORTANT]
> Make sure you run `colcon build` in the root of your ROS workspace, **NOT** in `src` for any other subfolder. If you accidentally do so, simply delete the `build`, `log`, and `install` folders generated in `src` or any other subfolder.

## Number Publisher Node
For the following parts, using a code editor such as [VSCode](https://code.visualstudio.com/) is recommended. VSCode can be downloaded at: https://code.visualstudio.com/Download or in Ubuntu App Center. Make sure to install the Python plugin to enable better syntax hightlighting and code hints. Open the `~/ros2_ws` folder in VSCode (`File` > `Open Folder...`) to edit files and run terminals (`Terminal` > `New Terminal` or `Ctrl` + `` ` ``) directly in VSCode.

> [!TIP]
> For Macbooks with M processors running Ubuntu in Virtualbox, please follow these instructions to install VSCode: https://creatronix.de/how-to-install-vs-code-on-ubuntu-for-arm64/

Now that we have a package, it is time to create a node that publishes numbers that we can use to play the FizzBuzz game. More information on creating ROS2 nodes in python can be found here: https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html.

### Starting off with a Minimal Publisher
#### Code
The number publisher node will live in the `fizzbuzz` folder inside the `fizzbuzz` package we made in the last section. Now we will start creating the number publishing node called `number_publisher_node.py`. Here is a minimal publisher node template from the ROS2 python node tutorial that we can use as a starting point. Create a new file at the path `~/ros2_ws/src/fizzbuzz/fizzbuzz/number_publisher_node.py` and paste in the following code (Right click on the `/src/fizzbuzz/fizzbuzz` folder in VSCode and select `New File...`).

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Explanation
The python file can be broken down into four parts.
1. The imports at the top of the file
1. The node class
1. The main function
1. And the if statement at the bottom

##### Imports
The two imports here are `rclpy` and the `String` message type from the `std_msgs` ROS2 package.

##### The `MinimalPublisher` Class
This class has two functions inside. 

The `__init__` function runs just once when an instance of the class is created. It first prints a log message to say that the node is starting. This is not necessary to the function of the node and just serves as a helpful way to see when the node starts.

Next a publisher is created with `self.create_publisher()`. This function has 3 arguments. The first is the message type.
In this case the node publishes strings so the `String` type is used. The second is the name of the topic that the publisher is publishing to. In this case it is boringly named `topic`. The final argument is the queue size and determines how many messages the publisher keeps around in case the subscribers are not receiving them fast enough. Ten is a good number.

Finally a timer is created with `self.create_timer()`. This timer calls the `self.timer_callback` function with a period between calls of 0.5 seconds.

The `timer_callback` function does the work of publishing the messages. When it is called it creates a string that says "Hello World" and starts counting up with each created message. Then it publishes the string with `self.publisher.publish()` and prints out a log statement with the current counter value.

#### The main function
The main function first initializes the `rclpy` library and then calls `rclpy.spin()` which keeps the nodes running and makes sure all the timers, publishers, and subscribers in a node work properly. When the program is told to stop, the node is destroyed and `rclpy` is shutdown.

#### The If Statement

```python
    if __name__ == '__main__':
        main()
```

If you have never seen this syntax before it is a way to only run code if the python file is run directly and not if it has been imported into another file. In this case it just calls the main function.

### Start Counting!
Right now this minimal publisher node publishes strings instead of numbers. We need to customize the example to our use case.

#### Rename the Node
First let's change all the places where it says `MinimalPublisher` or `minimal_publisher` to `NumberPublisher` or `number_publisher` respectively. Make sure you change the name in the `super().__init__` function call as well. This function lets the rest of ROS know what the node's name is. 

#### Change the Message Type
Right now the message being used is the `std_msgs/String` message. For the number publisher we want to publish integers. Find the appropriate message type to replace with String. The list of all of the messages in `std_msgs` can be found [here](https://github.com/ros2/common_interfaces/blob/jazzy/std_msgs/README.md). Make sure to replace the message type in the import, where the publisher is created, and in the callback. Rename the topic from `topic` to something more descriptive such as `numbers`. 

#### Make it Count!
Now the node is nearly all set up to publish integers instead of strings. All that is
left is to change the callback function to make that happen. Your mission, 
should you choose to accept it, is to publish integers starting at 0 and 
increasing by 1 every message.

### Adding the Node to the Build
To compile our node we need to let the `colcon` build tool know what the node needs and how to run it. To do that we need to add some lines to `package.xml` and `setup.py`.

Add the following lines to `package.xml` between `<license>...` and `<test_depend...>` sections.

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Then in `setup.py`, we need to provide an entry point to run `number_publisher_node.py`. Add the following line within the `console_scripts` brackets of the `entry_points` field:

```python
entry_points={
    'console_scripts': [
        'number_publisher = fizzbuzz.number_publisher_node:main'
    ],
},
```

This tells `colcon` build system that our package has an entry point named `number_publisher` which will invoke the `main` function in our `number_publisher_node` code of our `fizzbuzz` package.

## Testing the Number Publisher Node
Now it is time to see your hard work in action. Lets test the number
publisher node to make sure it is functioning properly.

First you will have to build your package again. To do this source your workspace setup file, which was created in the `install` directory last time we ran `colcon build`, and run `colcon build` from the root of your workspace.

```shell
cd ~/ros2_ws
source install/setup.bash
colcon build
```
> [!IMPORTANT]
> `source install/setup.bash` add your workspace as an overlay to your ROS2 installation such that ROS2 is made aware of your packages. Therefore, new terminals also has to go to your workspace and `source install/setup.bash` before doing anything related to your ROS2 workspace.

> [!TIP]
> `colcon build` has to be ran every time you made changes to your packages.

Step two is to open an additional terminal window. In VSCode, you can create a new terminal or split terminal (`Terminal` > `Split Terminal`) to view new terminal side-by-side. Remember to run `source install/setup.bash` in the new one as well.

Step three is to start our number publisher node in either terminal. For this you can use the `ros2 run [package_name] [entry_point]` command.

```shell
ros2 run fizzbuzz number_publisher
```

Step four is to set up the other terminal window to listen to a topic and print all of the messages it receives to the terminal. This way we can make sure that the node is publishing correctly. The following command does exactly that: `ros2 topic echo [topic_name]`.

```shell
ros2 topic echo numbers
```

Now you should see numbers counting up from the terminal running `ros2 topic echo`. That terminal is subscribed to the messages published by the `number_publisher_node` and prints them out to the screen.

You can use `Ctrl` + `C` to stop the processes in either terminal whenever you are done.

## FizzBuzz Message
### Custom FizzBuzz Message
To play fizzbuzz efficiently, we should create a custom message to describe our fizzbuzz game state. Our custom fizzbuzz message will look like this:

```
string fizzbuzz  # the result of the fizzbuzz function
float32 fizz_ratio  # the ratio of fizz results to non fizz results
float32 buzz_ratio  # the ratio of buzz results to non buzz results
float32 fizzbuzz_ratio  # the ratio of fizzbuzz results to non fizzbuzz results
int32 number_total  # the total number of numbers received.
```

The string `fizzbuzz` will either be `fizz` if the number is divisible by 3, `buzz` if the number is divisible by 5, `fizzbuzz` if the number is divisible by 15, or an empty string if none of the previous cases are true.

### FizzBuzz Custom Interfaces Package
To create a new message type in ROS2, we will have to make a new package for generating messages (and/or services) that other packages can depend on.

#### Create Custom Interfaces Package
We first create a c++ ROS2 package in the `src` folder of our `ros2_ws` workspace with `--build-type ament_cmake`. This is necessary because ROS2 package generator (`rosidl_default_generators`) runs on c++.

```shell
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake fizzbuzz_interfaces
```

#### Add Custom Message
To create our new message type in ROS2, we will have to make a file named `FizzBuzz.msg` in a new folder named `msg` within the `fizzbuzz_interfaces` package.

```shell
mkdir -p ~/ros2_ws/src/fizzbuzz_interfaces/msg
cd ~/ros2_ws/src/fizzbuzz_interfaces/msg
touch FizzBuzz.msg
```

Now the interfaces package structure should look like this.

```shell
tree ~/ros2_ws/src/fizzbuzz_interfaces
.
├── CMakeLists.txt
├── include
│   └── fizzbuzz_interfaces
├── msg
│   └── FizzBuzz.msg
├── package.xml
└── src
```

Now paste the message content above ([Custom FizzBuzz Message](#custom-fizzbuzz-message)) into the new file.

#### Add Custom Message to Build
Now that you have a new message file you need to add it to the build. To do this you will have to edit the file `CMakeLists.txt` as well as `package.xml` in the package.

Add the following to your `CMakeLists.txt` after `find_package(ament_cmake REQUIRED)`.

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FizzBuzz.msg  
) 
```

And these lines to your `package.xml` after `<buildtool_depend>ament_cmake</buildtool_depend>`.

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

#### Build and Test
After you have updated those two files, you will need to run `colcon build` to build the message. It is also good to source in the workspace setup file again whenever a new message or node is created.

```shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```

If everything worked correctly, you should be able to use `ros2 interface show <package>/msg/<message_name>` to see the contents of your new FizzBuzz message.

```shell
ros2 interface show fizzbuzz_interfaces/msg/FizzBuzz
```

## FizzBuzz Node
To implement the fizzbuzz node, we will return to our `fizzbuzz` package.

#### Dependency
To tell our `fizzbuzz` package that we need the custom message from our `fizzbuzz_interfaces` package, we need to add the following line to `package.xml` along other `<exec_depend>` statements.

```xml
<exec_depend>fizzbuzz_interfaces</exec_depend>
```

#### Code
To implement the fizzbuzz node, create a new file named `fizzbuzz_subscriber_node.py` in the same folder as our `number_publisher_node.py`. Here is some code to get you started.

```python
import rclpy
from rclpy.node import Node

# import our new fizzbuzz message type
from fizzbuzz_interfaces.msg import FizzBuzz 

# TODO import the number message used for the numbers topic

class FizzBuzzNode(Node):
    def __init__(self):
        super().__init__('fizzbuzz')
        self.get_logger().info("Starting fizzbuzz node")

        self.total_numbers = 0
        self.total_fizz = 0
        self.total_buzz = 0
        self.total_fizzbuzz = 0

        # create a publisher object to send data
        self.fizzbuzz_pub = self.create_publisher(FizzBuzz, "fizzbuzz_stats", 10)

        # TODO fill in the TOPIC_NAME and MESSAGE_TYPE
        self.number_sub = self.create_subscription(MESSAGE_TYPE, "TOPIC_NAME", self.number_callback, 10)

    def number_callback(self, msg):
        # this function is called whenever a number is received.

        number = msg.data 

        fizzbuzz_str = self.fizzbuzz(number)
        # loginfo to print the string to the terminal
        self.get_logger().info(fizzbuzz_str)

        fizzbuzz_msg = FizzBuzz()
        fizzbuzz_msg.fizzbuzz = fizzbuzz_str
        fizzbuzz_msg.fizz_ratio = 0 # TODO fill in this value
        fizzbuzz_msg.buzz_ratio = 0 # TODO fill in this value
        fizzbuzz_msg.fizzbuzz_ratio = 0 # TODO fill in this value
        fizzbuzz_msg.number_total = 0 # TODO fill in this value

        # publish the message
        self.fizzbuzz_pub.publish(fizzbuzz_msg)

    def fizzbuzz(self, number):
        # TODO complete this function
        # This should return a string equal to:
        #      "fizz" if number divisible my 3
        #      "buzz" if number divisible my 5
        #      "fizzbuzz" if number divisible my 15
        #      an empty string otherwise
        return ""


def main(args=None):
    rclpy.init()
    node = FizzBuzzNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

#### Explanation
At the top of the file we import the FizzBuzz message we just created from our `fizzbuzz_interfaces` package.

The `__init__` function first initializes a couple variables that might be helpful for completing the node. Then, it creates a publisher object that publishes FizzBuzz messages to a topic named `fizzbuzz_stats`.

Next it subscribes to a topic. It is your job to fill in both the topic name and the message type. When a new message is recived, it calls the `number_callback` function. In this way, it is similar to how the timer called the `timer_callback` function.

The `number_callback` function takes in the message received by the subscriber and publishes a FizzBuzz message with the correct information. To accomplish this it uses a function called `self.fizzbuzz` that returns a string containing the correct response for that number.

#### Completing the Node
If you complete the code in the places marked by `TODO`, you will have a functioning FizzBuzz Node.

Make sure you add it to the build with a new entry point by adding to `setup.py` as well. Let's call this entry point `fizzbuzz_subscriber`.

```python
entry_points={
    'console_scripts': [
        'number_publisher = fizzbuzz.number_publisher_node:main',
        'fizzbuzz_subscriber = fizzbuzz.fizzbuzz_subscriber_node:main'
    ],
},
```

## Testing the FizzBuzz Node
Let's check to make sure the FizzBuzz node is working correctly. Build the package again from the root of the workspace. Open three terminals and run the following commands.

#### Terminal 1
```shell
cd ~/ros2_ws
colcon build --packages-select fizzbuzz
source install/setup.bash
ros2 run fizzbuzz fizzbuzz_subscriber
```
This rebuild the `fizzbuzz` package and starts up the FizzBuzz node we just made. You should see the result of fizzbuzz printed to the terminal whenever this node receives a message from the numbers topic.

> [!TIP]
> `--packages-select` option specifies which package to build/rebuild instead of building every package in `/src` which may takes long for large ROS projects. The syntax for it is:
> ```shell 
> colcon build --packages-select <name-of-pkg> <name-of-another-pkg>
> ```
> Only 1 terminal needs to build the packages. Once the package is built, other terminals can just `source install/setup.bash`.

#### Terminal 2
```shell
cd ~/ros2_ws
source install/setup.bash
ros2 run fizzbuzz number_publisher
```
This runs the `number_publisher` node like earlier.

#### Terminal 3
```shell
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo fizzbuzz_stats
```
This command displays the messages published to the `fizzbuzz_stats` topic. You should see the result of FizzBuzz as well as the ratio and total count in this terminal after you start the number publisher.

## Making a Launch File
That was a lot of terminals we needed to play FizzBuzz! It would be great if there was a way to start multiple ROS nodes with one command. That is where launch files come in.

Launch files are files that contain instructions to run a group of ROS nodes. We will now make a launch file to start both the number publisher and the fizzbuzz node. Here is an example launch file that runs the `number_publisher`. You can copy the format to make it launch the `fizzbuzz_subscriber` as well. Launch files go in a directory called launch in the package to which they belong. You can name the new launch file `fizzbuzz_launch.py` and put it in a new folder at `src/fizzbuzz/launch`.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    number_publisher_node = Node(
        package='fizzbuzz',
        executable='number_publisher',
        output='screen'
    )

    fizzbuzz_subscriber_node = Node(
        # TODO complete the node description
    )

    return LaunchDescription([number_publisher_node, fizzbuzz_subscriber_node])
```

As you can see, we need to specify the package of the node and the filename of the node in order to properly start the node from a launch file. The output attribute specifies where the messages from the ROS log command get sent. By specifying the value screen they will appear in the terminal we use to start the launch file.

For more information about the format of a ROS2 launch file, see this page: https://docs.ros.org/en/humble/Tutorials/Launch/Launch-Main.html

To add the launch file to the build, add this to `setup.py` imports

```python
import os
from glob import glob
```

And everything from the `launch` folder to `setup.py`'s `datafiles`.

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Include all launch files.
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
],
```

And this to your `package.xml` along other `<exec_depend>` statements.

```xml
<exec_depend>ros2launch</exec_depend>
```

Run `colcon build` again and now if we want to play FizzBuzz again we can simply open two terminals and run the following commands. Make sure to navigate to the workspace folder and source the installation with `cd ~/ros2_ws && source install/setup.bash` in both terminals.

#### Terminal 1
```shell
cd ~/ros2_ws
colcon build --packages-select fizzbuzz
source install/setup.bash
ros2 launch fizzbuzz fizzbuzz_launch.py
```
The `ros2 launch` command will run the launch file for a package.

#### Terminal 2
```shell
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo fizzbuzz_stats
```
The `ros2 topic echo` command will display the fizzbuzz message on the other terminal.

## RQT
RQT is a useful tool to see what nodes are running and how messages are being transfered between them.

Use your launch file to begin a game of FizzBuzz and then open RQT using the following command in a new terminal.

```shell
rqt
```

This command will open up an empty window. Navigate in the top menu to `Plugins > Introspection > Node Graph`. This will show you all of the currently running ROS nodes as circles and the topics they communicate on as arrows. For FizzBuzz this graph is only two nodes with a single topic between them, but RQT is very helpful for visualizing the connections in larger projects. RQT has many other helpful tools for inspecting messages and graphing data that you might want to explore.

![FizzBuzz RQT](https://github.com/MASLAB/ROS2_FizzBuzz/assets/19534715/5c119fc3-9594-4604-8d5d-db8ac761c3bf)

ROS2 has many built in tools for seeing what topics being used (`ros2 topic list`), what nodes are being run (`ros2 node list`), recording and playing back messages (`ros2 bag`), diagnosing problems while ROS2 is running (`ros2 wtf`), and many more.

## Keep Exploring!
This tutorial has just scratched the surface of what ROS2 can do. To learn more check out the ROS2 Wiki here: https://docs.ros.org/en/jazzy/index.html Hopefully you found this tutorial helpful on your journey of learning ROS2.
