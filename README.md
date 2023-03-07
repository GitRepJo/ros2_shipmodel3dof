Jonas Mahler 03.2023

# Ros2 Cpp test example

This ros2 package is an extended 3 Degree of Freedom model of
a ship after Nomoto. It is most commonly used to
design a autopilot for course correction e.g. a
sliding mode controller.

For more information look into

    K.Nomoto 1956 " On the steering qualities of ships"
    in International shipbuilding progress

    Muhammad Ejaz and Mou Chen 2017 " Sliding mode control design of a ship
    steering autopilot with input saturation" in International Journal of Advanced
    Robotic Systems

The program has three "layers". The first one is the definition of the nomshipoto equation and a suitable observer for odeint integrator. The second "layer" is the object in which the integration function is called. This class also provides functions to compute a position value of the heading and a velocity. The third "layer" provides interfacing to the ros2 system and listens for velocity and rudderangle. This will be done repeatedly in an infinite loop.

The program my also serve as a template for ship
models with more sophisticated equations.

Odeint is used for integration, which is
included in boost. The differential equation
as well as the observer to the function has to
be passed to the odeint integrator function.

Odeint does not allow member functions to be
passed to the integrator function. Thus the ode
and the observer are functors. They can be found
in nomoto_ode.hpp and odeint_obs.hpp respectively.     

# Build the ros2 package

Use Ros2 Foxy, Ubuntu 20.04 and the custom package ella_interfaces for steering information.
Cmake and colcon is used to create the build files.  

By building this code, catch2 is downloaded     
and installed in the build folder by cmake.    
Therefore the first build may take some time.          

This will also build an executable with the tests     
that can be run by an ros2 launch test (see more in Test the code).         

Clone the code and run colcon build in you ros2 workspace. For example

```
cd /home/$USER/dev_ws && colcon build --packages-select shipmodel3dof
```

# Run the node 

In your ros2 sourced terminal session call as described below. This will call the program with a predefined location of the config file. Place the config file at the according location in the share diectory of the package if it is not found.
    
``` 
ros2 run shipmodel3dof shipmodel3dof 
``` 

To customize parameters call the program with a launch file that is supplied with the package

From your ros2 development directory call e.g.:    

```  
cd /home/$USER/dev_ws &&   
ros2 launch shipmodel3dof shipmodel3dof_launch.py

```  

Mind that folders that are called "launch" inside your main
work directory leads to a KeyError when executing `ros2 launch ..`.     

# Test the code 
There are two different tests.
The first will test the equation and the second the ros2 wrapper around it.
The test executable "tests" to test the equation is compiled in the build    
process. Cmake will download the catch2 code from         
official Git repository.     

For testing, figure 3 of the Nomoto 1956 publication is      
used (see top of readme). The figure shows the turnrate     
after 100 seconds of different K and T. The top and bottom    
line is used for testing. However, there are two limitations:             

- An accurate value can not be obtained from the figure,    
instead an estimate is used for the turn rate at 100 seconds.    

- Also, only a constant value of 15 degree is used as an rudder angle.        
In the figure, an ascending ruddder angle from 0 to 15 is used.

You can run the test by executing the test executable
```
cd /home/$USER/dev_ws && 
colcon test --packages-select nomoto --event-handlers console_direct+
```
# VS Code

If the Development Environment VS Code is used, build and test tasks can be called directly from within VSCode
Open the package in VS Code

```
cd /home/$USER/dev_ws/shipmodel3dof

code .
```
With Shift+Control+B the package can be build.

With Shift+Control+P -> Task: Run Test Task -> test tests can be run.

By pressing F5, the program can be debugged with for example breakpoints.    
Mind that you have to change the path to the executable and to your workspace according to your system.