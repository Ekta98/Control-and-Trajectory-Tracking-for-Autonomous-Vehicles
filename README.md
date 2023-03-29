# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Project Instructions

In the previous project you built a path planner for the autonomous vehicle. Now you will build the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

In the directory [/pid_controller](https://github.com/udacity/nd013-c6-control-starter/tree/master/project/pid_controller)  you will find the files [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.cpp)  and [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.h). This is where you will code your pid controller.
The function pid is called in [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp).

### Step 1: Build the PID controller object
Complete the TODO in the [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.h) and [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.cpp).

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.

  We can see that the car did not move in the following image:
  ![image](https://user-images.githubusercontent.com/28135189/228407278-f3c7a561-c5da-4c4f-a74b-531a50e01b74.png)

### Step 2: PID controller for throttle:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp), complete the TODO (step 2) to compute the error for the throttle pid. The error is the speed difference between the actual speed and the desired speed.

Useful variables:
- The last point of **v_points** vector contains the velocity computed by the path planner.
- **velocity** contains the actual velocity.
- The output of the controller should be inside [-1, 1].

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 3: PID controller for steer:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp), complete the TODO (step 3) to compute the error for the steer pid. The error is the angle difference between the actual steer and the desired steer to reach the planned position.

Useful variables:
- The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
- **yaw** gives the actual rotational angle of the car.
- The output of the controller should be inside [-1.2, 1.2].
- If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 4: Evaluate the PID efficiency
The values of the error and the pid command are saved in thottle_data.txt and steer_data.txt.
Plot the saved values using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You might need to install a few additional python modules: 

```
pip3 install pandas
pip3 install matplotlib
```

Answer the following questions:
- Add the plots to your report and explain them (describe what you see)
     ![image](https://user-images.githubusercontent.com/28135189/228315598-1bb4b3da-4d0f-46e3-9d73-366b14feb9c3.png)
The above figure shows the steering error and steering output.The Error Steering is low with few disturbances in some parts. The error becomes large when the vehicle was avoiding a car in front of it and had to change its path and take a right turn.
          
  ![image](https://user-images.githubusercontent.com/28135189/228316460-badc513a-71dd-4663-b618-8c834369f955.png)
For most part there is not much oscillation in throttle values, however there is some osciallation in error throttle towards rge end which will not provide smooth experience requires further tuning.
       
                
- What is the effect of the PID according to the plots, how each part of the PID affects the control command?     
  The PID controller doesn't overshoot and minimizes the error in a short range of time. PID controller can minimize the mistake between expect and actual speed / yaw angle of the vehicle by producing control output. Here I have adjusted the PID parameters to keep the drive smooth.      
  In PID P  is a proportional controller that provides an output proportional to the current error.      
  I is an integral controller that helps to reduce time to minimize the cross track error.            
  D is a derivative controller for ease of the error rate to reduce the overshooting and improves the stability of the system by compensation increasing the derivative, gain increases the speed of the response.              
  
                                        
- How would you design a way to automatically tune the PID parameters?
   Here I have tuned the PID parameters by trials and errors.     
   We can Implement the twiddle algorithm to optimize set of parameters which was covered in the course. By offering initial values for the PID parameters and probing intervals we can run the PID controller and analyze the control output and tweaking the parameters up and down by examining the outcome, calculates the new quality marker and replaces the parameters only if the result is better.
   
 - PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?      
    
    Pros:
    1. It is a conceptually simple controller that can be applied to many different problems without changes. Creating an initial version of the controller for a new problem is easy, expert knowledge is required about the underlying system, and the parameters to be tuned are general and easy to understand.      
    2. PID controller can run in real-time making it idealized for self-driving cars no need to understand the system, adapted to a complex system, computationally more efficient.     
    
    
    Cons:
    1. Uncertainty of the controller.       
    2. Difficult interpretation of the controller’s behaviour, cannot predict how the system will react to an unknown situation.      
    3. Tuning or learning the controller using data can be slow.        
    
    


### Tips:

- When you wil be testing your c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.
- When you will be tuning the PID parameters, try between those values:

