
# stretch_ctrl_dev
Experiments in controls

## Getting Started

### Unboxing and Powering Up

Your robot is a standard Stretch 3 minus the arm and head camera. Unbox the robot as described in the getting started guide.

Take some time to review the guide and become familiar with the robot.

https://docs.hello-robot.com/0.3/getting_started/hello_robot/



Once unboxed, plug in the charger. Place the charger in Linear Supply mode. The robot can be left powered on and the charger attached during development. 

Ensure the 5lb weight is attached to the Lift. The power on the robot.

Now that the robot is powered up,  attach a monitor, keyboard, and mouse to the base of the robot. 



### Home and Jog

Home the Lift. The Lift will go all the way up and then partially down. Make sure the workspace is clear. 

`stretch_lift_home.py`

```For use with S T R E T C H (R) from Hello Robot Inc.
---------------------------------------------------------------------

Homing Lift...
Hardstop detected at motor position (rad) 115.08666229248047
Marking Lift position to 1.100000 (m)
Marking Lift position to 0.285000 (m)
Lift homing successful
```

Now you can jog the lift:

`stretch_lift_jog.py`

```stretch_lift_jog.py 
For use with S T R E T C H (R) from Hello Robot Inc.
---------------------------------------------------------------------

--------------
m: menu
u / d : small up down
U / D : large up down
f: stiffness float
s: stiffness soft
h: stiffness hard
1: rate slow
2: rate default
3: rate fast
4: rate max
q: quit

Input?
```



# Explore the Control Framework

### Git Repos

Your Python code will live in the repo 

https://github.com/hello-robot/stretch_ctrl_dev

A local copy of Stretch Body on the branch `feature/controls_dev` is installed in `~/repos/stretch_body_feetech`

```pip3 list | grep stretch_body
hello-robot-stretch-body               0.7.27                               /home/hello-robot/repos/stretch_body_feetech/body
```

You can modify and push to this branch as needed as well.

A local copy of Stretch Firmware is in `~/repos/stretch_samd51_fimware_sandbox`, with branch `feature/controls_dev` checked out. 

You can modify, edit, and branch all repos as needed. Please check-in your code periodically, but note that all repos are public to our community.

Generally you'll be developing Python scripts under `stretch_ctrl_dev` and writing firmware controllers under `stretch_samd51_firmware_sandbox`.



### Writing Code

This tutorial introduces commanding motion. However, this robot doesn't support the robot.py instance. It will only support lift.py

https://docs.hello-robot.com/0.3/python/moving/

There is older documentation which goes into more detail on working with the low-level Stretch Body interfaces. Much of the Lift content will still apply (but may be out of date). The rest of the robot classes and functions can be igonored.

https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/



You'll be mainly working with the following classes shown [here](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_introduction/):

* Lift.py (Stretch Body)
* Stepper.py (Stretch Body)
* hello_stepper.ino (Firmware)

This script provides an example of programming the lift:

`cd ~/repos/stretch_ctrl_dev/demo_scripts`

and run

`./lift_test_motion.py`

Inspect the code. You can play with the feedfoward current (~3.8A max) that balances the test mass. You can adjust the controller stiffness and trajectory speed. 

### Tools

Next, try out our controller tuning gui. You may want to fork this and adapt to your own needs in the future:

`REx_stepper_ctrl_tuning.py --lift --pos_traj`

The `--pos_traj` flag will allow you to command motion through a trapezoidal profile. The `--pos-pid` directly command position with no smoothing/ramp. 



### Firmware Controllers

Firmware is found: `~/repos/stretch_samd51_fimware_sandbox/stretch_samd51_firmware/arduino/hello_stepper/`

This code implements a number of closed loop controllers for the Lift stepper using a hall effect encoder on the back of the motor.

In the firmware `hello_stepper` the main controllers are under `HelloController.cpp`. There's a lot of plumbing code of Python based RPC communication with the stepper, parameter management, etc. The main loop is run in `stepHelloController` at 1Khz. 

This loop ultimately computes a commanded current (`u`) that is sent to the motor driver. The block diagram is [here](https://docs.google.com/presentation/d/196_yxSHKrwC7ov7TnJAuINmrhZmf-r4upt1MDouw7r0/edit?usp=sharing):. 

### Flashing Firmware

Try flashing firmware via Arduino IDE. Run:

`arduino`

then open

`~/repos/stretch_samd51_fimware_sandbox/stretch_samd51_firmware/arduino/hello_stepper/hello_stepper.ino `

then select `hello_stepper` from the Tools/Boards menu,

then figure out which port the Lift is on (eg, tty ACM1) 

`ls -l /dev/hello-motor-lift`

and select that board from Tools/Port (it should be a SAMD51).

Now the Lift will drop when being programmed so ensure the clamp is under the carriage (or no weight is attached.)

When ready, do Sketch/Upload of hello_stepper sketch.

Next, reflash the motor encoder calibration to Flash memory:

`REx_stepper_calibration_YAML_to_flash.py hello-motor-lift`

Finally, you can check that the joint still moves under command:

`stretch_lift_jog.py`

### Lift Gravity Compensation and Params

The lift will automatically go into float mode after power-on.  

Try manually moving the 5lb weight up and down. It is being balanced by a feedforward current command. Explore how this value is being set and then used in the control loop. There are two relevant parameters:

```
hello-robot@stretch-se3-3043:~$ stretch_params.py | grep lift | grep feedforward
stretch_user_params.yaml                                               param.hello-motor-lift.gains.i_safety_feedforward                      1.2                           
stretch_user_params.yaml                                               param.lift.i_feedforward                                               1.2 
```

The default values can be edited in `~/stretch_user/stretch-se3-3043/stretch_user_params.yaml`

Explore how the gravity compensation is implemented, where the parameters are being read from. 

The low level controller (`stepper.py`) is commanding 1.2A feedforward (`param.hello-motor-lift.gains.i_safety_feedforward`) to balance the mass on power up. 

The YAML value is piped to the firmware in`HelloController.cpp` where the following line adds the feedforward current to the commanded current.

`u=u*stiffness_target+current_to_effort(cmd.i_feedforward);`

Try to trace the feedforward value from the user YAML all the way to the controller.

NOTE: When a instance of `lift.py` is created (eg `stretch_lift_jog.py`), the param `lift.gains.i_safety_feedforward` is being used instead of the `stepper.py` value.

### Guarded Contact

During motion the commanded current `u` is monitored within the HelloController loop. When `u` exceeds a threshold, the controller switches into a safety mode. In the case of the lift, the motion stops, but can then resume upon the next motion command. The result is that the lift will stop during motion when grabbed or run into something, providing a coarse safety mechanism.

Explore the parameters and code for guarded contact. See the homing procedure for the lift:

https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/prismatic_joint.py#L664

, where `wait_for_contact` is used. 

The contact parameters are as shown. Values are -100 to 100, where 100 is full scale current. Try modifying the contact parameters during motion (both programatically and via YAML)

```
stretch_params.py | grep lift | grep contact_thresh
stretch_body.robot_params.nominal_params                               param.lift.contact_models.effort_pct.contact_thresh_calibration_margin 10.0                          
stretch_body.robot_params.nominal_params                               param.lift.contact_models.effort_pct.contact_thresh_max                [-100, 100]                   
stretch_configuration_params.yaml                                      param.lift.contact_models.effort_pct.contact_thresh_default            [-19.668696534399892, 63.684764188878674]
stretch_configuration_params.yaml                                      param.lift.contact_models.effort_pct.contact_thresh_homing             [-30.0, 80.0]                 
hello-robot@stretch-se3-3043:~/repos/stretch_samd51_fimware_sandbox$ 

```

At the firmware level, a guarded event is triggered here:

https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_stepper/HelloController.cpp#L1277

### Advanced: The Trace Tool

The Trace tool allows for high rate data collection on the stepper microcontroller. This data can then be read back in Python, which can be useful for debugging controllers.

Information is here: https://forum.hello-robot.com/t/using-the-robot-trace-tool-to-debug-stretch-body/629



Note: The tool may not be working out of the box but we can fix it when there's a need for it.



## Advanced: Write Your Own Controller

Let's try writing your own controller. LMK when you're read for this. The general outline is:

* Create your own class YourLift that inherits from Lift
* Extend Lift with your own parameters. 
  * Pipe these all the way down to HelloController
* Add a new control mode to HelloController that can be addressed from YourLift
* Pipe new status data up from the HelloController into YourLift. You'll need to extend the protocol.
* Pipe new command data down to the HelloController
* Implement a simple controller down at HelloController. 
  * Try plotting control data from Python
  * Try plotting control data with Trace
