#!/usr/bin/env python3

import stretch_body.lift as lift
import time
import stretch_body.hello_utils as hu


l=lift.Lift()
if not l.startup(threaded=False):
    exit()

l.move_to(x_m=0.5)

#Execute the motion while recording first 1s via trace
l.motor.enable_firmware_trace()
l.push_command()
time.sleep(1.0)
l.motor.disable_firmware_trace()
l.push_command()
data=l.motor.read_firmware_trace()

time.sleep(5.0) #Finish motion

#Move back
l.move_to(x_m=1.0)
l.push_command()
time.sleep(6.0)
l.stop()


#Print out the trace data
for k in range(len(data)):
    lift_pos=l.motor_rad_to_translate_m(data[k]['debug']['f_3'])
    lift_target=l.motor_rad_to_translate_m(hu.deg_to_rad(data[k]['debug']['f_1']))
    traj_t= data[k]['debug']['f_2']
    print('Control cycle cnt: %f | Trajectory time (ms) %f |  Target %f | Pos (m) %f | Error (m) %f'%(data[k]['id'], traj_t,lift_target, lift_pos,lift_target-lift_pos))