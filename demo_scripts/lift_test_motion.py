#!/usr/bin/env python3

import stretch_body.lift as lift
import time

l=lift.Lift()
l.params['i_feedforward']=1.2 #Amps, counterbalance payload
stiffness=0.25#0-1, go from float mode to full stiffness

if not l.startup(threaded=False):
    exit()

rate='default' #'fast', 'slow', 'max'



l.move_to(x_m=0.5, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=True)
l.push_command()
time.sleep(6.0)

l.move_to(x_m=1.0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=True)
l.push_command()
time.sleep(6.0)
l.stop()
