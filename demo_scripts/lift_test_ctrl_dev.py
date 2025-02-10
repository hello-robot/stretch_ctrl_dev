#!/usr/bin/env python3
from stretch_body.stepper import Stepper
import stretch_body.hello_utils as hu
from stretch_body.lift import Lift
import time

class CtrlDevLift(Lift):
    """
    API to the Stretch Lift
    """
    def __init__(self,usb=None):
        Lift.__init__(self, usb=usb)

    def move_to(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None,contact_thresh_neg_N=None,
                req_calibration=True,contact_thresh_pos=None, contact_thresh_neg=None):
        """
        NOTE: This is just a copy of the PrismaticJoint move to for example

        x_m: commanded absolute position (meters). x_m=0 is down. x_m=~1.1 is up
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        req_calibration: Disallow motion prior to homing
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        """
        hu.check_deprecated_contact_model_prismatic_joint(self,'move_to', contact_thresh_pos_N, contact_thresh_neg_N,contact_thresh_pos,contact_thresh_neg )

        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return
            x_m = min(max(self.soft_motion_limits['current'][0], x_m), self.soft_motion_limits['current'][1]) #Only clip motion when calibrated

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_m_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_pos,i_contact_neg  = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg )

        self.motor.set_command(mode = Stepper.MODE_CTRL_DEV,
                                x_des=self.translate_m_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward,
                                i_contact_pos=i_contact_pos,
                                i_contact_neg=i_contact_neg)



l=CtrlDevLift()
#Configure / overide YAML parameters here
l.params['i_feedforward']=1.2 #Amps
l.motor.gains['ctrl_dev_A']=3.1
l.motor.gains['ctrl_dev_B']=3.0

if not l.startup(threaded=False):
    exit()

rate='default' #'fast', 'slow', 'max'
stiffness=1.0


l.move_to(x_m=0.5, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=True)
l.push_command()
for i in range(60):
    l.pull_status()
    print('I: %d | Pos %f Debug %f'%(i,l.status['pos'],l.motor.status['ctrl_dev_dbg']))
    time.sleep(0.1)

l.move_to(x_m=1.0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=True)
l.push_command()
for i in range(60):
    l.pull_status()
    print('I: %d | Pos %f Debug %f'%(i,l.status['pos'],l.motor.status['ctrl_dev_dbg']))
    time.sleep(0.1)
l.stop()
