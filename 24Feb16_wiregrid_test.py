
from nextline import disable_trace


import time
import datetime

with disable_trace():
    import numpy as np
    import sorunlib as run
    run.initialize()
    from ocs.ocs_client import OCSClient

UTC = datetime.timezone.utc
pysmurfs = run.CLIENTS['smurf']
acu = run.CLIENTS["acu"]

# HWP Params
use_pid = True
forward = True
hwp_freq = 2.0

def HWPPrep():
    iboot2 = OCSClient('power-iboot-hwp-2')
    iboot2.set_outlet(outlet = 1, state = 'on')
    iboot2.set_outlet(outlet = 2, state = 'on')

    pid = OCSClient('hwp-pid')
    pmx = OCSClient('hwp-pmx')
    global use_pid
    global forward

    if use_pid:
        pmx.use_ext()
    else:
        pmx.ign_ext()

    if forward:
        pid.set_direction(direction = '0')
    else:
        pid.set_direction(direction = '1')

def HWPPost():
    iboot2 = OCSClient('power-iboot-hwp-2')
    gripper = OCSClient('hwp-gripper')
    iboot2.set_outlet(outlet = 1, state = 'off')
    iboot2.set_outlet(outlet = 2, state = 'off')
    gripper.force(value = False)
    gripper.brake(state = True)
    gripper.power(state = False)

def HWPUngrip():
    gripper = OCSClient('hwp-gripper')
    gripper.reset()
    gripper.force(value=True)
    gripper.brake(state=False)
    gripper.power(state=True)
    gripper.home()
    time.sleep(5)
    gripper.brake(state=True)
    gripper.power(state=False)

def HWPGrip():
    gripper = OCSClient('hwp-gripper')
    gripper.reset()
    gripper.force(value=True)
    gripper.brake(state=False)
    gripper.power(state=True)
    gripper.home()

    finished_arr = [False, False, False]
    while True:
        for actuator, finished in enumerate(finished_arr):
            if not finished:
                gripper.move(mode='pos', actuator=actuator+1, distance=1.0)
                if bool(gripper.alarm()[2]['success']):
                    gripper.reset()
                    finished_arr[actuator] = True
            time.sleep(1)

        if all(finished_arr):
            break 

    time.sleep(5)
    gripper.brake(state=True)
    gripper.power(state=False)

def HWPSpinUp():
    pid = OCSClient('hwp-pid')
    pmx = OCSClient('hwp-pmx')
    global use_pid
    global forward
    global hwp_freq

    if use_pid:
        if forward:
            pid.set_direction(direction = '0')
        else:
            pid.set_direction(direction = '1')
    
        pid.declare_freq(freq = hwp_freq)
        pid.tune_freq()
        pmx.set_on()
    
        time.sleep(1)
        cur_freq = float(pid.get_freq()[2]['messages'][1][1].split(' ')[3])
    
        while abs(cur_freq - hwp_freq) > 0.005:
            cur_freq = float(pid.get_freq()[2]['messages'][1][1].split(' ')[3])
            print ('Current Frequency =', cur_freq, 'Hz    ', end = '\r')
    
        print('                                    ', end = '\r')
        print('Tuning finished')
    else:
        print('Error: Not using PID')

def HWPFastStop():
    iboot2 = OCSClient('power-iboot-hwp-2')
    pid = OCSClient('hwp-pid')
    pmx = OCSClient('hwp-pmx')
    global use_pid
    global forward

    if use_pid:
        print('Starting stop')
        if forward:
            pid.set_direction(direction = '1')
        else:
            pid.set_direction(direction = '0')
        
        pid.tune_stop()
        pmx.set_on()
        
        time.sleep(1)
        start_freq = float(pid.get_freq()[2]['messages'][1][1].split(' ')[3])
        time.sleep(15)
        cur_freq = float(pid.get_freq()[2]['messages'][1][1].split(' ')[3])
        if cur_freq > start_freq:
            if forward:
                pid.set_direction(direction = '0')
            else:
                pid.set_direction(direction = '1')

            start_freq = cur_freq
            time.sleep(15)
            cur_freq = float(pid.get_freq()[2]['messages'][1][1].split(' ')[3])
            if cur_freq > start_freq:
                print('Could not determine rotation direction, shutting off all power')
                pmx.set_off()
                iboot2.set_outlet(outlet = 1, state = 'off')
                iboot2.set_outlet(outlet = 2, state = 'off')
                time.sleep(60*30)

        while cur_freq > 0.2:
            cur_freq = float(pid.get_freq()[2]['messages'][1][1].split(' ')[3])
            print ('Current Frequency =', cur_freq, 'Hz    ', end = '\r')
        
        pmx.set_off()
        iboot2.set_outlet(outlet = 1, state = 'off')
        iboot2.set_outlet(outlet = 2, state = 'off')
        time.sleep(180)
        iboot2.set_outlet(outlet = 1, state = 'on')
        iboot2.set_outlet(outlet = 2, state = 'on')
        
        print('                                    ', end = '\r')
        print('CHWP stopped')
    else:
        print('Error: Not using PID')


#################################################
########### wire grid function ###########
def wg_prep():
    wg_actuator = OCSClient('wg-actuator')
    wg_kikusui = OCSClient('wg-kikusui')
    wg_kikusui.set_v(volt=12.0)
    wg_kikusui.set_c(current=3.0)
#################################################

############# Daily Relock ######################
run.smurf.uxm_relock(concurrent=True)

#################### Detector Setup #########################
print('Waiting until 2024-01-21 22:00:00+00:00 to start detector setup')
run.wait_until('2024-01-21T22:00:00+00:00')
run.smurf.take_bgmap(concurrent=True)
run.smurf.iv_curve(concurrent=False, settling_time=0.1)
run.smurf.bias_dets(concurrent=True)
time.sleep(180)
run.smurf.bias_step(concurrent=True)
#################### Detector Setup Over ####################

############# move to el = 50 and az = 121 ###############
run.acu.move_to(121, 50)
############################################################

####### HWP Start #######
HWPPrep()
forward = True
hwp_freq = 2.0
HWPSpinUp()
time.sleep(900)

wg_prep()
wg_actuator = OCSClient('wg-actuator')
wg_kikusui = OCSClient('wg-kikusui')

####### Wiregrid Continuous Rotation out of the windows #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_continuous,wg_out_of_window')
time.sleep(10)
wg_kikusui.set_on()
time.sleep(60)
wg_kikusui.set_off()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Insert/eject WG w/ continuous rotation #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_continuous,in_out_with_spinning')
wg_kikusui.set_on()
time.sleep(5)
wg_actuator.insert()
time.sleep(60)
wg_actuator.eject()
time.sleep(5)
wg_kikusui.set_off()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregride stepwise rotaion out of the view ##########
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise,wg_out_of_window')
time.sleep(10)
wg_kikusui.stepwise_rotation()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### WG Nominal Operation * 10 #######
for _i in range(10):
    run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise')
    wg_actuator.insert()
    time.sleep(5)
    wg_kikusui.stepwise_rotation()
    time.sleep(10)
    wg_actuator.eject()
    time.sleep(10)
    run.smurf.stream('off')
    time.sleep(30)

####### WG Not Rotating Insert/Ejection #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_no_rotate')
wg_actuator.insert()
time.sleep(30)
wg_actuator.eject()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregrid Step Rotation w/ 5s step wait #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise_5s')
wg_actuator.insert()
time.sleep(10)
wg_kikusui.stepwise_rotation(stopped_time=5.0)
time.sleep(10)
wg_actuator.eject()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregrid Step Rotation w/ 20s step wait #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise_20s')
wg_actuator.insert()
time.sleep(5)
wg_kikusui.stepwise_rotation(stopped_time=20.0)
time.sleep(5)
wg_actuator.eject()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

def change_hwp_freq(freq):
    global forward
    global hwp_freq
    forward = True
    hwp_freq = freq
    HWPSpinUp() 
    time.sleep(900)

####### Wiregrid Step Rotation w/ HWP Rotation Speed Varying #######  
freqs_hwp = np.arange(1.2, 2.4, 0.3)

for _i, freq in enumerate(freqs_hwp):
    freq_name_one = int(freq)
    freq_name_two = int(freq*10%10)
    tag_name = f'wiregrid,wg_stepwise,hwp_speed_{freq_name_one}_{freq_name_two}hz'
    run.smurf.stream('on', subtype='cal', tag=tag_name)
    print('HWP Rotation Speed = ', freq)
    change_hwp_freq(freq)
    time.sleep(30)
    wg_actuator.insert()
    time.sleep(5)
    wg_kikusui.stepwise_rotation()
    time.sleep(10)
    wg_actuator.eject()
    time.sleep(10)
    run.smurf.stream('off')
    time.sleep(30)


############### move to el = 90 and az = 121 ###############
run.acu.move_to(121, 90)
############################################################

####### HWP Start and rotate at 2.0 hz #######
change_hwp_freq(2.0)
#########################

####### Wiregrid Continuous Rotation out of the windows #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_continuous,wg_out_of_window')
time.sleep(10)
wg_kikusui.set_on()
time.sleep(60)
wg_kikusui.set_off()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Insert/eject WG w/ continuous rotation #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_continuous,in_out_with_spinning')
wg_kikusui.set_on()
time.sleep(5)
wg_actuator.insert()
time.sleep(60)
wg_actuator.eject()
time.sleep(5)
wg_kikusui.set_off()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregride stepwise rotaion out of the view ##########
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise,wg_out_of_window')
time.sleep(10)
wg_kikusui.stepwise_rotation()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### WG Nominal Operation * 10 #######
for _i in range(10):
    run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise')
    wg_actuator.insert()
    time.sleep(5)
    wg_kikusui.stepwise_rotation()
    time.sleep(10)
    wg_actuator.eject()
    time.sleep(10)
    run.smurf.stream('off')
    time.sleep(30)

####### WG Not Rotating Insert/Ejection #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_no_rotate')
wg_actuator.insert()
time.sleep(30)
wg_actuator.eject()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregrid Step Rotation w/ 5s step wait #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise_5s')
wg_actuator.insert()
time.sleep(10)
wg_kikusui.stepwise_rotation(stopped_time=5.0)
time.sleep(10)
wg_actuator.eject()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregrid Step Rotation w/ 20s step wait #######
run.smurf.stream('on', subtype='cal', tag='wiregrid,wg_stepwise_20s')
wg_actuator.insert()
time.sleep(5)
wg_kikusui.stepwise_rotation(stopped_time=20.0)
time.sleep(5)
wg_actuator.eject()
time.sleep(10)
run.smurf.stream('off')
time.sleep(30)

####### Wiregrid Step Rotation w/ HWP Rotation Speed Varying #######  
freqs_hwp = np.arange(1.2, 2.4, 0.3)

for _i, freq in enumerate(freqs_hwp):
    freq_name_one = int(freq)
    freq_name_two = int(freq*10%10)
    tag_name = f'wiregrid,wg_stepwise,hwp_speed_{freq_name_one}_{freq_name_two}hz'
    run.smurf.stream('on', subtype='cal', tag=tag_name)
    print('HWP Rotation Speed = ', freq)
    change_hwp_freq(freq)
    time.sleep(30)
    wg_actuator.insert()
    time.sleep(5)
    wg_kikusui.stepwise_rotation()
    time.sleep(10)
    wg_actuator.eject()
    time.sleep(10)
    run.smurf.stream('off')
    time.sleep(30)


####### HWP Stop #######
HWPFastStop()
HWPPost()

time.sleep(1)
