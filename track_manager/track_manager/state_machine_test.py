# this node would attempt to connect to the printer 
import time
from math import pi
import numpy as np
from transitions import Machine
import os

class StateMachine():

    # define all the states used in the state machine
    state = ['waiting', 'home', 'scan', 'printing', 'grasp', 'track']

    def __init__(self):
        # main condition checks
        self.arm_process_complete = True # this changes to false whenever we send a service request to the arm
        self.printer_process_complete = True # this changes to false whenever we send a service request to the printer

        # sub condition checks
        self.scan_all_tags = False

        self.machine = Machine(model = self, states = StateMachine.state, initial = 'waiting', queued=True)

        # define state transitions
        # define the trigger as mach_trig_NAME
        self.machine.add_transition(trigger='mach_trig_homing', source='*', dest='home', conditions=['is_arm_process_complete'])
        self.machine.add_transition(trigger='mach_trig_waiting', source='*', dest='waiting', conditions=['is_arm_process_complete'])
        self.machine.add_transition(trigger='', source='home', dest='waiting', conditions=['is_arm_process_complete'])
        
        self.machine.add_transition(trigger='mach_trig_scan', source='wait', dest='scan')
        self.machine.add_transition(trigger='mach_trig_trackGen', source='scan', dest='track', conditions=['is_scan_all_tags'])
        self.machine.add_transition(trigger='mach_trig_print', source='track', dest='printing', conditions=['is_printer_process_complete'])
        self.machine.add_transition(trigger='mach_trig_grasp', source='printing', dest='grasp', conditions=['is_arm_process_complete', 'is_printer_process_complete'])
        self.machine.add_transition(trigger='mach_trig_place', source='grasp', dest='scan', conditions=['is_arm_process_complete'])

        
        # define state functions
        # define the state as mach_state_NAME
        self.machine.on_enter_home('mach_state_home')
        self.machine.on_enter_waiting('mach_state_waiting')
        self.machine.on_enter_scan('mach_state_scan')
        self.machine.on_enter_track('mach_state_track')
        self.machine.on_enter_printing('mach_state_printing')
        self.machine.on_enter_grasp('mach_state_grasp')

        # define service       
        

        # define publisher

        # define subscriber
        # we need a printer status subscriber and update the printer_process_complete variable

        # define action

        # define timer 

    def is_arm_process_complete(self):
        return self.arm_process_complete
    
    def is_printer_process_complete(self):
        return self.printer_process_complete
    
    def is_scan_all_tags(self):
        return self.scan_all_tags
    

    def mach_state_home(self):
        # call the service to go home
        self.arm_process_complete = False
        print("going home")
        self.arm_process_complete = True
    
    def mach_state_waiting(self):
        print("waiting")
    
    def mach_state_scan(self):
        self.arm_process_complete = False
        # call the service to scan
        print("scanning")

    def mach_state_track(self):
        # call the service to generate the track
        print("generating track")

    def mach_state_printing(self):
        self.printer_process_complete = False
        # call the service to print
        print("printing")
    
    def mach_state_grasp(self):
        self.arm_process_complete = False
        # call the service to grasp
        print("grasping")


state = StateMachine()
state.mach_trig_homing()
