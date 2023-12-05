# this node would attempt to connect to the printer 
import time
from math import pi
import numpy as np
from transitions import Machine
import os

class StateMachine():

    # define all the states used in the state machine
    state = ['waiting', 'home', 'scan', 'printing', 'grasp', 'place', 'track']

    def __init__(self):
        super().__init__('state_machine')
        self.machine = Machine(model = self, states = StateMachine.state, initial = 'waiting')

        # define state transitions
        # define the trigger as mach_trig_NAME
        self.machine.add_transition(trigger='mach_trig_homing', source='*', dest='home')
        self.machine.add_transition(trigger='mach_trig_waiting', source='*', dest='waiting')

        self.machine.add_transition(trigger='mach_trig_scan', source='wait', dest='scan')

        
        # define state functions
        # define the state as mach_state_NAME
        self.machine.on_enter_home('mach_state_home')

        # define service       
    
        # define publisher

        # define subscriber

        # define action

        # define timer 
        
        self.get_logger().info("State machine initialized!")
        self.get_logger().info("current state " + self.state)
    
    def homing_callback(self, request, response):
        if request.data == True:
            response.success = True
            self.mach_trig_homing()
        return response
    
    def mach_state_home(self):
        # call the service to go home

        print("")

