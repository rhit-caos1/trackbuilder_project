# this node would attempt to connect to the printer 
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from std_srvs.srv import SetBool
from transitions import Machine
import os

class StateMachine(Node):

    # define all the states used in the state machine
    state = ['home', 'scan_coarse', 'scan_fine', 'printing', 'grasp', 'place']

    def __init__(self):
        super().__init__('state_machine')
        self.machine = Machine(model = self, states = StateMachine.state, initial = 'scan')

        # define state transitions
        # define the trigger as mach_trig_NAME
        self.machine.add_transition(trigger='mach_trig_homing', source='*', dest='home')
        self.machine.add_transition(trigger='mach_trig_scan_coarse', source='home', dest='scan_coarse')
        self.machine.add_transition(trigger='mach_trig_scan_fine', source='scan_coarse', dest='scan_fine')
        self.machine.add_transition(trigger='mach_trig_printing', source='scan_fine', dest='printing')
        self.machine.add_transition(trigger='mach_trig_grasp', source='printing', dest='grasp')
        self.machine.add_transition(trigger='mach_trig_place', source='grasp', dest='place')
        
        # define state functions
        # define the state as mach_state_NAME
        self.machine.on_enter_home('mach_state_home')

        # define service
        self.homing_service = self.create_service(SetBool, 'go_home', self.homing_callback)
       
    
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

        self.get_logger().info("Going home")



def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()