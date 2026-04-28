import numpy as np
import rclpy
from rclpy.node import Node
import serial
import time
from . import pump_lib as pump
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts

class MotorCtrl(Node):
    def __init__(self):
        super().__init__('motor_ctrl')
        self.init_pump()
        self.sub_motor_cmd = self.create_subscription(
                Int64, # Message type (e.g., Int32 for a 32-bit integer)
                'motor_cmd', # Topic name
                self.listener_callback, # Callback function
                1) # Queue size
        self.motor_flag_pub = self.create_publisher(
                Bool, 
                'motor_cmd_flag', 
                1) # Publisher to indicate whether the motor command has been executed or not.
        self.motor_cmd_flag = Bool()
        self.motor_cmd_flag.data = True # Initial flag value indicating that the motor is ready to receive commands.
        self.Timer_period = 0.1  # seconds
        self.flow_rate = 0  # Initial flow rate
        self.steps = 0
        self.current_pos = 0
        #self.motor_cmd_flag = True
        print("Controlling peristaltic pump for octopus tentacles")
        #     self.srv = self.create_service(AddTwoInts, 'motor_control', self.motor_req_callback)
        self.timer_ctrl = self.create_timer(self.Timer_period,
                                            self.automatic_ctrl_volume
                                            )


    # def motor_req_callback(self, request, response):
    #     self.get_logger().info('Motor control requested via service call.')
    #     self.steps = request.a
    #     self.automatic_ctrl_volume()
    #     response.sum = 1
    #     return response

    def init_pump(self):
        # FOR SBA: ideal speed = 5000000~8000000
        self.MAX_SPEED = 1700000 #128000000 # 50000000 #128000000 #2000000
        self.MAX_SPEED = int(input("Enter max speed for stepper motor (steps/s): "))
        self.MAX_ACCELERATION = 100000 #640000 #2000000  #
        self.STEP_MODE = 1
        self.CURRENT_LIMIT = 1152 # in mA
        self.motor_ID = np.asarray(['00470895']) 
        pump.init_motor_setup(self.motor_ID, 
                             self.STEP_MODE, 
                             self.CURRENT_LIMIT, 
                             self.MAX_SPEED,
                             self.MAX_ACCELERATION) # Motor Controller Setup
    
    def listener_callback(self, msg):
        #self.flow_rate = msg.data
        if self.motor_cmd_flag.data: # Only update the target position when the previous command has been executed.
            delta_step = msg.data
            print(f"Current Step: {self.steps} steps")
            self.steps += delta_step
            #print(f"Received flow rate command: {self.flow_rate} mL/min")
            print(f"Received step command: {delta_step} steps. Motor will move to {self.steps} steps.")
            self.motor_cmd_flag.data = False # Turn off when a cmd is received, and turn on when the cmd is executed in the timer callback function.
            self.motor_flag_pub.publish(self.motor_cmd_flag) # Publish the flag status to indicate that the motor command has been executed.
        

    def automatic_ctrl_volume(self):
        self.current_pos = pump.get_motor_position(self.motor_ID[0])
        print(f"Current Step: {self.current_pos} steps")
        while not self.motor_cmd_flag.data:
            if self.current_pos != self.steps:
                print(f"Moving motor to {self.steps} steps...")
                pump.run_target_position(self.motor_ID[0], self.steps)
                self.current_pos = pump.get_motor_position(self.motor_ID[0])
            else:
                print(f"Motor has reached the target position: {self.steps} steps.")
                self.motor_cmd_flag.data = True # Turn on when the cmd is executed, and turn off when a new cmd is received.
                self.motor_flag_pub.publish(self.motor_cmd_flag) # Publish the flag status to indicate that the motor command has been executed.

    # TEST FUNCTION
    def automatic_ctrl_volume(self):
        self.current_pos = pump.get_motor_position(self.motor_ID[0])
        print(f"Current Step: {self.current_pos} steps")
        while not self.motor_cmd_flag.data:
            if self.current_pos != self.steps:
                print(f"Moving motor to {self.steps} steps...")
                pump.run_target_position(self.motor_ID[0], self.steps)
                self.current_pos = pump.get_motor_position(self.motor_ID[0])
            else:
                print(f"Motor has reached the target position: {self.steps} steps.")
                self.motor_cmd_flag.data = True # Turn on when the cmd is executed, and turn off when a new cmd is received.
            
def main(args=None):
    try:
        rclpy.init(args=args)
        motorCtrl = MotorCtrl()
        rclpy.spin(motorCtrl)
    except KeyboardInterrupt:
        pass
    finally:
        motorCtrl.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
