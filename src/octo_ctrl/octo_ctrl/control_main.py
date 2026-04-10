import numpy as np
import rclpy
from rclpy.node import Node
import serial
import time
from . import pump_lib as pump
from . import color_gen_lib as color_gen

from std_srvs.srv import Trigger
from std_msgs.msg import Int64
from std_msgs.msg import Bool

class OctopusControl(Node):
    def __init__(self):
        super().__init__('octopus_control')
        self.init_ctrl_vars()
        self.init_arduino()
        self.init_pump()
        self.control_loop()
    
    def control_loop(self):
        while not self.exit:
            ans = input("Enter a control command: ")
            #ans = 'pump_ctrl'
            if ans in self.options:
                self.options[ans]()
                self.id += 1
            else:
                print("Invalid command. Please try again.")


    def init_ctrl_vars(self):
        self.id = 0 # COMMAND NUMBER IDENTIFIER
        self.exit = False
        self.TIMEOUT = 280 # Timeout for waiting for Arduino response (seconds)
        self.target_color = "#FFFFFF"
        self.color = "#FFFFFF"
        self.prev_color = "#FFFFFF"
        self.options = {
            'color_detect': self.recall_target_color,
            'color_gen': self.auto_color_generation,
            'pump_ctrl': self.peristaltic_pump_control,
            'EPM_ctrl': self.Manual_EPM_control,
            'linear_ext': self.linear_extension,
            'bending_ext': self.bending_extension,
            'servo_ctrl': self.servo_control,
            'exit': self.end_loop,
            'curvature_ctrl': self.curvature_control,
            'manual_color_gen': self.manual_color_generation,
            'skin_color_demo': self.skin_color_demo,
            'grasping_demo': self.grasping_demo
        }
        self.color_client = self.create_client(Trigger, 'color_detection')
        while not self.color_client.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')
        self.color_req = Trigger.Request()
        self.pub_motor_cmd = self.create_publisher(Int64, 'motor_cmd', 1) # Or Int64
        self.motor_cmd = Int64()
        self.motor_flag_sub = self.create_subscription(
                Bool, # Message type (e.g., Int32 for a 32-bit integer)
                'motor_cmd_flag', # Topic name
                self.motor_flag_callback, # Callback function
                1) # Queue size
        self.motor_flag = True
    
    def init_arduino(self):
        self.arduino_valve = serial.Serial(port='COM15', baudrate=9600, timeout=1) #14
        self.arduino_valve_data = None
        self.arduino_valve.reset_input_buffer()
        self.arduino_valve.reset_output_buffer()
        print("Arduino EPM initialized")
        
        self.arduino_color = serial.Serial(port='COM5', baudrate=9600, timeout=1)
        self.arduino_color_data = None
        self.arduino_color.reset_input_buffer()
        self.arduino_color.reset_output_buffer()
        print("Arduino color initialized")

    def init_pump(self):
        self.MAX_SPEED = 5000000 #128000000 # 50000000 #128000000 #2000000
        self.MAX_ACCELERATION = 80000 #640000 #2000000  #
        self.STEP_MODE = 1
        self.CURRENT_LIMIT = 1152 # in mA
        self.motor_ID = np.asarray(['00470895']) 
    
    def write_arduino(self, command, arduino):
        arduino.reset_input_buffer()
        arduino.write(bytes(command + '\n', 'utf-8'))
        arduino.flush()
        print(f'Command sent to Arduino: {command}')

    def read_arduino(self, arduino):
        try:
            arduino_data = arduino.read_until(b'\n').decode('utf-8', errors='ignore').rstrip()
            if arduino_data:
                print(f'Received from Arduino: {arduino_data}')
                return arduino_data
            else:
                print("No data received from Arduino.")
                return None
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
            return None

    def recall_target_color(self):
        future = self.color_client.call_async(self.color_req)
        # This will pump the executor until the service completes
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("color_detection service call failed or returned no result")
            return

        resp = future.result()
        self.target_color = resp.message
        self.get_logger().info(f"Target color set to: {self.target_color}")


    def manual_color_generation(self): # 000000, 3E5E8F
        #prev_color =input("Enter previous color in HEX format (e.g., #FF0000 for red): ")
        desired_color = input("Enter target color in HEX format (e.g., #FF0000 for red): ")
        
        if not desired_color.startswith('#') or len(desired_color) != 7:
            print("Invalid color format. Please enter a HEX color code (e.g., #FF0000 for red).")
            return
        desired_color_cmd = 'c_'
        _, _, _, step_plan, hex_color = color_gen.get_led_seq_from_rgb(self.prev_color, desired_color)

        print('Closest achievable dye color:', hex_color)
        print(f'Step Plan: {step_plan}')
        for i, (lights, duration) in enumerate(step_plan):
            uv, red, blue = lights 
            print(f'Lights:{lights}')
            try:
                idx = lights.index(1)
                delay = duration[idx]
            except ValueError:
                delay = 0  # or handle the missing value appropriately
            print(f'Generating UV:{uv} RED:{red} BLUE:{blue} for {duration} seconds')
            desired_color_cmd += f"{',' if i else ''}{uv},{red},{blue},{int(delay*1000)}"  # Convert to milliseconds
        print(desired_color_cmd + '_' + str(self.id))
        self.write_arduino(desired_color_cmd + '_' + str(self.id), self.arduino_color)
        
        self.prev_color = self.target_color
        try: 
            self.arduino_color_data =  self.read_arduino(self.arduino_color)  # Get initial status
            start_time = time.time()
            while self.arduino_color_data != 'DONE_' + str(self.id):
                if time.time() - start_time > self.TIMEOUT:  # Timeout after 10 seconds
                    raise TimeoutError("Timeout waiting for Color Generation to finish")
                print('Waiting for Color Generation to finish')
                self.arduino_color_data = self.read_arduino(self.arduino_color)
        except TimeoutError as e:
            print(f"Error: {e}")

        self.arduino_color_data = None
        print("COMPLETED COLOR GENERATION")
        
    def auto_color_generation(self):
        print("Generating colors for octopus tentacles")
        desired_color_cmd = 'c_'
        _, _, _, step_plan, hex_color = color_gen.get_led_seq_from_rgb(self.prev_color, self.target_color)
        #_, _, _, step_plan, hex_color = color_gen.plan_from_saturations(0, 0, 0.9, 0)

        print('Closest achievable dye color:', hex_color)
        print(f'Step Plan: {step_plan}')
        for i, (lights, duration) in enumerate(step_plan):
            uv, red, blue = lights 
            print(f'Lights:{lights}')
            try:
                idx = lights.index(1)
                delay = duration[idx]
            except ValueError:
                delay = 0  # or handle the missing value appropriately
            print(f'Generating UV:{uv} RED:{red} BLUE:{blue} for {duration} seconds')
            desired_color_cmd += f"{',' if i else ''}{uv},{red},{blue},{int(delay*1000)}"  # Convert to milliseconds
        print(desired_color_cmd + '_' + str(self.id))
        self.write_arduino(desired_color_cmd + '_' + str(self.id), self.arduino_color)
        
        # DEBUGGING PURPOSES ONLY - REMOVE FOR FINAL
        # uv = 1
        # red = 1
        # blue = 1
        # duration = input("Enter duration for color generation (seconds): ")
        # duration = int(float(duration) * 1000)  # Convert to milliseconds
        # desired_color_cmd  += f',{uv},{red},{blue},{int(duration)}'
        #self.write_arduino(desired_color_cmd + '_' + str(self.id))

        self.prev_color = self.target_color
        try: 
            self.arduino_color_data =  self.read_arduino(self.arduino_color)  # Get initial status
            start_time = time.time()
            while self.arduino_color_data != 'DONE_' + str(self.id):
                if time.time() - start_time > self.TIMEOUT:  # Timeout after 10 seconds
                    raise TimeoutError("Timeout waiting for Color Generation to finish")
                print('Waiting for Color Generation to finish')
                self.arduino_color_data = self.read_arduino(self.arduino_color)
        except TimeoutError as e:
            print(f"Error: {e}")

        self.arduino_color_data = None
        print("COMPLETED COLOR GENERATION")

    def peristaltic_pump_control_loop(self):
        #flow_rate = input("Enter flow rate (mL/min): ")
        volume = input("Enter volume to pump (mL): ")
        steps = pump.volume_to_steps(float(volume), self.STEP_MODE)
        t = input("sleep time between pump commands (seconds): ")
        t = float(t)
        for i in range(5):
            self.motor_cmd.data = steps
            self.pub_motor_cmd.publish(self.motor_cmd)
            time.sleep(t) # Short delay to ensure command is sent before waiting for completion
            self.motor_cmd.data = -steps
            self.pub_motor_cmd.publish(self.motor_cmd)
            time.sleep(t)
        #time.sleep(int(duration))
        print("Controlling peristaltic pump for octopus tentacles")
        #self.motor_cmd.data = 0
        #self.pub_motor_cmd.publish(self.motor_cmd)

    def peristaltic_pump_control(self):
        #flow_rate = input("Enter flow rate (mL/min): ")
        volume = input("Enter volume to pump (mL): ")
        steps = pump.volume_to_steps(float(volume), self.STEP_MODE)
        t = input("sleep time between pump commands (seconds): ")
        t = float(t)
        self.motor_cmd.data = steps
        self.pub_motor_cmd.publish(self.motor_cmd)
        time.sleep(t) # Short delay to ensure command is sent before waiting for completion
        print("Controlling peristaltic pump for octopus tentacles")
        
    def Manual_EPM_control(self):
        EPMs = ['RED', 'BLUE', 'GREEN']
        command = []
        for i, epm in enumerate(EPMs, start=1):
            ans = input(f"Turn {epm} EPM ON or OFF? (Y/N): ").upper()
            if ans in ['Y', 'N']:
                command.append(1 if ans == 'Y' else 0)
                separator = ","
                epm_cmd = separator.join(str(x) for x in command)
            else:
                print("Invalid input. Please enter Y or N.")

        print('e_' + epm_cmd + '_' + str(self.id))
        self.write_arduino('e_' + epm_cmd + '_' + str(self.id), self.arduino_valve)
        time.sleep(0.1)
        # Wait for completion
        try:
            self.arduino_valve_data =  self.read_arduino(self.arduino_valve)  # Get initial status
            start_time = time.time()
            while self.arduino_valve_data != 'DONE_' + str(self.id):
                if time.time() - start_time > self.TIMEOUT:  # Timeout after 10 seconds
                    raise TimeoutError("Timeout waiting for EPM control to finish")
                print('Waiting for EPM control to finish')
                self.arduino_valve_data = self.read_arduino(self.arduino_valve)
        except TimeoutError as e:
            print(f"Error: {e}")
        
        self.arduino_valve_data = None
        print("COMPLETED EPM CONTROL")

    def servo_control(self):
        SERVOS = ['RED', 'BLUE', 'GREEN']
        command = []
        for i, servo in enumerate(SERVOS, start=1):
            ans = input(f"Turn {servo} SERVO ON or OFF? (Y/N): ").upper()
            if ans in ['Y', 'N']:
                command.append(1 if ans == 'Y' else 0)
                separator = ","
                servo_cmd = separator.join(str(x) for x in command)
            else:
                print("Invalid input. Please enter Y or N.")

        print('s_' + servo_cmd + '_' + str(self.id))
        self.write_arduino('s_' + servo_cmd + '_' + str(self.id), self.arduino_valve)
        time.sleep(0.1)
        # Wait for completion
        try:
            self.arduino_valve_data =  self.read_arduino(self.arduino_valve)  # Get initial status
            start_time = time.time()
            while self.arduino_valve_data != 'DONE_' + str(self.id):
                if time.time() - start_time > self.TIMEOUT:  # Timeout after 10 seconds
                    raise TimeoutError("Timeout waiting for Servo control to finish")
                print('Waiting for Servo control to finish')
                self.arduino_valve_data = self.read_arduino(self.arduino_valve)
        except TimeoutError as e:
            print(f"Error: {e}")
        
        self.arduino_valve_data = None
        print("COMPLETED SERVO CONTROL")

    def end_loop(self): 
        self.exit = True 
        print("End Control")
    ###########################################################################
    '''
    Autonomous Control Functions
    '''
    def Auto_EPM_control(self, EPMs: list):
        #EPMs = ['RED', 'BLUE', 'GREEN']
        separator = ","
        epm_cmd = separator.join(str(x) for x in EPMs)
        print('e_' + epm_cmd + '_' + str(self.id))
        self.write_arduino('e_' + epm_cmd + '_' + str(self.id), self.arduino_valve)
        time.sleep(0.1)
        # Wait for completion
        self.arduino_valve_data =  self.read_arduino(self.arduino_valve)  # Get initial status
        while self.arduino_valve_data != 'DONE_' + str(self.id):
            print('Waiting for EPM control to finish')
            self.arduino_valve_data = self.read_arduino(self.arduino_valve)

        self.arduino_valve_data = None
        print("COMPLETED EPM CONTROL")

    def auto_pump_control(self, volume):
        steps = pump.volume_to_steps(float(volume), self.STEP_MODE)
        self.motor_cmd.data = steps
        self.pub_motor_cmd.publish(self.motor_cmd)
        print("Controlling peristaltic pump for octopus tentacles")

    def linear_extension(self, volume: float = 30.0):
        print("Controlling linear extension of octopus tentacles")
        volume = input("Enter volume to pump for linear extension (mL): ")
        # Color Generation -> EPM ON -> Pump Control (infuse) -> WAIT -> Pump Control (withdraw) -> EPM OFF
        self.recall_target_color()
        self.auto_color_generation()
        self.Auto_EPM_control([1, 1, 1]) # Turn on all EPMs
        time.sleep(0.1) # Wait for EPMs to activate
        self.auto_pump_control(float(volume)) # Infuse to extend
        time.sleep(10) # Wait for extension
        self.auto_pump_control(-float(volume)) # Withdraw to retract
        self.Auto_EPM_control([0, 0, 0]) # Turn off all EPMs

    def motor_flag_callback(self, msg):
        self.motor_flag = msg.data
        print(f"Motor command flag updated: {self.motor_flag}")

    def bending_extension(self, volume: float = 30.0, direction: list = [1, 0, 1]):
        print("Controlling bending extension of octopus tentacles")
        volume = input("Enter volume to pump for bending extension (mL): ")
        # Color Generation -> EPM ON -> Pump Control (infuse) -> WAIT -> Pump Control (withdraw) -> EPM OFF
        self.recall_target_color()
        #self.auto_color_generation()
        self.Auto_EPM_control(direction) # Turn on all EPMs
        time.sleep(0.1) # Wait for EPMs to activate
        for i in range(10):
            self.auto_pump_control(float(volume)) # Infuse to extend
            time.sleep(5) # Wait for extension
            self.auto_pump_control(-float(volume)) # Withdraw to retract
            time.sleep(5)
        self.Auto_EPM_control([0 if x else 0 for x in direction]) # Turn off all EPMs

    def curvature_control(self):
        direction_map = {
            'RED': [1, 0, 0],
            'BLUE': [0, 1, 0],
            'GREEN': [0, 0, 1]
        }
        print("Controlling curvature of octopus tentacles")
        direction_input = input("Enter direction for bending (RED, BLUE, GREEN): ").upper()
        if direction_input not in direction_map:
            direction_input = input("Enter direction for bending (RED, BLUE, GREEN): ").upper()
        direction = direction_map.get(direction_input, [1, 0, 0]) # Default to RED if invalid input
        volume_var = input("Enter volume to pump for curvature control (mL): ")
        volume_fixed = input("Enter volume to pump for fixed bending extension (mL): ")
        self.recall_target_color()
        self.auto_color_generation()
        self.Auto_EPM_control([1, 1, 1]) # Turn on EPMs for desired direction
        time.sleep(0.1) # Wait for EPMs to activate

        self.auto_pump_control(float(volume_fixed * 3)) # Infuse to extend
        while not self.motor_flag: # Wait until previous motor command is executed before sending new command
            time.sleep(0.1)

        self.Auto_EPM_control(direction) # Turn on EPMs for desired direction
        time.sleep(0.1) # Wait for EPMs to activate

        self.auto_pump_control(float(volume_var)) # Infuse to bend
        while not self.motor_flag: # Wait until previous motor command is executed before sending new command
            time.sleep(0.1)
        
        self.Auto_EPM_control([1, 1, 1]) # Keep EPMs on for desired direction
        time.sleep(0.1) # Wait for EPMs to activate
        self.auto_pump_control(-float(volume_var + volume_fixed * 3)) # Withdraw to unbend
        while not self.motor_flag: # Wait until previous motor command is executed before sending new command
            time.sleep(0.1)

        self.Auto_EPM_control([0 if x else 0 for x in direction]) # Turn off all EPMs

    # DEMO FUNCTIONS
    def skin_color_demo(self):
        desired_color_cmd = 'c_'
        color_options = {
            'Clear': '#FFFFFF',
            'Black': '#000000',
            'Yellow': '#FFFF00',
            'Blue': '#0000FF',
        }
        v = float(input("Pump volume (mL): "))

        print("Generating skin color for octopus tentacles")
        self.Auto_EPM_control([0, 0, 0]) # Turn off all EPMs
        self.auto_pump_control(float(v)) # Infuse Clear
        print("Infusing clear color for octopus skin")
        
        self.target_color = color_options['Black']
        self.auto_color_generation()
        print("Infusing black color for octopus skin")
        self.auto_pump_control(float(v)) # Withdraw Clear

        self.target_color = color_options['Yellow']
        self.auto_color_generation()
        print("Infusing yellow color for octopus skin")
        self.auto_pump_control(float(v)) # Infuse Yellow

        self.target_color = color_options['Blue']
        self.auto_color_generation()
        print("Infusing blue color for octopus skin")
        self.auto_pump_control(float(v)) # Infuse Blue

        self.target_color = color_options['Clear']
        self.auto_color_generation()
        print("Infusing clear color to wash out octopus skin")
        self.auto_pump_control(float(v)) # Withdraw to clear

        self.Auto_EPM_control([1, 1, 1]) # Turn off all EPMs

    def grasping_demo



def main(args=None):
    try:
        rclpy.init(args=args)
        octopusControl = OctopusControl()
        rclpy.spin(octopusControl)
    except KeyboardInterrupt:
        pass
    finally:
        octopusControl.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
