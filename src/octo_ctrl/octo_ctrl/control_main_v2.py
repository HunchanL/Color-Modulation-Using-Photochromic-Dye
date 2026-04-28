import numpy as np
import rclpy
from rclpy.node import Node
import serial
import time
from . import pump_lib as pump
from . import color_gen_lib as color_gen
import nidaqmx as ni

from std_srvs.srv import Trigger
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import Int64
from std_msgs.msg import Bool

class OctopusControl(Node):
    def __init__(self):
        super().__init__('octopus_control')
        self.init_ctrl_vars()
        self.init_arduino_daq()
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
            'color_gen': self.auto_color_generation,
            'pump_ctrl': self.peristaltic_pump_control,
            'epm_ctrl': self.Manual_EPM_control,
            'linear_ext': self.linear_extension,
            'bending_ext': self.bending_extension,
            'exit': self.end_loop,
            'curvature_ctrl': self.curvature_control,
            'manual_color_gen': self.manual_color_generation,
            'skin_color_demo': self.skin_color_demo,
            'grasping_demo': self.grasping_demo,
            'walk': self.walking_demo,
        }

        ## EXPERIMENTAL - AUTOMATIC Motor control service
        self.motor_client = self.create_client(AddTwoInts, 'motor_control')
        while not self.motor_client.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')
        self.motor_req = AddTwoInts.Request()
    
    def init_arduino_daq(self):
        DEVICE_NAME = "Dev3"
        # We add them individually to ensure index 0, 1, 2 mapping
        # port 0 ch 0 - COMMON PIN for group pin
        # port 0 ch 1, 2 - EPM #1
        # port 0 ch 3, 4 - EPM #2
        # port 0 ch 5, 6 - EPM #3

        CHANNELS = [f"{DEVICE_NAME}/port0/line0", f"{DEVICE_NAME}/port0/line1", f"{DEVICE_NAME}/port0/line2", 
                    f"{DEVICE_NAME}/port0/line3", f"{DEVICE_NAME}/port0/line4",
                     f"{DEVICE_NAME}/port0/line5", f"{DEVICE_NAME}/port0/line6", ]
        self.task = ni.Task()
        for ch in CHANNELS:
            self.task.do_channels.add_do_chan(ch)
        self.task.start()
        self.task.write([False for _ in range(len(CHANNELS))])
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

    # Pump Control
    def request_motor_control(self, steps):
        self.motor_req.a = steps
        self.motor_req.b = 0 # Placeholder for additional parameters if needed
        future = self.motor_client.call_async(self.motor_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("motor_control service call failed or returned no result")
            return
        
        resp = future.result()
        self.motor_flag = resp.sum
        if self.motor_flag == 1:
            self.get_logger().info("Motor command executed successfully")
        elif self.motor_flag == 0:
            self.get_logger().error("Motor command execution failed")

    def peristaltic_pump_control_loop(self):
        #flow_rate = input("Enter flow rate (mL/min): ")
        volume = input("Enter volume to pump (mL): ")
        steps = pump.volume_to_steps(float(volume), self.STEP_MODE)
        for i in range(5):
            self.request_motor_control(steps)
            self.request_motor_control(-steps)
        print("Controlling peristaltic pump for octopus tentacles")

    def peristaltic_pump_control(self):
        #flow_rate = input("Enter flow rate (mL/min): ")
        volume = input("Enter volume to pump (mL): ")
        steps = pump.volume_to_steps(float(volume), self.STEP_MODE)
        self.request_motor_control(steps)
        print("Controlling peristaltic pump for octopus tentacles")


    # Color Generation
    def manual_color_generation(self): # 000000, 3E5E8F
        #prev_color =input("Enter previous color in HEX format (e.g., #FF0000 for red): ")
        #desired_color = input("Enter target color in HEX format (e.g., #FF0000 for red): ")

        desired_color = '#273B64'
        prev_color = '#17241A'
        
        if not desired_color.startswith('#') or len(desired_color) != 7:
            print("Invalid color format. Please enter a HEX color code (e.g., #FF0000 for red).")
            return
        desired_color_cmd = 'c_'
        _, _, _, step_plan, hex_color = color_gen.get_led_seq_from_rgb(prev_color, desired_color)

        print('Closest achievable dye color:', hex_color)
        print(f'Step Plan: {step_plan}')
        for i, (lights, duration) in enumerate(step_plan):
            uv, red, blue = lights 
            print(f'Lights:{lights}')
            try:
                red = 255 if red == 1 else 0
                blue = 255 if blue == 1 else 0
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

        print('Closest achievable dye color:', hex_color)
        print(f'Step Plan: {step_plan}')
        for i, (lights, duration) in enumerate(step_plan):
            uv, red, blue = lights 
            print(f'Lights:{lights}')
            try:
                red = 255 if red == 1 else 0
                blue = 255 if blue == 1 else 0
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
        self.id += 1
        print("COMPLETED COLOR GENERATION")

    #EPM Control  
    def Manual_EPM_control(self):
        EPMs = ['RED', 'BLUE', 'GREEN']
        command = []
        for i, epm in enumerate(EPMs, start=1):
            ans = input(f"Turn {epm} EPM ON or OFF? (Y/N): ").upper()
            if ans in ['Y', 'N']:
                command.append(True if ans == 'Y' else False)
            else:
                print("Invalid input. Please enter Y or N.")
        self.Actuate_EPM(command)
        
        print("COMPLETED EPM CONTROL")

    def Actuate_EPM(self, epm_ctrl, duration_ms = 500):
        # epm_ctrl: list of booleans [EPM1_state, EPM2_state, EPM3_state]
        # Channel mapping: 0=common, 1-2=EPM#1, 3-4=EPM#2, 5-6=EPM#3
        cmd = [False] * 7
        cmd[0] = True  # Set common pin
        
        for i, state in enumerate(epm_ctrl):
            if state:
                # Set ON pin for EPM (i*2 + 1)
                cmd[i * 2 + 1] = True
                print(f'EPM {i+1}: ON')
            else:
                # Set OFF pin for EPM (i*2 + 2)
                cmd[i * 2 + 2] = True
                print(f'EPM {i+1}: OFF')
        
        self.task.write(cmd)
        time.sleep(duration_ms / 1000.0)
        self.task.write([False] * 7)    

    def end_loop(self): 
        self.exit = True 
        print("End Control")
    ###########################################################################
    '''
    Autonomous Control Functions
    '''

    def auto_pump_control(self, volume):
        steps = pump.volume_to_steps(float(volume), self.STEP_MODE)
        self.request_motor_control(steps)
        print("Controlling peristaltic pump for octopus tentacles")

    def linear_extension(self, volume: float = 30.0):
        print("Controlling linear extension of octopus tentacles")
        volume = input("Enter volume to pump for linear extension (mL): ")
        # Color Generation -> EPM ON -> Pump Control (infuse) -> WAIT -> Pump Control (withdraw) -> EPM OFF
        self.recall_target_color()
        self.auto_color_generation()
        self.Actuate_EPM([1, 1, 1]) # Turn on all EPMs
        time.sleep(0.1) # Wait for EPMs to activate
        self.auto_pump_control(float(volume)) # Infuse to extend
        time.sleep(10) # Wait for extension
        self.auto_pump_control(-float(volume)) # Withdraw to retract
        self.Actuate_EPM([0, 0, 0]) # Turn off all EPMs

    def bending_extension(self, volume: float = 30.0, direction: list = [1, 0, 1]):
        print("Controlling bending extension of octopus tentacles")
        volume = input("Enter volume to pump for bending extension (mL): ")
        # Color Generation -> EPM ON -> Pump Control (infuse) -> WAIT -> Pump Control (withdraw) -> EPM OFF
        self.recall_target_color()
        #self.auto_color_generation()
        self.Actuate_EPM(direction) # Turn on all EPMs
        time.sleep(0.1) # Wait for EPMs to activate
        for i in range(10):
            self.auto_pump_control(float(volume)) # Infuse to extend
            time.sleep(5) # Wait for extension
            self.auto_pump_control(-float(volume)) # Withdraw to retract
            time.sleep(5)
        self.Actuate_EPM([0 if x else 0 for x in direction]) # Turn off all EPMs

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
        self.Actuate_EPM([1, 1, 1]) # Turn on EPMs for desired direction
        time.sleep(0.1) # Wait for EPMs to activate

        self.auto_pump_control(float(volume_fixed * 3)) # Infuse to extend

        self.Actuate_EPM(direction) # Turn on EPMs for desired direction

        self.auto_pump_control(float(volume_var)) # Infuse to bend
        
        self.Actuate_EPM([1, 1, 1]) # Keep EPMs on for desired direction
        time.sleep(0.1) # Wait for EPMs to activate

        self.Actuate_EPM([0 if x else 0 for x in direction]) # Turn off all EPMs

    # DEMO FUNCTIONS
    def skin_color_demo(self):
        desired_color_cmd = 'c_'
        color_options = {
            'Clear': '#FFFFFF',
            'Black': '#17241A',
            'Yellow': '#979A43',
            'Blue': '#273B64',
        }
        v = float(input("Pump volume (mL): "))

        print("Generating skin color for octopus tentacles")
        self.Actuate_EPM([0, 0, 0]) # Turn off all EPMs
        print("Infusing clear color for octopus skin")
        self.auto_pump_control(float(v)) # Infuse Clear
        time.sleep(10) # Wait for infusion
        
        self.target_color = color_options['Black']
        self.auto_color_generation()
        print("Infusing black color for octopus skin")
        self.auto_pump_control(float(v)) # Withdraw Clear
        time.sleep(10) # Wait for withdrawal

        self.target_color = color_options['Yellow']
        self.auto_color_generation()
        print("Infusing yellow color for octopus skin")
        self.auto_pump_control(float(v)) # Infuse Yellow
        time.sleep(10) # Wait for infusion

        self.target_color = color_options['Blue']
        self.auto_color_generation()
        print("Infusing blue color for octopus skin")
        self.auto_pump_control(float(v)) # Infuse Blue
        time.sleep(10) # Wait for infusion

        self.target_color = color_options['Clear']
        self.auto_color_generation()
        print("Infusing clear color to wash out octopus skin")
        self.auto_pump_control(float(v)) # Withdraw to clear
        time.sleep(10) # Wait for infusion

        self.Actuate_EPM([1, 1, 1]) # Turn off all EPMs

    def grasping_demo(self):
        # RECALL GREEN COLOR
        def repeating_process(volume: float):
            #self.recall_target_color()
            #self.auto_color_generation()
            time.sleep(0.1) # Wait for EPMs to activate
            self.auto_pump_control(float(volume)) # Infuse to extend
            time.sleep(3)
            self.auto_pump_control(float(-volume*2)) # Infuse to extend

        q = input("Press Enter to continue to next part of demo...").upper()
        if q == 'Y':
            print("First Grasping demo completed")
            self.target_color = '#273B64'
            self.prev_color = '#17241A'
            self.auto_color_generation()
            self.Actuate_EPM([0, 0, 0]) # Turn on EPMs
            volume = 9.5
            self.auto_pump_control(float(volume)) # Infuse to extend
            self.Actuate_EPM([1, 0, 0]) # Turn on EPMs 
            volume = 18.0     
            repeating_process(float(volume))
            self.Actuate_EPM([0, 0, 0]) # Turn off all EPMs
            volume = 18.0
            self.auto_pump_control(-float(volume)) # Infuse to extend
            self.Actuate_EPM([1, 1, 1]) # Turn off all EPMs
        else: 
            print("Grasping demo completed")

    def walking_demo(self):
        def repeating_process(volume: float):
            time.sleep(0.1) # Wait for EPMs to activate
            self.auto_pump_control(float(volume)) # Infuse to extend
            time.sleep(3)
            self.auto_pump_control(float(-volume*1.5)) # Infuse to extend
            
        volume = 60.0
        for i in range(10):
            repeating_process(float(volume))

        self.target_color = '#273B64'
        self.prev_color = '#17241A'
        # self.auto_color_generation()
        # for i in range(5):
        #     repeating_process(float(volume))



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
