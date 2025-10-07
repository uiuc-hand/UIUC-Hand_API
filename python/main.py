import numpy as np

from uiuc_hand_utils.dynamixel_client import *
import uiuc_hand_utils.uiuc_hand_utils as lhu
import time
#######################################################
"""This can control and query the UIUC Hand

I recommend you only query when necessary and below 90 samples a second.  Used the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more in radians.

#UIUC hand conventions:
#3.14 rad is flat out home pose for the index, middle, ring, finger MCPs.
#Applying a positive angle closes the joints more and more to curl closed in radians.
#The MCP is centered at 3.14 and can move positive or negative to that in radians.

#The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side, MCP Forward, PIP, DIP for each finger.
#For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

"""
########################################################
class UIUCNode:
    def __init__(self):
        ####Some parameters
        # I recommend you keep the current limit from 350 for the lite, and 550 for the full hand
        # Increase KP if the hand is too weak, decrease if it's jittery.
        self.kP = 300  # Reduced from 600 for smoother response to noisy input
        self.kI = 0
        self.kD = 400  # Increased from 200 for better damping of noise
        self.curr_lim = 350  ##set this to 550 if you are using full motors!!!!
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_uiuc_hand(np.zeros(16))[:3]  # Only first 3 motors
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your UIUC Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0
        self.motors = motors = [1,2,3]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 57600)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 57600)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 57600)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        # Set PID gains for all motors with noise-optimized values
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness for all motors
        
        # Motor-specific tuning for noisy input:
        # Motor 1 (side-to-side): Lower P, higher D for smoothness
        self.dxl_client.sync_write([1], np.ones(1) * (self.kP * 0.3), 84, 2) # Even lower P for side-to-side
        self.dxl_client.sync_write([1], np.ones(1) * (self.kD * 2.0), 80, 2) # Higher D for side-to-side
        
        # Motor 2 (forward): Medium P, high D for noise rejection
        self.dxl_client.sync_write([2], np.ones(1) * (self.kP * 0.6), 84, 2) # Medium P for forward
        self.dxl_client.sync_write([2], np.ones(1) * (self.kD * 1.8), 80, 2) # High D for forward
        
        # Motor 3 (DIP): Medium P, high D for noise rejection  
        self.dxl_client.sync_write([3], np.ones(1) * (self.kP * 0.6), 84, 2) # Medium P for DIP
        self.dxl_client.sync_write([3], np.ones(1) * (self.kD * 1.8), 80, 2) # High D for DIP
        
        # Igain for all motors (usually 0)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive UIUC pose and directly control the robot
    def set_uiuc(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        
        # Apply calibration limits if available
        if hasattr(self, 'calibrated_limits'):
            # Clip to calibrated ranges (0-2π input mapped to calibrated min-max)
            for i, (min_val, max_val) in enumerate(self.calibrated_limits):
                if i < len(self.curr_pos):
                    # Map 0-2π to calibrated range
                    normalized = self.curr_pos[i] / (2 * np.pi)  # 0-1
                    self.curr_pos[i] = min_val + normalized * (max_val - min_val)
                    self.curr_pos[i] = np.clip(self.curr_pos[i], min_val, max_val)
        
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        
        # Load saved calibration if available
        self.load_calibration()
    
    def save_calibration(self):
        """Save calibration to file"""
        if hasattr(self, 'calibrated_limits') and self.calibrated_limits:
            import json
            import os
            
            # Convert to serializable format
            calib_data = {
                'motors': [1, 2, 3],
                'limits': [(float(min_val), float(max_val)) for min_val, max_val in self.calibrated_limits]
            }
            
            # Save to file
            calib_file = os.path.expanduser('~/.uiuc_hand_calibration.json')
            with open(calib_file, 'w') as f:
                json.dump(calib_data, f, indent=2)
            print(f"Calibration saved to {calib_file}")
    
    def load_calibration(self):
        """Load calibration from file"""
        import json
        import os
        
        calib_file = os.path.expanduser('~/.uiuc_hand_calibration.json')
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)
                
                # Convert back to tuples
                self.calibrated_limits = [tuple(limit) for limit in calib_data['limits']]
                #print(f"✓ Loaded calibration from {calib_file}")
                #for i, (min_val, max_val) in enumerate(self.calibrated_limits):
                    #print(f"Motor {i+1}: {min_val:.3f} to {max_val:.3f} rad")
            except Exception as e:
                print(f"Failed to load calibration: {e}")
                self.calibrated_limits = None
        else:
            self.calibrated_limits = None
    
    def calibrate_motors(self, motors_to_calibrate=[1, 2, 3]):
        """Record min/max positions while showing current position"""
        print("Calibration mode - Press Enter to save and exit")
        print("Torque disabled - rotate motors to find min/max positions")
        
        # Disable torque for calibration
        self.dxl_client.set_torque_enabled(motors_to_calibrate, False)
        
        # Initialize limits
        self.calibrated_limits = [(float('inf'), float('-inf'))] * len(motors_to_calibrate)
        
        import threading
        import sys
        
        # Flag to stop calibration
        stop_calibration = threading.Event()
        
        def check_input():
            input()  # Wait for Enter key
            stop_calibration.set()
        
        # Start input thread
        input_thread = threading.Thread(target=check_input)
        input_thread.daemon = True
        input_thread.start()
        
        try:
            while not stop_calibration.is_set():
                pos = self.read_pos()
                print("Position: " + str(pos))
                
                # Update min/max for each motor
                for i, motor_id in enumerate(motors_to_calibrate):
                    motor_pos = pos[motor_id - 1]  # Convert to 0-based index
                    min_val, max_val = self.calibrated_limits[i]
                    self.calibrated_limits[i] = (min(motor_pos, min_val), max(motor_pos, max_val))
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Re-enable torque
            self.dxl_client.set_torque_enabled(motors_to_calibrate, True)
            print("Torque re-enabled")
            
            # Save calibration
            print("\n✓ Calibration saved!")
            for i, motor_id in enumerate(motors_to_calibrate):
                min_val, max_val = self.calibrated_limits[i]
                print(f"Motor {motor_id}: {min_val:.3f} to {max_val:.3f} rad")
            
            # Save to permanent file
            self.save_calibration()
            print("Calibrated limits will be used by set_uiuc()")
    
    #allegro compatibility joint angles.  It adds 180 to make the fully open position at 0 instead of 180
    def set_allegro(self, pose):
        if len(pose) == 3:
            # Direct 3-motor input
            pose = lhu.allegro_to_uiuc_hand(np.concatenate([pose, np.zeros(13)]), zeros=False)[:3]
        else:
            pose = lhu.allegro_to_uiuc_hand(pose, zeros=False)[:3]
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility for policies, it assumes the ranges are [-1,1] and then convert to uiuc hand ranges.
    def set_ones(self, pose):
        if len(pose) == 3:
            # Direct 3-motor input
            pose = lhu.sim_ones_to_uiuc_hand(np.concatenate([pose, np.zeros(13)]))[:3]
        else:
            pose = lhu.sim_ones_to_uiuc_hand(np.array(pose))[:3]
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position of the robot
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    #These combined commands are faster FYI and return a list of data
    def pos_vel(self):
        return self.dxl_client.read_pos_vel()
    #These combined commands are faster FYI and return a list of data
    def pos_vel_eff_srv(self):
        return self.dxl_client.read_pos_vel_cur()
#init the node
def main(calibrate=False, **kwargs):
    uiuc_hand = UIUCNode()
    
    if calibrate:
        # Run calibration
        uiuc_hand.calibrate_motors([1, 2, 3])
        return
    
    while True:
        #Set to an open pose and read the joint angles 33hz
        uiuc_hand.set_allegro(np.zeros(3))  # Only 3 motors
        print("Position: " + str(uiuc_hand.read_pos()))
        time.sleep(0.03)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='UIUC Hand Control')
    parser.add_argument('--calibrate', action='store_true', help='Run motor calibration')
    args = parser.parse_args()
    
    main(calibrate=args.calibrate)
