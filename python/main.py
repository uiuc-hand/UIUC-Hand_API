import numpy as np
import time
import json
import os
import threading

from uiuc_hand_utils.dynamixel_client import *
import uiuc_hand_utils.uiuc_hand_utils as lhu
"""UIUC Hand Control - 15 motor robotic hand with calibration support"""
class UIUCNode:
    def __init__(self, calibration_mode=False):
        ####Some parameters
        # I recommend you keep the current limit from 350 for the lite, and 550 for the full hand
        # Increase KP if the hand is too weak, decrease if it's jittery.
        self.kP = 300  # Reduced from 600 for smoother response to noisy input
        self.kI = 0
        self.kD = 400  # Increased from 200 for better damping of noise
        self.curr_lim = 350  ##set this to 550 if you are using full motors!!!!
        # Load calibration first to get dynamic values
        self.load_calibration()
        
        if not calibration_mode:
            # Default position: max values, but every 3rd motor (3,6,9,12,15) uses midpoint, every 2nd motor (2,5,8,11,14) uses min
            default_positions = []
            for i in range(15):
                if hasattr(self, 'calibrated_limits') and self.calibrated_limits and i < len(self.calibrated_limits):
                    min_val, max_val = self.calibrated_limits[i]
                    if (i + 1) % 3 == 0:  # Every 3rd motor (3,6,9,12,15)
                        default_positions.append((min_val + max_val) / 2)  # midpoint
                    elif (i + 1) % 3 == 2:  # Every 2nd motor (2,5,8,11,14)
                        default_positions.append(min_val)  # min
                    else:  # Every 1st motor (1,4,7,10,13)
                        default_positions.append(max_val)  # max
                else:
                    # Fallback to default values if no calibration
                    default_positions.append(3.14)  # Default middle position
            
            self.prev_pos = self.pos = self.curr_pos = np.array(default_positions)
        else:
            # In calibration mode, just initialize with zeros (won't be used)
            self.prev_pos = self.pos = self.curr_pos = np.zeros(15)
        
        
        # Connect to UIUC Hand
        self.motors = list(range(1, 16))
        self.dxl_client = DynamixelClient(self.motors, '/dev/cu.usbserial-FT66WCAW', 1000000)
        self.dxl_client.connect()
        # Setup motor control
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        
        if not calibration_mode:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

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
    
    
    def save_calibration(self):
        if hasattr(self, 'calibrated_limits') and self.calibrated_limits:
            calib_data = {
                'motors': list(range(1, 16)),
                'limits': [(float(min_val), float(max_val)) for min_val, max_val in self.calibrated_limits]
            }
            calib_file = os.path.expanduser('~/.uiuc_hand_calibration.json')
            with open(calib_file, 'w') as f:
                json.dump(calib_data, f, indent=2)
            print(f"Calibration saved to {calib_file}")
    
    def load_calibration(self):
        calib_file = os.path.expanduser('~/.uiuc_hand_calibration.json')
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)
                self.calibrated_limits = [tuple(limit) for limit in calib_data['limits']]
            except Exception as e:
                print(f"Failed to load calibration: {e}")
                self.calibrated_limits = None
        else:
            self.calibrated_limits = None
    
    def calibrate_motors(self, motors_to_calibrate=list(range(1, 16))):
        print("Calibration mode - Press Enter to save and exit, Ctrl+C to exit without saving")
        print("Torque disabled - rotate motors to find min/max positions")
        
        # Disable torque for calibration
        self.dxl_client.set_torque_enabled(motors_to_calibrate, False)
        
        # Initialize limits
        self.calibrated_limits = [(float('inf'), float('-inf'))] * len(motors_to_calibrate)
        
        
        # Flag to stop calibration
        stop_calibration = threading.Event()
        cancelled = False
        
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
                print("\033[2J\033[H", end="")
                print("CALIBRATION MODE - Torque DISABLED")
                print("=" * 50)
                
                for i, p in enumerate(pos):
                    if hasattr(self, 'calibrated_limits') and self.calibrated_limits and i < len(self.calibrated_limits):
                        min_val, max_val = self.calibrated_limits[i]
                        n = (p - min_val) / (max_val - min_val) if max_val != min_val else 0.5
                    else:
                        n = p / 6.28
                    n = max(0, min(1, n))
                    bar = "█" * int(n * 20) + "░" * (20 - int(n * 20))
                    print(f"Motor {i+1:2d}: [{bar}] {p:6.3f} rad")
                
                print("=" * 50)
                print("Press Enter to save, Ctrl+C to cancel")
                
                # Update min/max for each motor
                for i, motor_id in enumerate(motors_to_calibrate):
                    motor_pos = pos[motor_id - 1]  # Convert to 0-based index
                    min_val, max_val = self.calibrated_limits[i]
                    self.calibrated_limits[i] = (min(motor_pos, min_val), max(motor_pos, max_val))
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n❌ Calibration cancelled - not saving")
            cancelled = True
        finally:
            # Re-enable torque
            self.dxl_client.set_torque_enabled(motors_to_calibrate, True)
            print("Torque re-enabled")
            
            # Save calibration only if not cancelled
            if not cancelled:
                print("\n✓ Calibration saved!")
                for i, motor_id in enumerate(motors_to_calibrate):
                    min_val, max_val = self.calibrated_limits[i]
                    print(f"Motor {motor_id}: {min_val:.3f} to {max_val:.3f} rad")
                
                # Save to permanent file
                self.save_calibration()
                print("Calibrated limits will be used by set_uiuc()")
    
    def read_pos(self):
        return self.dxl_client.read_pos()
def main(calibrate=False, **kwargs):
    uiuc_hand = UIUCNode()
    
    if calibrate:
        # Run calibration - don't set any initial position
        uiuc_hand = UIUCNode(calibration_mode=True)
        uiuc_hand.calibrate_motors()  # Will calibrate all 15 motors by default
        return
    
    while True:
        pos = uiuc_hand.read_pos()
        print("\033[2J\033[H", end="")
        print("UIUC Hand Position Monitor")
        print("=" * 50)
        
        for i, p in enumerate(pos):
            if hasattr(uiuc_hand, 'calibrated_limits') and uiuc_hand.calibrated_limits and i < len(uiuc_hand.calibrated_limits):
                min_val, max_val = uiuc_hand.calibrated_limits[i]
                n = (p - min_val) / (max_val - min_val) if max_val != min_val else 0.5
            else:
                n = p / 6.28
            n = max(0, min(1, n))
            bar = "█" * int(n * 20) + "░" * (20 - int(n * 20))
            print(f"Motor {i+1:2d}: [{bar}] {p:6.3f} rad")
        
        print("=" * 50)
        time.sleep(0.1)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='UIUC Hand Control')
    parser.add_argument('--calibrate', action='store_true', help='Run motor calibration')
    args = parser.parse_args()
    
    main(calibrate=args.calibrate)
