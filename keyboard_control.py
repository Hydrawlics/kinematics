#!/usr/bin/env python3
"""
Hydrawlics Keyboard Control Script

Controls hydraulic joints via serial commands using keyboard input.

Keyboard Controls:
- Numbers 0-3: Select joint
- W/Up Arrow:   Extend selected joint
- S/Down Arrow: Retract selected joint
- Space:        Stop all joints
- Q/ESC:        Quit

Serial Commands Sent:
- E<n>: Extend valve n (0-3)
- R<n>: Retract valve n (0-3)
- X:    Stop all valves
"""

import serial
import serial.tools.list_ports
import sys
import termios
import tty
import select
import time

class HydraulicController:
    def __init__(self, port=None, baud=115200):
        """Initialize the hydraulic controller"""
        self.baud = baud
        self.serial = None
        self.selected_joint = 0
        self.running = False

        # Find and connect to serial port
        if port is None:
            port = self.find_arduino_port()

        if port:
            self.connect(port)
        else:
            print("No Arduino found. Please specify port manually.")
            sys.exit(1)

    def find_arduino_port(self):
        """Automatically find Arduino serial port"""
        ports = serial.tools.list_ports.comports()

        # Look for common Arduino identifiers
        for port in ports:
            if 'Arduino' in port.description or 'ttyACM' in port.device or 'ttyUSB' in port.device:
                print(f"Found Arduino on {port.device}: {port.description}")
                return port.device

        # If no Arduino found, list available ports
        if ports:
            print("Available serial ports:")
            for i, port in enumerate(ports):
                print(f"  [{i}] {port.device}: {port.description}")
            return None
        else:
            print("No serial ports found!")
            return None

    def connect(self, port):
        """Connect to the Arduino"""
        try:
            self.serial = serial.Serial(port, self.baud, timeout=1)
            print(f"Connected to {port} at {self.baud} baud")
            time.sleep(2)  # Wait for Arduino to reset

            # Read initial messages from Arduino
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"Arduino: {line}")

            self.running = True

        except serial.SerialException as e:
            print(f"Error connecting to {port}: {e}")
            sys.exit(1)

    def send_command(self, command):
        """Send command to Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.write(command.encode())
            self.serial.flush()
            print(f"Sent: {command}")

    def extend_joint(self, joint_num):
        """Send extend command for specific joint"""
        self.send_command(f'E{joint_num}')

    def retract_joint(self, joint_num):
        """Send retract command for specific joint"""
        self.send_command(f'R{joint_num}')

    def stop_all(self):
        """Stop all joints"""
        self.send_command('X')

    def select_joint(self, joint_num):
        """Select a joint (0-3)"""
        if 0 <= joint_num <= 3:
            self.selected_joint = joint_num
            print(f"Selected Joint {joint_num}")

    def read_response(self):
        """Read and print responses from Arduino"""
        while self.serial and self.serial.in_waiting:
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"Arduino: {line}")

    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.stop_all()
            time.sleep(0.1)
            self.serial.close()
            print("\nSerial connection closed")

def get_key():
    """Get a single keypress from stdin (non-blocking)"""
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None

def print_controls():
    """Print control instructions"""
    print("\n" + "="*60)
    print("HYDRAWLICS KEYBOARD CONTROL")
    print("="*60)
    print("\nControls:")
    print("  Numbers 0-3:      Select joint")
    print("  W / Up Arrow:     Extend selected joint")
    print("  S / Down Arrow:   Retract selected joint")
    print("  Space:            Stop all joints")
    print("  Q / ESC:          Quit")
    print("\nJoint Labels:")
    print("  0: Azimuth (base rotation)")
    print("  1: Base-arm (shoulder)")
    print("  2: Elbow")
    print("  3: End-effector (wrist)")
    print("="*60 + "\n")

def main():
    """Main control loop"""
    # Parse command line arguments
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    # Initialize controller
    controller = HydraulicController(port)

    # Print instructions
    print_controls()
    print(f"Currently selected: Joint {controller.selected_joint}\n")

    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        # Set terminal to raw mode
        tty.setcbreak(sys.stdin.fileno())

        print("Ready for input (press 'q' to quit)...\n")

        while controller.running:
            # Read responses from Arduino
            controller.read_response()

            # Get keyboard input
            key = get_key()

            if key:
                # Handle key press
                if key == 'q' or key == '\x1b':  # q or ESC
                    print("\nQuitting...")
                    controller.running = False

                elif key in ['0', '1', '2', '3']:
                    # Select joint
                    joint_num = int(key)
                    controller.select_joint(joint_num)

                elif key == 'w' or key == '\x1b[A':  # w or up arrow
                    print(f"Extending Joint {controller.selected_joint}")
                    controller.extend_joint(controller.selected_joint)

                elif key == 's' or key == '\x1b[B':  # s or down arrow
                    print(f"Retracting Joint {controller.selected_joint}")
                    controller.retract_joint(controller.selected_joint)

                elif key == ' ':  # space
                    print("STOPPING all joints")
                    controller.stop_all()

                elif key == '\x03':  # Ctrl+C
                    print("\nInterrupted!")
                    controller.running = False

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        controller.close()
        print("Goodbye!")

if __name__ == "__main__":
    main()