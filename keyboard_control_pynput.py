#!/usr/bin/env python3
"""
Hydrawlics Keyboard Control Script (pynput version)

Controls hydraulic joints via serial commands using keyboard input.
This version uses pynput library for easier keyboard handling.

Installation:
    pip install pyserial pynput

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
import time
from pynput import keyboard

class HydraulicController:
    def __init__(self, port=None, baud=115200):
        """Initialize the hydraulic controller"""
        self.baud = baud
        self.serial = None
        self.selected_joint = 0
        self.running = True
        self.current_command = None  # Track currently active command

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
            print("\nAvailable serial ports:")
            for i, port in enumerate(ports):
                print(f"  [{i}] {port.device}: {port.description}")

            # Ask user to select
            try:
                choice = int(input(f"\nSelect port [0-{len(ports)-1}]: "))
                if 0 <= choice < len(ports):
                    return ports[choice].device
            except (ValueError, IndexError):
                pass

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

        except serial.SerialException as e:
            print(f"Error connecting to {port}: {e}")
            sys.exit(1)

    def send_command(self, command):
        """Send command to Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.write(command.encode())
            self.serial.flush()

    def extend_joint(self, joint_num):
        """Send extend command for specific joint"""
        cmd = f'E{joint_num}'
        self.send_command(cmd)
        self.current_command = cmd
        print(f"→ EXTEND Joint {joint_num}")

    def retract_joint(self, joint_num):
        """Send retract command for specific joint"""
        cmd = f'R{joint_num}'
        self.send_command(cmd)
        self.current_command = cmd
        print(f"← RETRACT Joint {joint_num}")

    def stop_all(self):
        """Stop all joints"""
        self.send_command('X')
        self.current_command = None
        print("⏹ STOP all joints")

    def select_joint(self, joint_num):
        """Select a joint (0-3)"""
        if 0 <= joint_num <= 3:
            self.selected_joint = joint_num
            joint_names = ["Azimuth", "Base-arm", "Elbow", "End-effector"]
            print(f"\n✓ Selected: Joint {joint_num} ({joint_names[joint_num]})")

    def read_response(self):
        """Read and print responses from Arduino"""
        while self.serial and self.serial.in_waiting:
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if line and not line.startswith("EXTEND") and not line.startswith("RETRACT") and not line.startswith("STOP"):
                print(f"  Arduino: {line}")

    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.stop_all()
            time.sleep(0.1)
            self.serial.close()
            print("\n✓ Serial connection closed")

def print_controls():
    """Print control instructions"""
    print("\n" + "="*60)
    print(" "*15 + "HYDRAWLICS KEYBOARD CONTROL")
    print("="*60)
    print("\n📋 Controls:")
    print("  0, 1, 2, 3:       Select joint")
    print("  W / ↑:            Extend selected joint")
    print("  S / ↓:            Retract selected joint")
    print("  Space:            Stop all joints")
    print("  Q / ESC:          Quit")
    print("\n🔧 Joint Labels:")
    print("  [0] Azimuth       - Base rotation")
    print("  [1] Base-arm      - Shoulder")
    print("  [2] Elbow         - Elbow")
    print("  [3] End-effector  - Wrist")
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
    print(f"Current selection: Joint {controller.selected_joint}\n")
    print("Ready! Press keys to control the arm...\n")

    def on_press(key):
        """Handle key press events"""
        try:
            # Number keys to select joints
            if hasattr(key, 'char') and key.char in ['0', '1', '2', '3']:
                joint_num = int(key.char)
                controller.select_joint(joint_num)

            # W or Up arrow to extend
            elif (hasattr(key, 'char') and key.char == 'w') or key == keyboard.Key.up:
                controller.extend_joint(controller.selected_joint)

            # S or Down arrow to retract
            elif (hasattr(key, 'char') and key.char == 's') or key == keyboard.Key.down:
                controller.retract_joint(controller.selected_joint)

            # Space to stop
            elif key == keyboard.Key.space:
                controller.stop_all()

            # Q or ESC to quit
            elif (hasattr(key, 'char') and key.char == 'q') or key == keyboard.Key.esc:
                print("\n👋 Quitting...")
                controller.running = False
                return False  # Stop listener

        except AttributeError:
            pass

    def on_release(key):
        """Handle key release events - auto-stop when key released"""
        try:
            # Auto-stop when extend/retract keys are released
            if (hasattr(key, 'char') and key.char in ['w', 's']) or key in [keyboard.Key.up, keyboard.Key.down]:
                if controller.current_command:
                    controller.stop_all()

        except AttributeError:
            pass

    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        # Main loop - just read Arduino responses
        while controller.running:
            controller.read_response()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n⚠ Keyboard interrupt received")

    finally:
        listener.stop()
        controller.close()
        print("Goodbye! 👋")

if __name__ == "__main__":
    main()