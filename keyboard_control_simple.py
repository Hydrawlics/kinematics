#!/usr/bin/env python3
"""
Hydrawlics Keyboard Control Script (Simple Terminal Version)

Works on all systems including Wayland/Linux by using simple terminal input.
No special keyboard libraries required - just pyserial.

Installation:
    pip install pyserial

Controls:
- Type joint number (0-3) and press Enter to select
- Type 'w' and press Enter to extend selected joint
- Type 's' and press Enter to retract selected joint
- Type 'x' and press Enter to stop all joints
- Type 'q' and press Enter to quit

For continuous control, just hold Enter after typing the command!
"""

import serial
import serial.tools.list_ports
import sys
import time
import threading

class HydraulicController:
    def __init__(self, port=None, baud=115200):
        """Initialize the hydraulic controller"""
        self.baud = baud
        self.serial = None
        self.selected_joint = 0
        self.running = True
        self.current_command = None

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
                choice = input(f"\nSelect port [0-{len(ports)-1}]: ")
                choice = int(choice)
                if 0 <= choice < len(ports):
                    return ports[choice].device
            except (ValueError, IndexError, KeyboardInterrupt):
                pass

        return None

    def connect(self, port):
        """Connect to the Arduino"""
        try:
            self.serial = serial.Serial(port, self.baud, timeout=0.1)
            print(f"Connected to {port} at {self.baud} baud")
            time.sleep(2)  # Wait for Arduino to reset

            # Read initial messages from Arduino
            time.sleep(0.5)
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"  {line}")

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
        print(f"  → EXTENDING Joint {joint_num}")

    def retract_joint(self, joint_num):
        """Send retract command for specific joint"""
        cmd = f'R{joint_num}'
        self.send_command(cmd)
        self.current_command = cmd
        print(f"  ← RETRACTING Joint {joint_num}")

    def stop_all(self):
        """Stop all joints"""
        self.send_command('X')
        self.current_command = None
        print("  ⏹ STOPPED all joints")

    def select_joint(self, joint_num):
        """Select a joint (0-3)"""
        if 0 <= joint_num <= 3:
            self.selected_joint = joint_num
            joint_names = ["Azimuth (base)", "Base-arm (shoulder)", "Elbow", "End-effector (wrist)"]
            print(f"\n✓ Selected: Joint {joint_num} - {joint_names[joint_num]}")

    def read_response(self):
        """Read and print responses from Arduino"""
        try:
            while self.serial and self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line and not any(x in line for x in ["EXTEND", "RETRACT", "STOP", "SELECTED"]):
                    print(f"  Arduino: {line}")
        except:
            pass

    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.stop_all()
            time.sleep(0.2)
            self.serial.close()
            print("\n✓ Connection closed")

def print_controls():
    """Print control instructions"""
    print("\n" + "="*70)
    print(" "*20 + "HYDRAWLICS KEYBOARD CONTROL")
    print("="*70)
    print("\n📋 Quick Reference:")
    print("  0, 1, 2, 3  → Select joint")
    print("  w           → Extend selected joint")
    print("  s           → Retract selected joint")
    print("  x           → Stop all joints")
    print("  q           → Quit")
    print("\n💡 Tip: Commands take effect when you press Enter")
    print("\n🔧 Joints:")
    print("  [0] Azimuth       - Base rotation")
    print("  [1] Base-arm      - Shoulder")
    print("  [2] Elbow         - Elbow joint")
    print("  [3] End-effector  - Wrist")
    print("="*70 + "\n")

def main():
    """Main control loop"""
    # Parse command line arguments
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    # Initialize controller
    try:
        controller = HydraulicController(port)
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(0)

    # Start background thread to read Arduino responses
    def read_loop():
        while controller.running:
            controller.read_response()
            time.sleep(0.05)

    reader_thread = threading.Thread(target=read_loop, daemon=True)
    reader_thread.start()

    # Print instructions
    print_controls()
    print(f"Current selection: Joint {controller.selected_joint}\n")
    print("Ready! Type commands and press Enter...\n")

    try:
        while controller.running:
            # Show prompt with current selection
            joint_names = ["Azimuth", "Base", "Elbow", "Wrist"]
            prompt = f"[J{controller.selected_joint}:{joint_names[controller.selected_joint]}] > "

            try:
                cmd = input(prompt).strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            if not cmd:
                continue

            # Process command
            if cmd == 'q' or cmd == 'quit' or cmd == 'exit':
                print("Quitting...")
                break

            elif cmd in ['0', '1', '2', '3']:
                controller.select_joint(int(cmd))

            elif cmd == 'w' or cmd == 'up':
                controller.extend_joint(controller.selected_joint)

            elif cmd == 's' or cmd == 'down':
                controller.retract_joint(controller.selected_joint)

            elif cmd == 'x' or cmd == 'stop':
                controller.stop_all()

            elif cmd == 'h' or cmd == 'help':
                print_controls()

            else:
                print(f"  ⚠ Unknown command: '{cmd}' (type 'h' for help)")

    except KeyboardInterrupt:
        print("\n\nInterrupted!")

    finally:
        controller.running = False
        time.sleep(0.1)
        controller.close()
        print("Goodbye!\n")

if __name__ == "__main__":
    main()