#!/usr/bin/env python3
"""
Hydrawlics Keyboard Control Script (Curses Version)

Real-time keyboard control using curses (works great on Wayland/Linux!)
No external keyboard libraries needed - uses Python's built-in curses.

Installation:
    pip install pyserial

Usage:
    python keyboard_control_curses.py [serial_port]

Controls (press keys directly, no Enter needed):
- Numbers 0-3: Select joint
- W / Up:      Extend selected joint (hold to keep extending)
- S / Down:    Retract selected joint (hold to keep retracting)
- X / Space:   Stop all joints
- Q / ESC:     Quit
"""

import serial
import serial.tools.list_ports
import sys
import time
import curses
import threading

class HydraulicController:
    def __init__(self, port=None, baud=115200):
        """Initialize the hydraulic controller"""
        self.baud = baud
        self.serial = None
        self.selected_joint = 0
        self.running = True
        self.current_command = None
        self.log_messages = []
        self.max_log_lines = 10

        # Find and connect to serial port
        if port is None:
            port = self.find_arduino_port()

        if port:
            self.connect(port)
        else:
            raise Exception("No Arduino found")

    def find_arduino_port(self):
        """Automatically find Arduino serial port"""
        ports = serial.tools.list_ports.comports()

        # Look for common Arduino identifiers
        for port in ports:
            if 'Arduino' in port.description or 'ttyACM' in port.device or 'ttyUSB' in port.device:
                return port.device

        # If no Arduino found, list and ask
        if ports:
            print("\nAvailable serial ports:")
            for i, port in enumerate(ports):
                print(f"  [{i}] {port.device}: {port.description}")

            try:
                choice = int(input(f"\nSelect port [0-{len(ports)-1}]: "))
                if 0 <= choice < len(ports):
                    return ports[choice].device
            except (ValueError, IndexError, KeyboardInterrupt):
                pass

        return None

    def connect(self, port):
        """Connect to the Arduino"""
        try:
            self.serial = serial.Serial(port, self.baud, timeout=0.1)
            self.add_log(f"Connected to {port} at {self.baud} baud")
            time.sleep(2)  # Wait for Arduino to reset

            # Read initial messages
            time.sleep(0.5)
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.add_log(line)

        except serial.SerialException as e:
            raise Exception(f"Error connecting to {port}: {e}")

    def add_log(self, message):
        """Add a message to the log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_messages.append(f"[{timestamp}] {message}")
        if len(self.log_messages) > self.max_log_lines:
            self.log_messages.pop(0)

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

    def retract_joint(self, joint_num):
        """Send retract command for specific joint"""
        cmd = f'R{joint_num}'
        self.send_command(cmd)
        self.current_command = cmd

    def stop_all(self):
        """Stop all joints"""
        self.send_command('X')
        self.current_command = None

    def select_joint(self, joint_num):
        """Select a joint (0-3)"""
        if 0 <= joint_num <= 3:
            self.selected_joint = joint_num

    def read_response(self):
        """Read and log responses from Arduino"""
        try:
            while self.serial and self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.add_log(f"Arduino: {line}")
        except:
            pass

    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.stop_all()
            time.sleep(0.2)
            self.serial.close()

def draw_ui(stdscr, controller):
    """Draw the user interface"""
    stdscr.clear()
    height, width = stdscr.getmaxyx()

    joint_names = ["Azimuth (base)", "Base-arm (shoulder)", "Elbow", "End-effector (wrist)"]

    # Title
    title = "HYDRAWLICS KEYBOARD CONTROL"
    stdscr.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)

    # Controls
    y = 2
    stdscr.addstr(y, 2, "Controls:", curses.A_BOLD)
    y += 1
    stdscr.addstr(y, 4, "0-3: Select Joint  |  W/↑: Extend  |  S/↓: Retract  |  X/Space: Stop  |  Q/ESC: Quit")

    # Joint status
    y += 2
    stdscr.addstr(y, 2, "Joints:", curses.A_BOLD)
    y += 1

    for i in range(4):
        marker = "►" if i == controller.selected_joint else " "
        state = ""

        if controller.current_command:
            cmd_type, cmd_joint = controller.current_command[0], int(controller.current_command[1])
            if i == cmd_joint:
                if cmd_type == 'E':
                    state = " [EXTENDING]"
                elif cmd_type == 'R':
                    state = " [RETRACTING]"

        line = f"{marker} [{i}] {joint_names[i]}{state}"
        attr = curses.A_REVERSE if i == controller.selected_joint else curses.A_NORMAL
        stdscr.addstr(y, 4, line, attr)
        y += 1

    # Current command
    y += 1
    stdscr.addstr(y, 2, "Status:", curses.A_BOLD)
    y += 1

    if controller.current_command:
        cmd_type, cmd_joint = controller.current_command[0], int(controller.current_command[1])
        action = "EXTENDING" if cmd_type == 'E' else "RETRACTING"
        stdscr.addstr(y, 4, f"Active: {action} Joint {cmd_joint} ({joint_names[int(cmd_joint)]})", curses.A_BOLD)
    else:
        stdscr.addstr(y, 4, "Idle - All joints stopped")

    # Log messages
    y += 2
    stdscr.addstr(y, 2, "Log:", curses.A_BOLD)
    y += 1

    for msg in controller.log_messages[-8:]:  # Show last 8 messages
        if y < height - 1:
            stdscr.addstr(y, 4, msg[:width-6])
            y += 1

    stdscr.refresh()

def main_curses(stdscr, controller):
    """Main curses loop"""
    # Setup curses
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(1)   # Non-blocking input
    stdscr.timeout(50)  # 50ms timeout for getch()

    # Start background thread to read Arduino responses
    def read_loop():
        while controller.running:
            controller.read_response()
            time.sleep(0.05)

    reader_thread = threading.Thread(target=read_loop, daemon=True)
    reader_thread.start()

    controller.add_log("Ready! Use keyboard to control joints")

    last_key = None

    try:
        while controller.running:
            # Draw UI
            draw_ui(stdscr, controller)

            # Get key press
            try:
                key = stdscr.getch()
            except:
                key = -1

            if key == -1:
                continue

            # Convert to character
            try:
                if key == 27:  # ESC
                    controller.add_log("Quitting...")
                    controller.running = False
                    break
                elif key == curses.KEY_UP or key == ord('w') or key == ord('W'):
                    controller.extend_joint(controller.selected_joint)
                    controller.add_log(f"Extending joint {controller.selected_joint}")
                elif key == curses.KEY_DOWN or key == ord('s') or key == ord('S'):
                    controller.retract_joint(controller.selected_joint)
                    controller.add_log(f"Retracting joint {controller.selected_joint}")
                elif key == ord(' ') or key == ord('x') or key == ord('X'):
                    controller.stop_all()
                    controller.add_log("STOPPED all joints")
                elif key == ord('q') or key == ord('Q'):
                    controller.add_log("Quitting...")
                    controller.running = False
                    break
                elif ord('0') <= key <= ord('3'):
                    joint_num = key - ord('0')
                    controller.select_joint(joint_num)
                    controller.add_log(f"Selected joint {joint_num}")

            except Exception as e:
                controller.add_log(f"Error: {e}")

    except KeyboardInterrupt:
        controller.add_log("Interrupted!")

    finally:
        controller.running = False

def main():
    """Main entry point"""
    # Parse arguments
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    # Initialize controller (before curses so we can see error messages)
    try:
        print("Connecting to Arduino...")
        controller = HydraulicController(port)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Run curses interface
    try:
        curses.wrapper(main_curses, controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        print("\nConnection closed. Goodbye!\n")

if __name__ == "__main__":
    main()