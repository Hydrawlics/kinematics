# Hydraulics Keyboard Control

This system allows you to control hydraulic joints via keyboard input through serial commands.

## System Overview

The system consists of two parts:
1. **Arduino Code** (`main_button_test.cpp`) - Listens for serial commands and controls valves
2. **Python Script** - Captures keyboard input and sends serial commands to Arduino

## Arduino Setup

### Serial Command Protocol

The Arduino firmware accepts the following commands:

| Command | Description |
|---------|-------------|
| `E<n>` | Extend valve n (0-3) |
| `R<n>` | Retract valve n (0-3) |
| `S<n>` | Select valve n (for SELECT_BTN_MODE) |
| `X` | Stop all valves |

### Operating Modes

The Arduino code supports two modes (configured via `#define SELECT_BTN_MODE`):

#### 1. DIRECT Mode (Default)
- Control each valve independently
- Commands require valve number: `E0`, `R2`, etc.
- Example: `E0` extends joint 0, `R3` retracts joint 3

#### 2. SELECT Mode
- Select one joint, then extend/retract it
- Commands: `S<n>` to select, `E` to extend, `R` to retract
- Example: `S2` selects joint 2, then `E` extends it

### Upload to Arduino

1. Make sure `platformio.ini` is configured for the correct environment
2. Upload the code:
   ```bash
   pio run -e button-test -t upload
   ```

## Python Script Setup

Multiple versions are provided for different systems and preferences:

### ⭐ RECOMMENDED for Linux/Wayland: Curses Version (keyboard_control_curses.py)

Uses Python's built-in curses library. Works perfectly on Wayland/Linux without permissions issues!

**Install dependencies:**
```bash
pip install pyserial
```

**Run:**
```bash
python keyboard_control_curses.py [serial_port]
```

**Features:**
- ✅ Works on Wayland (no X11/display server permissions needed)
- Real-time keyboard input (no need to press Enter)
- Nice TUI with live status display
- Shows current joint states and log messages
- Auto-stops when you release keys

### Option 2: Simple Terminal Version (keyboard_control_simple.py)

Basic command-line interface - type commands and press Enter.

**Install dependencies:**
```bash
pip install pyserial
```

**Run:**
```bash
python keyboard_control_simple.py [serial_port]
```

**Features:**
- Works everywhere (most compatible)
- Simple text-based interface
- Type 'w' + Enter to extend, 's' + Enter to retract

### Option 3: Pynput Version (keyboard_control_pynput.py)

Uses pynput library. **Note:** May not work on Wayland without X11 compatibility.

**Install dependencies:**
```bash
pip install pyserial pynput
```

**Run:**
```bash
python keyboard_control_pynput.py [serial_port]
```

**Features:**
- Works well on X11/Windows/macOS
- Auto-stop on key release
- May require X11 on Wayland systems

### Option 4: Standard Version (keyboard_control.py)

Basic terminal input with manual key reading. May have issues on some systems.

**Run:**
```bash
python keyboard_control.py [serial_port]
```

## Keyboard Controls

Both scripts use the same keyboard layout:

| Key | Action |
|-----|--------|
| `0`, `1`, `2`, `3` | Select joint |
| `W` or `↑` | Extend selected joint |
| `S` or `↓` | Retract selected joint |
| `Space` | Stop all joints |
| `Q` or `ESC` | Quit program |

## Joint Mapping

The joints are numbered as follows:

| Number | Joint | Description |
|--------|-------|-------------|
| 0 | Azimuth | Base rotation |
| 1 | Base-arm | Shoulder joint |
| 2 | Elbow | Elbow joint |
| 3 | End-effector | Wrist joint |

## Usage Example

1. **Upload Arduino code** to your board
2. **Connect Arduino** via USB
3. **Run Python script**:
   ```bash
   python keyboard_control_pynput.py
   ```
4. **Control the arm**:
   - Press `1` to select the base-arm joint
   - Press and hold `W` to extend it
   - Release `W` to stop (pynput version only)
   - Press `2` to select elbow
   - Press and hold `S` to retract it
   - Press `Space` to emergency stop all joints

## Troubleshooting

### Serial Port Not Found

If the script can't find the Arduino:

1. **List available ports:**
   ```bash
   python -m serial.tools.list_ports
   ```

2. **Manually specify port:**
   ```bash
   python keyboard_control.py /dev/ttyACM0  # Your port here
   ```

3. **Check permissions (Linux):**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

### Arduino Not Responding

1. Open Arduino IDE Serial Monitor to check if Arduino is receiving commands
2. Verify baud rate is 115200
3. Try unplugging and reconnecting the Arduino
4. Check that the correct code is uploaded

### Keyboard Input Not Working

**On Wayland (Linux/Gnome):**
- **Use the curses version** (`keyboard_control_curses.py`) - it works perfectly on Wayland!
- The pynput version may not work due to Wayland's security restrictions
- Alternatively, use the simple version with Enter-based input

**General fixes:**
- Make sure the terminal window has focus
- Check that your terminal supports the required features
- The curses version is most compatible across Linux systems

## Advanced Configuration

### Change Baud Rate

In Arduino code:
```cpp
Serial.begin(115200);  // Change this value
```

In Python script:
```python
controller = HydraulicController(port, baud=115200)  # Match Arduino
```

### Add Custom Commands

Modify `processSerialCommands()` in `main_button_test.cpp`:
```cpp
else if (cmd == 'C' || cmd == 'c') {
    // Your custom command here
}
```

### Auto-Detect and Auto-Reconnect

Both scripts support auto-detection of Arduino serial ports. If connection is lost, restart the script to reconnect.

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **Always have the STOP command ready** - Press Space to stop all joints immediately
2. **Be aware of mechanical limits** - Don't over-extend or over-retract joints
3. **Monitor the system** - Watch for unusual sounds or movements
4. **Emergency shutoff** - Have a physical emergency stop if available
5. **Test in stages** - Test each joint individually before complex movements

## Contributing

Found a bug or want to add features? Feel free to modify the code and submit improvements!

## License

Part of the Hydrawlics project. See main project README for license information.