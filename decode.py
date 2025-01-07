#!/bin/env python3

from enum import Enum, IntEnum
import fileinput
import re

# -------------------------------------------------------------------------------------------
# Burst and pause length configuration functions
#
#   <   start  >< 0  ><    1     ><  intra-frame space  ><    1     >< 0  >< stop           >
#
#   |||||||     |||   |||         |||                    |||         |||   |||
#   |||||||     |||   |||         |||                    |||         |||   ||| 
#   |||||||     |||   |||         |||                    |||         |||   |||
# __|||||||_____|||___|||_________|||____________________|||_________|||___|||_______________
#
#   <-----><---><-><-><-><------->   <------------------>                  <-><-------------> 
#      |     |   |  |  |     |                 |                            |         |
#     9ms    | 0.7ms|0.7ms   |               19.8ms                       0.7ms       |
#          4.5ms  0.5ms    1.6ms                                                    >25ms
#
# Details of the NEC infrared protocol: https://www.sbprojects.net/knowledge/ir/nec.php
# -------------------------------------------------------------------------------------------

# Check for start pulse of a standard 8-byte NEC-like data frame
def is_long_start(hi: int, lo: int):
    # 9ms high, 4.5ms low
    return 8000 < hi and lo < -4000

# Check for start pulse of a short, 2-byte "I FEEL" data frame
def is_short_start(hi: int, lo: int):
    # 6ms high, 3ms low
    return 5000 < hi < 7000 and -3500 < lo < -2500

# Check for bit lead burst
def is_bit(hi: int):
    # 700us high (standard NEC protocol would be 560us, but median is around 700 in the GREE remote)
    return 600 < hi < 800

# Check for a "zero" bit encoding
def is_zero(hi: int, lo: int):
    # 700us high, 500us low pulse
    return is_bit(hi) and -600 < lo < -400

# Check for a "one" bit encoding
def is_one(hi: int, lo: int):
    # 700us high, 1.6ms low pulse
    return is_bit(hi) and -1700 < lo < -1500

# Check for intra-frame space
def is_space(hi: int, lo: int):
    # 700us high, 19.8ms low
    return is_bit(hi) and lo < -19000


# -------------------------------------------------------------------------------------------
# Clean and process ESPHome remote_receiver component logs
#
# Depending on your actual IR receiver hardware module, it might output active-low or active-high
# signal. Make sure that the raw output starts with a positive number. If not, you might need to
# set the receiver to use inverted input and optionally the internal pullup resistor.
#
# The GREE remotes transmit a 19-20ms intra-frame space (low signal) between the first 4 bytes and
# the second 4 bytes transmitted. Therefore, the default 10ms idle setting needs to be increased
# to detect the complete frames correctly.
#
# This function processes the output of the `raw` decoding.
#
# Example ESPHome configuration for TSOP4838 IR receiver connected to an ESP-32:
#
# remote_receiver:
#   pin:
#     number: GPIO22
#     inverted: true
#     mode:
#       input: true
#       pullup: true
#   idle: 25ms
#   dump:
#     - raw
#
# Documentation: https://esphome.io/components/remote_receiver
# -------------------------------------------------------------------------------------------

current_frame = []

# Process one line of the ESPHome log output
def next_frame(line: str) -> list[int]:
    # Skip everything in the logs that is not the remote_receiver raw dump
    if "[I][remote.raw" not in line:
        return None

    # We're interested in the IR pulse timings. The number is positive for high signal
    # (microseconds spent in high), and negative for low signal (microseconds spent in low).
    payload = line.split("]:")[1]
    timings = [int(i) for i in re.findall("-?\\d+", payload)]

    # The numbers span over multiple lines in the logs, but the "Received Raw:" string only
    # appears once on the beginning of a frame.
    # https://github.com/esphome/esphome/blob/dev/esphome/components/remote_base/raw_protocol.cpp#L12
    global current_frame
    if payload.lstrip().startswith("Received"):
        if len(current_frame) > 0:
            print("Error: extra bits:", current_frame)
        current_frame = timings
    else:
        current_frame += timings

    # We expect two kinds of frame: standard (with 70 bursts: high/low pairs plus a closing high)
    # and short (18 bursts). If we collected all, we can return the complete frame.
    if is_long_start(current_frame[0], current_frame[1]) and len(current_frame) >= 139:
        res = current_frame
        current_frame = []
        return res
    if is_short_start(current_frame[0], current_frame[1]) and len(current_frame) >= 35:
        res = current_frame
        current_frame = []
        return res
    return None


# -------------------------------------------------------------------------------------------
# Convert burst and pause timings into binary - sequence of bits and bytes
#
# Standard code frames largely follow the NEC protocol:
#  - Start bit
#  - 4 data bytes in LSB first bit order
#  - Stop bit
#
# But in contrast to NEC, the GREE remote control transmits 8 bytes of data in a single frame,
# so it inserts 4 extra synchronization bits in the between the two 4-byte payload parts. The
# sync bits are a 0101 sequence with the last 1 being extra long (19.8 ms instead of 1.6 ms).
#
#        0xCA                    0x35                     0xF0                    0x0F
#   <   byte 0    >         <   byte 4    > < sync > <   byte 5    >         <   byte 8    > 
# S 0 1 0 1 0 0 1 1   ...   1 0 1 0 1 1 0 0 0 1 0 1* 0 0 0 0 1 1 1 1   ...   1 1 1 1 0 0 0 0 $
# | |             |                               |                                          |
# | |    MSB (bit 7)                        extra long                                       |
# | LSB (bit 0)                         (intra-frame space)                                  |
# start bit                                                                           stop bit
# <-------------------------------- 70 bits (64 bit payload) -------------------------------->  
#
# The short code frames are much simpler. They begin with a shorter start burst and only
# contain payload bits for the 2 bytes (also LSB first), nothing else.
#
#        0xCA             0x0F
#   <   byte 0    > <   byte 1    > 
# s 0 1 0 1 0 0 1 1 1 1 1 1 0 0 0 0 $
# | |             |                 |
# | |    MSB (bit 7)                |
# | LSB (bit 0)                     |
# start bit                  stop bit
# <---- 18 bits (16 bit payload) --->  
#
# Check the climate_ir component of ESPHome how intra-frame sync is implemented:
# https://github.com/esphome/esphome/blob/dev/esphome/components/gree/gree.cpp#L88
# -------------------------------------------------------------------------------------------

# Convert a frame of IR burst timings into bit sequence
def next_code(timings: list[int]) -> str:
    # There is a closing high pulse to determine the length of the last low pulse
    if len(timings) % 2 == 0:
        print("Error: even bits:", timings)
        return None
    
    # Convert timings of high and low signal pairs into various "bits" (code elements)
    # S or s: start bit for standard or short frame
    # 0 or 1: data bit
    # _: intra-frame space
    # $: stop bit
    # x: ambiguous/error bit
    current_code = ""
    pairs = [(timings[i], timings[i+1]) for i in range(0, len(timings) - 1, 2)]
    for h, l in pairs:
        if is_long_start(h, l):
            current_code += "S"
        elif is_short_start(h, l):
            current_code += "s"
        elif is_zero(h, l):
            current_code += "0"
        elif is_one(h, l):
            current_code += "1"
        elif is_space(h, l):
            current_code += "_"
        else:
            current_code += "x"
    current_code += "$" if is_bit(timings[-1]) else "x"

    # Check code integrity and basic structure: correct number of start, space, error, stop and data bits
    if "x" in current_code:
        print("Error: invalid bit:", timings, current_code)
        return None
    
    if current_code[0] == "S" and "_" in current_code and current_code[-1] == "$" and len(current_code) == 70:
        return current_code
    if current_code[0] == "s" and "_" not in current_code and current_code[-1] == "$" and len(current_code) == 18:
        return current_code

    print("Error: invalid code structure: ", timings, current_code)
    return None


# Convert bit sequence into proper data bytes
def next_bytes(code: str) -> list[int]:
    # Remove start and stop bits, and intra-frame sync bits and space
    bits = code.lstrip("Ss").replace("010_", "").rstrip("$")
    if len(bits) % 8 != 0:
        print("Error: invalid code length: ", len(bits), code)
        return None
    
    # Convert the remaining payload data bits into bytes: bits are transmitted in LSB->MSB order.
    return [int(''.join(reversed(bits[i:i+8])), 2) for i in range(0, len(bits), 8)]


# -------------------------------------------------------------------------------------------
# Decode the payload of the GREE remote control frames.
#
# There are 4 kind of frames that the YAP1F remote sends:
# - Basic frame (standard): actively changes the state of the air conditioner by telling the
#   state of all possible values and functions.
# - Timer frame (standard): can only change the most important aspects of the air conditioner.
#   The main purpose of this frame is to set the time when the air conditioner should turn
#   on or off.
# - Footer frame (standard): it carries no information. It just signals the unit that there
#   are no more frames to process.
# - Temp frame (short): it relates to the so called "IFEEL" function. It tells the air
#   conditioner what is the current temperature as sensed by the remote control. The AC then
#   can use this as a reference room temperature instead of what is measures at the indoor unit.
#
# The standard frames are sent when triggered by a human pressing buttons on the remote control,
# while the Temp frame is sent after keypress and also automatically once every 15 minutes.
#
# These are the various combinations the frames are sent after each other:
# - Button press if no timer is set and IFEEL is inactive:
#   <Basic><Footer>
# - Button press if timer is set but IFEEL is inactive:
#   <Basic><Timer><Footer>
# - Button press if no timer is set but IFEEL is active:
#   <Basic><Footer><Temp>
# - Button press if timer is set and IFEEL is active:
#   <Basic><Timer><Footer><Temp>
# - Automatically if IFEEL is active:
#   <Temp>
#
# Sidenote: the remote control is stateful. It only sends commands if it makes sense in the
# current state, eg.:
# - No button press other than Timer, ON/OFF, and Light triggers sending frames if the AC is off
# - Setting the clock on the remote does not trigger sending frames if no timer is active
# - Functions that make no sense in a give operation mode does not trigger sending
# - etc.
# -------------------------------------------------------------------------------------------

# Helper class: an IntEnum variant that only prints the name of the enum value for short output
class ShortEnum(IntEnum):
    def __str__(self):
        return self.name
    
# Possible operation modes of the indoor unit
class Mode(ShortEnum):
    Auto = 0,
    Cool = 1,
    Dry = 2,
    Fan = 3,
    Heat = 4,

# Possible fan speeds
class Fan(ShortEnum):
    Auto = 0,
    Low = 1,
    Med = 2,
    High = 3,

# Possible horizontal (left-right) air guide (louver) states
class HGuide(ShortEnum):
    # Keep stationary as it is (or as it was last set)
    Closed = 0,

    # Stationary positions
    Left = 2,
    MidLeft = 3,
    Mid = 4,
    MidRight = 5,
    Right = 6,
    # Blow away from centre
    Out = 12,

    # Continuous swinging
    SwingLeftRight = 1,
    SwingInOut = 13,

# Possible vertical (up-down) air guide (louver) states
class VGuide(ShortEnum):
    # Keep stationary as it is (or as it was last set)
    Closed = 0,

    # Stationary positions
    Up = 2,
    MidUp = 3,
    Mid = 4,
    MidDown = 5,
    Down = 6,

    # Continuous swinging
    # Full-range swing
    SwingUpDown = 1,
    # Half-range swings
    SwingDown = 7,
    SwingMid = 9,
    SwingUp = 11,

# What temperature value is displayed on the indoor unit
class TempDisplay(ShortEnum):
    Default = 0,
    Set = 1,
    Room = 2,
    Outdoor = 3,


# Decode the first 4 bytes of the standard frames (Basic and Timer)
def decode_common_data(bytes: list[int]):
    if (bytes[3] & 0x02) != 0:
        print(f"Error: nonzero 3[1] func2-unused 0x{(bytes[3] & 0x02) >> 1:02X}")
    
    return {
        # Byte 0 - Basic functions

        # Sleep mode (silent operation and slight breeze)
        "sleep": (bytes[0] & 0x80) != 0,
        # Continuous swinging is active in one or more directions (see also `h_guide` and `v_guide`)
        "swing": (bytes[0] & 0x40) != 0,
        # Fan speed
        "fan": Fan((bytes[0] & 0x30) >> 4),
        # The unit is on or off
        "on": (bytes[0] & 0x08) != 0,
        # The operating mode
        "mode": Mode((bytes[0] & 0x07) >> 0),

        # Byte 1 - Temperature and & coarse timer

        # Timer active for on and/or off
        "timer": (bytes[1] & 0x80) != 0,
        # Hours left until the closest timed event (turning on or off), rounded to the nearest half hour
        "timer_hours": ((bytes[1] & 0x60) >> 5) * 10.0 + (bytes[2] & 0x0F) * 1.0 + ((bytes[1] & 0x10) >> 4) * 0.5,
        # Target temperature set in degrees celsius
        # If Fahrenheit scale is used (see `fahrenheit`), it can be set at 0.5C accuracy, otherwise it is 1.0C steps.
        "temp": (bytes[1] & 0x0F) + (16.5 if (bytes[3] & 0x04) != 0 else 16.0),

        # Byte 2 - Functions

        # Keep fan on to dry evaporator (only in Cool or Dry more) after turning AC off
        "x_fan": (bytes[2] & 0x80) != 0,
        # Turn on air ionizer
        "health": (bytes[2] & 0x40) != 0,
        # Turn on or off the LED light and display on the front of the indoor unit
        "light": (bytes[2] & 0x20) != 0,
        # Boost fan speed and performance to heat or cool faster
        "turbo": (bytes[2] & 0x10) != 0,

        # Byte 3 - More functions

        # Display and set in degrees Fahrenheit instead of Celsius
        "fahrenheit": (bytes[3] & 0x08) != 0,
        # Open fresh air valve to let outdoor air in the room
        "fresh_air": (bytes[3] & 0x01) != 0,
    }

# Decode the second 4 bytes of the Basic frame
def decode_basic_data(bytes: list[int]):
    if (bytes[5] & 0x80) == 0:
        print(f"Error: zero 5[7] func3-lead 0x{(bytes[5] & 0x80) >> 7:02X}")
    if (bytes[5] & 0x38) != 0:
        print(f"Error: nonzero 5[5:3] func3-unused 0x{(bytes[5] & 0x38) >> 3:02X}")
    if bytes[6] != 0:
        print(f"Error: nonzero 6[7:0] unused 0x{bytes[6]:02X}")
    if (bytes[7] & 0x0B) != 0:
        print(f"Error: nonzero 7[3,1:0] control-unused 0x{(bytes[5] & 0x0B) >> 0:02X}")
    
    res = {"type": "basic"}
    res.update(decode_common_data(bytes))
    res.update({
        # Byte 4 - Air guide settings

        # Horizontal (left-right) air guide state
        "h_guide": HGuide((bytes[4] & 0xF0) >> 4),
        # Vertical (up-down) air guide state
        "v_guide": VGuide((bytes[4] & 0x0F) >> 0),

        # Byte 5 - Even more functions
        "wifi": (bytes[5] & 0x40) != 0,
        "ifeel": (bytes[5] & 0x04) != 0,
        "temp_display": TempDisplay((bytes[5] & 0x03) >> 0),

        # Byte 6 - Unused

        # Byte 7 - Extra functions

        # Energy saving when in Cooling mode and Absence when in Heating mode
        "energy_save": (bytes[7] & 0x04) != 0,
    })

    if ("Swing" in res["h_guide"].name or "Swing" in res["v_guide"].name) == res["swing"]:
        del res["swing"]

    return res

# Decode the second 4 bytes of the Timer frame
def decode_timer_data(bytes: list[int]):
    if (bytes[5] & 0x08) == 0:
        print(f"Error: zero 5[3] on-lead 0x{(bytes[5] & 0x08) >> 3:02X}")
    if (bytes[7] & 0x0C) != 0:
        print(f"Error: nonzero 7[3:2] control-unused 0x{(bytes[7] & 0x0C) >> 2:02X}")

    res = {"type": "timer"}
    res.update(decode_common_data(bytes))
    res.update({
        # Byte 4 & 5 - Turn on time (minutes from now)
        "on_mins": ((bytes[5] & 0x07) << 8) | bytes[4] << 0,

        # This bit is set when the on and off times set by the user are too close to each other.
        # If this case, the remote sends slightly modified off time value to have at least 15 mins between the two.
        "overlap": (bytes[5] & 0x80) != 0,

        # Byte 5 & 6 - Turn off time (minutes from now)
        "off_mins": (bytes[6] << 4 | (bytes[5] & 0x70) >> 4),

        # Byte 7 - Timer status

        # The timer for turning on the AC is set
        "on_set": (bytes[7] & 0x02) != 0,
        # The timer for turning off the AC is set
        "off_set": (bytes[7] & 0x01) != 0,
    })
    return res

# Decode the Footer frame
def decode_footer_data(bytes: list[int]):
    # All bits should be zero (apart the type and checksum)
    if bytes[0] != 0 or bytes[1] != 0 or bytes[2] != 0 or bytes[4] != 0 or bytes[5] != 0 or bytes[6] != 0:
        print("Error: nonzero footer unused bytes", bytes)
    if (bytes[3] & 0x0F) != 0 or (bytes[7] & 0x0F) != 0:
        print("Error: nonzero unused bits", bytes)

    return {
        "type": "footer",

        # No meaningful data encoded
    }

# Decode the Temp frame
def decode_temp_data(bytes: list[int]):
    # The second byte is a fix magic byte (0xA5) instead of a checksum to detect transmit errors
    if bytes[1] != 0xA5:
        print("Error: invalid magic byte", bytes)

    return {
        "type": "temp",

        # Current temperature in degrees celsius (1.0C accuracy).
        "temp": bytes[0],
    }

# Calculate and compare checksum of the standard frames.
# All standard frames have a proper 4-bit checksum at bits 7..4 of byte 7 (the very last bits transmitted).
# The checksum is the modulo 16 sum of selected 4 bits from each of the 8 data bytes.
# This is a rudimental way to detect possible transmission errors.
#
# Original source: https://github.com/esphome/esphome/blob/dev/esphome/components/gree/gree.cpp#L62
# There was a bug in the original source in the ESPHome source code: byte 4 was not incorporated
# in the checksum. On the other hand, byte 7 was incorporeted twice (once with its current value
# and once as 0x0A). The checksum calculation shown here is the correct one, and it matches the
# value sent by the actual remote control.
def match_checksum(bytes: list[int]):
    # The checksum received
    orig = (bytes[7] & 0xF0) >> 4

    # The checksum recalculated from the payload
    calc = ((bytes[0] & 0x0F) + (bytes[1] & 0x0F) + (bytes[2] & 0x0F) + (bytes[3] & 0x0F) +
            ((bytes[4] & 0xF0) >> 4) + ((bytes[5] & 0xF0) >> 4) + ((bytes[6] & 0xF0) >> 4) + 0x0A) & 0x0F
    
    if orig != calc:
        print(f"Error: checksum mismatch 0x{orig:02X} != 0x{calc:02X}") 
    return orig == calc

# Decode the payload into actual air conditioner control values
def next_data(bytes: list[int]):
    if len(bytes) == 8:
        # All standard frames have proper checksum at bits 7..4 of byte 7
        match_checksum(bytes)

        # Bits 7..4 of byte 3 appears to encode the frame type
        if (bytes[3] & 0xF0) == 0x50:
            return decode_basic_data(bytes)
        if (bytes[3] & 0xF0) == 0x60:
            return decode_timer_data(bytes)
        if (bytes[3] & 0xF0) == 0xA0:
            return decode_footer_data(bytes)
    if len(bytes) == 2:
        return decode_temp_data(bytes)

    print("Error: unknown code type", bytes)
    return None


# -------------------------------------------------------------------------------------------
# Script entry point. The script expects an ESPHome log stream as in input and outputs the
# decoded remote command frames.
#
# Usage:
#
# For processing live log stream:
#
#   $ esphome logs irtest.yaml | python raw2bin.py
#
# For processing a saved log file:
#
#   $ python raw2bin.py log.txt
# -------------------------------------------------------------------------------------------

if __name__ == "__main__":
    for line in fileinput.input():
        frame = next_frame(line)
        if frame is None:
            continue
        
        code = next_code(frame)
        if code is None:
            continue
        
        bytes = next_bytes(code)
        if bytes is None:
            continue
        # print("[" + ", ".join([f"0x{b:02X}" for b in bytes]) + "]")

        data = next_data(bytes)
        if data is None:
            continue
        print("{" + ", ".join([f"{k} = {data[k]}" for k in data]) + "}")
