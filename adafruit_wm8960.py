# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_wm8960`
================================================================================

Barebones CircuitPython driver for WM8960 Stereo CODEC


* Author(s): Scott Shawcroft

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from adafruit_bus_device import i2c_device

import time

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_WM8960.git"


class WM8960:
    def __init__(self, i2c_bus, address=0x1a):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._buf = bytearray(2)

    def _write_reg(self, reg, value):
        self._buf[0] = reg << 1 | value >> 8
        self._buf[1] = value & 0xff
        with self.i2c_device as i2c:
            i2c.write(self._buf)

    def start_i2s_out(self):
        # playback only 
        self._write_reg(0xf, 0) # Reset
        self._write_reg(0x7, 0x2) # 16-bit I2S format
        self._write_reg(0x4, 0x0) # SYSCLK = MCLK 11.2 mhz

        self._write_reg(0xa, 0x1ff) # Left DAC volume
        self._write_reg(0xb, 0x1ff) # Right DAC volume

        self._write_reg(0x2, 0x079) # Left headphone volume
        self._write_reg(0x3, 0x179) # Right headphone volume

        self._write_reg(0x19, 0xc0) # 2 x 50k divider enabled and enable vref
        time.sleep(0.1)

        self._write_reg(0x1a, 0x180) # DAC L + R

        self._write_reg(0x22, 0x100) # Left DAC to Left Mixer
        self._write_reg(0x25, 0x100) # Right DAC to Right Mixer

        self._write_reg(0x2f, 0xc) # L + R Mixer output

        self._write_reg(0x1a, 0x1e0) # DACL, DACR, LOUT1, ROUT1

        # Unmute DAC
        self._write_reg(0x5, 0)
