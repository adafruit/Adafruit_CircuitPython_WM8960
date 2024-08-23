# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

"""
Demonstrates analog audio input (on INPUT2s), sets volume control, and headphone output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT2" inputs, they are labeled "RIN2" and "LIN2" on the board.

This example will pass your audio source through the mixers and gain stages of the codec using all of the analog bypass paths.

It will output the sound on the headphone outputs.
It is setup to do a capless headphone setup, so connect your headphones ground to "OUT3" and this provides a buffered VMID.

You can now control the volume of the codecs built in headphone buffers using this function:
codec.setHeadphoneVolumeDB(6.00); Valid inputs are -74.00 (MUTE) up to +6.00, (1.00dB steps).

HARDWARE CONNECTIONS

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)

**********************
CODEC ------- AUDIO IN
**********************
GND --------- TRS INPUT SLEEVE        *ground for line level input
LINPUT2 ----- TRS INPUT TIP           *left audio
RINPUT2 ----- TRS INPUT RING1         *right audio

**********************
CODEC -------- AUDIO OUT
**********************
OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
HPL ---------- TRS OUTPUT TIP             *left HP output
HPR ---------- TRS OUTPUT RING1           *right HP output

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
"""

import board
import adafruit_wm8960

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow through the analog audio bypass connections

# INPUT1 must pass through the Mic Boost
codec.mic = True
codec.mic_inverting_input = True
codec.mic_input = adafruit_wm8960.MIC_VMID  # Non-inverting input
codec.mic_mute = False
codec.mic_boost_gain = 0.0
codec.mic_boost = True
codec.mic_volume = 0.0

# In order to use INPUT2 or INPUT3 with the Mic Boost (PGA)
# codec.mic_input = adafruit_wm8960.MIC_INPUT2
# codec.mic_input = adafruit_wm8960.MIC_INPUT3
# If codec.mic_inverting_input is enabled, mic boost uses input as balanced input between INPUT1 & INPUT2/3

# Set input boosts to get INPUT2 or INPUT3 (both left and right) to the boost mixers and bypass the Mic Boost (PGA)
codec.input2_boost = 0.0
codec.input3_boost = 0.0

# Enable input boost mixers
codec.input = True

# Connect LB2LO (booster to output mixer [aka analog bypass])
codec.mic_output = True

# Set gainstage between boost mixer and output mixers (analog bypass)
codec.mic_output_volume = 0.0

# Enable output mixers
codec.output = True

# Enable Headphone Amp with OUT3 as capless buffer for headphone ground
codec.headphone = True
codec.mono_output = True
codec.headphone_volume = 0.0
