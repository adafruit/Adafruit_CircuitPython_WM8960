# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates analog audio input (on INPUT1s), sets volume control, and headphone output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT1" inputs, they are labeled "RIN1" and "LIN1" on the board.

This example will pass your audio source through the mixers and gain stages of the codec using all of the analog bypass paths.

It will output the sound on the headphone outputs. It is setup to do a capless headphone setup, so connect your headphones ground to "OUT3" and this provides a buffered VMID.

You can now control the volume of the codecs built in headphone buffers using this function:
codec.setHeadphoneVolumeDB(6.00)
Valid inputs are -74.00 (MUTE) up to +6.00, (1.00dB steps).

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
LINPUT1 ----- TRS INPUT TIP           *left audio
RINPUT1 ----- TRS INPUT RING1         *right audio

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
'''

import board
import adafruit_wm8960

print("Example 3 - INPUT1")

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow through the analog audio bypass connections
codec.enableLMIC()
codec.enableRMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectLMN1()
codec.connectRMN1()

# Disable mutes on PGA inputs (aka INTPUT1)
codec.disableLINMUTE()
codec.disableRINMUTE()

# Set input boosts to get inputs 1 to the boost mixers
codec.setLMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)
codec.setRMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)

codec.connectLMIC2B()
codec.connectRMIC2B()

# Enable boost mixers
codec.enableAINL()
codec.enableAINR()

# Connect LB2LO (booster to output mixer (analog bypass)
codec.enableLB2LO()
codec.enableRB2RO()

# Set gainstage between booster mixer and output mixer
codec.setLB2LOVOL(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_0DB)
codec.setRB2ROVOL(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_0DB)

# Enable output mixers
codec.enableLOMIX()
codec.enableROMIX()

codec.enableHeadphones()
codec.enableOUT3MIX() # Provides VMID as buffer for headphone ground

print("Volume set to +0dB")
codec.setHeadphoneVolumeDB(0.00)

print("Example complete. Listen to INPUT1 on headphone outputs.")
