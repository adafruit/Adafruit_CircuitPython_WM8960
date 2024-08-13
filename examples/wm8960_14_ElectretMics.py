# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates electret microphone analog audio input (on INPUT1/INPUT2 as "pseudo-differential MIC configuration"). Sets the PGA gain, sets the non-inverting pga input to INPUT2s, sets volume control, and headphone output on the WM8960 Codec.

Note, most of the examples in this library set all gain stages at 0dB, but for this electret mic example, we need a bit more gain on the initial input stage (the PGA), so we set it to +24dB (aka "57" as the argument to the function).
We are also bumping up the headphone output gainstage to max, +6dB (aka "127").

Electret mics should be connected to left and right inputs.

MicBias should be provided to the "+" side of each mic via an in-series 2.2K resistor.

The Mic "-" should be connected to each INPUT2. Note, this is also GND in this example, but enables a "psuedo-differential setup".

This example will pass your audio source through the mixers and gain stages of the codec using all of the analog bypass paths.

It will output the sound on the headphone outputs. It is setup to do a capless headphone setup, so connet your headphones ground to "OUT3" and this provides a buffered VMID.

You can now control the volume of the codecs built in headphone buffers using this function:
codec.setHeadphoneVolumeDB(6.00)
Valid inputs are -74.00 (MUTE) up to +6.00, (1.00dB steps).

HARDWARE CONNECTIONS
See Datasheet page 25 for electret mic hardware hookup example diagrams.
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)

**********************
CODEC ------- AUDIO IN (ELECTRET MICS)
**********************
GND --------- LEFT MIC -
LINPUT1 ----- LEFT MIC -
LINPUT2 ----- LEFT MIC +
MICBIAS ----- 2.2K RESISTOR ----- LEFT MIC+
GND --------- RIGHT MIC -
RINPUT1 ----- RIGHT MIC -
RINPUT2 ----- RIGHT MIC +
MICBIAS ----- 2.2K RESISTOR ----- RIGHT MIC+

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

print("Example 14 - Electret Mics")

codec = adafruit_wm8960.WM8960(board.I2C())

codec.enableMicBias()

# WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or
# WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
codec.setMicBiasVoltage(adafruit_wm8960.WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD)
print("Mic Bias enabled (0.9*AVDD)")

# Setup signal flow through the analog audio bypass connections
codec.enableLMIC()
codec.enableRMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectLMN1()
codec.connectRMN1()

# Disable mutes on PGA inputs (aka INTPUT1)
codec.disableLINMUTE()
codec.disableRINMUTE()

# Set pga volumes
codec.setLINVOLDB(24.00) # Valid options are -17.25dB to +30.00dB
codec.setRINVOLDB(24.00) # Valid options are -17.25dB to +30.00dB
print("PGA gain set to +24dB")

# Set input boosts to get inputs 1 to the boost mixers
codec.setLMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)
codec.setRMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)
print("Mic boost stage set to 0dB")

# For MIC+ signal of differential mic signal
codec.pgaLeftNonInvSignalSelect(adafruit_wm8960.WM8960_PGAL_LINPUT2)

# For MIC+ signal of differential mic signal
codec.pgaRightNonInvSignalSelect(adafruit_wm8960.WM8960_PGAR_RINPUT2)
print("Pga non-inverting inputs set to INPUT2s")

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

print("Headphone output buffer volume set to +6dB (max)")
codec.setHeadphoneVolumeDB(6.00)

print("Example complete. Listen to Electret mics on headphone outputs.")
