# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

"""
Demonstrates how to use the automatic level control feature of the WM8960 Codec.

Attach a potentiomenter to GND/A0/3V3 to actively adjust the ALC target setting.

This example sets up the codec for analog audio input (on INPUT1s), ADC/DAC Loopback, sets hp volume, and Headphone output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT1" inputs, they are labeled "RIN1" and "LIN1" on the board.

This example will pass your audio source through the mixers and gain stages of the codec into the ADC. Turn on Loopback (so ADC is feed directly to DAC).
Then send the output of the DAC to the headphone outs.

We will use the user input via potentiometer on A0 to set the ALC target value. The ALC will adjust the gain of the pga input buffer to try and keep the signal level at the target.

HARDWARE CONNECTIONS

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)

**********************
MCU --------- POTENTIOMTER (aka blue little trimpot)
**********************
GND --------- "right-side pin"
A0 ---------- center pin            *aka center tap connection
3V3 --------- "left-side pin"

**********************
CODEC ------- AUDIO IN
**********************
GND --------- TRS INPUT SLEEVE        *ground for line level input
LINPUT1 ----- TRS INPUT TIP           *left audio
RINPUT1 ----- TRS INPUT RING1         *right audio

**********************
CODEC ------- AUDIO OUT
**********************
OUT3 -------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
HPL --------- TRS OUTPUT TIP             *left HP output
HPR --------- TRS OUTPUT RING1           *right HP output

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
"""

import board, time
from analogio import AnalogIn
from adafruit_simplemath import map_range
import adafruit_wm8960

analog_in = AnalogIn(board.A0)

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow to the ADC
codec.mic = True

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs
codec.mic_inverting_input = True

# Disable mutes on PGA inputs (aka INTPUT1)
codec.mic_mute = False

# Set input boosts to get inputs 1 to the boost mixers
codec.mic_boost_gain = adafruit_wm8960.MIC_BOOST_GAIN_0DB
codec.mic_boost = True
codec.input = True  # Enable boost mixers

# Disconnect LB2LO (booster to output mixer (analog bypass)
# For this example, we are going to pass audio throught the ADC and DAC
codec.mic_output = False

# Connect from DAC outputs to output mixer
codec.dac_output = True

# Set gainstage between booster mixer and output mixer
# For this loopback example, we are going to keep these as low as they go
codec.mic_output_volume = adafruit_wm8960.OUTPUT_VOLUME_MIN

# Enable output mixers
codec.output = True

# Setup clock and mode
codec.sample_rate = 44100
codec.master_mode = True
codec.gpio_output = True  # Note, should not be changed while ADC is enabled.

# Enable ADCs and DACs
codec.adc = codec.dac = True

# Loopback sends ADC data directly into DAC
codec.loopback = True

# Default is "soft mute" on, so we must disable mute to make channels active
codec.dac_mute = False

# Enable headphone amp output
codec.headphone = True
codec.headphone_volume = 0.0

# Enables capless mode using the VMID as buffer for headphone ground on OUT3
codec.mono_output = True

# Automatic Level control configuration

# Only allows mic gain stages at a "zero crossover" point in audio stream.
# Minimizes "zipper" noise when chaning gains.
codec.mic_zero_cross = True

codec.alc = True
codec.alc_target = -6.0
codec.alc_attack_time = 0.024
codec.alc_hold_time = 0.0
codec.alc_decay_time = 0.192
codec.alc_max_gain = adafruit_wm8960.ALC_MAX_GAIN_MAX
codec.alc_min_gain = adafruit_wm8960.ALC_MIN_GAIN_MIN

while True:
    codec.alc_target = map_range(
        analog_in.value,
        0,
        65536,
        adafruit_wm8960.ALC_TARGET_MIN,
        adafruit_wm8960.ALC_TARGET_MAX,
    )
    time.sleep(1.0)