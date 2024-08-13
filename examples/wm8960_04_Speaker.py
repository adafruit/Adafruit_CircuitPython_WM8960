# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates analog audio input (on INPUT1s), sets volume control, and Speaker output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT1" inputs, they are labeled "RIN1" and "LIN1" on the board.

This example will pass your audio source through the mixers and gain stages of the codec using all of the analog bypass paths.

It will output the sound on the Speaker outputs.

You can now control the volume of the codecs built in class-d amp using this function:
codec.setSpeakerVolumeDB(6.00)
Valid inputs are -73.00 to 6.00 (1.00 dB steps)

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
SL+ --------- Left Speaker +
SL- --------- Left Speaker -
SR+ --------- Right Speaker +
SR- --------- Right Speaker -

*Note, with a class-d speaker amp like this, you need to connections like above.
Each speaker must be connected to its correct + and -.
You cannot connect the "-" side of the speaker to GND.
You cannot share the "-" side of two speakers (like with common "TRS-wired"
headphones).

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
'''

import board
import adafruit_wm8960

print("Example 4 - Speaker")

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow through the analog audio bypass connections
codec.enableMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectMN1()

# Disable mutes on PGA inputs (aka INPUT1)
codec.disableINMUTE()

# Set input boosts to get inputs 1 to the boost mixers
codec.setMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)
codec.connectMIC2B()

# Enable boost mixers
codec.enableAIN()

# Connect booster to output mixer (analog bypass)
codec.enableBoost2OutputMixer()

# Set gainstage between booster mixer and output mixer
codec.setBoost2MixerGain(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_0DB)

# Enable output mixers
codec.enableOutputMixer()

# Set up clock for 44.1KHz
codec.configureI2S(44100)

# Enable speakers at default volume of 0dB
codec.configureSpeakers()

print("Example complete. Listen to left/right INPUT1 on Speaker outputs.")
