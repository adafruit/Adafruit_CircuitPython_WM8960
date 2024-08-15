# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
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
'''

import board, time
from analogio import AnalogIn
from adafruit_simplemath import map_range
import adafruit_wm8960

print("Example 12 - Automatic Level Control")

# Used to store incoming potentiometer settings to set ADC digital volume setting
userInputA0 = 0
analog_in = AnalogIn(board.A0)

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow to the ADC
codec.enableMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectMN1()

# Disable mutes on PGA inputs (aka INTPUT1)
codec.disableINMUTE()

# Set input boosts to get inputs 1 to the boost mixers
codec.setMICBOOST(adafruit_wm8960.MIC_BOOST_GAIN_0DB)
codec.connectMIC2B()

# Enable boost mixers
codec.enableAIN()

# Disconnect LB2LO (booster to output mixer (analog bypass)
# For this example, we are going to pass audio throught the ADC and DAC
codec.disableB2O()

# Connect from DAC outputs to output mixer
codec.enableD2O()

# Set gainstage between booster mixer and output mixer
# For this loopback example, we are going to keep these as low as they go
codec.setB2OVOL(adafruit_wm8960.OUTPUT_MIXER_GAIN_NEG_21DB)

# Enable output mixers
codec.enableOMIX()

# Setup clock and mode
codec.setSampleRate(44100)
codec.enableMasterMode()
codec.setALRCGPIO() # Note, should not be changed while ADC is enabled.

# Enable ADCs and DACs
codec.enableAdc()
codec.enableDac()

# Loopback sends ADC data directly into DAC
codec.enableLoopBack()

# Default is "soft mute" on, so we must disable mute to make channels active
codec.disableDacMute()

print("Headphopne Amp Volume set to +0dB")
codec.configureHeadphones(dB=0.0, capless=True) # Capless provides VMID as buffer for headphone ground

# Automatic Level control stuff

# Only allows pga gain stages at a "zero crossover" point in audio stream.
# Minimizes "zipper" noise when chaning gains.
codec.enablePgaZeroCross()

codec.enableAlc(adafruit_wm8960.ALC_MODE_STEREO)
codec.setAlcTarget(adafruit_wm8960.ALC_TARGET_LEVEL_NEG_6DB)
codec.setAlcDecay(adafruit_wm8960.ALC_DECAY_TIME_192MS)
codec.setAlcAttack(adafruit_wm8960.ALC_ATTACK_TIME_24MS)
codec.setAlcMaxGain(adafruit_wm8960.ALC_MAX_GAIN_LEVEL_30DB)
codec.setAlcMinGain(adafruit_wm8960.ALC_MIN_GAIN_LEVEL_NEG_17_25DB)
codec.setAlcHold(adafruit_wm8960.ALC_HOLD_TIME_0MS)

print("Codec setup complete. Listen to left/right INPUT1 on Headphone outputs.")

while True:
    # Take a bunch of readings and average them, to smooth out the value
    for i in range(250):
        userInputA0 += analog_in.value
        time.sleep(0.001)
    userInputA0 /= 250.0

    # Map it from 0-4096, to a value that is acceptable for the setting
    alcTarget = map_range(userInputA0, 0.0, 65536.0, 15.0, 0.0)

    print("alcTarget: ", alcTarget)

    codec.setAlcTarget(alcTarget) # Valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS

    time.sleep(1.0)
