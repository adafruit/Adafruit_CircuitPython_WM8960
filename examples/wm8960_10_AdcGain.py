# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates how to control the volume using the codec's ADC digital volume control.

Attach a potentiomenter to GND/A0/3V3 to actively adjust the setting.

This example sets up the codec for analog audio input (on INPUT1s), ADC/DAC Loopback, sets hp volume, and Headphone output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT1" inputs, they are labeled "RIN1" and "LIN1" on the board.

This example will pass your audio source through the mixers and gain stages of the codec into the ADC. Turn on Loopback (so ADC is feed directly to DAC).
Then send the output of the DAC to the headphone outs.

We will use the gain stage at the ADC to control the volume of the signal.
This is capable of more precision, with 255 available settings.

** ADC digital volume
** Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
** -97.50 (or lower) = MUTE
** -97.00 = -97.00dB (MIN)
** ... 0.5dB steps ...
** 30.00 = +30.00dB  (MAX)

You can also control the volume of the codecs built in headphone amp using this function:
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
CODEC -------- AUDIO OUT
**********************
OUT3 --------- TRS OUTPUT SLEEVE
HPL ---------- TRS OUTPUT TIP             *left HP output
HPR ---------- TRS OUTPUT RING1           *right HP output

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
'''

import board, time
from analogio import AnalogIn
from adafruit_simplemath import map_range
import adafruit_wm8960

# Used to store incoming potentiometer settings to set ADC digital volume setting
userInputA0 = 0
analog_in = AnalogIn(board.A0)

print("Example 10 - ADC Digital Volume Control")

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

print("Headphone Amp Volume set to +0dB")
codec.configureHeadphones(dB=0.0, capless=True) # Capless provides VMID as buffer for headphone ground

print("Codec setup complete. Listen to left/right INPUT1 on Headphone outputs.")

while True:
    # Take a bunch of readings and average them, to smooth out the value
    for i in range(250):
        userInputA0 += analog_in.value
        time.sleep(0.001)

    # After taking a bunch of samples, divide down to the average single reading
    userInputA0 /= 250.0

    # Map it from 0-4096, to a dB value that is acceptable in the ADC digital volume control (-97.50 [MUTE] to +30dB)
    adcVolumeDB = map_range(userInputA0, 0.0, 65536.0, 30.0, -97.5)

    print("adcVolumeDB: ", adcVolumeDB)

    codec.setAdcDigitalVolumeDB(adcVolumeDB) # -97.50 to +30.00dB

    time.sleep(0.05)
