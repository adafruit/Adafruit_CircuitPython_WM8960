# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates reading I2S audio from the ADC, and plotting the audio samples on the Arduino Serial Plotter.

This example sets up analog audio input (on INPUT1s), ADC enabled as I2S peripheral, sets volume control, and Headphone output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT1" inputs, they are labeled "RIN1" and "LIN1" on the board.

This example will pass your audio source through the mixers and gain stages of the codec into the ADC. Read the audio from the ADC via I2S.

The analog bypass paths is also setup, so your audio will pass through the codec and playback on HP outs.

HARDWARE CONNECTIONS

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)
AUDIO_BCLK -- BCK         *aka BCLK/I2S_SCK/"bit clock", this is the clock for I2S audio, can be controlled via controller or peripheral.
AUDIO_RXD --- ADAT        *aka ADC_DATA/I2S_SD/"serial data in", this carries the I2S audio data from codec's ADC to MCU I2S bus.
AUDIO_SYNC -- ALR        *aka I2S_WS/LRC/"word select"/"left-right-channel", this toggles for left or right channel data.

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

You can now control the volume of the codecs built in headphone amp using this fuction:
codec.setHeadphoneVolumeDB(6.00)
Valid inputs are -74.00 (MUTE) up to +6.00, (1.00dB steps).

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
'''

import board, time
import audiobusio
import adafruit_wm8960

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow to the ADC
codec.enableMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectMN1()

# Disable mutes on PGA inputs (aka INTPUT1)
codec.disableINMUTE()

# Set pga volumes
codec.setINVOLDB(0.00) # Valid options are -17.25dB to +30dB (0.75dB steps)

# Set input boosts to get inputs 1 to the boost mixers
codec.setMICBOOST(adafruit_wm8960.MIC_BOOST_GAIN_0DB)

# Connect from MIC inputs (aka pga output) to boost mixers
codec.connectMIC2B()

# Enable boost mixers
codec.enableAIN()

# Connect LB2LO (booster to output mixer (analog bypass)
codec.enableB2O()

# Disconnect from DAC outputs to output mixer
codec.disableD2O()

# Set gainstage between booster mixer and output mixer
codec.setB2OVOL(adafruit_wm8960.OUTPUT_MIXER_GAIN_0DB)

# Enable output mixers
codec.enableOMIX()

# Configure I2S
codec.configureI2S(sample_rate=44100, word_length=adafruit_wm8960.WL_16BIT, master=False)

# Enable ADCs, and disable DACs
codec.enableAdc()
codec.disableDac()
codec.disableDacMute()

# Loopback sends ADC data directly into DAC
codec.disableLoopBack()

# Default is "soft mute" on, so we must disable mute to make channels active
codec.enableDacMute()

codec.configureHeadphones(dB=0.0, capless=True) # Capless provides VMID as buffer for headphone ground

# Set up I2S
audio = audiobusio.I2SOut(board.AUDIO_BCLK, board.AUDIO_SYNC, board.AUDIO_TXD)

sBuffer = [0 for i in range(64)]

while True:
    # False print statements to "lock range" on serial plotter display
    # Change rangelimit value to adjust "sensitivity"
    rangelimit = 3000
    print(rangelimit * -1, " ", rangelimit, " ", end=None)

    # Get I2S data and place in data buffer
    # TODO: Read from I2SIn

    # Read I2S data buffer
    mean = 0.0
    for i in range(64):
        mean += sBuffer[i]

    # Average the data reading
    mean /= 64.0

    # Print to serial plotter
    print(mean)
