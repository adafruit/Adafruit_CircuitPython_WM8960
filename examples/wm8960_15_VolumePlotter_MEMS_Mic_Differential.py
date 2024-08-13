# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
This example is very similar to example 11, however it uses a MEMS mic with differential signal as the source of the sound. Here we are using just one of VM2020 Breakouts plugged into the left channel.

Note, with this VM2020 differential mic signal, we are going to add much more gain than in previous examples. We are going to use a MICBOOST of 29dB (max), and set the PGA to +8.25dB.

Demonstrates reading I2S audio from the ADC, and plotting the audio samples on the Arduino Serial Plotter.

This example sets up differential analog audio input (on LINPUT1/LINPUT2), ADC enabled as I2S peripheral, sets volume control, and Headphone output on the WM8960 Codec.

A MEMS mic should be connected to the left channel INPUT1 and INPUT2, they are labeled "LIN1" and "LIN2" on the board. These will provide the input connections for the positive and negative signals from the MEMS mic.

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
AUDIO_SYNC -- ALR         *aka I2S_WS/LRC/"word select"/"left-right-channel", this toggles for left or right channel data.

**********************
CODEC ------- MIC IN
**********************
GND --------- GND        *Ground
AVDD -------- VCC        *3.3V (default on the Codec breakout)
LINPUT1 ----- OUT-       *Mic signal "-"
LINPUT2 ----- OUT+       *Mic signal "+"

**********************
CODEC ------- AUDIO OUT
**********************
OUT3 -------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
HPL --------- TRS OUTPUT TIP             *left HP output
HPR --------- TRS OUTPUT RING1           *right HP output

You can now control the volume of the codecs built in headphone amp using this fuction:
codec.setHeadphoneVolumeDB(6.00)
Valid inputs are -74.00 (MUTE) up to +6.00, (1.00dB steps).

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
'''

import board
import audiobusio
import adafruit_wm8960

sBuffer = [0 for i in range(64)]

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow to the ADC
codec.enableMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectMN1()

# Disable mutes on PGA inputs (aka INTPUT1)
codec.disableINMUTE()

# Set pga volumes
codec.setINVOLDB(8.25) # Valid options are -17.25dB to +30dB (0.75dB steps)

# Set input boosts to get inputs 1 to the boost mixers
codec.setMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_29DB)

# For MIC+ signal of differential mic signal
codec.pgaNonInvSignalSelect(adafruit_wm8960.WM8960_PGA_INPUT2)

# Connect from MIC inputs (aka pga output) to boost mixers
codec.connectMIC2B()

# Enable boost mixers
codec.enableAIN()

# Connect LB2LO (booster to output mixer (analog bypass)
codec.enableBoost2OutputMixer()

# Disconnect from DAC outputs to output mixer
codec.disableDac2OutputMixer()

# Set gainstage between booster mixer and output mixer
codec.setBoost2MixerGain(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_0DB)

# Enable output mixers
codec.enableOutputMixer()

# Set sample rate, word length, and mode
codec.configureI2S(sample_rate=44100, word_length=adafruit_wm8960.WM8960_WL_16BIT, master=False)

# Enable ADCs, and disable DACs
codec.enableAdc()
codec.disableDac()
codec.disableDacMute()

# Loopback sends ADC data directly into DAC
codec.disableLoopBack()

# Default is "soft mute" on, so we must disable mute to make channels active
codec.enableDacMute()

codec.configureHeadphones(dB=6.0, capless=True) # Capless provides VMID as buffer for headphone ground

# Set up I2S
audio = audiobusio.I2SOut(board.AUDIO_BCLK, board.AUDIO_SYNC, board.AUDIO_TXD)

while True:
    # False print statements to "lock range" on serial plotter display
    # Change rangelimit value to adjust "sensitivity"
    rangelimit = 3000
    print(rangelimit * -1, " ", rangelimit, " ", end=None)

    # Get I2S data and place in data buffer
    # TODO: Read from I2SIn

    # Read I2S data buffer
    mean = 0.0
    # Only looking at left signal samples in the buffer (e.g. 0,2,4,6,8...)
    # Notice in our for loop here, we are incrementing the index by 2.
    for i in range(64):
        mean += sBuffer[i]

    # Average the data reading
    # We're only concerned with left input for this example. So we must
    # divide by "half of samples read" (because it is stereo I2S audio data)
    mean /= 64.0 / 2.0

    # Print to serial plotter
    print(mean)
