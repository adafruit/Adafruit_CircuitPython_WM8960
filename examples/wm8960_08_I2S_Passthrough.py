# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates reading I2S audio from the ADC, and passing that back to the DAC.

This example sets up analog audio input (on INPUT1s), ADC/DAC enabled as I2S peripheral, sets volume control, and Headphone output on the WM8960 Codec.

Audio should be connected to both the left and right "INPUT1" inputs, they are labeled "RIN1" and "LIN1" on the board.

This example will pass your audio source through the mixers and gain stages of the codec into the ADC. Read the audio from the ADC via I2S. Then send audio immediately back to the DAC via I2S. Then send the output of the DAC to the headphone outs.

HARDWARE CONNECTIONS

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)
AUDIO_TXD --- DDT         *aka DAC_DATA/I2S_SDO/"serial data out", this carries the I2S audio data from MCU to codec DAC
AUDIO_BCLK -- BCK         *aka BCLK/I2S_SCK/"bit clock", this is the clock for I2S audio, can be controlled via controller or peripheral.
AUDIO_RXD --- ADAT        *aka ADC_DATA/I2S_SD/"serial data in", this carries the I2S audio data from codec's ADC to MCU I2S bus.
AUDIO_SYNC -- DLRC        *aka I2S_WS/LRC/"word select"/"left-right-channel", this toggles for left or right channel data.
AUDIO_SYNC -- ALR         *for this example WS is shared for both the ADC WS (ALR) and the DAC WS (DLRC)

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

import board
import adafruit_wm8960
import audiobusio

print("Example 8 - I2S Passthough")

codec = adafruit_wm8960.WM8960(board.I2C())

# Setup signal flow to the ADC
codec.enableMIC()

# Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
codec.connectMN1()

# Disable mutes on PGA inputs (aka INTPUT1)
codec.disableINMUTE()

# Set pga volumes
codec.setINVOLDB(0.0) # Valid options are -17.25dB to +30dB (0.75dB steps)

# Set input boosts to get inputs 1 to the boost mixers
codec.setMICBOOST(adafruit_wm8960.WM8960_MIC_BOOST_GAIN_0DB)

# Connect from MIC inputs (aka pga output) to boost mixers
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
codec.setB2OVOL(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_NEG_21DB)

# Enable output mixers
codec.enableOMIX()

# Set sample rate, word length, and mode
codec.configureI2S(sample_rate=44100, word_length=adafruit_wm8960.WM8960_WL_16BIT, master=False)

# Enable ADCs and DACs
codec.enableAdc()
codec.enableDac()

# Loopback sends ADC data directly into DAC
codec.disableLoopBack()

# Default is "soft mute" on, so we must disable mute to make channels active
codec.disableDacMute()

print("Volume set to +0dB")
codec.configureHeadphones(dB=0.0, capless=True) # Capless provides VMID as buffer for headphone ground

print("Codec Setup complete. Listen to left/right INPUT1 on Headphone outputs.")

# Set up I2S
audio = audiobusio.I2SOut(board.AUDIO_BCLK, board.AUDIO_SYNC, board.AUDIO_TXD)

while True:
  # Get I2S data and place in data buffer
  # TODO: Read from I2SIn and loopback into I2SOut
  pass

  # DelayMicroseconds(300) # Only hear to demonstrate how much time you have
  # to do things.
  # Do not do much in this main loop, or the audio won't pass through correctly.
  # With default settings (64 samples in buffer), you can spend up to 300
  # microseconds doing something in between passing each buffer of data
  # You can tweak the buffer length to get more time if you need it.
  # When bufferlength is 64, then you get ~300 microseconds
  # When bufferlength is 128, then you get ~600 microseconds
  # Note, as you increase bufferlength, then you are increasing latency between
  # ADC input to DAC output.
  # Latency may or may not be desired, depending on the project.
