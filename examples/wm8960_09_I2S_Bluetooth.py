# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates how to receive audio via Bluetooth and play it back via I2S to the codec DAC.

This example sets up the MCU as a bluetooth sink device, with its output set
to I2S audio.

This example sets up the WM8960 codec as an I2S peripheral, sets volume
control, and Headphone output.

A bluetooth device, such as your phone or laptop, can connect to the MCU and
then begin playing an audio file.

The MCU will send the I2S audio to the DAC of the codec. The DAC is
connected to the HP outputs.

HARDWARE CONNECTIONS

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)
AUDIO_TXD --- DDT         *aka DAC_DATA/I2S_SDO/"serial data out", this carries the I2S audio data from MCU to codec DAC
AUDIO_BCLK -- BCK         *aka BCLK/I2S_SCK/"bit clock", this is the clock for I2S audio, can be controlled via controller or peripheral.
AUDIO_SYNC -- DLRC        *aka I2S_WS/LRC/"word select"/"left-right-channel", this toggles for left or right channel data.

**********************
CODEC -------- AUDIO OUT
**********************
OUT3 --------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
HPL ---------- TRS OUTPUT TIP             *left HP output
HPR ---------- TRS OUTPUT RING1           *right HP output

Note, once connected and playing a sound file, your bluetooth source device
(i.e. your phone) can control volume with its own volume control.

You can also control the volume of the codecs built in headphone amp using this fuction:
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

print("Example 9 - Bluetooth Audio")

codec = adafruit_wm8960.WM8960(board.I2C())

# Connect from DAC outputs to output mixer
codec.enableLD2LO()
codec.enableRD2RO()

# Set gainstage between booster mixer and output mixer
# For this loopback example, we are going to keep these as low as they go
codec.setLB2LOVOL(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_NEG_21DB)
codec.setRB2ROVOL(adafruit_wm8960.WM8960_OUTPUT_MIXER_GAIN_NEG_21DB)

# Enable output mixers
codec.enableLOMIX()
codec.enableROMIX()

# CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d freq at 705.6kHz
codec.enablePLL() # Needed for class-d amp clock
codec.setPLLPRESCALE(adafruit_wm8960.WM8960_PLLPRESCALE_DIV_2)
codec.setSMD(adafruit_wm8960.WM8960_PLL_MODE_FRACTIONAL)
codec.setCLKSEL(adafruit_wm8960.WM8960_CLKSEL_PLL)
codec.setSYSCLKDIV(adafruit_wm8960.WM8960_SYSCLK_DIV_BY_2)
codec.setBCLKDIV(4)
codec.setDCLKDIV(adafruit_wm8960.WM8960_DCLKDIV_16)
codec.setPLLN(7)
codec.setPLLK(0x86C226)
#codec.setADCDIV(0) # Default is 000 (what we need for 44.1KHz)
#codec.setDACDIV(0) # Default is 000 (what we need for 44.1KHz)
codec.setWL(adafruit_wm8960.WM8960_WL_16BIT)

codec.enablePeripheralMode()
#codec.enableMasterMode()
#codec.setALRCGPIO() # Note, should not be changed while ADC is enabled.

# Enable DACs
codec.enableDac()

# Loopback sends ADC data directly into DAC
codec.disableLoopBack()

# Default is "soft mute" on, so we must disable mute to make channels active
codec.disableDacMute()

codec.enableHeadphones()
codec.enableOUT3MIX() # Provides VMID as buffer for headphone ground

print("Volume set to +0dB")
codec.setHeadphoneVolumeDB(0.00)

print("Codec Setup complete. Connect via Bluetooth, play music, and listen on Headphone outputs.")

# Set up I2S
audio = audiobusio.I2SOut(board.AUDIO_BCLK, board.AUDIO_SYNC, board.AUDIO_TXD)

# TODO: Bluetooth setup
