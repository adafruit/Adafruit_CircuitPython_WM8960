# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
This example demonstrates control of the mic bias feature of WM8960 Codec.

Electret Mics are powered with a mic bias voltage applied to their signal line - usually with a 2.2K resistor in series. This Codec can provide a clean mic bias.

This example turns on the mic bias, set's it to each available output voltage, and then turns it off to demonstrate disable.

Measure the voltage with a multimeter to verfy you are getting the correct voltages you desire on mic bias.

You can later use this mic bias voltage to power an electret mic in a more advanced example (example 14).

HARDWARE CONNECTIONS

**********************
MCU --------- CODEC
**********************
QWIIC ------- QWIIC       *Note this connects GND/3.3V/SDA/SCL
GND --------- GND         *optional, but not a bad idea
5V ---------- VIN         *needed to power codec's onboard AVDD (3.3V vreg)

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
'''

import board, time
import adafruit_wm8960

print("Example 7 - MicBias Control")

codec = adafruit_wm8960.WM8960(board.I2C())

codec.enableMicBias()

# WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or
# WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
codec.setMicBiasVoltage(adafruit_wm8960.WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD)
print("Mic Bias enabled (0.9*AVDD)")
delay(3000)

# WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) or
# WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
codec.setMicBiasVoltage(adafruit_wm8960.WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD)
print("Mic Bias enabled (0.65*AVDD)")
delay(3000)

codec.disableMicBias()
print("Mic Bias disabled")

print("Example Complete. Hit reset to begin again.")
