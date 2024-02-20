# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

# Sounds like an alarm clock. Tested on iMX RT 1060 EVK.

import audiobusio
import board
import synthio
import adafruit_wm8960
import time
import digitalio

dac = adafruit_wm8960.WM8960(board.I2C())
dac.start_i2s_out()
audio = audiobusio.I2SOut(board.AUDIO_BCLK, board.AUDIO_SYNC, board.AUDIO_TXD, main_clock=board.AUDIO_MCLK)
synth = synthio.Synthesizer(sample_rate=22050)
audio.play(synth)

led = digitalio.DigitalInOut(board.LED)
led.switch_to_output()

while True:
    print("note on")
    synth.press(65) # midi note 65 = F4
    led.value = True
    time.sleep(0.5)
    synth.release(65) # release the note we pressed
    led.value = False
    print("note off")
    time.sleep(0.5)
