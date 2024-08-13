# SPDX-FileCopyrightText: 2023 Tod Kurt (@todbot)
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT

'''
Demonstrates I2C Output on WM8960 Codec by generating "a swirling ominous wub that evolves over time" using synthio.

Modified from original code by @todbot / Tod Kurt: https://github.com/todbot/circuitpython-synthio-tricks/blob/main/examples/eighties_dystopia/code.py

It will output the sound on the headphone outputs.
It is setup to do a capless headphone setup, so connect your headphones ground to "OUT3" and this provides a buffered VMID.

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
CODEC ------- AUDIO OUT
**********************
OUT3 -------- TRS OUTPUT SLEEVE          *buffered "vmid" (aka "HP GND")
HPL --------- TRS OUTPUT TIP             *left HP output
HPR --------- TRS OUTPUT RING1           *right HP output
'''

import board, audiobusio
import audiomixer, synthio
import adafruit_wm8960
import time, random
import digitalio
import ulab.numpy as np

codec = adafruit_wm8960.WM8960(board.I2C())

# Connect from DAC outputs to output mixer
codec.enableLD2LO()
codec.enableRD2RO()

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

# Enable DACs
codec.enableDacLeft()
codec.enableDacRight()
codec.disableDacMute()

# Loopback sends ADC data directly into DAC
codec.disableLoopBack()

# Default is "soft mute" on, so we must disable mute to make channels active
codec.disableDacMute()

# Enable Headphones
codec.enableHeadphones()
codec.enableOUT3MIX() # Provides VMID as buffer for headphone ground
codec.setHeadphoneVolumeDB(0.0)

audio = audiobusio.I2SOut(board.AUDIO_BCLK, board.AUDIO_SYNC, board.AUDIO_TXD)

led = digitalio.DigitalInOut(board.LED)
led.switch_to_output()
led.value = True

notes = (33, 34, 31) # possible notes to play MIDI A1, A1#, G1
note_duration = 15   # how long each note plays for
num_voices = 5       # how many voices for each note
lpf_basef = 500      # filter lowest frequency
lpf_resonance = 1.5  # filter q

mixer = audiomixer.Mixer(channel_count=2, sample_rate=44100, buffer_size=8192)
synth = synthio.Synthesizer(channel_count=2, sample_rate=44100)
audio.play(mixer)
mixer.voice[0].play(synth)
mixer.voice[0].level = 0.8

# our oscillator waveform, a 512 sample downward saw wave going from +/-30k
wave_saw = np.linspace(30000, -30000, num=512, dtype=np.int16)  # max is +/-32k but gives us headroom
amp_env = synthio.Envelope(attack_level=1, sustain_level=1)

# set up the voices (aka "Notes" in synthio-speak) w/ initial values
voices = []
for i in range(num_voices):
    voices.append( synthio.Note( frequency=0, envelope=amp_env, waveform=wave_saw ) )

# set all the voices to the "same" frequency (with random detuning)
# zeroth voice is sub-oscillator, one-octave down
def set_notes(n):
    for voice in voices:
        #f = synthio.midi_to_hz( n ) + random.uniform(0,1.0)  # what orig sketch does
        f = synthio.midi_to_hz( n + random.uniform(0,0.4) ) # more valid if we move up the scale
        voice.frequency = f
    voices[0].frequency = voices[0].frequency/2  # bass note one octave down

# the LFO that modulates the filter cutoff
lfo_filtermod = synthio.LFO(rate=0.05, scale=2000, offset=2000)
# we can't attach this directly to a filter input, so stash it in the blocks runner
synth.blocks.append(lfo_filtermod)

note = notes[0]
last_note_time = time.monotonic()
last_filtermod_time = time.monotonic()

# start the voices playing
set_notes(note)
synth.press(voices)

while True:
    # continuosly update filter, no global filter, so update each voice's filter
    for v in voices:
        v.filter = synth.low_pass_filter( lpf_basef + lfo_filtermod.value, lpf_resonance )

    if time.monotonic() - last_filtermod_time > 1:
        last_filtermod_time = time.monotonic()
        # randomly modulate the filter frequency ('rate' in synthio) to make more dynamic
        lfo_filtermod.rate = 0.01 + random.random() / 8
        print("filtermod",lfo_filtermod.rate)

    if time.monotonic() - last_note_time > note_duration:
        last_note_time = time.monotonic()
        # pick new note, but not one we're currently playing
        note = random.choice([n for n in notes if n != note])
        set_notes(note)
        print("note", note, ["%3.2f" % v.frequency for v in voices] )
