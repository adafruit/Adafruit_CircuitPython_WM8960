# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: Unlicense

'''
Demonstrates I2C Input and Output on WM8960 Codec.

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

You can now control the volume of the codecs built in headphone buffers using this function:
codec.setHeadphoneVolumeDB(6.00)
Valid inputs are -74.00 (MUTE) up to +6.00, (1.00dB steps).

For information on the data sent to and received from the CODEC, refer to the WM8960 datasheet at:
https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf
'''

import board
import adafruit_wm8960
import array
import rp2pio
import adafruit_pioasm

CHANNELS = 2
BITS = 16
SAMPLE_RATE = 48000
BUFFER_SIZE = 2048
BUFFER_TYPE = "L"
BUFFER_WIDTH = 32

import busio
codec = adafruit_wm8960.WM8960(busio.I2C(board.GP17, board.GP16))

# Setup INPUT1
codec.enableMIC()
codec.connectMN1()
codec.disableINMUTE()
codec.setINVOLDB(0.0)
codec.setMICBOOST(adafruit_wm8960.MIC_BOOST_GAIN_0DB)
codec.connectMIC2B()
codec.enableAIN()
codec.disableB2O()
codec.setB2OVOL(adafruit_wm8960.OUTPUT_MIXER_GAIN_NEG_21DB)

# Setup DAC
codec.enableD2O()
codec.enableOMIX()
codec.disableLoopBack()

# Set up codec as peripheral device with clock configured for desired sample rate (and 16-bit words)
codec.configureI2S(SAMPLE_RATE)

# Enable ADCs and DACs
codec.enableAdc()
codec.enableDac()
codec.disableDacMute()

# Enable headphone output with OUT3 as capless buffer for headphone ground, adjust volume with dB
codec.configureHeadphones(dB=0.0, capless=True)

def i2s_codec(
    channels=2,
    sample_rate=48000,
    bits=16,
    bclk_pin=None,
    out_pin=None,
    in_pin=None,
):
    i2s_clock = sample_rate * channels * bits
    pio_clock = 4 * i2s_clock
    pio_code = """
        .program i2s_codec
        .side_set 2
                            ; at program start we initialize the bit count top
                            ; (which may be >32) with data
                            ; pulled from the input fifo
            pull noblock    ; first empty the input fifo
            pull noblock
            pull noblock
            pull noblock
            out null, 32    ; then clear OSR so we can get a new value
            pull block      ; then get the bit count top value from the fifo
                            ;        /--- LRCLK
                            ;        |/-- BCLK
                            ;        ||
            mov x, osr;       side 0b01 [1] ; save it in x
            out null, 32      side 0b00 [1]
            mov y, x          side 0b01 [1] ; start of main loop (wrap target=8)
        bitloop1:
            out pins 1        side 0b00
            in pins 1         side 0b00
            jmp y-- bitloop1  side 0b01 [1]
            out pins 1        side 0b10
            in pins 1         side 0b10
            mov y, x          side 0b11 [1]
        bitloop0:
            out pins 1        side 0b10
            in pins 1         side 0b10
            jmp y-- bitloop0  side 0b11 [1]
            out pins 1        side 0b00
            in pins 1         side 0b00
    """
    pio_params = {
        "frequency": pio_clock,
        "first_out_pin": out_pin,
        "first_in_pin": in_pin,
        "first_sideset_pin": bclk_pin,
        "sideset_pin_count": 2,
        "auto_pull": True,
        "auto_push": True,
        "out_shift_right": False,
        "in_shift_right": False,
        "pull_threshold": bits,
        "push_threshold": bits,
        "wait_for_txstall": False,
        "wrap_target": 8,
    }
    pio_instructions = adafruit_pioasm.assemble(pio_code)
    i2s_clock = sample_rate * channels * bits
    pio_clock = 4 * i2s_clock
    pio = rp2pio.StateMachine(pio_instructions, **pio_params)
    return pio

def spaced_samples(length, bits):
    max_int = (1 << bits) - 1
    if length == 1:
        return [0]
    step = max_int / (length - 1)
    result = [round(i * step) for i in range(length)]
    result[0] = 0
    result[-1] = max_int
    return result

# initialize pio bit count top value by sending it at the start of output data
bit_count_top = BITS * (CHANNELS // 2) - 2

PIO = i2s_codec(
    channels=CHANNELS,
    bits=BITS,
    sample_rate=SAMPLE_RATE,
    out_pin=board.GP20,
    in_pin=board.GP21,
    bclk_pin=board.GP18, # L/R signal will be one pin higher, i.e. GP19
)

buffer_in = array.array(BUFFER_TYPE, [0] * BUFFER_SIZE)
while True:
    buffer_out = array.array(BUFFER_TYPE, [bit_count_top] + [d << (BUFFER_WIDTH - BITS) for d in buffer_in])
    PIO.write_readinto(buffer_out, buffer_in)
    PIO.clear_rxfifo()
    del buffer_out
