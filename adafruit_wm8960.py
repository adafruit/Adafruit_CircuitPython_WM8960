# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT
"""
`adafruit_wm8960`
================================================================================

CircuitPython driver for WM8960 Stereo CODEC

* Author(s): Scott Shawcroft, Cooper Dalrymple

Originally authored by Pete Lewis @ SparkFun Electronics, October 14th, 2022
https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from busio import I2C
from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_WM8960.git"

# I2C address (7-bit format for Wire library)
WM8960_ADDR = 0x1A

# WM8960 register addresses
WM8960_REG_LEFT_INPUT_VOLUME = 0x00
WM8960_REG_RIGHT_INPUT_VOLUME = 0x01
WM8960_REG_LOUT1_VOLUME = 0x02
WM8960_REG_ROUT1_VOLUME = 0x03
WM8960_REG_CLOCKING_1 = 0x04
WM8960_REG_ADC_DAC_CTRL_1 = 0x05
WM8960_REG_ADC_DAC_CTRL_2 = 0x06
WM8960_REG_AUDIO_INTERFACE_1 = 0x07
WM8960_REG_CLOCKING_2 = 0x08
WM8960_REG_AUDIO_INTERFACE_2 = 0x09
WM8960_REG_LEFT_DAC_VOLUME = 0x0A
WM8960_REG_RIGHT_DAC_VOLUME = 0x0B
WM8960_REG_RESET = 0x0F
WM8960_REG_3D_CONTROL = 0x10
WM8960_REG_ALC1 = 0x11
WM8960_REG_ALC2 = 0x12
WM8960_REG_ALC3 = 0x13
WM8960_REG_NOISE_GATE = 0x14
WM8960_REG_LEFT_ADC_VOLUME = 0x15
WM8960_REG_RIGHT_ADC_VOLUME = 0x16
WM8960_REG_ADDITIONAL_CONTROL_1 = 0x17
WM8960_REG_ADDITIONAL_CONTROL_2 = 0x18
WM8960_REG_PWR_MGMT_1 = 0x19
WM8960_REG_PWR_MGMT_2 = 0x1A
WM8960_REG_ADDITIONAL_CONTROL_3 = 0x1B
WM8960_REG_ANTI_POP_1 = 0x1C
WM8960_REG_ANTI_POP_2 = 0x1D
WM8960_REG_ADCL_SIGNAL_PATH = 0x20
WM8960_REG_ADCR_SIGNAL_PATH = 0x21
WM8960_REG_LEFT_OUT_MIX_1 = 0x22
WM8960_REG_RIGHT_OUT_MIX_2 = 0x25
WM8960_REG_MONO_OUT_MIX_1 = 0x26
WM8960_REG_MONO_OUT_MIX_2 = 0x27
WM8960_REG_LOUT2_VOLUME = 0x28
WM8960_REG_ROUT2_VOLUME = 0x29
WM8960_REG_MONO_OUT_VOLUME = 0x2A
WM8960_REG_INPUT_BOOST_MIXER_1 = 0x2B
WM8960_REG_INPUT_BOOST_MIXER_2 = 0x2C
WM8960_REG_BYPASS_1 = 0x2D
WM8960_REG_BYPASS_2 = 0x2E
WM8960_REG_PWR_MGMT_3 = 0x2F
WM8960_REG_ADDITIONAL_CONTROL_4 = 0x30
WM8960_REG_CLASS_D_CONTROL_1 = 0x31
WM8960_REG_CLASS_D_CONTROL_3 = 0x33
WM8960_REG_PLL_N = 0x34
WM8960_REG_PLL_K_1 = 0x35
WM8960_REG_PLL_K_2 = 0x36
WM8960_REG_PLL_K_3 = 0x37

# PGA input selections
WM8960_PGAL_LINPUT2 = 0
WM8960_PGAL_LINPUT3 = 1
WM8960_PGAL_VMID = 2
WM8960_PGAR_RINPUT2 = 0
WM8960_PGAR_RINPUT3 = 1
WM8960_PGAR_VMID = 2

# Mic (aka PGA) BOOST gain options
WM8960_MIC_BOOST_GAIN_0DB = 0
WM8960_MIC_BOOST_GAIN_13DB = 1
WM8960_MIC_BOOST_GAIN_20DB = 2
WM8960_MIC_BOOST_GAIN_29DB = 3

'''
Boost Mixer gain options
These are used to control the gain (aka volume) at the following settings:
LIN2BOOST
LIN3BOOST
RIN2BOOST
RIN3BOOST
'''
WM8960_BOOST_MIXER_GAIN_MUTE = 0
WM8960_BOOST_MIXER_GAIN_NEG_12DB = 1
WM8960_BOOST_MIXER_GAIN_NEG_9DB = 2
WM8960_BOOST_MIXER_GAIN_NEG_6DB = 3
WM8960_BOOST_MIXER_GAIN_NEG_3DB = 4
WM8960_BOOST_MIXER_GAIN_0DB = 5
WM8960_BOOST_MIXER_GAIN_3DB = 6
WM8960_BOOST_MIXER_GAIN_6DB = 7

'''
Output Mixer gain options
These are used to control the gain (aka volume) at the following settings:
LI2LOVOL
LB2LOVOL
RI2LOVOL
RB2LOVOL
These are useful as analog bypass signal path options.
'''
WM8960_OUTPUT_MIXER_GAIN_0DB = 0
WM8960_OUTPUT_MIXER_GAIN_NEG_3DB = 1
WM8960_OUTPUT_MIXER_GAIN_NEG_6DB = 2
WM8960_OUTPUT_MIXER_GAIN_NEG_9DB = 3
WM8960_OUTPUT_MIXER_GAIN_NEG_12DB = 4
WM8960_OUTPUT_MIXER_GAIN_NEG_15DB = 5
WM8960_OUTPUT_MIXER_GAIN_NEG_18DB = 6
WM8960_OUTPUT_MIXER_GAIN_NEG_21DB = 7

# Mic Bias voltage options
WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD = 0
WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD = 1

# SYSCLK divide
WM8960_SYSCLK_DIV_BY_1 = 0
WM8960_SYSCLK_DIV_BY_2 = 2
WM8960_CLKSEL_MCLK = 0
WM8960_CLKSEL_PLL = 1
WM8960_PLL_MODE_INTEGER = 0
WM8960_PLL_MODE_FRACTIONAL = 1
WM8960_PLLPRESCALE_DIV_1 = 0
WM8960_PLLPRESCALE_DIV_2 = 1

# Class d clock divide
WM8960_DCLKDIV_16 = 7

# Word length settings (aka bits per sample)
# Audio Data Word Length
WM8960_WL_16BIT = 0
WM8960_WL_20BIT = 1
WM8960_WL_24BIT = 2
WM8960_WL_32BIT = 3

'''
Additional Digital Audio Interface controls
LRP (aka left-right-polarity)
Right, left and I2S modes – LRCLK polarity
0 = normal LRCLK polarity
1 = inverted LRCLK polarity
'''
WM8960_LR_POLARITY_NORMAL = 0
WM8960_LR_POLARITY_INVERT = 1

'''
ALRSWAP (aka ADC left/right swap)
Left/Right ADC channel swap
1 = Swap left and right ADC data in audio interface
0 = Output left and right data as normal
'''
WM8960_ALRSWAP_NORMAL = 0
WM8960_ALRSWAP_SWAP = 1

# Gain mins, maxes, offsets and step-sizes for all the amps within the codec.
WM8960_PGA_GAIN_MIN = -17.25
WM8960_PGA_GAIN_MAX = 30.00
WM8960_PGA_GAIN_OFFSET = 17.25
WM8960_PGA_GAIN_STEPSIZE = 0.75
WM8960_HP_GAIN_MIN = -73.00
WM8960_HP_GAIN_MAX = 6.00
WM8960_HP_GAIN_OFFSET = 121.00
WM8960_HP_GAIN_STEPSIZE = 1.00
WM8960_SPEAKER_GAIN_MIN = -73.00
WM8960_SPEAKER_GAIN_MAX = 6.00
WM8960_SPEAKER_GAIN_OFFSET = 121.00
WM8960_SPEAKER_GAIN_STEPSIZE = 1.00
WM8960_ADC_GAIN_MIN = -97.00
WM8960_ADC_GAIN_MAX = 30.00
WM8960_ADC_GAIN_OFFSET = 97.50
WM8960_ADC_GAIN_STEPSIZE = 0.50
WM8960_DAC_GAIN_MIN = -97.00
WM8960_DAC_GAIN_MAX = 30.00
WM8960_DAC_GAIN_OFFSET = 97.50
WM8960_DAC_GAIN_STEPSIZE = 0.50

# Automatic Level Control Modes
WM8960_ALC_MODE_OFF = 0
WM8960_ALC_MODE_RIGHT_ONLY = 1
WM8960_ALC_MODE_LEFT_ONLY = 2
WM8960_ALC_MODE_STEREO = 3

# Automatic Level Control Target Level dB
WM8960_ALC_TARGET_LEVEL_NEG_22_5DB = 0
WM8960_ALC_TARGET_LEVEL_NEG_21DB = 1
WM8960_ALC_TARGET_LEVEL_NEG_19_5DB = 2
WM8960_ALC_TARGET_LEVEL_NEG_18DB = 3
WM8960_ALC_TARGET_LEVEL_NEG_16_5DB = 4
WM8960_ALC_TARGET_LEVEL_NEG_15DB = 5
WM8960_ALC_TARGET_LEVEL_NEG_13_5DB = 6
WM8960_ALC_TARGET_LEVEL_NEG_12DB = 7
WM8960_ALC_TARGET_LEVEL_NEG_10_5DB = 8
WM8960_ALC_TARGET_LEVEL_NEG_9DB = 9
WM8960_ALC_TARGET_LEVEL_NEG_7_5DB = 10
WM8960_ALC_TARGET_LEVEL_NEG_6DB = 11
WM8960_ALC_TARGET_LEVEL_NEG_4_5DB = 12
WM8960_ALC_TARGET_LEVEL_NEG_3DB = 13
WM8960_ALC_TARGET_LEVEL_NEG_1_5DB = 14

# Automatic Level Control Max Gain Level dB
WM8960_ALC_MAX_GAIN_LEVEL_NEG_12DB = 0
WM8960_ALC_MAX_GAIN_LEVEL_NEG_6DB = 1
WM8960_ALC_MAX_GAIN_LEVEL_0DB = 2
WM8960_ALC_MAX_GAIN_LEVEL_6DB = 3
WM8960_ALC_MAX_GAIN_LEVEL_12DB = 4
WM8960_ALC_MAX_GAIN_LEVEL_18DB = 5
WM8960_ALC_MAX_GAIN_LEVEL_24DB = 6
WM8960_ALC_MAX_GAIN_LEVEL_30DB = 7

# Automatic Level Control Min Gain Level dB
WM8960_ALC_MIN_GAIN_LEVEL_NEG_17_25DB = 0
WM8960_ALC_MIN_GAIN_LEVEL_NEG_11_25DB = 1
WM8960_ALC_MIN_GAIN_LEVEL_NEG_5_25DB = 2
WM8960_ALC_MIN_GAIN_LEVEL_0_75DB = 3
WM8960_ALC_MIN_GAIN_LEVEL_6_75DB = 4
WM8960_ALC_MIN_GAIN_LEVEL_12_75DB = 5
WM8960_ALC_MIN_GAIN_LEVEL_18_75DB = 6
WM8960_ALC_MIN_GAIN_LEVEL_24_75DB = 7

# Automatic Level Control Hold Time (MS and SEC)
WM8960_ALC_HOLD_TIME_0MS = 0
WM8960_ALC_HOLD_TIME_3MS = 1
WM8960_ALC_HOLD_TIME_5MS = 2
WM8960_ALC_HOLD_TIME_11MS = 3
WM8960_ALC_HOLD_TIME_21MS = 4
WM8960_ALC_HOLD_TIME_43MS = 5
WM8960_ALC_HOLD_TIME_85MS = 6
WM8960_ALC_HOLD_TIME_170MS = 7
WM8960_ALC_HOLD_TIME_341MS = 8
WM8960_ALC_HOLD_TIME_682MS = 9
WM8960_ALC_HOLD_TIME_1365MS = 10
WM8960_ALC_HOLD_TIME_3SEC = 11
WM8960_ALC_HOLD_TIME_5SEC = 12
WM8960_ALC_HOLD_TIME_10SEC = 13
WM8960_ALC_HOLD_TIME_23SEC = 14
WM8960_ALC_HOLD_TIME_44SEC = 15

# Automatic Level Control Decay Time (MS and SEC)
WM8960_ALC_DECAY_TIME_24MS = 0
WM8960_ALC_DECAY_TIME_48MS = 1
WM8960_ALC_DECAY_TIME_96MS = 2
WM8960_ALC_DECAY_TIME_192MS = 3
WM8960_ALC_DECAY_TIME_384MS = 4
WM8960_ALC_DECAY_TIME_768MS = 5
WM8960_ALC_DECAY_TIME_1536MS = 6
WM8960_ALC_DECAY_TIME_3SEC = 7
WM8960_ALC_DECAY_TIME_6SEC = 8
WM8960_ALC_DECAY_TIME_12SEC = 9
WM8960_ALC_DECAY_TIME_24SEC = 10

# Automatic Level Control Attack Time (MS and SEC)
WM8960_ALC_ATTACK_TIME_6MS = 0
WM8960_ALC_ATTACK_TIME_12MS = 1
WM8960_ALC_ATTACK_TIME_24MS = 2
WM8960_ALC_ATTACK_TIME_482MS = 3
WM8960_ALC_ATTACK_TIME_964MS = 4
WM8960_ALC_ATTACK_TIME_1928MS = 5
WM8960_ALC_ATTACK_TIME_3846MS = 6
WM8960_ALC_ATTACK_TIME_768MS = 7
WM8960_ALC_ATTACK_TIME_1536MS = 8
WM8960_ALC_ATTACK_TIME_3SEC = 9
WM8960_ALC_ATTACK_TIME_6SEC = 10

# Speaker Boost Gains (DC and AC)
WM8960_SPEAKER_BOOST_GAIN_0DB = 0
WM8960_SPEAKER_BOOST_GAIN_2_1DB = 1
WM8960_SPEAKER_BOOST_GAIN_2_9DB = 2
WM8960_SPEAKER_BOOST_GAIN_3_6DB = 3
WM8960_SPEAKER_BOOST_GAIN_4_5DB = 4
WM8960_SPEAKER_BOOST_GAIN_5_1DB = 5

# VMIDSEL settings
WM8960_VMIDSEL_DISABLED = 0
WM8960_VMIDSEL_2X50KOHM = 1
WM8960_VMIDSEL_2X250KOHM = 2
WM8960_VMIDSEL_2X5KOHM = 3

'''
VREF to Analogue Output Resistance
(Disabled Outputs)
0 = 500 VMID to output
1 = 20k VMID to output
'''
WM8960_VROI_500 = 0
WM8960_VROI_20K = 1

'''
Analogue Bias Optimisation
00 = Reserved
01 = Increased bias current optimized for
AVDD=2.7V
1X = Lowest bias current, optimized for
AVDD=3.3V
'''

WM8960_VSEL_INCREASED_BIAS_CURRENT = 1
WM8960_VSEL_LOWEST_BIAS_CURRENT = 3

WM8960_REGISTER_DEFAULTS = [
    0x0097, # R0 (0x00)
    0x0097, # R1 (0x01)
    0x0000, # R2 (0x02)
    0x0000, # R3 (0x03)
    0x0000, # R4 (0x04)
    0x0008, # F5 (0x05)
    0x0000, # R6 (0x06)
    0x000A, # R7 (0x07)
    0x01C0, # R8 (0x08)
    0x0000, # R9 (0x09)
    0x00FF, # R10 (0x0a)
    0x00FF, # R11 (0x0b)
    0x0000, # R12 (0x0C) RESERVED
    0x0000, # R13 (0x0D) RESERVED
    0x0000, # R14 (0x0E) RESERVED
    0x0000, # R15 (0x0F) RESERVED
    0x0000, # R16 (0x10)
    0x007B, # R17 (0x11)
    0x0100, # R18 (0x12)
    0x0032, # R19 (0x13)
    0x0000, # R20 (0x14)
    0x00C3, # R21 (0x15)
    0x00C3, # R22 (0x16)
    0x01C0, # R23 (0x17)
    0x0000, # R24 (0x18)
    0x0000, # R25 (0x19)
    0x0000, # R26 (0x1A)
    0x0000, # R27 (0x1B)
    0x0000, # R28 (0x1C)
    0x0000, # R29 (0x1D)
    0x0000, # R30 (0x1E) RESERVED
    0x0000, # R31 (0x1F) RESERVED
    0x0100, # R32 (0x20)
    0x0100, # R33 (0x21)
    0x0050, # R34 (0x22)
    0x0000, # R35 (0x23) RESERVED
    0x0000, # R36 (0x24) RESERVED
    0x0050, # R37 (0x25)
    0x0000, # R38 (0x26)
    0x0000, # R39 (0x27)
    0x0000, # R40 (0x28)
    0x0000, # R41 (0x29)
    0x0040, # R42 (0x2A)
    0x0000, # R43 (0x2B)
    0x0000, # R44 (0x2C)
    0x0050, # R45 (0x2D)
    0x0050, # R46 (0x2E)
    0x0000, # R47 (0x2F)
    0x0002, # R48 (0x30)
    0x0037, # R49 (0x31)
    0x0000, # R50 (0x32) RESERVED
    0x0080, # R51 (0x33)
    0x0008, # R52 (0x34)
    0x0031, # R53 (0x35)
    0x0026, # R54 (0x36)
    0x00e9, # R55 (0x37)
]

class WM8960:
    def __init__(self, i2c_bus:I2C, address:int = WM8960_ADDR):
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._buf = bytearray(2)

        '''
        The WM8960 does not support I2C reads
        This means we must keep a local copy of all the register values
        We will instantiate with default values by copying from WM8960_REGISTER_DEFAULTS during reset()
        As we write to the device, we will also make sure to update our local copy as well, stored here in this array.
        Each register is 9-bits
        They are in order from R0-R55, and we even keep blank spots for the "reserved" registers. This way we can use the register address macro defines above to easiy access each local copy of each register.
        Example: self._registerLocalCopy[WM8960_REG_LEFT_INPUT_VOLUME]
        '''
        self._registerLocalCopy = [0x0000 for i in range(len(WM8960_REGISTER_DEFAULTS))]
        self.reset()

        # General setup
        self.enableVREF()
        self.enableVMID()

    def isConnected(self) -> bool:
        # TODO: Check I2C or I2CDevice
        return True

    '''
    Necessary for all other functions of the CODEC
    VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
    VREF is bit 6, 0 = power down, 1 = power up
    Returns 1 if successful, 0 if something failed (I2C error)
    '''
    def enableVREF(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 1)

    '''
    Use this to save power
    VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
    VREF is bit 6, 0 = power down, 1 = power up
    Returns 1 if successful, 0 if something failed (I2C error)
    '''
    def disableVREF(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 0)

    # Resets all registers to their default state
    def reset(self) -> None:
        # Doesn't matter which bit we flip, writing anything will cause the reset
        self._writeRegisterBit(WM8960_REG_RESET, 7, 1)
        # Update our local copy of the registers to reflect the reset
        for i in range(len(WM8960_REGISTER_DEFAULTS)):
            self._registerLocalCopy[i] = WM8960_REGISTER_DEFAULTS[i]

    def enableAINL(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 1)
    def disableAINL(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 0)

    def enableAINR(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 1)
    def disableAINR(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 0)

    def enableLMIC(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1)
    def disableLMIC(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0)

    def enableRMIC(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1)
    def disableRMIC(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0)

    def enableLMICBOOST(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1)
    def disableLMICBOOST(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0)

    def enableRMICBOOST(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1)
    def disableRMICBOOST(self) -> None:
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0)

    # PGA input signal select
    # Each PGA (left and right) has a switch on its non-inverting input.
    # On PGA_LEFT:
    # You can select between VMID, LINPUT2 or LINPUT3
    # Note, the inverting input of PGA_LEFT is perminantly connected to
    # LINPUT1
    # On PGA_RIGHT:
    # You can select between VMIN, RINPUT2 or RINPUT3
    # Note, the inverting input of PGA_RIGHT is perminantly connected to
    # RINPUT1

    # 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
    def pgaLeftNonInvSignalSelect(self, signal:int):
        '''
        Clear LMP2 and LMP3
        Necessary because the previous setting could have either set,
        And we don't want to confuse the codec.
        Only 1 input can be selected.
        '''

        # LMP3
        self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 0)

        # LMP2
        self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 0)

        if signal == WM8960_PGAL_LINPUT2:
            # LMP2
            self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 1)
        elif signal == WM8960_PGAL_LINPUT3:
            # LMP3
            self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 1)
        elif signal == WM8960_PGAL_VMID:
            # Don't set any bits. When both LMP2 and LMP3 are cleared, then the signal is set to VMID
            pass

    # 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
    def pgaRightNonInvSignalSelect(self, signal:int):
        '''
        Clear RMP2 and RMP3
        Necessary because the previous setting could have either set,
        And we don't want to confuse the codec.
        Only 1 input can be selected.
        '''

        # RMP3
        self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 0)

        # RMP2
        self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 0)

        if signal == WM8960_PGAR_RINPUT2:
            # RMP2
            self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 1)
        elif signal == WM8960_PGAR_RINPUT3:
            # RMP3
            self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 1)
        elif signal == WM8960_PGAR_VMID:
            # Don't set any bits. When both RMP2 and RMP3 are cleared, then the signal is set to VMID
            pass

    # Connections from each INPUT1 to the inverting input of its PGA

    # Connect LINPUT1 to inverting input of Left Input PGA
    def connectLMN1(self):
        self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 1)

    # Disconnect LINPUT1 from inverting input of Left Input PGA
    def disconnectLMN1(self):
        self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 0)

    # Connect RINPUT1 to inverting input of Right Input PGA
    def connectRMN1(self):
        self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 1)

    # Disconnect RINPUT1 from inverting input of Right Input PGA
    def disconnectRMN1(self):
        self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 0)
    
    # Connection from output of PGAs to downstream "boost mixers"

    # Connect Left Input PGA to Left Input Boost mixer
    def connectLMIC2B(self):
        self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 1)

    # Disconnect Left Input PGA to Left Input Boost mixer
    def disconnectLMIC2B(self):
        self._writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 0)

    # Connect Right Input PGA to Right Input Boost mixer
    def connectRMIC2B(self):
        self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 1)

    # Disconnect Right Input PGA to Right Input Boost mixer
    def disconnectRMIC2B(self):
        self._writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 0)

    # 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
    def setLINVOL(self, volume:int):
        # Limit incoming values max
        if volume > 63:
            volume = 63
        self._writeRegisterMultiBits(WM8960_REG_LEFT_INPUT_VOLUME, 5, 0, volume)
        self.pgaLeftIPVUSet()

    '''
    Sets the volume of the PGA input buffer amp to a specified dB value 
    passed in as a float argument.
    Valid dB settings are -17.25 up to +30.00
    -17.25 = -17.25dB (MIN)
    ... 0.75dB steps ...
    30.00 = +30.00dB  (MAX)
    '''
    def setLINVOLDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setLINVOL()
        volume = self.convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX)
        self.setLINVOL(volume)

    # 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
    def setRINVOL(self, volume:int):
        # Limit incoming values max
        if volume > 63:
            volume = 63
        self._writeRegisterMultiBits(WM8960_REG_RIGHT_INPUT_VOLUME, 5, 0, volume)
        self.pgaRightIPVUSet()

    '''
    Sets the volume of the PGA input buffer amp to a specified dB value 
    passed in as a float argument.
    Valid dB settings are -17.25 up to +30.00
    -17.25 = -17.25dB (MIN)
    ... 0.75dB steps ...
    30.00 = +30.00dB  (MAX)
    '''
    def setRINVOLDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setLINVOL()
        volume = self.convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX)
        self.setRINVOL(volume)

    # Zero Cross prevents zipper sounds on volume changes
    # Sets both left and right PGAs
    def enablePgaZeroCross(self):
        self._writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 1)
        self._writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 1)
    def disablePgaZeroCross(self):
        self._writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 0)
        self._writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 0)

    def enableLINMUTE(self):
        self._writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 1)
    def disableLINMUTE(self):
        self._writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 0)
        self._writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1)

    def enableRINMUTE(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 1)
    def disableRINMUTE(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 0)
        self._writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1)

    # Causes left and right input PGA volumes to be updated
    # (LINVOL and RINVOL)
    def pgaLeftIPVUSet(self):
        self._writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1)

    # Causes left and right input PGA volumes to be updated
    # (LINVOL and RINVOL)
    def pgaRightIPVUSet(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1)

    # Boosts

    # WM8960_MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
    def setLMICBOOST(self, boost_gain:int):
        # Limit incoming values max
        if boost_gain > 3:
            boost_gain = 3
        self._writeRegisterMultiBits(WM8960_REG_ADCL_SIGNAL_PATH, 5, 4, boost_gain)

    # WM8960_MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
    def setRMICBOOST(self, boost_gain:int):
        # Limit incoming values max
        if boost_gain > 3:
            boost_gain = 3
        self._writeRegisterMultiBits(WM8960_REG_ADCR_SIGNAL_PATH, 5, 4, boost_gain)

    # WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
    def setLIN3BOOST(self, boost_gain:int):
        # Limit incoming values max
        if boost_gain > 7:
            boost_gain = 7
        self._writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1, 6, 4, boost_gain)

    # WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
    def setLIN2BOOST(self, boost_gain:int):
        # Limit incoming values max
        if boost_gain > 7:
            boost_gain = 7
        self._writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1, 3, 1, boost_gain)

    # WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
    def setRIN3BOOST(self, boost_gain:int):
        # Limit incoming values max
        if boost_gain > 7:
            boost_gain = 7
        self._writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2, 6, 4, boost_gain)

    # WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
    def setRIN2BOOST(self, boost_gain:int):
        # Limit incoming values max
        if boost_gain > 7:
            boost_gain = 7
        self._writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2, 3, 1, boost_gain)

    # Mic Bias control
    def enableMicBias(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 1)
    def disableMicBias(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 0)

    # WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD)
    # or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
    def setMicBiasVoltage(self, voltage:bool):
        self._writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 0, voltage)

    ## ADC

    def enableAdcLeft(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 1)
    def disableAdcLeft(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 0)

    def enableAdcRight(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 1)
    def disableAdcRight(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 0)

    def enableAdc(self):
        self.enableAdcLeft()
        self.enableAdcRight()
    def disableAdc(self):
        self.disableAdcLeft()
        self.disableAdcRight()

    # ADC digital volume
    # Note, also needs to handle control of the ADCVU bits (volume update).
    # Valid inputs are 0-255
    # 0 = mute
    # 1 = -97dB
    # ... 0.5dB steps up to
    # 195 = 0dB
    # 255 = +30dB
    def setAdcLeftDigitalVolume(self, volume:int):
        self._writeRegisterMultiBits(WM8960_REG_LEFT_ADC_VOLUME, 7, 0, volume)
        self.adcLeftADCVUSet()
    def setAdcRightDigitalVolume(self, volume:int):
        self._writeRegisterMultiBits(WM8960_REG_RIGHT_ADC_VOLUME, 7, 0, volume)
        self.adcRightADCVUSet()

    '''
    ADC digital volume DB
    Sets the volume of the ADC to a specified dB value passed in as a float 
    argument.
    Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
    -97.50 (or lower) = MUTE
    -97.00 = -97.00dB (MIN)
    ... 0.5dB steps ...
    30.00 = +30.00dB  (MAX)
    '''
    def setAdcLeftDigitalVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setAdcLeftDigitalVolume()
        volume = self.convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX)
        self.setAdcLeftDigitalVolume(volume)
    def setAdcRightDigitalVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setAdcRightDigitalVolume()
        volume = self.convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX)
        self.setAdcRightDigitalVolume(volume)

    # Causes left and right input ADC volumes to be updated
    def adcLeftADCVUSet(self):
        self._writeRegisterBit(WM8960_REG_LEFT_ADC_VOLUME, 8, 1)

    # Causes left and right input ADC volumes to be updated
    def adcRightADCVUSet(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_ADC_VOLUME, 8, 1)

    # Control ADC volume in a stereo pair
    def setAdcDigitalVolume(self, volume:int):
        self.setAdcLeftDigitalVolume(volume)
        self.setAdcRightDigitalVolume(volume)

    def setAdcDigitalVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setAdcLeftDigitalVolume()
        volume = self.convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX)
        self.setAdcDigitalVolume(volume)

    ## ALC

    # Automatic Level Control
    # Note that when the ALC function is enabled, the settings of
    # Registers 0 and 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and
    # RINMUTE) are ignored.

    # Also sets alc sample rate to match global sample rate.
    def enableAlc(self, mode:int = WM8960_ALC_MODE_STEREO):
        bit8 = mode >> 1
        bit7 = mode & 0x1
        self._writeRegisterBit(WM8960_REG_ALC1, 8, bit8)
        self._writeRegisterBit(WM8960_REG_ALC1, 7, bit7)
    
    def disableAlc(self):
        self._writeRegisterBit(WM8960_REG_ALC1, 8, 0)
        self._writeRegisterBit(WM8960_REG_ALC1, 7, 0)

    # Valid inputs are 0-15
    # 0 = -22.5dB FS ... 1.5dB steps ... 15 = -1.5dB FS
    def setAlcTarget(self, target:int):
        # Limit incoming values max
        if target > 15:
            target = 15
        self._writeRegisterMultiBits(WM8960_REG_ALC1,3,0,target)

    # Valid inputs are 0-10, 0 = 24ms, 1 = 48ms ... 10 = 24.58seconds
    def setAlcDecay(self, decay:int):
        # Limit incoming values max
        if decay > 10:
            decay = 10
        self._writeRegisterMultiBits(WM8960_REG_ALC3, 7, 4, decay)

    # Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms ...
    # 10 = 6.14seconds
    def setAlcAttack(self, attack:int):
        # Limit incoming values max
        if attack > 10:
            attack = 10
        self._writeRegisterMultiBits(WM8960_REG_ALC3, 3, 0, attack)

    # Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
    def setAlcMaxGain(self, maxGain:int):
        # Limit incoming values max
        if maxGain > 7:
            maxGain = 7
        self._writeRegisterMultiBits(WM8960_REG_ALC1, 6, 4, maxGain)

    # Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
    def setAlcMinGain(self, minGain:int):
        # Limit incoming values max
        if minGain > 7:
            minGain = 7
        self._writeRegisterMultiBits(WM8960_REG_ALC2, 6, 4, minGain)

    # Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
    def setAlcHold(self, hold:int):
        # Limit incoming values max
        if hold > 15:
            hold = 15
        self._writeRegisterMultiBits(WM8960_REG_ALC2, 3, 0, hold)

    # Peak Limiter
    def enablePeakLimiter(self):
        self._writeRegisterBit(WM8960_REG_ALC3, 8, 1)

    def disablePeakLimiter(self):
        self._writeRegisterBit(WM8960_REG_ALC3, 8, 0)

    # Noise Gate
    def enableNoiseGate(self):
        self._writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 1)
    def disableNoiseGate(self):
        self._writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 0)

    # 0-31, 0 = -76.5dBfs, 31 = -30dBfs
    def setNoiseGateThreshold(self, threshold:int):
        # TODO
        pass

    ## DAC

    # Enable/disble each channel
    def enableDacLeft(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 1)
    def disableDacLeft(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 0)

    def enableDacRight(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 1)
    def disableDacRight(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 0)

    def enableDac(self):
        self.enableDacLeft()
        self.enableDacRight()
    def disableDac(self):
        self.disableDacLeft()
        self.disableDacRight()

    # DAC digital volume
    # Valid inputs are 0-255
    # 0 = mute
    # 1 = -127dB
    # ... 0.5dB steps up to
    # 255 = 0dB
    def setDacLeftDigitalVolume(self, volume:int):
        self._writeRegisterMultiBits(WM8960_REG_LEFT_DAC_VOLUME, 7, 0, volume)
        self.dacLeftDACVUSet()

    def setDacRightDigitalVolume(self, volume:int):
        self._writeRegisterMultiBits(WM8960_REG_RIGHT_DAC_VOLUME, 7, 0, volume)
        self.dacRightDACVUSet()

    '''
    DAC digital volume DB
    Sets the volume of the DAC to a specified dB value passed in as a float argument.
    Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
    -97.50 (or lower) = MUTE
    -97.00 = -97.00dB (MIN)
    ... 0.5dB steps ...
    30.00 = +30.00dB  (MAX)
    '''
    def setDacLeftDigitalVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setDacLeftDigitalVolume()
        volume = self.convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX)
        self.setDacLeftDigitalVolume(volume)

    def setDacRightDigitalVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setDacRightDigitalVolume()
        volume = self.convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX)
        self.setDacRightDigitalVolume(volume)

    # Causes left and right input DAC volumes to be updated
    def dacLeftDACVUSet(self):
        self._writeRegisterBit(WM8960_REG_LEFT_DAC_VOLUME, 8, 1)

    # Causes left and right input DAC volumes to be updated
    def dacRightDACVUSet(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_DAC_VOLUME, 8, 1)

    # DAC mute
    def enableDacMute(self):
        self._writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 1)
    def disableDacMute(self):
        self._writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 0)

    # DE-Emphasis

    # 3D Stereo Enhancement
    # 3D enable/disable
    def enable3d(self):
        self._writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 1)
    def disable3d(self):
        self._writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 0)
    def set3dDepth(self, depth:int): # 0 = 0%, 15 = 100%
        # Limit incoming values max
        if depth > 15:
            depth = 15
        self._writeRegisterMultiBits(WM8960_REG_3D_CONTROL, 4, 1, depth)

    # 3D upper/lower cut-off frequencies.

    # DAC output -6dB attentuation enable/disable
    def enableDac6dbAttenuation(self):
        self._writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 1)
    def disableDac6dbAttentuation(self):
        self._writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 0)

    ## OUTPUT mixers

    # What's connected to what? Oh so many options...
    # LOMIX	Left Output Mixer
    # ROMIX	Right Output Mixer
    # OUT3MIX		Mono Output Mixer

    # Enable/disable left and right output mixers
    def enableLOMIX(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 1)
    def disableLOMIX(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 0)

    def enableROMIX(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 1)
    def disableROMIX(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 0)

    def enableOUT3MIX(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 1)
    def disableOUT3MIX(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 0)

    # Enable/disable audio path connections/vols to/from output mixers
    # See datasheet page 35 for a nice image of all the connections.

    def enableLI2LO(self):
        self._writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 1)
    def disableLI2LO(self):
        self._writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 0)

    # 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
    def setLI2LOVOL(self, volume:int):
        self._writeRegisterMultiBits(WM8960_REG_LEFT_OUT_MIX_1, 6, 4, volume)

    def enableLB2LO(self):
        self._writeRegisterBit(WM8960_REG_BYPASS_1, 7, 1)
    def disableLB2LO(self):
        self._writeRegisterBit(WM8960_REG_BYPASS_1, 7, 0)

    # 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
    def setLB2LOVOL(self, volume:int):
        # Limit incoming values max
        if volume > 7:
            volume = 7
        self._writeRegisterMultiBits(WM8960_REG_BYPASS_1, 6, 4, volume)

    def enableLD2LO(self):
        self._writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 1)
    def disableLD2LO(self):
        self._writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 0)

    def enableRI2RO(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 1)
    def disableRI2RO(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 0)

    # 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
    def setRI2ROVOL(self, volume:int):
        # Limit incoming values max
        if volume > 7:
            volume = 7
        self._writeRegisterMultiBits(WM8960_REG_RIGHT_OUT_MIX_2, 6, 4, volume)

    def enableRB2RO(self):
        self._writeRegisterBit(WM8960_REG_BYPASS_2, 7, 1)
    def disableRB2RO(self):
        self._writeRegisterBit(WM8960_REG_BYPASS_2, 7, 0)

    # 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
    def setRB2ROVOL(self, volume:int):
        # Limit incoming values max
        if volume > 7:
            volume = 7
        self._writeRegisterMultiBits(WM8960_REG_BYPASS_2, 6, 4, volume)

    def enableRD2RO(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 1)
    def disableRD2RO(self):
        self._writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 0)

    # Mono Output mixer.
    # Note, for capless HPs, we'll want this to output a buffered VMID.
    # To do this, we need to disable both of these connections.
    def enableLI2MO(self):
        self._writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 1)
    def disableLI2MO(self):
        self._writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 0)

    def enableRI2MO(self):
        self._writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 1)
    def disableRI2MO(self):
        self._writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 0)

    # Paired stereo functions to enable/disable output mixers
    def enableAdc2OutputMixer(self):
        self.enableLI2LO()
        self.enableRI2RO()
    def disableAdc2OutputMixer(self):
        self.disableLI2LO()
        self.disableRI2RO()
    def setAdc2MixerGain(self, volume:int):
        self.setLI2LOVOL(volume)
        self.setRI2ROVOL(volume)

    def enableBoost2OutputMixer(self):
        self.enableLB2LO()
        self.enableRB2RO()
    def disableBoost2OutputMixer(self):
        self.disableLB2LO()
        self.disableRB2RO()
    def setBoost2MixerGain(self, volume:int):
        self.setLB2LOVOL(volume)
        self.setRB2ROVOL(volume)

    def enableDac2OutputMixer(self):
        self.enableLD2LO()
        self.enableRD2RO()
    def disableDac2OutputMixer(self):
        self.disableLD2LO()
        self.disableRD2RO()
    
    def enableAdc2MonoOutput(self):
        self.enableLI2MO()
        self.enableRI2MO()
    def disableAdc2MonoOutput(self):
        self.disableLI2MO()
        self.disableRI2MO()

    def enableOutputMixer(self):
        self.enableLOMIX()
        self.enableROMIX()
    def disableOutputMixer(self):
        self.disableLOMIX()
        self.disableROMIX()

    # Sets the VMID signal to one of three possible settings.
    # 4 options:
    # WM8960_VMIDSEL_DISABLED
    # WM8960_VMIDSEL_2X50KOHM (playback / record)
    # WM8960_VMIDSEL_2X250KOHM (for low power / standby)
    # WM8960_VMIDSEL_2X5KOHM (for fast start-up)
    def setVMID(self, setting:int = WM8960_VMIDSEL_2X50KOHM):
        self._writeRegisterMultiBits(WM8960_REG_PWR_MGMT_1, 8, 7, setting)
    
    # Enables VMID in the WM8960_REG_PWR_MGMT_2 register, and set's it to 
    # playback/record settings of 2*50Kohm.
    # Note, this function is only here for backwards compatibility with the
    # original releases of this library. It is recommended to use the
    # setVMID() function instead.
    def enableVMID(self):
        self.setVMID(WM8960_VMIDSEL_2X50KOHM)
    def disableVMID(self):
        self.setVMID(WM8960_VMIDSEL_DISABLED)

    # This will disable both connections, thus enable VMID on OUT3. Note,
    # to enable VMID, you also need to enable OUT3 in the
    # WM8960_REG_PWR_MGMT_2 [1]
    def enableOUT3asVMID(self):
        self.disableLI2MO()
        self.disableRI2MO()
        self.enableOUT3MIX()
        self.enableVMID()
    
    ## Headphones

    # Enable and disable headphones (mute)
    def enableHeadphones(self):
        self.enableRightHeadphone()
        self.enableLeftHeadphone()
    def disableHeadphones(self):
        self.disableRightHeadphone()
        self.disableLeftHeadphone()

    def enableRightHeadphone(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 1)
    def disableRightHeadphone(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 0)
    def enableLeftHeadphone(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 1)
    def disableLeftHeadphone(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 0)

    def enableHeadphoneStandby(self):
        self._writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 1)
    def disableHeadphoneStandby(self):
        self._writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 0)

    # Set headphone volume
    # Although you can control each headphone output independently, here
    # we are going to assume you want both left and right to do the same
    # thing.

    # Valid inputs are 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ...
    # 127 = +6dB
    def setHeadphoneVolume(self, volume:int):
        # Updates both left and right channels
        # Handles the OUT1VU (volume update) bit control, so that it happens at the 
        # same time on both channels. Note, we must also make sure that the outputs 
        # are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]
        # Grab local copy of register
        # Modify the bits we need to
        # Write register in device, including the volume update bit write
        # If successful, save locally.

        # Limit inputs
        if volume > 127:
            volume = 127

        # LEFT
        self._writeRegisterMultiBits(WM8960_REG_LOUT1_VOLUME, 6, 0, volume)

        # RIGHT
        self._writeRegisterMultiBits(WM8960_REG_ROUT1_VOLUME, 6, 0, volume)

        # UPDATES
        # Updated left channel
        self._writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 8, 1)

        # Updated right channel
        self._writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 8, 1)

    # Set headphone volume dB
    # Sets the volume of the headphone output buffer amp to a speicified
    # dB value passed in as a float argument.
    # Valid dB settings are -74.0 up to +6.0
    # User input will be rounded to nearest whole integer
    # -74 (or lower) = MUTE
    # -73 = -73dB (MIN)
    # ... 1dB steps ...
    # 0 = 0dB
    # ... 1dB steps ...
    # 6 = +6dB  (MAX)
    def setHeadphoneVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setHeadphoneVolume()
        volume = self.convertDBtoSetting(dB, WM8960_HP_GAIN_OFFSET, WM8960_HP_GAIN_STEPSIZE, WM8960_HP_GAIN_MIN, WM8960_HP_GAIN_MAX)
        self.setHeadphoneVolume(volume)

    # Zero Cross prevents zipper sounds on volume changes
    # Sets both left and right Headphone outputs
    def enableHeadphoneZeroCross(self):
        # Left
        self._writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 1)
        # Right
        self._writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 1)

    def disableHeadphoneZeroCross(self):
        # Left
        self._writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 0)
        # Right
        self._writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 0)

    ## Speakers

    # Enable and disable speakers (mute)
    def enableSpeakers(self):
        self.enableRightSpeaker()
        self.enableLeftSpeaker()
    def disableSpeakers(self):
        self.disableRightSpeaker()
        self.disableLeftSpeaker()

    def enableRightSpeaker(self):
        # SPK_OP_EN
        self._writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 1)
        # SPKR
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 1)

    def disableRightSpeaker(self):
        # SPK_OP_EN
        self._writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 0)
        # SPKR
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 0)

    def enableLeftSpeaker(self):
        # SPK_OP_EN
        self._writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 1)
        # SPKL
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 1)

    def disableLeftSpeaker(self):
        # SPK_OP_EN
        self._writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 0)
        # SPKL
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 0)

    # Set Speaker output volume
    # Although you can control each Speaker output independently, here we
    # are going to assume you want both left and right to do the same thing.
    # Valid inputs are 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ...
    # 127 = +6dB

    def setSpeakerVolume(self, volume:int):
        # Updates both left and right channels
        # Handles the SPKVU (volume update) bit control, so that it happens at the 
        # same time on both channels. Note, we must also make sure that the outputs 
        # are enabled in the WM8960_REG_PWR_MGMT_2 [4:3], and the class D control 
        # reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

        # Limit inputs
        if volume > 127:
            volume = 127

        # LEFT
        self._writeRegisterMultiBits(WM8960_REG_LOUT2_VOLUME, 6, 0, volume)
        # RIGHT
        self._writeRegisterMultiBits(WM8960_REG_ROUT2_VOLUME, 6, 0, volume)

        # SPKVU
        # Updated left channel
        self._writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 8, 1)
        # Updated right channel
        self._writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 8, 1)

    def setSpeakerVolumeDB(self, dB:float):
        # Create an unsigned integer volume setting variable we can send to setSpeakerVolume()
        volume = self.convertDBtoSetting(dB, WM8960_SPEAKER_GAIN_OFFSET, WM8960_SPEAKER_GAIN_STEPSIZE, WM8960_SPEAKER_GAIN_MIN, WM8960_SPEAKER_GAIN_MAX)
        self.setSpeakerVolume(volume)

    # Zero Cross prevents zipper sounds on volume changes
    # Sets both left and right Speaker outputs
    def enableSpeakerZeroCross(self):
        # Left
        self._writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1)
        # Right
        self._writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 1)

    def disableSpeakerZeroCross(self):
        # Left
        self._writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 0)
        # Right
        self._writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 0)

    # DC and AC gain - allows signal to be higher than the DACs swing
    # (use only if your SPKVDD is high enough to handle a larger signal)
    # Valid inputs are 0-5
    # 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
    def setSpeakerDcGain(self, gain:int):
        # Limit incoming values max
        if gain > 5:
            gain = 5
        self._writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3, 5, 3, gain)

    def setSpeakerAcGain(self, gain:int):
        # Limit incoming values max
        if gain > 5:
            gain = 5
        self._writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3, 2, 0, gain)

    ## Digital audio interface control

    # Defaults to I2S, peripheral-mode, 24-bit word length

    # Loopback
    # When enabled, the output data from the ADC audio interface is fed
    # directly into the DAC data input.
    def enableLoopBack(self):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 1)
    def disableLoopBack(self):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 0)

    ## Clock controls

    # Getting the Frequency of SampleRate as we wish
    # Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
    # According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a
    # SR of 44.1KHz. To get that Desired Output (SYSCLK), we need the
    # following settings on the PLL stuff:
    # As found on table 45 (ds pg 61).
    # PRESCALE DIVIDE (PLLPRESCALE): 2
    # POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
    # FIXED POST-DIVIDE: 4
    # R: 7.5264
    # N: 7h
    # K: 86C226h

    # Example at bottom of table 46, shows that we should be in fractional
    # mode for a 44.1KHz.

    # In terms of registers, this is what we want for 44.1KHz
    # PLLEN=1			(PLL enable)
    # PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down
    # to 12MHZ for F2
    # PLLN=7h			(PLL N value) *this is "int R"
    # PLLK=86C226h		(PLL K value) *this is int ( 2^24 * (R- intR))
    # SDM=1			(Fractional mode)
    # CLKSEL=1			(PLL select)
    # MS=0				(Peripheral mode)
    # WL=00			(16 bits)
    # SYSCLKDIV=2		(Divide by 2)
    # ADCDIV=000		(Divide by 1) = 44.1kHz
    # DACDIV=000		(Divide by 1) = 44.1kHz
    # BCLKDIV=0100		(Divide by 4) = 64fs
    # DCLKDIV=111		(Divide by 16) = 705.6kHz

    # And now for the functions that will set these registers...
    def enablePLL(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 1)
    def disablePLL(self):
        self._writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 0)

    # Valid options are WM8960_PLLPRESCALE_DIV_1, WM8960_PLLPRESCALE_DIV_2
    def setPLLPRESCALE(self, div:bool):
        self._writeRegisterBit(WM8960_REG_PLL_N, 4, div)

    def setPLLN(self, n:int):
        self._writeRegisterMultiBits(WM8960_REG_PLL_N, 3, 0, n)

    # Send each nibble of 24-bit value for value K
    def setPLLK(self, k:int):
        self._writeRegisterMultiBits(WM8960_REG_PLL_K_1, 5, 0, (k >> 16) & 0x1F)
        self._writeRegisterMultiBits(WM8960_REG_PLL_K_2, 8, 0, (k >> 8) & 0xFF)
        self._writeRegisterMultiBits(WM8960_REG_PLL_K_3, 8, 0, k & 0xFF)

    # 0=integer, 1=fractional
    def setSMD(self, mode:bool):
        self._writeRegisterBit(WM8960_REG_PLL_N, 5, mode)

    # 0=MCLK, 1=PLL_output
    def setCLKSEL(self, sel:bool):
        self._writeRegisterBit(WM8960_REG_CLOCKING_1, 0, sel)

    # (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
    def setSYSCLKDIV(self, div:int):
        self._writeRegisterMultiBits(WM8960_REG_CLOCKING_1, 2, 1, div)

    # 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
    def setADCDIV(self, div:int):
        self._writeRegisterMultiBits(WM8960_REG_CLOCKING_1, 8, 6, div)

    # 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
    def setDACDIV(self, div:int):
        self._writeRegisterMultiBits(WM8960_REG_CLOCKING_1, 5, 3, div)

    # 0100 (4) = sufficiently high for 24bit, div by 4 allows for max word
    # length of 32bit
    def setBCLKDIV(self, div:int):
        self._writeRegisterMultiBits(WM8960_REG_CLOCKING_2, 3, 0, div)

    # Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
    def setDCLKDIV(self, div:int):
        self._writeRegisterMultiBits(WM8960_REG_CLOCKING_2, 8, 6, div)

    # Set LR clock to be the same for ADC & DAC (needed for loopback mode)
    def setALRCGPIO(self):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 6, 1)

    def enableMasterMode(self):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 1)
    
    def enablePeripheralMode(self):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 0)

    def setWL(self, word_length:int):
        self._writeRegisterMultiBits(WM8960_REG_AUDIO_INTERFACE_1, 3, 2, word_length)

    def setLRP(self, polarity:bool):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 4, polarity)

    def setALRSWAP(self, swap:bool):
        self._writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 8, swap)

    def setVROI(self, setting:bool):
        self._writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_3, 6, setting)

    def setVSEL(self, setting:int):
        self._writeRegisterMultiBits(WM8960_REG_ADDITIONAL_CONTROL_1, 7, 6, setting)

    # General-purpose register write
    def writeRegister(self, reg:int, value:int) -> None:
        self._buf[0] = reg << 1 | value >> 8
        self._buf[1] = value & 0xff
        with self.i2c_device as i2c:
            i2c.write(self._buf)
        self._registerLocalCopy[reg] = value

    # **The WM8960 does not support reading registers!!!

    # Writes a 0 or 1 to the desired bit in the desired register
    def _writeRegisterBit(self, registerAddress:int, bitNumber:int, bitValue:bool) -> None:
        regvalue = self._registerLocalCopy[registerAddress]
        if bitValue:
            regvalue |= 1<<bitNumber
        else:
            regvalue &= ~(1<<bitNumber)

        self.writeRegister(registerAddress, regvalue)

    '''
    This function writes data into the desired bits within the desired register
    Some settings require more than just flipping a single bit within a register.
    For these settings use this more advanced register write helper function.

    For example, to change the LIN2BOOST setting to +6dB,
    I need to write a setting of 7 (aka +6dB) to the bits [3:1] in the
    WM8960_REG_INPUT_BOOST_MIXER_1 register. Like so...
    _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1, 3, 1, 7);
    '''
    def _writeRegisterMultiBits(self, registerAddress:int, settingMsbNum:int, settingLsbNum:int, setting:int):
        regvalue = self._registerLocalCopy[registerAddress]

        # Clear bits we care about
        numOfBits = (settingMsbNum - settingLsbNum) + 1
        for i in range(numOfBits):
            regvalue &= ~(1 << (settingLsbNum + i))

        # Shift and set the bits from in incoming desired setting value
        regvalue |= setting << settingLsbNum

        # Write modified value to device
        self.writeRegister(registerAddress, regvalue)

    def convertDBtoSetting(self, dB:float, offset:float, stepSize:float, minDB:float, maxDB:float) -> int:
        '''
        Limit incoming dB values to acceptable range. Note, the minimum limit we
        want to limit this too is actually one step lower than the minDB, because
        that is still an acceptable dB level (it is actually "true mute").
        Note, the PGA amp does not have a "true mute" setting available, so we
        must check for its unique minDB of -17.25.
        '''

        # Limit max. This is the same for all amps.
        if dB > maxDB:
            dB = maxDB

        '''
        PGA amp doesn't have mute setting, so minDB should be limited to minDB
        Let's check for the PGAs unique minDB (-17.25) to know we are currently
        converting a PGA setting.
        '''
        if minDB == WM8960_PGA_GAIN_MIN:
            if dB < minDB:
                dB = minDB
        else: # Not PGA. All other amps have a mute setting below minDb
            if dB < minDB - stepSize:
                dB = minDB - stepSize

        '''
        Adjust for offset
        Offset is the number that gets us from the minimum dB option of an amp
        up to the minimum setting value in the register.
        '''
        dB = dB + offset

        '''
        Find out how many steps we are above the minimum (at this point, our
        minimum is "0". Note, because dB comes in as a float, the result of this
        division (volume) can be a partial number. We will round that next.
        '''
        volume = dB / stepSize
        volume = round(volume) # round to the nearest setting value.

        return int(volume) & 0xff # cast from float to integer
