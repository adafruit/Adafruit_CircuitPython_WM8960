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
https://github.com/sparkfun/SparkFun_Arduino_Library

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_WM8960.git"

from busio import I2C
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_simplemath import constrain, map_range
import math
from micropython import const

try:
    from typing import Optional, Type, Protocol
    from circuitpython_typing.device_drivers import I2CDeviceDriver
except ImportError:
    pass

# I2C address
_DEFAULT_I2C_ADDR = const(0x1A)

# WM8960 register addresses
_REG_LEFT_INPUT_VOLUME = const(0x00)
_REG_RIGHT_INPUT_VOLUME = const(0x01)
_REG_LOUT1_VOLUME = const(0x02)
_REG_ROUT1_VOLUME = const(0x03)
_REG_CLOCKING_1 = const(0x04)
_REG_ADC_DAC_CTRL_1 = const(0x05)
_REG_ADC_DAC_CTRL_2 = const(0x06)
_REG_AUDIO_INTERFACE_1 = const(0x07)
_REG_CLOCKING_2 = const(0x08)
_REG_AUDIO_INTERFACE_2 = const(0x09)
_REG_LEFT_DAC_VOLUME = const(0x0A)
_REG_RIGHT_DAC_VOLUME = const(0x0B)
_REG_RESET = const(0x0F)
_REG_3D_CONTROL = const(0x10)
_REG_ALC1 = const(0x11)
_REG_ALC2 = const(0x12)
_REG_ALC3 = const(0x13)
_REG_NOISE_GATE = const(0x14)
_REG_LEFT_ADC_VOLUME = const(0x15)
_REG_RIGHT_ADC_VOLUME = const(0x16)
_REG_ADDITIONAL_CONTROL_1 = const(0x17)
_REG_ADDITIONAL_CONTROL_2 = const(0x18)
_REG_PWR_MGMT_1 = const(0x19)
_REG_PWR_MGMT_2 = const(0x1A)
_REG_ADDITIONAL_CONTROL_3 = const(0x1B)
_REG_ANTI_POP_1 = const(0x1C)
_REG_ANTI_POP_2 = const(0x1D)
_REG_ADCL_SIGNAL_PATH = const(0x20)
_REG_ADCR_SIGNAL_PATH = const(0x21)
_REG_LEFT_OUT_MIX = const(0x22)
_REG_RIGHT_OUT_MIX = const(0x25)
_REG_MONO_OUT_MIX_1 = const(0x26)
_REG_MONO_OUT_MIX_2 = const(0x27)
_REG_LOUT2_VOLUME = const(0x28)
_REG_ROUT2_VOLUME = const(0x29)
_REG_MONO_OUT_VOLUME = const(0x2A)
_REG_INPUT_BOOST_MIXER_1 = const(0x2B)
_REG_INPUT_BOOST_MIXER_2 = const(0x2C)
_REG_BYPASS_1 = const(0x2D)
_REG_BYPASS_2 = const(0x2E)
_REG_PWR_MGMT_3 = const(0x2F)
_REG_ADDITIONAL_CONTROL_4 = const(0x30)
_REG_CLASS_D_CONTROL_1 = const(0x31)
_REG_CLASS_D_CONTROL_3 = const(0x33)
_REG_PLL_N = const(0x34)
_REG_PLL_K_1 = const(0x35)
_REG_PLL_K_2 = const(0x36)
_REG_PLL_K_3 = const(0x37)

# PGA input selections
PGA_INPUT2 = 0
PGA_INPUT3 = 1
PGA_VMID = 2

# Mic (aka PGA) BOOST gain options
MIC_BOOST_GAIN_0DB = 0
MIC_BOOST_GAIN_13DB = 1
MIC_BOOST_GAIN_20DB = 2
MIC_BOOST_GAIN_29DB = 3

'''
Boost Mixer gain options
These are used to control the gain (aka volume) at the following settings:
LIN2BOOST
LIN3BOOST
RIN2BOOST
RIN3BOOST
'''
BOOST_MIXER_GAIN_MUTE = 0
BOOST_MIXER_GAIN_NEG_12DB = 1
BOOST_MIXER_GAIN_NEG_9DB = 2
BOOST_MIXER_GAIN_NEG_6DB = 3
BOOST_MIXER_GAIN_NEG_3DB = 4
BOOST_MIXER_GAIN_0DB = 5
BOOST_MIXER_GAIN_3DB = 6
BOOST_MIXER_GAIN_6DB = 7

# Mic Bias voltage options
MIC_BIAS_VOLTAGE_0_9_AVDD = 0
MIC_BIAS_VOLTAGE_0_65_AVDD = 1

# SYSCLK divide
_SYSCLK_DIV_BY_1 = const(0)
_SYSCLK_DIV_BY_2 = const(2)

# Gain/Level mins, maxes, offsets and step-sizes
_PGA_GAIN_MIN = -17.25
_PGA_GAIN_MAX = 30.00

_ADC_VOLUME_MIN = -97.00
_ADC_VOLUME_MAX = 30.00

_DAC_VOLUME_MIN = -127.00
_DAC_VOLUME_MAX = 0.00

_ALC_TARGET_MIN = -22.50
_ALC_TARGET_MAX = -1.50

_ALC_MAX_GAIN_MIN = -12.00
_ALC_MAX_GAIN_MAX = 30.00

_ALC_MIN_GAIN_MIN = -17.25
_ALC_MIN_GAIN_MAX = 24.75

_GATE_THRESHOLD_MIN = -76.50
_GATE_THRESHOLD_MAX = -30.00

_OUTPUT_VOLUME_MIN = -21.00
_OUTPUT_VOLUME_MAX = 0.00

_AMP_VOLUME_MIN = -73.00
_AMP_VOLUME_MAX = 6.00

# ALC Time mapping
_ALC_ATTACK_MAX = const(10)
_ALC_ATTACK_TIME_MIN = 0.006
_ALC_ATTACK_TIME_MAX = 6.140

_ALC_DECAY_MAX = const(10)
_ALC_DECAY_TIME_MIN = 0.024
_ALC_DECAY_TIME_MAX = 24.580

_ALC_HOLD_TIME_MIN = 0.00267
_ALC_HOLD_TIME_MAX = 43.691

# Speaker Boost Gains (DC and AC)
SPEAKER_BOOST_GAIN_0DB = 0
SPEAKER_BOOST_GAIN_2_1DB = 1
SPEAKER_BOOST_GAIN_2_9DB = 2
SPEAKER_BOOST_GAIN_3_6DB = 3
SPEAKER_BOOST_GAIN_4_5DB = 4
SPEAKER_BOOST_GAIN_5_1DB = 5

# VMIDSEL settings
VMIDSEL_DISABLED = 0
VMIDSEL_PLAYBACK = 1 # 2X50KOHM
VMIDSEL_LOWPOWER = 2 # 2X250KOHM
VMIDSEL_FASTSTART = 3 # 2X5KOHM

# Clock Divider Tables
_BCLKDIV = [
    1.0,
    1.5,
    2.0,
    3.0,
    4.0,
    5.5,
    6.0,
    8.0,
    11.0,
    12.0,
    16.0,
    22.0,
    24.0,
    32.0
]

_DCLKDIV = [
    1.5,
    2.0,
    3.0,
    4.0,
    6.0,
    8.0,
    12.0,
    16.0
]

_ADCDACDIV = [
    1.0,
    1.5,
    2.0,
    3.0,
    4.0,
    5.5,
    6.0
]

class WOBit:

    def __init__(
        self,
        register_address: int,
        bit: int,
    ) -> None:
        self.register_address = register_address
        self.bit = bit
        self.bit_mask = 1 << (bit % 8) # the bitmask *within* the byte!
        self.byte = 1 - (bit // 8) # the byte number within the buffer

    def _set(self, obj:Optional[I2CDeviceDriver], value:bool) -> None:
        if value:
            obj._registers[self.register_address][self.byte] |= self.bit_mask
        else:
            obj._registers[self.register_address][self.byte] &= ~self.bit_mask
    
    def reset(self, obj: Optional[I2CDeviceDriver]) -> None:
        if not hasattr(self, 'default'):
            self.default = self.__get__(obj)
        self._set(obj, self.default)

    def __get__(
        self,
        obj: Optional[I2CDeviceDriver],
        objtype: Optional[Type[I2CDeviceDriver]] = None,
    ) -> bool:
        return bool(obj._registers[self.register_address][self.byte] & self.bit_mask)

    def __set__(self, obj: Optional[I2CDeviceDriver], value: bool) -> None:
        self._set(obj, value)
        with obj.i2c_device as i2c:
            i2c.write(obj._registers[self.register_address])

class WOBits:

    def __init__(  # pylint: disable=too-many-arguments
        self,
        num_bits: int,
        register_address: int,
        lowest_bit: int,
    ) -> None:
        self.register_width = 1
        self.register_address = register_address
        self.bit_mask = ((1 << num_bits) - 1) << lowest_bit
        self.lowest_bit = lowest_bit

    def _set(self, obj:Optional[I2CDeviceDriver], value:int) -> None:
        value <<= self.lowest_bit  # shift the value over to the right spot
        reg = 0
        for i in range(2):
            reg = (reg << 8) | obj._registers[self.register_address][i]
        reg &= ~self.bit_mask  # mask off the bits we're about to change
        reg |= value  # then or in our new value
        for i in range(1, -1, -1):
            obj._registers[self.register_address][i] = reg & 0xFF
            reg >>= 8
    
    # Must call first before using object
    def reset(self, obj:Optional[I2CDeviceDriver]) -> None:
        if not hasattr(self, 'default'):
            self.default = self.__get__(obj)
        self._set(obj, self.default)

    def __get__(
        self,
        obj: Optional[I2CDeviceDriver],
        objtype: Optional[Type[I2CDeviceDriver]] = None,
    ) -> int:
        # read the number of bytes into a single variable
        reg = 0
        for i in range(2):
            reg = (reg << 8) | obj._registers[self.register_address][i]
        reg = (reg & self.bit_mask) >> self.lowest_bit
        return reg

    def __set__(self, obj: Optional[I2CDeviceDriver], value: int) -> None:
        self._set(obj, value)
        with obj.i2c_device as i2c:
            i2c.write(obj._registers[self.register_address])

class WM8960:

    # Power

    vref = WOBit(_REG_PWR_MGMT_1, 6)

    analogInputLeft = WOBit(_REG_PWR_MGMT_1, 5)
    analogInputRight = WOBit(_REG_PWR_MGMT_1, 4)

    @property
    def analogInput(self) -> bool:
        return self.analogInputLeft and self.analogInputRight
    @analogInput.setter
    def analogInput(self, value:bool) -> None:
        self.analogInputLeft = self.analogInputRight = value

    # PGA
    
    ## PWR_MGMT

    micLeft = WOBit(_REG_PWR_MGMT_3, 5)
    micRight = WOBit(_REG_PWR_MGMT_3, 4)

    @property
    def mic(self) -> bool:
        return self.micLeft and self.micRight
    @mic.setter
    def mic(self, value:bool) -> None:
        self.micLeft = self.micRight = value

    ## SIGNAL_PATH

    _micInput1Left = WOBit(_REG_ADCL_SIGNAL_PATH, 8)
    _micInput2Left = WOBit(_REG_ADCL_SIGNAL_PATH, 6)
    _micInput3Left = WOBit(_REG_ADCL_SIGNAL_PATH, 7)

    _micInput1Right = WOBit(_REG_ADCR_SIGNAL_PATH, 8)
    _micInput2Right = WOBit(_REG_ADCR_SIGNAL_PATH, 6)
    _micInput3Right = WOBit(_REG_ADCR_SIGNAL_PATH, 7)

    @property
    def micNonInvSignalLeft(self) -> int:
        if self._micInput2Left:
            return PGA_INPUT2
        elif self._micInput3Left:
            return PGA_INPUT3
        else:
            return PGA_VMID
    @micNonInvSignalLeft.setter
    def micNonInvSignalLeft(self, signal:int) -> None:
        self._micInput2Left = signal == PGA_INPUT2
        self._micInput3Left = signal == PGA_INPUT3
    
    @property
    def micNonInvSignalRight(self) -> int:
        if self._micInput2Right:
            return PGA_INPUT2
        elif self._micInput3Right:
            return PGA_INPUT3
        else:
            return PGA_VMID
    @micNonInvSignalRight.setter
    def micNonInvSignalRight(self, signal:int) -> None:
        self._micInput2Right = signal == PGA_INPUT2
        self._micInput3Right = signal == PGA_INPUT3

    @property
    def micNonInvSignal(self) -> int:
        # NOTE: Not checking right signal
        return self.micNonInvSignalLeft
    @micNonInvSignal.setter
    def micNonInvSignal(self, signal:int) -> None:
        self.micNonInvSignalLeft = self.micNonInvSignalRight = signal

    ## Boost

    micLeftBoost = WOBit(_REG_ADCL_SIGNAL_PATH, 3)
    micRightBoost = WOBit(_REG_ADCR_SIGNAL_PATH, 3)

    @property
    def micBoost(self) -> None:
        return self.micLeftBoost and self.micRightBoost
    @micBoost.setter
    def micBoost(self, value:int) -> None:
        self.micLeftBoost = self.micRightBoost = value

    micBoostGainLeft = WOBits(2, _REG_ADCL_SIGNAL_PATH, 4)
    micBoostGainRight = WOBits(2, _REG_ADCR_SIGNAL_PATH, 4)

    @property
    def micBoostGain(self) -> None:
        return max(self.micBoostGainLeft, self.micBoostGainRight)
    @micBoostGain.setter
    def micBoostGain(self, value:int) -> None:
        self.micGainLeft = self.micGainRight = value

    ## Volume

    _micLeftVolume = WOBits(6, _REG_LEFT_INPUT_VOLUME, 0)
    _micLeftVolumeSet = WOBit(_REG_LEFT_INPUT_VOLUME, 8)

    _micRightVolume = WOBits(6, _REG_RIGHT_INPUT_VOLUME, 0)
    _micRightVolumeSet = WOBit(_REG_RIGHT_INPUT_VOLUME, 8)

    @property
    def micLeftVolume(self) -> int:
        return self._micLeftVolume
    @micLeftVolume.setter
    def micLeftVolume(self, value:int) -> None:
        self._micLeftVolume = value
        self._micLeftVolumeSet = True

    @property
    def micLeftVolumeDb(self) -> float:
        return map_range(self.micLeftVolume, 0, 63, _PGA_GAIN_MIN, _PGA_GAIN_MAX)
    @micLeftVolumeDb.setter
    def micLeftVolumeDb(self, value:float) -> None:
        self.micLeftVolume = round(map_range(value, _PGA_GAIN_MIN, _PGA_GAIN_MAX, 0, 63))
    
    @property
    def micRightVolume(self) -> int:
        return self._micRightVolume
    @micRightVolume.setter
    def micRightVolume(self, value:int) -> None:
        self._micRightVolume = value
        self._micRightVolumeSet = True

    @property
    def micRightVolumeDb(self) -> float:
        return map_range(self.micRightVolume, 0, 63, _PGA_GAIN_MIN, _PGA_GAIN_MAX)
    @micRightVolumeDb.setter
    def micRightVolumeDb(self, value:float) -> None:
        self.micRightVolume = round(map_range(value, _PGA_GAIN_MIN, _PGA_GAIN_MAX, 0, 63))
    
    @property
    def micVolume(self) -> int:
        return self.micLeftVolume
    @micVolume.setter
    def micVolume(self, value:int) -> None:
        self.micLeftVolume = self.micRightVolume = value

    @property
    def micVolumeDb(self) -> float:
        return self.micLeftVolumeDb
    @micVolumeDb.setter
    def micVolumeDb(self, value:float) -> None:
        self.micVolume = round(map_range(value, _PGA_GAIN_MIN, _PGA_GAIN_MAX, 0, 63))

    ## Zero Cross

    micLeftZeroCross = WOBit(_REG_LEFT_INPUT_VOLUME, 6)
    micRightZeroCross = WOBit(_REG_RIGHT_INPUT_VOLUME, 6)

    @property
    def micZeroCross(self) -> bool:
        return self.micLeftZeroCross and self.micRightZeroCross
    @micZeroCross.setter
    def micZeroCross(self, value:bool) -> None:
        self.micLeftZeroCross = self.micRightZeroCross = value

    ## Mute

    _micLeftMute = WOBit(_REG_LEFT_INPUT_VOLUME, 7)
    _micRightMute = WOBit(_REG_RIGHT_INPUT_VOLUME, 7)

    @property
    def micLeftMute(self) -> bool:
        return self._micLeftMute
    @micLeftMute.setter
    def micLeftMute(self, value:bool) -> None:
        self._micLeftMute = value
        self._micLeftVolumeSet = True

    @property
    def micRightMute(self) -> bool:
        return self._micRightMute
    @micRightMute.setter
    def micRightMute(self, value:bool) -> None:
        self._micRightMute = value
        self._micRightVolumeSet = True

    @property
    def micMute(self) -> bool:
        return self._micLeftMute and self._micRightMute
    @micMute.setter
    def micMute(self, value:bool) -> None:
        self._micLeftMute = self._micRightMute = value

    # Boost Mixer

    input2LeftBoost = WOBits(3, _REG_INPUT_BOOST_MIXER_1, 1)
    input2RightBoost = WOBits(3, _REG_INPUT_BOOST_MIXER_2, 1)

    @property
    def input2Boost(self) -> int:
        return self.input2LeftBoost
    @input2Boost.setter
    def input2Boost(self, value:int) -> None:
        self.input2LeftBoost = self.input2RightBoost = value

    input3LeftBoost = WOBits(3, _REG_INPUT_BOOST_MIXER_1, 4)
    input3RightBoost = WOBits(3, _REG_INPUT_BOOST_MIXER_2, 4)

    # Mic Bias

    micBias = WOBit(_REG_PWR_MGMT_1, 1)
    micBiasVoltage = WOBit(_REG_ADDITIONAL_CONTROL_4, 0)

    @property
    def input3Boost(self) -> int:
        return self.input3LeftBoost
    @input3Boost.setter
    def input3Boost(self, value:int) -> None:
        self.input3LeftBoost = self.input3RightBoost = value

    # ADC

    adcLeft = WOBit(_REG_PWR_MGMT_1, 3)
    adcRight = WOBit(_REG_PWR_MGMT_1, 2)

    @property
    def adc(self) -> bool:
        return self.adcLeft and self.adcRight
    @adc.setter
    def adc(self, value:bool) -> None:
        self.adcLeft = self.adcRight = value
    
    ## Volume

    _adcLeftVolume = WOBits(8, _REG_LEFT_ADC_VOLUME, 0)
    _adcLeftVolumeSet = WOBit(_REG_LEFT_ADC_VOLUME, 8)

    @property
    def adcLeftVolume(self) -> int:
        return self._adcLeftVolume
    @adcLeftVolume.setter
    def adcLeftVolume(self, value:int) -> None:
        self._adcLeftVolume = value
        self._adcLeftVolumeSet = True

    @property
    def adcLeftVolumeDb(self) -> float:
        return map_range(max(self.adcLeftVolume, 1), 1, 255, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX)
    @adcLeftVolumeDb.setter
    def adcLeftVolumeDb(self, value:float) -> None:
        self.adcLeftVolume = round(map_range(value, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX, 0, 254) + 1.0)

    _adcRightVolume = WOBits(8, _REG_RIGHT_ADC_VOLUME, 0)
    _adcRightVolumeSet = WOBit(_REG_RIGHT_ADC_VOLUME, 8)

    @property
    def adcRightVolume(self) -> int:
        return self._adcRightVolume
    @adcRightVolume.setter
    def adcRightVolume(self, value:int) -> None:
        self._adcRightVolume = value
        self._adcRightVolumeSet = True

    @property
    def adcRightVolumeDb(self) -> float:
        return map_range(max(self.adcRightVolume, 1), 1, 255, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX)
    @adcRightVolumeDb.setter
    def adcRightVolumeDb(self, value:float) -> None:
        self.adcRightVolume = round(map_range(value, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX, 0, 254) + 1.0)

    @property
    def adcVolume(self) -> int:
        return self.adcLeftVolume
    @adcVolume.setter
    def adcVolume(self, value:float) -> None:
        self.adcLeftVolume = self.adcRightVolume = value
    
    @property
    def adcVolumeDb(self) -> int:
        return self.adcLeftVolumeDb
    @adcVolume.setter
    def adcVolume(self, value:float) -> None:
        self.adcLeftVolume = self.adcRightVolume = round(map_range(value, _ADC_VOLUME_MIN, _ADC_VOLUME_MAX, 0, 254) + 1.0)

    # ALC

    alcLeft = WOBit(_REG_ALC1, 7)
    alcRight = WOBit(_REG_ALC1, 8)

    @property
    def alc(self) -> bool:
        return self.alcLeft and self.alcRight
    @alc.setter
    def alc(self, value:bool) -> None:
        self.alcLeft = self.alcRight = value

    alcTarget = WOBits(4, _REG_ALC1, 0)

    @property
    def alcTargetDb(self) -> float:
        return map_range(self.alcTarget, 0, 15, _ALC_TARGET_MIN, _ALC_TARGET_MAX)
    @alcTargetDb.setter
    def alcTargetDb(self, value:float) -> None:
        self.alcTarget = round(map_range(value, _ALC_TARGET_MIN, _ALC_TARGET_MAX, 0, 15))

    alcMaxGain = WOBits(3, _REG_ALC1, 4)

    @property
    def alcMaxGainDb(self) -> float:
        return map_range(self.alcMaxGain, 0, 7, _ALC_MAX_GAIN_MIN, _ALC_MAX_GAIN_MAX)
    @alcMaxGainDb.setter
    def alcMaxGainDb(self, value:float) -> None:
        self.alcMaxGain = round(map_range(value, _ALC_MAX_GAIN_MIN, _ALC_MAX_GAIN_MAX, 0, 7))

    alcMinGain = WOBits(3, _REG_ALC2, 4)

    @property
    def alcMinGainDb(self) -> float:
        return map_range(self.alcMinGain, 0, 7, _ALC_MIN_GAIN_MIN, _ALC_MIN_GAIN_MAX)
    @alcMinGainDb.setter
    def alcMinGainDb(self, value:float) -> None:
        self.alcMinGain = round(map_range(value, _ALC_MIN_GAIN_MIN, _ALC_MIN_GAIN_MAX, 0, 7))

    _alcAttack = WOBits(4, _REG_ALC3, 0)

    @property
    def alcAttack(self) -> int:
        return self._alcAttack
    @alcAttack.setter
    def alcAttack(self, value:int) -> None:
        self._alcAttack = min(value, _ALC_ATTACK_MAX)
    
    @property
    def alcAttackTime(self) -> float:
        return _ALC_ATTACK_TIME_MIN * pow(2, self.alcAttack)
    @alcAttackTime.setter
    def alcAttackTime(self, value:float) -> None:
        self.alcAttack = round(math.log2((constrain(value, _ALC_ATTACK_TIME_MIN, _ALC_ATTACK_TIME_MAX) - _ALC_ATTACK_TIME_MIN) / _ALC_ATTACK_TIME_MIN))

    _alcDecay = WOBits(4, _REG_ALC3, 4)

    @property
    def alcDecay(self) -> int:
        return self._alcDecay
    @alcDecay.setter
    def alcDecay(self, value:int) -> None:
        self._alcDecay = min(value, _ALC_DECAY_MAX)

    @property
    def alcDecayTime(self) -> float:
        return _ALC_DECAY_TIME_MIN * pow(2, self.alcDecay)
    @alcDecayTime.setter
    def alcDecayTime(self, value:float) -> None:
        self.alcDecay = round(math.log2((constrain(value, _ALC_DECAY_TIME_MIN, _ALC_DECAY_TIME_MAX) - _ALC_DECAY_TIME_MIN) / _ALC_DECAY_TIME_MIN))

    alcHold = WOBits(4, _REG_ALC2, 0)

    @property
    def alcHoldTime(self) -> float:
        value = self.alcHold
        if value == 0: return 0.0
        return _ALC_HOLD_TIME_MIN * pow(2, self.alcHold - 1)
    @alcHoldTime.setter
    def alcHoldTime(self, value:float) -> None:
        if value <= 0.0:
            self.alcHold = 0
        else:
            self.alcHold = round(math.log2((constrain(value, _ALC_HOLD_TIME_MIN, _ALC_HOLD_TIME_MAX) - _ALC_HOLD_TIME_MIN) / _ALC_HOLD_TIME_MIN) + 1.0)

    alcLimiter = WOBit(_REG_ALC3, 8)

    # Noise Gate

    noiseGate = WOBit(_REG_NOISE_GATE, 0)
    noiseGateThreshold = WOBits(5, _REG_NOISE_GATE, 3)

    @property
    def noiseGateThresholdDb(self) -> float:
        return map_range(self.noiseGateThreshold, 0, 31, _GATE_THRESHOLD_MIN, _GATE_THRESHOLD_MAX)
    @noiseGateThresholdDb.setter
    def noiseGateThresholdDb(self, value:float) -> None:
        self.noiseGateThreshold = round(map_range(value, _GATE_THRESHOLD_MIN, _GATE_THRESHOLD_MAX, 0, 31))

    # DAC

    dacLeft = WOBit(_REG_PWR_MGMT_2, 8)
    dacRight = WOBit(_REG_PWR_MGMT_2, 7)

    @property
    def dac(self) -> bool:
        return self.dacLeft and self.dacRight
    @dac.setter
    def dac(self, value:bool) -> None:
        self.dacLeft = self.dacRight = value
    
    _dacLeftVolume = WOBits(8, _REG_LEFT_DAC_VOLUME, 0)
    _dacLeftVolumeSet = WOBit(_REG_LEFT_DAC_VOLUME, 8)

    @property
    def dacLeftVolume(self) -> int:
        return self._dacLeftVolume
    @dacLeftVolume.setter
    def dacLeftVolume(self, value:int) -> None:
        self._dacLeftVolume = value
        self._dacLeftVolumeSet = True

    @property
    def dacLeftVolumeDb(self) -> float:
        return map_range(max(self.dacLeftVolume, 1), 1, 255, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX)
    @dacLeftVolumeDb.setter
    def dacLeftVolumeDb(self, value:float) -> None:
        self.dacLeftVolume = round(map_range(value, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX, 0, 254) + 1.0)

    _dacRightVolume = WOBits(8, _REG_RIGHT_DAC_VOLUME, 0)
    _dacRightVolumeSet = WOBit(_REG_RIGHT_DAC_VOLUME, 8)

    @property
    def dacRightVolume(self) -> int:
        return self._dacRightVolume
    @dacRightVolume.setter
    def dacRightVolume(self, value:int) -> None:
        self._dacRightVolume = value
        self._dacRightVolumeSet = True

    @property
    def dacRightVolumeDb(self) -> float:
        return map_range(max(self.dacRightVolume, 1), 1, 255, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX)
    @dacRightVolumeDb.setter
    def dacRightVolumeDb(self, value:float) -> None:
        self.dacRightVolume = round(map_range(value, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX, 0, 254) + 1.0)

    @property
    def dacVolume(self) -> int:
        return self.dacLeftVolume
    @dacVolume.setter
    def dacVolume(self, value:float) -> None:
        self.dacLeftVolume = self.dacRightVolume = value
    
    @property
    def dacVolumeDb(self) -> int:
        return self.dacLeftVolumeDb
    @dacVolume.setter
    def dacVolume(self, value:float) -> None:
        self.dacLeftVolume = self.dacRightVolume = round(map_range(value, _DAC_VOLUME_MIN, _DAC_VOLUME_MAX, 0, 254) + 1.0)

    dacMute = WOBit(_REG_ADC_DAC_CTRL_1, 3)
    dacSoftMute = WOBit(_REG_ADC_DAC_CTRL_2, 3)
    dacSlowSoftMute = WOBit(_REG_ADC_DAC_CTRL_2, 2)
    dacAttenuation = WOBit(_REG_ADC_DAC_CTRL_1, 7)

    # 3D Enhance

    enhance = WOBit(_REG_3D_CONTROL, 0)
    enhanceDepth = WOBits(4, _REG_3D_CONTROL, 1)
    enhanceFilterLPF = WOBit(_REG_3D_CONTROL, 6)
    enhanceFilterHPF = WOBit(_REG_3D_CONTROL, 5)

    # Output Mixer

    leftOutput = WOBit(_REG_PWR_MGMT_3, 3)
    rightOutput = WOBit(_REG_PWR_MGMT_3, 2)

    @property
    def output(self) -> bool:
        return self.leftOutput and self.rightOutput
    @output.setter
    def output(self, value:bool):
        self.leftOutput = self.rightOutput = value

    ## DAC Output

    dacLeftOutput = WOBit(_REG_LEFT_OUT_MIX, 8)
    dacRightOutput = WOBit(_REG_RIGHT_OUT_MIX, 8)

    @property
    def dacOutput(self) -> bool:
        return self.dacLeftOutput and self.dacRightOutput
    @dacOutput.setter
    def dacOutput(self, value:bool) -> None:
        self.dacLeftOutput = self.dacRightOutput = value

    ## Input 3 Output

    input3LeftOutput = WOBit(_REG_LEFT_OUT_MIX, 7)
    input3RightOutput = WOBit(_REG_RIGHT_OUT_MIX, 7)
    
    @property
    def input3Output(self) -> bool:
        return self.input3LeftOutput and self.input3RightOutput
    @input3Output.setter
    def input3Output(self, value:bool) -> None:
        self.input3LeftOutput = self.input3RightOutput = value

    input3LeftOutputVolume = WOBits(3, _REG_LEFT_OUT_MIX, 4)

    @property
    def input3LeftOutputVolumeDb(self) -> float:
        return map_range(self.input3LeftOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @input3LeftOutputVolumeDb.setter
    def input3LeftOutputVolumeDb(self, value:float) -> None:
        self.input3LeftOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    input3RightOutputVolume = WOBits(3, _REG_RIGHT_OUT_MIX, 4)

    @property
    def input3RightOutputVolumeDb(self) -> float:
        return map_range(self.input3RightOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @input3RightOutputVolumeDb.setter
    def input3RightOutputVolumeDb(self, value:float) -> None:
        self.input3RightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    @property
    def input3OutputVolume(self) -> int:
        return self.input3LeftOutputVolume
    @input3OutputVolume.setter
    def input3OutputVolume(self, value:int) -> None:
        self.input3LeftOutputVolume = self.input3RightOutputVolume = value

    @property
    def input3OutputVolumeDb(self) -> float:
        return self.input3LeftOutputVolumeDb
    @input3OutputVolumeDb.setter
    def input3OutputVolumeDb(self, value:float) -> None:
        self.input3LeftOutputVolume = self.input3RightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    ## PGA Boost Mixer Output

    micBoostLeftOutput = WOBit(_REG_BYPASS_1, 7)
    micBoostRightOutput = WOBit(_REG_BYPASS_2, 7)

    @property
    def micBoostOutput(self) -> bool:
        return self.micBoostLeftOutput and self.micBoostRightOutput

    micBoostLeftOutputVolume = WOBits(3, _REG_BYPASS_1, 4)

    @property
    def micBoostLeftOutputVolumeDb(self) -> float:
        return map_range(self.micBoostLeftOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @micBoostLeftOutputVolumeDb.setter
    def micBoostLeftOutputVolumeDb(self, value:float) -> None:
        self.micBoostLeftOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    micBoostRightOutputVolume = WOBits(3, _REG_BYPASS_2, 4)

    @property
    def micBoostRightOutputVolumeDb(self) -> float:
        return map_range(self.micBoostRightOutputVolume, 0, 7, _OUTPUT_VOLUME_MAX, _OUTPUT_VOLUME_MIN)
    @micBoostRightOutputVolumeDb.setter
    def micBoostRightOutputVolumeDb(self, value:float) -> None:
        self.micBoostRightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    @property
    def micBoostOutputVolume(self) -> int:
        return self.micBoostLeftOutputVolume
    @micBoostOutputVolume.setter
    def micBoostOutputVolume(self, value:int) -> None:
        self.micBoostLeftOutputVolume = self.micBoostRightOutputVolume = value

    @property
    def micBoostOutputVolumeDb(self) -> float:
        return self.micBoostLeftOutputVolumeDb
    @micBoostOutputVolumeDb.setter
    def micBoostOutputVolumeDb(self, value:float) -> None:
        self.micBoostLeftOutputVolume = self.micBoostRightOutputVolume = round(map_range(value, _OUTPUT_VOLUME_MIN, _OUTPUT_VOLUME_MAX, 7, 0))

    ## Mono Output

    monoOutput = WOBit(_REG_PWR_MGMT_2, 1)

    monoLeftMix = WOBit(_REG_MONO_OUT_MIX_1, 7)
    monoRightMix = WOBit(_REG_MONO_OUT_MIX_2, 7)

    @property
    def monoMix(self) -> bool:
        return self.monoLeftMix and self.monoRightMix
    @monoMix.setter
    def monoMix(self, value:bool) -> None:
        self.monoLeftMix = self.monoRightMix = value

    monoOutputAttenuation = WOBit(_REG_MONO_OUT_VOLUME, 6)

    vmid = WOBits(2, _REG_PWR_MGMT_1, 7)

    # Amplifier

    ## Headphones

    leftHeadphone = WOBit(_REG_PWR_MGMT_2, 5)
    rightHeadphone = WOBit(_REG_PWR_MGMT_2, 6)

    @property
    def headphone(self) -> bool:
        return self.leftHeadphone and self.rightHeadphone
    @headphone.setter
    def headphone(self, value:bool) -> None:
        self.leftHeadphone = self.rightHeadphone = value

    headphoneStandby = WOBit(_REG_ANTI_POP_1, 0)

    _leftHeadphoneVolume = WOBits(7, _REG_LOUT1_VOLUME, 0)
    _leftHeadphoneVolumeSet = WOBit(_REG_LOUT1_VOLUME, 8)

    @property
    def leftHeadphoneVolume(self) -> int:
        return self._leftHeadphoneVolume
    @leftHeadphoneVolume.setter
    def leftHeadphoneVolume(self, value:int) -> None:
        self._leftHeadphoneVolume = value
        self._leftHeadphoneVolumeSet = True

    @property
    def leftHeadphoneVolumeDb(self) -> float:
        return map_range(max(self.leftHeadphoneVolume, 48), 48, 127, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @leftHeadphoneVolumeDb.setter
    def leftHeadphoneVolumeDb(self, value:float) -> None:
        self.leftHeadphoneVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    _rightHeadphoneVolume = WOBits(7, _REG_ROUT1_VOLUME, 0)
    _rightHeadphoneVolumeSet = WOBit(_REG_ROUT1_VOLUME, 8)

    @property
    def rightHeadphoneVolume(self) -> int:
        return self._rightHeadphoneVolume
    @rightHeadphoneVolume.setter
    def rightHeadphoneVolume(self, value:int) -> None:
        self._rightHeadphoneVolume = value
        self._rightHeadphoneVolumeSet = True

    @property
    def rightHeadphoneVolumeDb(self) -> float:
        return map_range(max(self.rightHeadphoneVolume, 48), 48, 255, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @rightHeadphoneVolumeDb.setter
    def rightHeadphoneVolumeDb(self, value:float) -> None:
        self.rightHeadphoneVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    @property
    def headphoneVolume(self) -> int:
        return self.leftHeadphoneVolume
    @headphoneVolume.setter
    def headphoneVolume(self, value:float) -> None:
        self.leftHeadphoneVolume = self.rightHeadphoneVolume = value
    
    @property
    def headphoneVolumeDb(self) -> int:
        return self.leftHeadphoneVolumeDb
    @headphoneVolumeDb.setter
    def headphoneVolumeDb(self, value:float) -> None:
        self.leftHeadphoneVolume = self.rightHeadphoneVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127) + 1.0)

    leftHeadphoneZeroCross = WOBit(_REG_LOUT1_VOLUME, 7)
    rightHeadphoneZeroCross = WOBit(_REG_LOUT1_VOLUME, 7)

    @property
    def headphoneZeroCross(self) -> bool:
        return self.leftHeadphoneZeroCross and self.rightHeadphoneZeroCross
    @headphoneZeroCross.setter
    def headphoneZeroCross(self, value:bool) -> None:
        self.leftHeadphoneZeroCross = self.rightHeadphoneZeroCross = value

    ## Speakers

    _leftSpeaker = WOBit(_REG_PWR_MGMT_2, 4)
    _leftSpeakerAmp = WOBit(_REG_CLASS_D_CONTROL_1, 6)

    @property
    def leftSpeaker(self) -> bool:
        return self._leftSpeaker and self._leftSpeakerAmp
    @leftSpeaker.setter
    def leftSpeaker(self, value:bool) -> None:
        self._leftSpeaker = self._leftSpeakerAmp = value
    
    _rightSpeaker = WOBit(_REG_PWR_MGMT_2, 3)
    _rightSpeakerAmp = WOBit(_REG_CLASS_D_CONTROL_1, 7)

    @property
    def rightSpeaker(self) -> bool:
        return self._rightSpeaker and self._rightSpeakerAmp
    @rightSpeaker.setter
    def rightSpeaker(self, value:bool) -> None:
        self._rightSpeaker = self._rightSpeakerAmp = value

    @property
    def speaker(self) -> bool:
        return self.leftSpeaker and self.rightSpeaker
    @speaker.setter
    def speaker(self, value:bool) -> None:
        self.leftSpeaker = self.rightSpeaker = value

    _leftSpeakerVolume = WOBits(7, _REG_LOUT2_VOLUME, 0)
    _leftSpeakerVolumeSet = WOBit(_REG_LOUT2_VOLUME, 8)

    @property
    def leftSpeakerVolume(self) -> int:
        return self._leftSpeakerVolume
    @leftSpeakerVolume.setter
    def leftSpeakerVolume(self, value:int) -> None:
        self._leftSpeakerVolume = value
        self._leftSpeakerVolumeSet = True

    @property
    def leftSpeakerVolumeDb(self) -> float:
        return map_range(max(self.leftSpeakerVolume, 48), 48, 127, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @leftSpeakerVolumeDb.setter
    def leftSpeakerVolumeDb(self, value:float) -> None:
        self.leftSpeakerVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    _rightSpeakerVolume = WOBits(7, _REG_ROUT2_VOLUME, 0)
    _rightSpeakerVolumeSet = WOBit(_REG_ROUT2_VOLUME, 8)

    @property
    def rightSpeakerVolume(self) -> int:
        return self._rightSpeakerVolume
    @rightSpeakerVolume.setter
    def rightSpeakerVolume(self, value:int) -> None:
        self._rightSpeakerVolume = value
        self._rightSpeakerVolumeSet = True

    @property
    def rightSpeakerVolumeDb(self) -> float:
        return map_range(max(self.rightSpeakerVolume, 48), 48, 255, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX)
    @rightSpeakerVolumeDb.setter
    def rightSpeakerVolumeDb(self, value:float) -> None:
        self.rightSpeakerVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127))

    @property
    def speakerVolume(self) -> int:
        return self.leftSpeakerVolume
    @speakerVolume.setter
    def speakerVolume(self, value:float) -> None:
        self.leftSpeakerVolume = self.rightSpeakerVolume = value
    
    @property
    def speakerVolumeDb(self) -> int:
        return self.leftSpeakerVolumeDb
    @speakerVolume.setter
    def speakerVolume(self, value:float) -> None:
        self.leftSpeakerVolume = self.rightSpeakerVolume = round(map_range(value, _AMP_VOLUME_MIN, _AMP_VOLUME_MAX, 48, 127) + 1.0)

    leftSpeakerZeroCross = WOBit(_REG_LOUT2_VOLUME, 7)
    rightSpeakerZeroCross = WOBit(_REG_LOUT2_VOLUME, 7)

    @property
    def speakerZeroCross(self) -> bool:
        return self.leftSpeakerZeroCross and self.rightSpeakerZeroCross
    @speakerZeroCross.setter
    def speakerZeroCross(self, value:bool) -> None:
        self.leftSpeakerZeroCross = self.rightSpeakerZeroCross = value

    _speakerDcGain = WOBits(3, _REG_CLASS_D_CONTROL_3, 3)

    @property
    def speakerDcGain(self) -> int:
        return self._speakerDcGain
    @speakerDcGain.setter
    def speakerDcGain(self, value:int) -> None:
        self._speakerDcGain = min(value, 5)

    _speakerAcGain = WOBits(3, _REG_CLASS_D_CONTROL_3, 0)

    @property
    def speakerAcGain(self) -> int:
        return self._speakerAcGain
    @speakerAcGain.setter
    def speakerAcGain(self, value:int) -> None:
        self._speakerAcGain = min(value, 5)

    # Digital Audio Interface Control

    loopback = WOBit(_REG_AUDIO_INTERFACE_2, 0)
    
    pll = WOBit(_REG_PWR_MGMT_2, 0)
    pllPrescaleDiv2 = WOBit(_REG_PWR_MGMT_2, 4)
    pllN = WOBits(4, _REG_PLL_N, 0)

    _pllK1 = WOBits(6, _REG_PLL_K_1, 0)
    _pllK2 = WOBits(9, _REG_PLL_K_2, 0)
    _pllK3 = WOBits(9, _REG_PLL_K_3, 0)

    @property
    def pllK(self) -> int:
        return self._pllK1 << 18 + self._pllK2 << 9 + self._pllK3
    @pllK.setter
    def pllK(self, value:int) -> None:
        self._pllK1 = (value >> 18) & 0b111111
        self._pllK2 = (value >> 9) & 0b111111111
        self._pllK3 = value & 0b111111111

    clockFractionalMode = WOBit(_REG_PLL_N, 5)

    clockFromPLL = WOBit(_REG_CLOCKING_1, 0)

    _systemClockDivider = WOBits(2, _REG_CLOCKING_1, 1)
    
    @property
    def systemClockDiv2(self) -> bool:
        return self._systemClockDivider == _SYSCLK_DIV_BY_2
    @systemClockDiv2.setter
    def systemClockDiv2(self, value:bool) -> None:
        self._systemClockDivider = _SYSCLK_DIV_BY_2 if value else _SYSCLK_DIV_BY_1
    
    _adcClockDivider = WOBits(3, _REG_CLOCKING_1, 6)

    @property
    def adcClockDivider(self) -> int:
        return _ADCDACDIV[min(self._adcClockDivider, len(_ADCDACDIV))]
    @adcClockDivider.setter
    def adcClockDivider(self, value:int) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._adcClockDivider = _ADCDACDIV.index(value)

    _dacClockDivider = WOBits(3, _REG_CLOCKING_1, 3)

    @property
    def dacClockDivider(self) -> int:
        return _ADCDACDIV[min(self._dacClockDivider, len(_ADCDACDIV))]
    @dacClockDivider.setter
    def dacClockDivider(self, value:int) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._dacClockDivider = _ADCDACDIV.index(value)

    _baseClockDivider = WOBits(4, _REG_CLOCKING_2, 0)

    @property
    def baseClockDivider(self) -> float:
        return _BCLKDIV[min(self._baseClockDivider, len(_BCLKDIV))]
    @baseClockDivider.setter
    def baseClockDivider(self, value:float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _BCLKDIV:
            self._baseClockDivider = _BCLKDIV.index(value)
        

    _ampClockDivider = WOBits(3, _REG_CLOCKING_2, 6)

    @property
    def ampClockDivider(self) -> float:
        return _DCLKDIV[min(self._ampClockDivider, len(_DCLKDIV))]
    @ampClockDivider.setter
    def ampClockDivider(self, value:float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _DCLKDIV:
            self._ampClockDivider = _DCLKDIV.index(value)

    ## Mode

    masterMode = WOBit(_REG_AUDIO_INTERFACE_1, 6)

    _wordLength = WOBits(2, _REG_AUDIO_INTERFACE_1, 2)

    @property
    def wordLength(self) -> int:
        value = self._wordLength
        if value == 3:
            return 32
        else:
            return 16 + 4 * value
    @wordLength.setter
    def wordLength(self, value:int) -> None:
        self._wordLength = (min(value, 28) - 16) // 4

    wordSelectInvert = WOBit(_REG_AUDIO_INTERFACE_1, 4)

    adcChannelSwap = WOBit(_REG_AUDIO_INTERFACE_1, 8)

    _vrefOutputDisable = WOBit(_REG_ADDITIONAL_CONTROL_3, 6)

    @property
    def vrefOutput(self) -> bool:
        return not self._vrefOutputDisable
    @vrefOutput.setter
    def vrefOutput(self, value:bool) -> None:
        self._vrefOutputDisable = not value

    _vsel = WOBits(2, _REG_ADDITIONAL_CONTROL_1, 6)

    @property
    def powerSupply(self) -> float:
        return (constrain(self._vsel, 1, 2) - 1) * 0.6 + 2.7
    @powerSupply.setter
    def powerSupply(self, value:float) -> None:
        self._vsel = (constrain(value, 2.7, 3.3) - 2.7) // 0.6 * 2 + 1

    ## GPIO

    gpioOutput = WOBit(_REG_AUDIO_INTERFACE_2, 6)
    gpioOutputMode = WOBits(3, _REG_ADDITIONAL_CONTROL_4, 4)
    gpioOutputInvert = WOBit(_REG_ADDITIONAL_CONTROL_4, 7)
    
    _gpioClockDivider = WOBits(3, _REG_CLOCKING_2, 6)

    @property
    def gpioClockDivider(self) -> int:
        return self._gpioClockDivider
    @gpioClockDivider.setter
    def gpioClockDivider(self, value:int) -> None:
        self._gpioClockDivider = min(value, 5)
    
    @property
    def sampleRate(self) -> int:
        return self._sampleRate
    @sampleRate.setter
    def sampleRate(self, value:int) -> None:
        # MCLK = 24 MHz
        self.pll = True # Needed for class-d amp clock
        self.clockFractionalMode = True
        self.clockFromPLL = True

        self.pllPrescaleDiv2 = True
        self.systemClockDiv2 = True
        self.baseClockDivider = 4.0
        self.ampClockDivider = 16.0

        if value in [8000, 12000, 16000, 24000, 32000, 48000]:
            # SYSCLK = 12.288 MHz
            # DCLK = 768.0kHz
            self.pllN = 8
            self.pllK = 0x3126E8
            self.adcClockDivider = self.dacClockDivider = 48000 / value
            
        elif value in [11025, 22050, 44100]:
            # SYSCLK = 11.2896 MHz
            # DCLK = 705.6kHz
            self.pllN = 7
            self.pllK = 0x86C226
            self.adcClockDivider = self.dacClockDivider = 44100 / value
            
        else:
            raise Exception("Invalid sample rate")

        self._sampleRate = value
    
    def __init__(self, i2c_bus:I2C, address:int = _DEFAULT_I2C_ADDR) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._sampleRate = None

        self._registers = [
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
        for i in range(len(self._registers)):
            self._registers[i] = bytearray(self._registers[i].to_bytes(2, 'big'))
            self._registers[i][0] |= i << 1

        self.reset()

        # General setup
        self.vref = True
        self.vmid = VMIDSEL_PLAYBACK

    # Resets all registers to their default state
    _reset = WOBit(_REG_RESET, 7)
    def reset(self) -> None:
        self._reset = True
        for name in dir(self):
            if not name.startswith('_') and isinstance(getattr(self, name), (WOBit, WOBits)):
                getattr(self, name).reset(self)
