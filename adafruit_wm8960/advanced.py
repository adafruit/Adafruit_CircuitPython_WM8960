# SPDX-FileCopyrightText: Copyright (c) 2022 Pete Lewis for SparkFun Electronics
# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
#
# SPDX-License-Identifier: MIT
"""
`adafruit_wm8960.advanced`
================================================================================

CircuitPython driver for WM8960 Stereo CODEC with advanced control

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

import math

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_simplemath import constrain, map_range
from busio import I2C
from micropython import const

try:
    from typing import Optional, Type

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

# ALC Time mapping

ALC_ATTACK_TIME_MIN = 0.006
"""The minimum amount of time allowed by the attack (Gain Ramp-Down) time of the automatic level
control (ALC) of the WM8960 in seconds. Used by the attribute
:attr:`WM8960_Advanced.alc_attack_time`.
"""

ALC_ATTACK_TIME_MAX = 6.140
"""The maximum amount of time allowed by the attack (Gain Ramp-Down) time of the automatic level
control (ALC) of the WM8960 in seconds. Used by the attribute
:attr:`WM8960_Advanced.alc_attack_time`.
"""

ALC_DECAY_TIME_MIN = 0.024
"""The minimum amount of time allowed by the decay (Gain Ramp-Up) time of the automatic level
control (ALC) of the WM8960 in seconds. Used by the attribute
:attr:`WM8960_Advanced.alc_decay_time`.
"""

ALC_DECAY_TIME_MAX = 24.580
"""The maximum amount of time allowed by the decay (Gain Ramp-Up) time of the automatic level
control (ALC) of the WM8960 in seconds. Used by the attribute
:attr:`WM8960_Advanced.alc_decay_time`.
"""

ALC_HOLD_TIME_MIN = 0.00267
"""The minimum amount of time allowed by the hold time of the automatic level control (ALC) of the
WM8960 in seconds. Used by the attribute :attr:`WM8960_Advanced.alc_hold_time`.
"""

ALC_HOLD_TIME_MAX = 43.691
"""The maximum amount of time allowed by the hold time of the automatic level control (ALC) of the
WM8960 in seconds. Used by the attribute :attr:`WM8960_Advanced.alc_hold_time`.
"""

# Gain/Level min/max

BOOST_GAIN_MIN = -12.00
"""The minimum gain allowed by the WM8960 input boost mixers in decibels. Used by the attributes
:attr:`WM8960_Advanced.left_input2_boost`, :attr:`WM8960_Advanced.right_input2_boost`,
:attr:`WM8960_Advanced.input2_boost`, :attr:`WM8960_Advanced.left_input3_boost`,
:attr:`WM8960_Advanced.right_input3_boost`, and :attr:`WM8960_Advanced.input3_boost`.
"""

BOOST_GAIN_MAX = 6.0
"""The maximum gain allowed by the WM8960 input boost mixers in decibels. Used by the attributes
:attr:`WM8960_Advanced.left_input2_boost`, :attr:`WM8960_Advanced.right_input2_boost`,
:attr:`WM8960_Advanced.input2_boost`, :attr:`WM8960_Advanced.left_input3_boost`,
:attr:`WM8960_Advanced.right_input3_boost`, and :attr:`WM8960_Advanced.input3_boost`.
"""

MIC_GAIN_MIN = -17.25
"""The minimum gain allowed by the WM8960 PGA (microphone) amplifier in decibels. Used by the
attributes :attr:`WM8960_Advanced.left_mic_volume`, :attr:`WM8960_Advanced.right_mic_volume`, and
:attr:`WM8960_Advanced.mic_volume`.
"""

MIC_GAIN_MAX = 30.00
"""The maximum gain allowed by the WM8960 PGA (microphone) amplifier in decibels. Used by the
attributes :attr:`WM8960_Advanced.left_mic_volume`, :attr:`WM8960_Advanced.right_mic_volume`, and
:attr:`WM8960_Advanced.mic_volume`.
"""

ADC_VOLUME_MIN = -97.00
"""The minimum digital volume allowed by the WM8960 ADC in decibels. Used by the attributes
:attr:`WM8960_Advanced.left_adc_volume`, :attr:`WM8960_Advanced.right_adc_volume`, and
:attr:`WM8960_Advanced.adc_volume`.
"""

ADC_VOLUME_MAX = 30.00
"""The maximum digital volume allowed by the WM8960 ADC in decibels. Used by the attributes
:attr:`WM8960_Advanced.left_adc_volume`, :attr:`WM8960_Advanced.right_adc_volume`, and
:attr:`WM8960_Advanced.adc_volume`.
"""

DAC_VOLUME_MIN = -127.00
"""The minimum digital volume allowed by the WM8960 DAC in decibels. Used by the attributes
:attr:`WM8960_Advanced.left_dac_volume`, :attr:`WM8960_Advanced.right_dac_volume`, and
:attr:`WM8960_Advanced.dac_volume`.
"""

DAC_VOLUME_MAX = 0.00
"""The maximum digital volume allowed by the WM8960 DAC in decibels. Used by the attributes
:attr:`WM8960_Advanced.left_dac_volume`, :attr:`WM8960_Advanced.right_dac_volume`, and
:attr:`WM8960_Advanced.dac_volume`.
"""

ALC_TARGET_MIN = -22.50
"""The minimum target level of the WM8960 automatic level control (ALC) in decibels. Used by the
attribute :attr:`WM8960_Advanced.alc_target`.
"""

ALC_TARGET_MAX = -1.50
"""The maximum target level of the WM8960 automatic level control (ALC) in decibels. Used by the
attribute :attr:`WM8960_Advanced.alc_target`.
"""

ALC_MAX_GAIN_MIN = -12.00
"""The minimum gain of the maximum allowed gain of the WM8960 automatic level control (ALC) in
decibels. Used by the attribute :attr:`WM8960_Advanced.alc_max_gain`.
"""

ALC_MAX_GAIN_MAX = 30.00
"""The maximum gain of the maximum allowed gain of the WM8960 automatic level control (ALC) in
decibels. Used by the attribute :attr:`WM8960_Advanced.alc_max_gain`.
"""

ALC_MIN_GAIN_MIN = -17.25
"""The minimum gain of the minimum allowed gain of the WM8960 automatic level control (ALC) in
decibels. Used by the attribute :attr:`WM8960_Advanced.alc_min_gain`.
"""

ALC_MIN_GAIN_MAX = 24.75
"""The maximum gain of the minimum allowed gain of the WM8960 automatic level control (ALC) in
decibels. Used by the attribute :attr:`WM8960_Advanced.alc_min_gain`.
"""

GATE_THRESHOLD_MIN = -76.50
"""The minimum level to trigger the noise gate of the WM8960 in decibels. Used by the attribute
:attr:`WM8960_Advanced.noise_gate_threshold`.
"""

GATE_THRESHOLD_MAX = -30.00
"""The maximum level to trigger the noise gate of the WM8960 in decibels. Used by the attribute
:attr:`WM8960_Advanced.noise_gate_threshold`.
"""

OUTPUT_VOLUME_MIN = -21.00
"""The minimum volume level used by the analog bypass output mixers of the WM8960 in decibels. Used
by the attributes :attr:`WM8960_Advanced.left_input3_output_volume`,
:attr:`WM8960_Advanced.right_input3_output_volume`, :attr:`WM8960_Advanced.input3_output_volume`,
:attr:`WM8960_Advanced.left_mic_output_volume`, :attr:`WM8960_Advanced.right_mic_output_volume`,
and :attr:`WM8960_Advanced.mic_output_volume`.
"""

OUTPUT_VOLUME_MAX = 0.00
"""The maximum volume level used by the analog bypass output mixers of the WM8960 in decibels. Used
by the attributes :attr:`WM8960_Advanced.left_input3_output_volume`,
:attr:`WM8960_Advanced.right_input3_output_volume`, :attr:`WM8960_Advanced.input3_output_volume`,
:attr:`WM8960_Advanced.left_mic_output_volume`, :attr:`WM8960_Advanced.right_mic_output_volume`, and
:attr:`WM8960_Advanced.mic_output_volume`.
"""

AMP_VOLUME_MIN = -73.00
"""The minimum volume level used by the headphone and speaker amplifiers of the WM8960 in decibels.
Used by the attributes :attr:`WM8960_Advanced.left_headphone_volume`,
:attr:`WM8960_Advanced.right_headphone_volume`, :attr:`WM8960_Advanced.headphone_volume`,
:attr:`WM8960_Advanced.left_speaker_volume`, :attr:`WM8960_Advanced.right_speaker_volume`, and
:attr:`WM8960_Advanced.speaker_volume`.
"""

AMP_VOLUME_MAX = 6.00
"""The maximum volume level used by the headphone and speaker amplifiers of the WM8960 in decibels.
Used by the attributes :attr:`WM8960_Advanced.left_headphone_volume`,
:attr:`WM8960_Advanced.right_headphone_volume`, :attr:`WM8960_Advanced.headphone_volume`,
:attr:`WM8960_Advanced.left_speaker_volume`, :attr:`WM8960_Advanced.right_speaker_volume`, and
:attr:`WM8960_Advanced.speaker_volume`.
"""


class Mic_Input:
    """An enum-like class representing the microphone amplifier (PGA) input modes. Used for the
    attributes :attr:`WM8960_Advanced.left_mic_input`, :attr:`WM8960_Advanced.right_mic_input`, and
    :attr:`WM8960_Advanced.mic_input`.
    """

    VMID = 0
    """Disconnect both input 2 and input 3 from the non-inverting input of the amplifier and instead
    connect it to the internally buffered VMID refernce (see :attr:`WM8960_Advanced.vmid`).
    """

    INPUT2 = 1
    """Connect the signal of input 2 to the non-inverting input of the amplifier."""

    INPUT3 = 2
    """Connect the signal of input 3 to the non-inverting input of the amplifier."""


# pylint: enable=too-few-public-methods


# Microphone Boost gain options
_MIC_BOOST_GAIN = [0.0, 13.0, 20.0, 29.0]  # in dB

# SYSCLK divide
_SYSCLK_DIV_BY_1 = const(0)
_SYSCLK_DIV_BY_2 = const(2)

# ALC Time mapping
_ALC_DECAY_MAX = const(10)
_ALC_ATTACK_MAX = const(10)

# Speaker Boost Gains (DC and AC)
_SPEAKER_BOOST_GAIN = [0.0, 2.1, 2.9, 3.6, 4.5, 5.1]  # in dB


class Vmid_Mode:
    """An enum-like class representing possible Vmid reference voltage modes used in conjunction
    with the attribute :attr:`WM8960_Advanced.vmid`.
    """

    DISABLED = 0
    """Disable the vmid reference voltage used by both the microphone amplifier (PGA) and OUT3 when
    mono output is disabled as a buffered headphone ground.
    """

    PLAYBACK = 1  # 2X50KOHM
    """Set the internal voltage reference to a 2*50kOhm potential divider. This setting is typical
    for normal playback and power consumption and is used for the microphone amplifier and as a
    buffered headphone ground.
    """

    LOWPOWER = 2  # 2X250KOHM
    """Set the internal voltage reference to a 2*250kOhm potential divider. Use this setting for low
    power maintenance when all other operations of the WM8960 are disabled.
    """

    FASTSTART = 3  # 2X5KOHM
    """Set the internal voltage reference to a 2*5kOhm potential divider. Use this setting if a fast
    start-up of playback operation is desired. This setting will consume more power.
    """


# pylint: enable=too-few-public-methods


# Clock Divider Tables
_BCLKDIV = [1.0, 1.5, 2.0, 3.0, 4.0, 5.5, 6.0, 8.0, 11.0, 12.0, 16.0, 22.0, 24.0, 32.0]
_DCLKDIV = [1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0]
_ADCDACDIV = [1.0, 1.5, 2.0, 3.0, 4.0, 5.5, 6.0]
_OPCLKDIV = [1.0, 2.0, 3.0, 4.0, 5.5, 6.0]

# Default Register Map
_REG_DEFAULTS = [
    const(0x0097),  # R0 (0x00)
    const(0x0097),  # R1 (0x01)
    const(0x0000),  # R2 (0x02)
    const(0x0000),  # R3 (0x03)
    const(0x0000),  # R4 (0x04)
    const(0x0008),  # F5 (0x05)
    const(0x0000),  # R6 (0x06)
    const(0x000A),  # R7 (0x07)
    const(0x01C0),  # R8 (0x08)
    const(0x0000),  # R9 (0x09)
    const(0x00FF),  # R10 (0x0a)
    const(0x00FF),  # R11 (0x0b)
    const(0x0000),  # R12 (0x0C) RESERVED
    const(0x0000),  # R13 (0x0D) RESERVED
    const(0x0000),  # R14 (0x0E) RESERVED
    const(0x0000),  # R15 (0x0F) RESERVED
    const(0x0000),  # R16 (0x10)
    const(0x007B),  # R17 (0x11)
    const(0x0100),  # R18 (0x12)
    const(0x0032),  # R19 (0x13)
    const(0x0000),  # R20 (0x14)
    const(0x00C3),  # R21 (0x15)
    const(0x00C3),  # R22 (0x16)
    const(0x01C0),  # R23 (0x17)
    const(0x0000),  # R24 (0x18)
    const(0x0000),  # R25 (0x19)
    const(0x0000),  # R26 (0x1A)
    const(0x0000),  # R27 (0x1B)
    const(0x0000),  # R28 (0x1C)
    const(0x0000),  # R29 (0x1D)
    const(0x0000),  # R30 (0x1E) RESERVED
    const(0x0000),  # R31 (0x1F) RESERVED
    const(0x0100),  # R32 (0x20)
    const(0x0100),  # R33 (0x21)
    const(0x0050),  # R34 (0x22)
    const(0x0000),  # R35 (0x23) RESERVED
    const(0x0000),  # R36 (0x24) RESERVED
    const(0x0050),  # R37 (0x25)
    const(0x0000),  # R38 (0x26)
    const(0x0000),  # R39 (0x27)
    const(0x0000),  # R40 (0x28)
    const(0x0000),  # R41 (0x29)
    const(0x0040),  # R42 (0x2A)
    const(0x0000),  # R43 (0x2B)
    const(0x0000),  # R44 (0x2C)
    const(0x0050),  # R45 (0x2D)
    const(0x0050),  # R46 (0x2E)
    const(0x0000),  # R47 (0x2F)
    const(0x0002),  # R48 (0x30)
    const(0x0037),  # R49 (0x31)
    const(0x0000),  # R50 (0x32) RESERVED
    const(0x0080),  # R51 (0x33)
    const(0x0008),  # R52 (0x34)
    const(0x0031),  # R53 (0x35)
    const(0x0026),  # R54 (0x36)
    const(0x00E9),  # R55 (0x37)
]


class WOBit:
    def __init__(
        self,
        register_address: int,
        bit: int,
    ) -> None:
        self.register_address = register_address
        self.bit = bit
        self.bit_mask = 1 << (bit % 8)  # the bitmask *within* the byte!
        self.byte = 1 - (bit // 8)  # the byte number within the buffer

    def _set(self, obj: Optional[I2CDeviceDriver], value: bool) -> None:
        if value:
            obj._registers[self.register_address][self.byte] |= self.bit_mask
        else:
            obj._registers[self.register_address][self.byte] &= ~self.bit_mask

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
    def __init__(
        self,
        num_bits: int,
        register_address: int,
        lowest_bit: int,
    ) -> None:
        self.register_width = 1
        self.register_address = register_address
        self.bit_mask = ((1 << num_bits) - 1) << lowest_bit
        self.lowest_bit = lowest_bit

    def _set(self, obj: Optional[I2CDeviceDriver], value: int) -> None:
        value <<= self.lowest_bit  # shift the value over to the right spot
        reg = 0
        for i in range(2):
            reg = (reg << 8) | obj._registers[self.register_address][i]
        reg &= ~self.bit_mask  # mask off the bits we're about to change
        reg |= value  # then or in our new value
        for i in range(1, -1, -1):
            obj._registers[self.register_address][i] = reg & 0xFF
            reg >>= 8

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


class WM8960_Advanced:
    """Driver for interacting directly with a WM8960 audio codec to control analog and digital audio
    pathways over an I2C connection."""

    # Power

    power: bool = WOBit(_REG_PWR_MGMT_1, 6)
    """Control the power state of all functions of the WM8960.

    :default: `True`
    """

    vmid: int = WOBits(2, _REG_PWR_MGMT_1, 7)
    """The buffered reference voltage mode. Use constants from class :class:`Vmid_Mode`.

    :default: :const:`Vmid_Mode.PLAYBACK`
    """

    left_input: bool = WOBit(_REG_PWR_MGMT_1, 5)
    """The power state of the left channel boost stage and microphone amplifier (if :attr:`left_mic`
    is set to `True`).

    :default: `False`
    """

    right_input: bool = WOBit(_REG_PWR_MGMT_1, 4)
    """The power state of the right channel boost stage and microphone amplifier (if
    :attr:`right_mic` is set to `True`).

    :default: `False`
    """

    @property
    def input(self) -> bool:
        """The power state of the stereo boost stage and microphone amplifier (if :attr:`mic` is set
        to `True`).

        :default: `False`
        """
        return self.left_input and self.right_input

    @input.setter
    def input(self, value: bool) -> None:
        self.left_input = self.right_input = value

    # MIC

    ## PWR_MGMT

    left_mic: bool = WOBit(_REG_PWR_MGMT_3, 5)
    """The power state of the left channel microphone amplifier (if :attr:`left_input` is set to
    `True`).

    :default: `False`
    """

    right_mic: bool = WOBit(_REG_PWR_MGMT_3, 4)
    """The power state of the right channel microphone amplifier (if :attr:`right_input` is set to
    `True`).

    :default: `False`
    """

    @property
    def mic(self) -> bool:
        """The power state of the stereo microphone amplifier (if :attr:`input` is set to `True`).

        :default: `False`
        """
        return self.left_mic and self.right_mic

    @mic.setter
    def mic(self, value: bool) -> None:
        self.left_mic = self.right_mic = value

    ## SIGNAL_PATH

    left_mic_inverting_input: bool = WOBit(_REG_ADCL_SIGNAL_PATH, 8)
    """The connection of the left inverting input of the microphone amplifier (PGA) to the left
    input 1.

    :default: `False`
    """

    right_mic_inverting_input: bool = WOBit(_REG_ADCR_SIGNAL_PATH, 8)
    """The connection the right inverting input of the microphone amplifier (PGA) to the right input
    1.

    :default: `False`
    """

    @property
    def mic_inverting_input(self) -> bool:
        """The connection of the inverting input of the microphone amplifier (PGA) to input 1.

        :default: `False`
        """
        return self.left_mic_inverting_input and self.right_mic_inverting_input

    @mic_inverting_input.setter
    def mic_inverting_input(self, value: bool) -> None:
        self.left_mic_inverting_input = self.right_mic_inverting_input = value

    _left_mic_input2: bool = WOBit(_REG_ADCL_SIGNAL_PATH, 6)
    _left_mic_input3: bool = WOBit(_REG_ADCL_SIGNAL_PATH, 7)

    @property
    def left_mic_input(self) -> int:
        """The connection to the left non-inverting input of the microphone amplifier. Use constants
        from class :class:`Mic_Input`.

        :default: :const:`Mic_Input.VMID`
        """
        if self._left_mic_input2:
            return Mic_Input.INPUT2
        if self._left_mic_input3:
            return Mic_Input.INPUT3
        return Mic_Input.VMID

    @left_mic_input.setter
    def left_mic_input(self, value: int) -> None:
        self._left_mic_input2 = value == Mic_Input.INPUT2
        self._left_mic_input3 = value == Mic_Input.INPUT3

    _right_mic_input2: bool = WOBit(_REG_ADCR_SIGNAL_PATH, 6)
    _right_mic_input3: bool = WOBit(_REG_ADCR_SIGNAL_PATH, 7)

    @property
    def right_mic_input(self) -> int:
        """The connection to the right non-inverting input of the microphone amplifier. Use
        constants from class :class:`Mic_Input`.

        :default: :const:`Mic_Input.VMID`
        """
        if self._right_mic_input2:
            return Mic_Input.INPUT2
        if self._right_mic_input3:
            return Mic_Input.INPUT3
        return Mic_Input.VMID

    @right_mic_input.setter
    def right_mic_input(self, value: int) -> None:
        self._right_mic_input2 = value == Mic_Input.INPUT2
        self._right_mic_input3 = value == Mic_Input.INPUT3

    @property
    def mic_input(self) -> int:
        """The connection to the non-inverting input of the microphone amplifier. Use constants from
        class :class:`Mic_Input`.

        :default: :const:`Mic_Input.VMID`
        """
        # NOTE: Not checking right signal
        return self.left_mic_input

    @mic_input.setter
    def mic_input(self, value: int) -> None:
        self.left_mic_input = self.right_mic_input = value

    ## Boost

    left_mic_boost: bool = WOBit(_REG_ADCL_SIGNAL_PATH, 3)
    """The connection to the left input boost of the microphone amplifier.

    :default: `False`
    """

    right_mic_boost: bool = WOBit(_REG_ADCR_SIGNAL_PATH, 3)
    """The connection to the right input boost of the microphone amplifier.

    :default: `False`
    """

    @property
    def mic_boost(self) -> bool:
        """The connection to the input boost of the microphone amplifier.

        :default: `False`
        """
        return self.left_mic_boost and self.right_mic_boost

    @mic_boost.setter
    def mic_boost(self, value: bool) -> None:
        self.left_mic_boost = self.right_mic_boost = value

    @staticmethod
    def _get_mic_boost_gain(value: float) -> int:
        for i in reversed(range(len(_MIC_BOOST_GAIN))):
            if value >= _MIC_BOOST_GAIN[i]:
                return i
        return 0

    _left_mic_boost_gain: int = WOBits(2, _REG_ADCL_SIGNAL_PATH, 4)

    @property
    def left_mic_boost_gain(self) -> float:
        """The amount of gain of the left input boost of the microphone amplifier in decibels.
        Allowed values are 0.0, 13.0, 20.0, and 29.0. :attr:`WM8960_Advanced.left_mic_boost` must be
        set to `True`.

        :default: 0.0
        """
        return _MIC_BOOST_GAIN[self._left_mic_boost_gain]

    @left_mic_boost_gain.setter
    def left_mic_boost_gain(self, value: float) -> None:
        self._left_mic_boost_gain = WM8960_Advanced._get_mic_boost_gain(value)

    _right_mic_boost_gain: int = WOBits(2, _REG_ADCR_SIGNAL_PATH, 4)

    @property
    def right_mic_boost_gain(self) -> float:
        """The amount of gain of the right input boost of the microphone amplifier in decibels.
        Allowed values are 0.0, 13.0, 20.0, and 29.0. :attr:`WM8960_Advanced.right_mic_boost` must
        be set to `True`.

        :default: 0.0
        """
        return _MIC_BOOST_GAIN[self._right_mic_boost_gain]

    @right_mic_boost_gain.setter
    def right_mic_boost_gain(self, value: float) -> None:
        self._right_mic_boost_gain = WM8960_Advanced._get_mic_boost_gain(value)

    @property
    def mic_boost_gain(self) -> float:
        """The amount of gain of the input boost of the microphone amplifier in decibels. Allowed
        values are 0.0, 13.0, 20.0, and 29.0. :attr:`WM8960_Advanced.mic_boost` must be set to
        `True`.

        :default: 0.0
        """
        return max(self.left_mic_boost_gain, self.right_mic_boost_gain)

    @mic_boost_gain.setter
    def mic_boost_gain(self, value: float) -> None:
        self._left_mic_boost_gain = self._right_mic_boost_gain = (
            WM8960_Advanced._get_mic_boost_gain(value)
        )

    ## Volume

    _left_mic_volume: int = WOBits(6, _REG_LEFT_INPUT_VOLUME, 0)
    _left_mic_volume_set: bool = WOBit(_REG_LEFT_INPUT_VOLUME, 8)

    _right_mic_volume: int = WOBits(6, _REG_RIGHT_INPUT_VOLUME, 0)
    _right_mic_volume_set: bool = WOBit(_REG_RIGHT_INPUT_VOLUME, 8)

    @property
    def left_mic_volume(self) -> float:
        """The level of the left microphone amplifier in decibels. Value ranges from a minimum of
        -17.25dB (:const:`MIC_GAIN_MIN`) to a maximum of +30.0dB (:const:`MIC_GAIN_MAX`).
        :attr:`WM8960_Advanced.left_mic` must be set to `True`.

        :default: 0.0
        """
        return map_range(self._left_mic_volume, 0, 63, MIC_GAIN_MIN, MIC_GAIN_MAX)

    @left_mic_volume.setter
    def left_mic_volume(self, value: float) -> None:
        self._left_mic_volume = round(map_range(value, MIC_GAIN_MIN, MIC_GAIN_MAX, 0, 63))
        self._left_mic_volume_set = True

    @property
    def right_mic_volume(self) -> float:
        """The level of the right microphone amplifier in decibels. Value ranges from a minimum of
        -17.25dB (:const:`MIC_GAIN_MIN`) to a maximum of +30.0dB (:const:`MIC_GAIN_MAX`).
        :attr:`WM8960_Advanced.right_mic` must be set to `True`.

        :default: 0.0
        """
        return map_range(self._right_mic_volume, 0, 63, MIC_GAIN_MIN, MIC_GAIN_MAX)

    @right_mic_volume.setter
    def right_mic_volume(self, value: float) -> None:
        self._right_mic_volume = round(map_range(value, MIC_GAIN_MIN, MIC_GAIN_MAX, 0, 63))
        self._right_mic_volume_set = True

    @property
    def mic_volume(self) -> float:
        """The level of the microphone amplifier in decibels. Value ranges from a minimum of
        -17.25dB (:const:`MIC_GAIN_MIN`) to a maximum of +30.0dB (:const:`MIC_GAIN_MAX`).
        :attr:`WM8960_Advanced.mic` must be set to `True`.

        :default: 0.0
        """
        return max(self.left_mic_volume, self.right_mic_volume)

    @mic_volume.setter
    def mic_volume(self, value: float) -> None:
        self._left_mic_volume = self._right_mic_volume = round(
            map_range(value, MIC_GAIN_MIN, MIC_GAIN_MAX, 0, 63)
        )
        self._left_mic_volume_set = self._right_mic_volume_set = True

    ## Zero Cross

    left_mic_zero_cross: bool = WOBit(_REG_LEFT_INPUT_VOLUME, 6)
    """Whether or not the volume of the left microphone amplifier channel will be adjusted when a
    "zero" input level is detected in order to avoid harsh jumps in the waveform.

    :default: `False`
    """

    right_mic_zero_cross: bool = WOBit(_REG_RIGHT_INPUT_VOLUME, 6)
    """Whether or not the volume of the right microphone amplifier channel will be adjusted when a
    "zero" input level is detected in order to avoid harsh jumps in the waveform.

    :default: `False`
    """

    @property
    def mic_zero_cross(self) -> bool:
        """Whether or not the volume of the microphone amplifier will be adjusted when a "zero"
        input level is detected in order to avoid harsh jumps in the waveform.

        :default: `False`
        """
        return self.left_mic_zero_cross and self.right_mic_zero_cross

    @mic_zero_cross.setter
    def mic_zero_cross(self, value: bool) -> None:
        self.left_mic_zero_cross = self.right_mic_zero_cross = value

    ## Mute

    _left_mic_mute: bool = WOBit(_REG_LEFT_INPUT_VOLUME, 7)
    _right_mic_mute: bool = WOBit(_REG_RIGHT_INPUT_VOLUME, 7)

    @property
    def left_mic_mute(self) -> bool:
        """The muted state of the left channel of the microphone amplifier.

        :default: `False`
        """
        return self._left_mic_mute

    @left_mic_mute.setter
    def left_mic_mute(self, value: bool) -> None:
        self._left_mic_mute = value
        self._left_mic_volume_set = True

    @property
    def right_mic_mute(self) -> bool:
        """The muted state of the right channel of the microphone amplifier.

        :default: `False`
        """
        return self._right_mic_mute

    @right_mic_mute.setter
    def right_mic_mute(self, value: bool) -> None:
        self._right_mic_mute = value
        self._right_mic_volume_set = True

    @property
    def mic_mute(self) -> bool:
        """The muted state of the microphone amplifier.

        :default: `False`
        """
        return self._left_mic_mute and self._right_mic_mute

    @mic_mute.setter
    def mic_mute(self, value: bool) -> None:
        self._left_mic_mute = self._right_mic_mute = value

    # Boost Mixer

    _left_input2_boost: int = WOBits(3, _REG_INPUT_BOOST_MIXER_1, 1)

    @property
    def left_input2_boost(self) -> float:
        """The gain of the left channel of input 2 boost amp into the boost mixer before the ADC in
        decibels. Accepts a minimum value of -12.0dB (:const:`BOOST_GAIN_MIN`) and a maximum of
        +6.0dB (:const:`BOOST_GAIN_MAX`). In order to mute line-level input 2 into the ADC, set this
        value below -12.0dB. If muted, a value of `None` will be returned.

        :default: `None`
        """
        value = self._left_input2_boost
        if value == 0:
            return None
        return map_range(value, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX)

    @left_input2_boost.setter
    def left_input2_boost(self, value: float) -> None:
        self._left_input2_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7))
        )

    _right_input2_boost: int = WOBits(3, _REG_INPUT_BOOST_MIXER_2, 1)

    @property
    def right_input2_boost(self) -> float:
        """The gain of the right channel of input 2 boost amp into the boost mixer before the ADC in
        decibels. Accepts a minimum value of -12.0dB (:const:`BOOST_GAIN_MIN`) and a maximum of
        +6.0dB (:const:`BOOST_GAIN_MAX`). In order to mute line-level input 2 into the ADC, set this
        value below -12.0dB. If muted, a value of `None` will be returned.

        :default: `None`
        """
        value = self._right_input2_boost
        if value == 0:
            return None
        return map_range(value, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX)

    @right_input2_boost.setter
    def right_input2_boost(self, value: float) -> None:
        self._right_input2_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7))
        )

    @property
    def input2_boost(self) -> float:
        """The gain of both channels of input 2 boost amp into the boost mixer before the ADC in
        decibels. Accepts a minimum value of -12.0dB (:const:`BOOST_GAIN_MIN`) and a maximum of
        +6.0dB (:const:`BOOST_GAIN_MAX`). In order to mute line-level input 2 into the ADC, set this
        value below -12.0dB. If muted, a value of `None` will be returned.

        :default: `None`
        """
        value = max(self._left_input2_boost, self._right_input2_boost)
        return None if value == 0 else map_range(value, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX)

    @input2_boost.setter
    def input2_boost(self, value: float) -> None:
        self._left_input2_boost = self._right_input2_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7))
        )

    _left_input3_boost: int = WOBits(3, _REG_INPUT_BOOST_MIXER_1, 4)

    @property
    def left_input3_boost(self) -> float:
        """The gain of the left channel of input 3 boost amp into the boost mixer before the ADC in
        decibels. Accepts a minimum value of -12.0dB (:const:`BOOST_GAIN_MIN`) and a maximum of
        +6.0dB (:const:`BOOST_GAIN_MAX`). In order to mute line-level input 3 into the ADC, set this
        value below -12.0dB. If muted, a value of `None` will be returned.

        :default: `None`
        """
        value = self._left_input3_boost
        if value == 0:
            return None
        return map_range(value, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX)

    @left_input3_boost.setter
    def left_input3_boost(self, value: float) -> None:
        self._left_input3_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7))
        )

    _right_input3_boost: int = WOBits(3, _REG_INPUT_BOOST_MIXER_2, 4)

    @property
    def right_input3_boost(self) -> float:
        """The gain of the right channel of input 3 boost amp into the boost mixer before the ADC in
        decibels. Accepts a minimum value of -12.0dB (:const:`BOOST_GAIN_MIN`) and a maximum of
        +6.0dB (:const:`BOOST_GAIN_MAX`). In order to mute line-level input 3 into the ADC, set this
        value below -12.0dB. If muted, a value of `None` will be returned.

        :default: `None`
        """
        value = self._right_input3_boost
        if value == 0:
            return None
        return map_range(value, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX)

    @right_input3_boost.setter
    def right_input3_boost(self, value: float) -> None:
        self._right_input3_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7))
        )

    @property
    def input3_boost(self) -> float:
        """The gain of both channels of input 3 boost amp into the boost mixer before the ADC in
        decibels. Accepts a minimum value of -12.0dB (:const:`BOOST_GAIN_MIN`) and a maximum of
        +6.0dB (:const:`BOOST_GAIN_MAX`). In order to mute line-level input 3 into the ADC, set this
        value below -12.0dB. If muted, a value of `None` will be returned.

        :default: `None`
        """
        value = max(self._left_input3_boost, self._right_input3_boost)
        return None if value == 0 else map_range(value, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX)

    @input3_boost.setter
    def input3_boost(self, value: float) -> None:
        self._left_input3_boost = self._right_input3_boost = (
            0
            if value < BOOST_GAIN_MIN
            else round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7))
        )

    # Mic Bias

    mic_bias: bool = WOBit(_REG_PWR_MGMT_1, 1)
    """Whether or not the WM8960 is outputting a reference voltage on the MICBIAS pin for biasing
    electret type microphones.

    :default: `False`
    """

    mic_bias_voltage: bool = WOBit(_REG_ADDITIONAL_CONTROL_4, 0)
    """The electret type microphone bias mode. :attr:`WM8960.mic_bias` must be set to `True` to
    utilize this setting.
    `False` = the reference voltage is 0.9 * AVDD or 1.8 * VMID
    `True` = the reference voltage is 0.65 * AVDD or 1.3 * VMID

    :default: `False`
    """

    # ADC

    left_adc: bool = WOBit(_REG_PWR_MGMT_1, 3)
    """Whether or not the left channel of the ADC is enabled.

    :default: `False`
    """

    right_adc: bool = WOBit(_REG_PWR_MGMT_1, 2)
    """Whether or not the right channel of the ADC is enabled.

    :default: `False`
    """

    @property
    def adc(self) -> bool:
        """Whether or not the both channels of the ADC are enabled.

        :default: `False`
        """
        return self.left_adc and self.right_adc

    @adc.setter
    def adc(self, value: bool) -> None:
        self.left_adc = self.right_adc = value

    ## Volume

    _left_adc_volume: int = WOBits(8, _REG_LEFT_ADC_VOLUME, 0)
    _left_adc_volume_set: bool = WOBit(_REG_LEFT_ADC_VOLUME, 8)

    @property
    def left_adc_volume(self) -> float:
        """The digital volume of the left channel ADC in decibels. Accepts a minimum value of
        -97.0dB (:const:`ADC_VOLUME_MIN`) and a maximum of +30.0dB (:const:`ADC_VOLUME_MAX`). In
        order to digitally mute the left channel of the ADC, set this value below -97.0dB. If muted,
        a value of `None` will be returned.

        :default: `None`
        """
        value = self._left_adc_volume
        if value == 0:
            return None
        return map_range(value, 1, 255, ADC_VOLUME_MIN, ADC_VOLUME_MAX)

    @left_adc_volume.setter
    def left_adc_volume(self, value: float) -> None:
        self._left_adc_volume = (
            0
            if value < ADC_VOLUME_MIN
            else round(map_range(value, ADC_VOLUME_MIN, ADC_VOLUME_MAX, 1, 255))
        )
        self._left_adc_volume_set = True

    _right_adc_volume: int = WOBits(8, _REG_RIGHT_ADC_VOLUME, 0)
    _right_adc_volume_set: bool = WOBit(_REG_RIGHT_ADC_VOLUME, 8)

    @property
    def right_adc_volume(self) -> float:
        """The digital volume of the right channel ADC in decibels. Accepts a minimum value of
        -97.0dB (:const:`ADC_VOLUME_MIN`) and a maximum of +30.0dB (:const:`ADC_VOLUME_MAX`). In
        order to digitally mute the right channel of the ADC, set this value below -97.0dB. If
        muted, a value of `None` will be returned.

        :default: `None`
        """
        value = self._right_adc_volume
        if value == 0:
            return None
        return map_range(value, 1, 255, ADC_VOLUME_MIN, ADC_VOLUME_MAX)

    @right_adc_volume.setter
    def right_adc_volume(self, value: float) -> None:
        self._right_adc_volume = (
            0
            if value < ADC_VOLUME_MIN
            else round(map_range(value, ADC_VOLUME_MIN, ADC_VOLUME_MAX, 1, 255))
        )
        self._right_adc_volume_set = True

    @property
    def adc_volume(self) -> float:
        """The digital volume of both channels of the ADC in decibels. Accepts a minimum value of
        -97.0dB (:const:`ADC_VOLUME_MIN`) and a maximum of +30.0dB (:const:`ADC_VOLUME_MAX`). In
        order to digitally mute the ADC, set this value below -97.0dB. If muted, a value of `None`
        will be returned.

        :default: `None`
        """
        return max(self.left_adc_volume, self.right_adc_volume)

    @adc_volume.setter
    def adc_volume(self, value: float) -> None:
        self._left_adc_volume = self._right_adc_volume = (
            0
            if value < ADC_VOLUME_MIN
            else round(map_range(value, ADC_VOLUME_MIN, ADC_VOLUME_MAX, 1, 255))
        )
        self._left_adc_volume_set = self._right_adc_volume_set = True

    # ALC

    left_alc: bool = WOBit(_REG_ALC1, 7)
    """Whether or not the left channel of the automatic level control (ALC) is enabled.

    :default: `False`
    """

    right_alc: bool = WOBit(_REG_ALC1, 8)
    """Whether or not the right channel of the automatic level control (ALC) is enabled.

    :default: `False`
    """

    @property
    def alc(self) -> bool:
        """Whether or not both channels of the automatic level control (ALC) are enabled. The ALC
        will automatically adjust the gain of incoming signal into the microphone amplifier.

        :default: `False`
        """
        return self.left_alc and self.right_alc

    @alc.setter
    def alc(self, value: bool) -> None:
        self.left_alc = self.right_alc = value

    _alc_target: int = WOBits(4, _REG_ALC1, 0)

    @property
    def alc_target(self) -> float:
        """The target level of the input signal out of the microphone amplifier as controlled by the
        ALC in decibels. Accepts a value between -22.5dB (:const:`ALC_TARGET_MIN`) and +6.0dB
        (:const:`ALC_TARGET_MAX`).

        :default: `False`
        """
        return map_range(self._alc_target, 0, 15, ALC_TARGET_MIN, ALC_TARGET_MAX)

    @alc_target.setter
    def alc_target(self, value: float) -> None:
        self._alc_target = round(map_range(value, ALC_TARGET_MIN, ALC_TARGET_MAX, 0, 15))

    _alc_max_gain: int = WOBits(3, _REG_ALC1, 4)

    @property
    def alc_max_gain(self) -> float:
        """The maximum potential gain of the ALC to be applied to the microphone amplifier in
        decibels. Accepts a value between -12.0dB (:const:`ALC_MAX_GAIN_MIN`) and +30.0dB
        (:const:`ALC_MAX_GAIN_MAX`).

        :default: 30.0
        """
        return map_range(self._alc_max_gain, 0, 7, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX)

    @alc_max_gain.setter
    def alc_max_gain(self, value: float) -> None:
        self._alc_max_gain = round(map_range(value, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX, 0, 7))

    _alc_min_gain: int = WOBits(3, _REG_ALC2, 4)

    @property
    def alc_min_gain(self) -> float:
        """The minimum allowed gain of the ALC to be applied to the microphone amplifier in
        decibels. Accepts a value between -17.25dB (:const:`ALC_MIN_GAIN_MIN`) and +24.75dB
        (:const:`ALC_MIN_GAIN_MAX`).

        :default: -17.25
        """
        return map_range(self._alc_min_gain, 0, 7, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX)

    @alc_min_gain.setter
    def alc_min_gain(self, value: float) -> None:
        self._alc_min_gain = round(map_range(value, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX, 0, 7))

    _alc_hold: int = WOBits(4, _REG_ALC2, 0)

    @property
    def alc_hold_time(self) -> float:
        """The time delay between the peak level detected being below target and the microphone
        amplifier gain beginning to ramp up. However, there is no delay before ramping the gain down
        when the signal level is above target. Accepts a minimum value of 0.00267s
        (:const:`ALC_HOLD_TIME_MIN`) increasing exponentially to a maximum of 43.691s
        (:const:`ALC_HOLD_TIME_MAX`). This value can also be set to 0.

        :default: 30.0
        """
        value = self._alc_hold
        if value == 0:
            return 0.0
        return ALC_HOLD_TIME_MIN * pow(2, value - 1)

    @alc_hold_time.setter
    def alc_hold_time(self, value: float) -> None:
        self._alc_hold = (
            round(
                math.log(
                    (constrain(value, ALC_HOLD_TIME_MIN, ALC_HOLD_TIME_MAX) - ALC_HOLD_TIME_MIN)
                    / ALC_HOLD_TIME_MIN,
                    2.0,
                )
                + 1.0
            )
            if value > 0.0
            else 0
        )

    _alc_decay: int = WOBits(4, _REG_ALC3, 4)

    @property
    def alc_decay_time(self) -> float:
        """The time that it takes for the microphone amplifier gain to ramp up and return to its
        target value, :attr:`alc_target`. This time is relative depending on the gain adjustment
        required. Accepts a minimum value of 0.024s (:const:`ALC_HOLD_TIME_MIN`) increasing
        exponentially to a maximum of 43.691s (:const:`ALC_HOLD_TIME_MAX`).

        :default: 0.024
        """
        return ALC_DECAY_TIME_MIN * pow(2, self._alc_decay)

    @alc_decay_time.setter
    def alc_decay_time(self, value: float) -> None:
        self._alc_decay = min(
            round(
                math.log(
                    (constrain(value, ALC_DECAY_TIME_MIN, ALC_DECAY_TIME_MAX) - ALC_DECAY_TIME_MIN)
                    / ALC_DECAY_TIME_MIN,
                    2.0,
                )
            ),
            _ALC_DECAY_MAX,
        )

    _alc_attack: int = WOBits(4, _REG_ALC3, 0)

    @property
    def alc_attack_time(self) -> float:
        """The time that it takes for the microphone amplifier gain to ramp down and adjust the
        incoming signal to its target level, :attr:`alc_target`. This time is relative depending on
        the gain adjustment required. Accepts a minimum value of 0.006s (:const:`ALC_HOLD_TIME_MIN`)
        increasing exponentially to a maximum of 6.14s (:const:`ALC_HOLD_TIME_MAX`).

        :default: 0.024
        """
        return ALC_ATTACK_TIME_MIN * pow(2, self._alc_attack)

    @alc_attack_time.setter
    def alc_attack_time(self, value: float) -> None:
        self._alc_attack = min(
            round(
                math.log(
                    (
                        constrain(value, ALC_ATTACK_TIME_MIN, ALC_ATTACK_TIME_MAX)
                        - ALC_ATTACK_TIME_MIN
                    )
                    / ALC_ATTACK_TIME_MIN,
                    2.0,
                )
            ),
            _ALC_ATTACK_MAX,
        )

    alc_limiter: bool = WOBit(_REG_ALC3, 8)
    """Whether or not the ALC limiter is enabled to prevent clipping. This function is automatically
    enabled whenever :attr:`alc` is set to `True`.

    :default: `False`
    """

    # Noise Gate

    noise_gate: bool = WOBit(_REG_NOISE_GATE, 0)
    """Whether or not the noise gate is enabled. The ALC, :attr:`alc`, must be set to `True` for
    this functionality to work. This feature will help prevent "noise pumping" during periods of
    quiet input signal. Only applicable to signal into the microphone amplifier.

    :default: `False`
    """

    _noise_gate_threshold: int = WOBits(5, _REG_NOISE_GATE, 3)

    @property
    def noise_gate_threshold(self) -> float:
        """The input signal level below at which to engage the noise gate in decibels.

        :default: -76.5
        """
        return map_range(self._noise_gate_threshold, 0, 31, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX)

    @noise_gate_threshold.setter
    def noise_gate_threshold(self, value: float) -> None:
        self._noise_gate_threshold = round(
            map_range(value, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX, 0, 31)
        )

    # DAC

    left_dac: bool = WOBit(_REG_PWR_MGMT_2, 8)
    """Whether or not the left channel of the DAC is enabled.

    :default: `False`
    """

    right_dac: bool = WOBit(_REG_PWR_MGMT_2, 7)
    """Whether or not the right channel of the DAC is enabled.

    :default: `False`
    """

    @property
    def dac(self) -> bool:
        """Whether or not the boths channels of the DAC are enabled.

        :default: `False`
        """
        return self.left_dac and self.right_dac

    @dac.setter
    def dac(self, value: bool) -> None:
        self.left_dac = self.right_dac = value

    _left_dac_volume: int = WOBits(8, _REG_LEFT_DAC_VOLUME, 0)
    _left_dac_volume_set: bool = WOBit(_REG_LEFT_DAC_VOLUME, 8)

    @property
    def left_dac_volume(self) -> float:
        """The digital volume of the left channel DAC in decibels. Accepts a minimum value of
        -127.0dB (:const:`DAC_VOLUME_MIN`) and a maximum of +0.0dB (:const:`DAC_VOLUME_MAX`).

        :default: 0.0
        """
        return map_range(self._left_dac_volume, 1, 255, DAC_VOLUME_MIN, DAC_VOLUME_MAX)

    @left_dac_volume.setter
    def left_dac_volume(self, value: float) -> None:
        self._left_dac_volume = round(map_range(value, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 1, 255))
        self._left_dac_volume_set = True

    _right_dac_volume: int = WOBits(8, _REG_RIGHT_DAC_VOLUME, 0)
    _right_dac_volume_set: bool = WOBit(_REG_RIGHT_DAC_VOLUME, 8)

    @property
    def right_dac_volume(self) -> float:
        """The digital volume of the right channel DAC in decibels. Accepts a minimum value of
        -127.0dB (:const:`DAC_VOLUME_MIN`) and a maximum of +0.0dB (:const:`DAC_VOLUME_MAX`).

        :default: 0.0
        """
        return map_range(self._right_dac_volume, 1, 255, DAC_VOLUME_MIN, DAC_VOLUME_MAX)

    @right_dac_volume.setter
    def right_dac_volume(self, value: float) -> None:
        self._right_dac_volume = round(map_range(value, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 1, 255))
        self._right_dac_volume_set = True

    @property
    def dac_volume(self) -> float:
        """The digital volume of both channels of the DAC in decibels. Accepts a minimum value of
        -127.0dB (:const:`DAC_VOLUME_MIN`) and a maximum of +0.0dB (:const:`DAC_VOLUME_MAX`).

        :default: 0.0
        """
        return max(self.left_dac_volume, self.right_dac_volume)

    @dac_volume.setter
    def dac_volume(self, value: float) -> None:
        self._left_dac_volume = self._right_dac_volume = round(
            map_range(value, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 1, 255)
        )
        self._left_dac_volume_set = self._right_dac_volume_set = True

    dac_mute: bool = WOBit(_REG_ADC_DAC_CTRL_1, 3)
    """Whether or not the DAC is soft-muted. By default, this feature is enabled. To play back an
    audio signal from the DAC, this must first set to `False`.

    :default: `True`
    """

    dac_soft_mute: bool = WOBit(_REG_ADC_DAC_CTRL_2, 3)
    """Prevents a sudden volume increase when disabling :attr:`dac_mute` during playback to avoid
    creating a popping noise.

    :default: `False`
    """

    dac_slow_soft_mute: bool = WOBit(_REG_ADC_DAC_CTRL_2, 2)
    """The rate at which the DAC soft mute, :attr:`dac_mute`, ramps up or down. When set to `False`,
    it takes roughly 10.7ms to mute or unmute (when :attr:`sample_rate` is set to 48000). When set
    to `True`, it takes roughly 171ms.

    :default: `False`
    """

    dac_attenuation: bool = WOBit(_REG_ADC_DAC_CTRL_1, 7)
    """When set to `True`, the signal of the DAC will be attenuated by -6dB. This is commonly used
    when :attr:`enhance` is set to `True` to avoid limiting.

    :default: `False`
    """

    # 3D Enhance

    enhance: bool = WOBit(_REG_3D_CONTROL, 0)
    """Whether or not the digital 3D stereo enhancement feature on the DAC is enabled. This option
    will artificially increase the separation between the left and right channels.

    :default: `False`
    """

    enhance_filter_lpf: bool = WOBit(_REG_3D_CONTROL, 6)
    """Whether or not a low-pass filter is applied before 3D enhancement processing. Recommended
    when :attr:`sample_rate` is less than 32000.

    :default: `False`
    """

    enhance_filter_hpf: bool = WOBit(_REG_3D_CONTROL, 5)
    """Whether or not a high-pass filter is applied before 3D enhancement processing. Recommended
    when :attr:`sample_rate` is less than 32000.

    :default: `False`
    """

    _enhance_depth: int = WOBits(4, _REG_3D_CONTROL, 1)

    @property
    def enhance_depth(self) -> float:
        """The depth of the 3D stereo enhancement effect. Accepts a monotonic value of 0.0 to 1.0.

        :default: 0.0
        """
        return self._enhance_depth / 15.0

    @enhance_depth.setter
    def enhance_depth(self, value: float) -> None:
        self._enhance_depth = round(map_range(value, 0.0, 1.0, 0, 15))

    # Output Mixer

    left_output: bool = WOBit(_REG_PWR_MGMT_3, 3)
    """Whether or not the left channel of the output mixer is enabled.

    :default: `False`
    """

    right_output: bool = WOBit(_REG_PWR_MGMT_3, 2)
    """Whether or not the right channel of the output mixer is enabled.

    :default: `False`
    """

    @property
    def output(self) -> bool:
        """Whether or not both channels of the output mixer are enabled.

        :default: `False`
        """
        return self.left_output and self.right_output

    @output.setter
    def output(self, value: bool) -> None:
        self.left_output = self.right_output = value

    ## DAC Output

    left_dac_output: bool = WOBit(_REG_LEFT_OUT_MIX, 8)
    """Whether or not the left channel of the DAC is connected to the left channel of the output
    mixer.

    :default: `False`
    """

    right_dac_output: bool = WOBit(_REG_RIGHT_OUT_MIX, 8)
    """Whether or not the right channel of the DAC is connected to the right channel of the output
    mixer.

    :default: `False`
    """

    @property
    def dac_output(self) -> bool:
        """Whether or not both channels of the DAC are connected to the output mixer. Required for
        playback of the DAC through the headphone or speaker amplifier.

        :default: `False`
        """
        return self.left_dac_output and self.right_dac_output

    @dac_output.setter
    def dac_output(self, value: bool) -> None:
        self.left_dac_output = self.right_dac_output = value

    ## Input 3 Output

    left_input3_output: bool = WOBit(_REG_LEFT_OUT_MIX, 7)
    """The connection of the left channel of input 3 directly to the output mixer, bypassing the
    ADC.

    :default: `False`
    """

    right_input3_output: bool = WOBit(_REG_RIGHT_OUT_MIX, 7)
    """The connection of the right channel of input 3 directly to the output mixer, bypassing the
    ADC.

    :default: `False`
    """

    @property
    def input3_output(self) -> bool:
        """The connection of both channels of input 3 directly to the output mixer, bypassing the
        ADC.

        :default: `False`
        """
        return self.left_input3_output and self.right_input3_output

    @input3_output.setter
    def input3_output(self, value: bool) -> None:
        self.left_input3_output = self.right_input3_output = value

    _left_input3_output_volume: int = WOBits(3, _REG_LEFT_OUT_MIX, 4)

    @property
    def left_input3_output_volume(self) -> float:
        """The level of the left channel of input 3 into the output mixer in decibels. Accepts a
        value from -21.0dB (:const:`OUTPUT_VOLUME_MIN`) to +0.0dB (:const:`OUTPUT_VOLUME_MAX`).

        :default: -21.0
        """
        return map_range(
            self._left_input3_output_volume, 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN
        )

    @left_input3_output_volume.setter
    def left_input3_output_volume(self, value: float) -> None:
        self._left_input3_output_volume = round(
            map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)
        )

    _right_input3_output_volume: int = WOBits(3, _REG_RIGHT_OUT_MIX, 4)

    @property
    def right_input3_output_volume(self) -> float:
        """The level of the right channel of input 3 into the output mixer in decibels. Accepts a
        value from -21.0dB (:const:`OUTPUT_VOLUME_MIN`) to +0.0dB (:const:`OUTPUT_VOLUME_MAX`).

        :default: -21.0
        """
        return map_range(
            self._right_input3_output_volume, 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN
        )

    @right_input3_output_volume.setter
    def right_input3_output_volume(self, value: float) -> None:
        self._right_input3_output_volume = round(
            map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)
        )

    @property
    def input3_output_volume(self) -> float:
        """The level of both channels of input 3 into the output mixer in decibels. Accepts a value
        from -21.0dB (:const:`OUTPUT_VOLUME_MIN`) to +0.0dB (:const:`OUTPUT_VOLUME_MAX`).

        :default: -21.0
        """
        return max(self.left_input3_output_volume, self.right_input3_output_volume)

    @input3_output_volume.setter
    def input3_output_volume(self, value: float) -> None:
        self._left_input3_output_volume = self._right_input3_output_volume = round(
            map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)
        )

    ## MIC Boost Mixer Output

    left_mic_output: bool = WOBit(_REG_BYPASS_1, 7)
    """The connection of the left channel of the microphone amplifier directly to the output mixer,
    bypassing the ADC.

    :default: `False`
    """

    right_mic_output: bool = WOBit(_REG_BYPASS_2, 7)
    """The connection of the right channel of the microphone amplifier directly to the output mixer,
    bypassing the ADC.

    :default: `False`
    """

    @property
    def mic_output(self) -> bool:
        """The connection of both channels of the microphone amplifier directly to the output mixer,
        bypassing the ADC.

        :default: `False`
        """
        return self.left_mic_output and self.right_mic_output

    @mic_output.setter
    def mic_output(self, value: bool) -> None:
        self.left_mic_output = self.right_mic_output = value

    _left_mic_output_volume: int = WOBits(3, _REG_BYPASS_1, 4)

    @property
    def left_mic_output_volume(self) -> float:
        """The level of the left channel of the microphone amplifier into the output mixer in
        decibels. Accepts a value from -21.0dB (:const:`OUTPUT_VOLUME_MIN`) to +0.0dB
        (:const:`OUTPUT_VOLUME_MAX`).

        :default: -21.0
        """
        return map_range(self._left_mic_output_volume, 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN)

    @left_mic_output_volume.setter
    def left_mic_output_volume(self, value: float) -> None:
        self._left_mic_output_volume = round(
            map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)
        )

    _right_mic_output_volume: int = WOBits(3, _REG_BYPASS_2, 4)

    @property
    def right_mic_output_volume(self) -> float:
        """The level of the right channel of the microphone amplifier into the output mixer in
        decibels. Accepts a value from -21.0dB (:const:`OUTPUT_VOLUME_MIN`) to +0.0dB
        (:const:`OUTPUT_VOLUME_MAX`).

        :default: -21.0
        """
        return map_range(self._right_mic_output_volume, 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN)

    @right_mic_output_volume.setter
    def right_mic_output_volume(self, value: float) -> None:
        self._right_mic_output_volume = round(
            map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)
        )

    @property
    def mic_output_volume(self) -> float:
        """The level of both channels of the microphone amplifier into the output mixer in decibels.
        Accepts a value from -21.0dB (:const:`OUTPUT_VOLUME_MIN`) to +0.0dB
        (:const:`OUTPUT_VOLUME_MAX`).

        :default: -21.0
        """
        return max(self.left_mic_output_volume, self.right_mic_output_volume)

    @mic_output_volume.setter
    def mic_output_volume(self, value: float) -> None:
        self._left_mic_output_volume = self._right_mic_output_volume = round(
            map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)
        )

    ## Mono Output

    mono_output: bool = WOBit(_REG_PWR_MGMT_2, 1)
    """Whether or not the mono output mixer is enabled. This can be used to provide a mono mix of
    the left and right channels of the output mixer (if :attr:`mono_left_mix` or
    :attr:`mono_right_mix` is set to `True`) or as a buffer for the headphone amplifier to allow
    capless headphone output.

    :default: `False`
    """

    mono_left_mix: bool = WOBit(_REG_MONO_OUT_MIX_1, 7)
    """Whether or not the left channel of the output mixer is connected to the mono output.

    :default: `False`
    """

    mono_right_mix: bool = WOBit(_REG_MONO_OUT_MIX_2, 7)
    """Whether or not the right channel of the output mixer is connected to the mono output.

    :default: `False`
    """

    @property
    def mono_mix(self) -> bool:
        """Whether or not both channels of the output mixer are connected to the mono output.

        :default: `False`
        """
        return self.mono_left_mix and self.mono_right_mix

    @mono_mix.setter
    def mono_mix(self, value: bool) -> None:
        self.mono_left_mix = self.mono_right_mix = value

    mono_output_attenuation: bool = WOBit(_REG_MONO_OUT_VOLUME, 6)
    """When set to `True`, the signal of the mono output will be attenuated by -6dB. This can help
    prevent clipping. This property should not be modified when :attr:`mono_output` is set to `True`
    as this may cause an audible click noise.

    :default: `False`
    """

    # Amplifier

    ## Headphones

    left_headphone: bool = WOBit(_REG_PWR_MGMT_2, 6)
    """Whether or not the left channel of the headphone amplifier is powered on.

    :default: `False`
    """

    right_headphone: bool = WOBit(_REG_PWR_MGMT_2, 5)
    """Whether or not the right channel of the headphone amplifier is powered on.

    :default: `False`
    """

    @property
    def headphone(self) -> bool:
        """Whether or not the headphone amplifier is powered on.

        :default: `False`
        """
        return self.left_headphone and self.right_headphone

    @headphone.setter
    def headphone(self, value: bool) -> None:
        self.left_headphone = self.right_headphone = value

    headphone_standby: bool = WOBit(_REG_ANTI_POP_1, 0)
    """Headphone amplifier standby mode.

    :default: `False`
    """

    _left_headphone_volume: int = WOBits(7, _REG_LOUT1_VOLUME, 0)
    _left_headphone_volume_set: bool = WOBit(_REG_LOUT1_VOLUME, 8)

    @property
    def left_headphone_volume(self) -> float:
        """The volume level of the left channel of the headphone amplifier in decibels. Accepts a
        minimum value of -73.0dB (:const:`AMP_VOLUME_MIN`) and a maximum of +6.0dB
        (:const:`AMP_VOLUME_MAX`). If set to a value less than the minimum (-73.0dB), the left
        channel will be muted and this property will return a value of `None`.

        :default: `None`
        """
        value = self._left_headphone_volume
        if value < 48:
            return None
        return map_range(value, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX)

    @left_headphone_volume.setter
    def left_headphone_volume(self, value: float) -> None:
        self._left_headphone_volume = (
            round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127))
            if value >= AMP_VOLUME_MIN
            else 0
        )
        self._left_headphone_volume_set = True

    _right_headphone_volume: int = WOBits(7, _REG_ROUT1_VOLUME, 0)
    _right_headphone_volume_set: bool = WOBit(_REG_ROUT1_VOLUME, 8)

    @property
    def right_headphone_volume(self) -> float:
        """The volume level of the right channel of the headphone amplifier in decibels. Accepts a
        minimum value of -73.0dB (:const:`AMP_VOLUME_MIN`) and a maximum of +6.0dB
        (:const:`AMP_VOLUME_MAX`). If set to a value less than the minimum (-73.0dB), the right
        channel will be muted and this property will return a value of `None`.

        :default: `None`
        """
        value = self._right_headphone_volume
        if value < 48:
            return None
        return map_range(value, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX)

    @right_headphone_volume.setter
    def right_headphone_volume(self, value: float) -> None:
        self._right_headphone_volume = (
            round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127))
            if value >= AMP_VOLUME_MIN
            else 0
        )
        self._right_headphone_volume_set = True

    @property
    def headphone_volume(self) -> float:
        """The volume level of both channels of the headphone amplifier in decibels. Accepts a
        minimum value of -73.0dB (:const:`AMP_VOLUME_MIN`) and a maximum of +6.0dB
        (:const:`AMP_VOLUME_MAX`). If set to a value less than the minimum (-73.0dB), the amplifier
        will be muted and this property will return a value of `None`.

        :default: `None`
        """
        left = self.left_headphone_volume
        right = self.right_headphone_volume
        if left is None or right is None:
            if left is None and right is None:
                return None
            return right if left is None else left
        return max(left, right)

    @headphone_volume.setter
    def headphone_volume(self, value: float) -> None:
        self._left_headphone_volume = self._right_headphone_volume = (
            round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127))
            if value >= AMP_VOLUME_MIN
            else 0
        )
        self._left_headphone_volume_set = self._right_headphone_volume_set = True

    left_headphone_zero_cross: bool = WOBit(_REG_LOUT1_VOLUME, 7)
    """Whether or not the volume of the left headphone amplifier channel will be adjusted when a
    "zero" input level is detected in order to avoid harsh jumps in the output.

    :default: `False`
    """

    right_headphone_zero_cross: bool = WOBit(_REG_LOUT1_VOLUME, 7)
    """Whether or not the volume of the right headphone amplifier channel will be adjusted when a
    "zero" input level is detected in order to avoid harsh jumps in the output.

    :default: `False`
    """

    @property
    def headphone_zero_cross(self) -> bool:
        """Whether or not the volume of the headphone amplifier channel will be adjusted when a
        "zero" input level is detected in order to avoid harsh jumps in the output.

        :default: `False`
        """
        return self.left_headphone_zero_cross and self.right_headphone_zero_cross

    @headphone_zero_cross.setter
    def headphone_zero_cross(self, value: bool) -> None:
        self.left_headphone_zero_cross = self.right_headphone_zero_cross = value

    ## Speakers

    _left_speaker: bool = WOBit(_REG_PWR_MGMT_2, 4)
    _left_speaker_amp: bool = WOBit(_REG_CLASS_D_CONTROL_1, 6)

    @property
    def left_speaker(self) -> bool:
        """Whether or not the left channel of the speaker amplifier is powered on.

        :default: `False`
        """
        return self._left_speaker and self._left_speaker_amp

    @left_speaker.setter
    def left_speaker(self, value: bool) -> None:
        self._left_speaker = self._left_speaker_amp = value

    _right_speaker: bool = WOBit(_REG_PWR_MGMT_2, 3)
    _right_speaker_amp: bool = WOBit(_REG_CLASS_D_CONTROL_1, 7)

    @property
    def right_speaker(self) -> bool:
        """Whether or not the right channel of the speaker amplifier is powered on.

        :default: `False`
        """
        return self._right_speaker and self._right_speaker_amp

    @right_speaker.setter
    def right_speaker(self, value: bool) -> None:
        self._right_speaker = self._right_speaker_amp = value

    @property
    def speaker(self) -> bool:
        """Whether or not the speaker amplifier is powered on.

        :default: `False`
        """
        return self.left_speaker and self.right_speaker

    @speaker.setter
    def speaker(self, value: bool) -> None:
        self.left_speaker = self.right_speaker = value

    _left_speaker_volume: int = WOBits(7, _REG_LOUT2_VOLUME, 0)
    _left_speaker_volume_set: bool = WOBit(_REG_LOUT2_VOLUME, 8)

    @property
    def left_speaker_volume(self) -> float:
        """The volume level of the left channel of the speaker amplifier in decibels. Accepts a
        minimum value of -73.0dB (:const:`AMP_VOLUME_MIN`) and a maximum of +6.0dB
        (:const:`AMP_VOLUME_MAX`). If set to a value less than the minimum (-73.0dB), the left
        channel will be muted and this property will return a value of `None`.

        :default: `None`
        """
        value = self._left_speaker_volume
        if value < 48:
            return None
        return map_range(value, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX)

    @left_speaker_volume.setter
    def left_speaker_volume(self, value: float) -> None:
        self._left_speaker_volume = (
            round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127))
            if value >= AMP_VOLUME_MIN
            else 0
        )
        self._left_speaker_set = True

    _right_speaker_volume: int = WOBits(7, _REG_ROUT2_VOLUME, 0)
    _right_speaker_volume_set: bool = WOBit(_REG_ROUT2_VOLUME, 8)

    @property
    def right_speaker_volume(self) -> float:
        """The volume level of the right channel of the speaker amplifier in decibels. Accepts a
        minimum value of -73.0dB (:const:`AMP_VOLUME_MIN`) and a maximum of +6.0dB
        (:const:`AMP_VOLUME_MAX`). If set to a value less than the minimum (-73.0dB), the right
        channel will be muted and this property will return a value of `None`.

        :default: `None`
        """
        value = self._right_speaker_volume
        if value < 48:
            return None
        return map_range(value, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX)

    @right_speaker_volume.setter
    def right_speaker_volume(self, value: float) -> None:
        self._right_speaker_volume = (
            round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127))
            if value >= AMP_VOLUME_MIN
            else 0
        )
        self._right_speaker_volume_set = True

    @property
    def speaker_volume(self) -> float:
        """The volume level of the speaker amplifier in decibels. Accepts a minimum value of -73.0dB
        (:const:`AMP_VOLUME_MIN`) and a maximum of +6.0dB (:const:`AMP_VOLUME_MAX`). If set to a
        value less than the minimum (-73.0dB), the amplifier will be muted and this property will
        return a value of `None`.

        :default: `None`
        """
        left = self.left_speaker_volume
        right = self.right_speaker_volume
        if left is None or right is None:
            if left is None and right is None:
                return None
            return right if left is None else left
        return max(left, right)

    @speaker_volume.setter
    def speaker_volume(self, value: float) -> None:
        self._left_speaker_volume = self._right_speaker_volume = (
            round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127))
            if value >= AMP_VOLUME_MIN
            else 0
        )
        self._left_speaker_volume_set = self._right_speaker_volume_set = True

    left_speaker_zero_cross: bool = WOBit(_REG_LOUT2_VOLUME, 7)
    """Whether or not the volume of the left speaker amplifier channel will be adjusted when a
    "zero" input level is detected in order to avoid harsh jumps in the output.

    :default: `False`
    """

    right_speaker_zero_cross: bool = WOBit(_REG_LOUT2_VOLUME, 7)
    """Whether or not the volume of the right speaker amplifier channel will be adjusted when a
    "zero" input level is detected in order to avoid harsh jumps in the output.

    :default: `False`
    """

    @property
    def speaker_zero_cross(self) -> bool:
        """Whether or not the volume of the speaker amplifier will be adjusted when a "zero" input
        level is detected in order to avoid harsh jumps in the output.

        :default: `False`
        """
        return self.left_speaker_zero_cross and self.right_speaker_zero_cross

    @speaker_zero_cross.setter
    def speaker_zero_cross(self, value: bool) -> None:
        self.left_speaker_zero_cross = self.right_speaker_zero_cross = value

    @staticmethod
    def _get_speaker_boost_gain(value: float) -> int:
        for i in reversed(range(len(_SPEAKER_BOOST_GAIN))):
            if value >= _SPEAKER_BOOST_GAIN[i]:
                return i
        return 0

    _speaker_dc_gain: int = WOBits(3, _REG_CLASS_D_CONTROL_3, 3)

    @property
    def speaker_dc_gain(self) -> float:
        """Speaker DC output level boost on both left and right channels in decibels. Accepts values
        of +0.0dB, +2.1dB, +2.9dB, +3.6dB, +4.5dB, and +5.1dB.

        :default: 0.0
        """
        return _SPEAKER_BOOST_GAIN[self._speaker_dc_gain]

    @speaker_dc_gain.setter
    def speaker_dc_gain(self, value: float) -> None:
        self._speaker_dc_gain = WM8960_Advanced._get_speaker_boost_gain(value)

    _speaker_ac_gain: int = WOBits(3, _REG_CLASS_D_CONTROL_3, 0)

    @property
    def speaker_ac_gain(self) -> float:
        """Speaker AC output level boost on both left and right channels in decibels. Accepts values
        of +0.0dB, +2.1dB, +2.9dB, +3.6dB, +4.5dB, and +5.1dB.

        :default: 0.0
        """
        return _SPEAKER_BOOST_GAIN[self._speaker_ac_gain]

    @speaker_ac_gain.setter
    def speaker_ac_gain(self, value: float) -> None:
        self._speaker_ac_gain = WM8960_Advanced._get_speaker_boost_gain(value)

    # Digital Audio Interface Control

    loopback: bool = WOBit(_REG_AUDIO_INTERFACE_2, 0)
    """Whether or not digital loopback between the ADC and DAC is enabled.

    :default: `False`
    """

    pll: bool = WOBit(_REG_PWR_MGMT_2, 0)
    """Whether or not the phase-locked loop (PLL) clock is powered on.

    :default: `False`
    """

    pll_prescale_div2: bool = WOBit(_REG_PLL_N, 4)
    """Divide MCLK by 2 before input to PLL.

    :default: `False`
    """

    pll_n: int = WOBits(4, _REG_PLL_N, 0)
    """The intenger (N) part of PLL input/output ratio. Accepts a value greater than 5 and less than
    13.

    :default: 8
    """

    _pll_k1: int = WOBits(6, _REG_PLL_K_1, 0)
    _pll_k2: int = WOBits(9, _REG_PLL_K_2, 0)
    _pll_k3: int = WOBits(9, _REG_PLL_K_3, 0)

    @property
    def pll_k(self) -> int:
        """The fractional (K) part of PLL input/output ratio. Accepts a 24-bit unsigned integer.

        :default: 0x3126E9
        """
        return self._pll_k1 << 18 + self._pll_k2 << 9 + self._pll_k3

    @pll_k.setter
    def pll_k(self, value: int) -> None:
        self._pll_k1 = (value >> 18) & 0b111111
        self._pll_k2 = (value >> 9) & 0b111111111
        self._pll_k3 = value & 0b111111111

    clock_fractional_mode: bool = WOBit(_REG_PLL_N, 5)
    """Whether the integer mode (`False`) or fractional mode (`True`) of the PLL is used to
    calculate the output clock.

    :default: `False`
    """

    clock_from_pll: bool = WOBit(_REG_CLOCKING_1, 0)
    """Whether the SYSCLK is derived from MCLK (`False`) or the output of the PLL (`True`).

    :default: `False`
    """

    _system_clock_divider: int = WOBits(2, _REG_CLOCKING_1, 1)

    @property
    def system_clock_div2(self) -> bool:
        """Whether the clock source of the SYSCLK (see :attr:`clock_from_pll`) is divided by 1
        (`False`) or 2 (`True`).

        :default: `False`
        """
        return self._system_clock_divider == _SYSCLK_DIV_BY_2

    @system_clock_div2.setter
    def system_clock_div2(self, value: bool) -> None:
        self._system_clock_divider = _SYSCLK_DIV_BY_2 if value else _SYSCLK_DIV_BY_1

    _adc_clock_divider: int = WOBits(3, _REG_CLOCKING_1, 6)

    @property
    def adc_clock_divider(self) -> float:
        """The sample rate divisor of the ADC as SYSCLK / (value * 256). Accepts a value of 1.0,
        1.5, 2.0, 3.0, 4.0, 5.5, or 6.0.

        :default: 1.0
        """
        return _ADCDACDIV[min(self._adc_clock_divider, len(_ADCDACDIV))]

    @adc_clock_divider.setter
    def adc_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._adc_clock_divider = _ADCDACDIV.index(value)

    _dac_clock_divider: int = WOBits(3, _REG_CLOCKING_1, 3)

    @property
    def dac_clock_divider(self) -> float:
        """The sample rate divisor of the DAC as SYSCLK / (value * 256). Accepts a value of 1.0,
        1.5, 2.0, 3.0, 4.0, 5.5, or 6.0.

        :default: 1.0
        """
        return _ADCDACDIV[min(self._dac_clock_divider, len(_ADCDACDIV))]

    @dac_clock_divider.setter
    def dac_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _ADCDACDIV:
            self._dac_clock_divider = _ADCDACDIV.index(value)

    _base_clock_divider: int = WOBits(4, _REG_CLOCKING_2, 0)

    @property
    def base_clock_divider(self) -> float:
        """The divisor of the SYSCLK (when :attr:`master_mode` is set to `True`) when determining
        BCLK frequency as SYSCLK / value. Accepts a value of 1.0, 1.5, 2.0, 3.0, 4.0, 5.5, 6.0, 8.0,
        11.0, 12.0, 16.0, 22.0, 24.0, or 32.0.

        :default: 1.0
        """
        return _BCLKDIV[min(self._base_clock_divider, len(_BCLKDIV))]

    @base_clock_divider.setter
    def base_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _BCLKDIV:
            self._base_clock_divider = _BCLKDIV.index(value)

    _amp_clock_divider: int = WOBits(3, _REG_CLOCKING_2, 6)

    @property
    def amp_clock_divider(self) -> float:
        """The divisor of the SYSCLK used to operating the Class D amplifier switching clock as
        SYSCLK / value. Accepts a value of 1.5 (not recommended), 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, and
        16.0.

        :default: 16.0
        """
        return _DCLKDIV[min(self._amp_clock_divider, len(_DCLKDIV))]

    @amp_clock_divider.setter
    def amp_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _DCLKDIV:
            self._amp_clock_divider = _DCLKDIV.index(value)

    ## Mode

    master_mode: bool = WOBit(_REG_AUDIO_INTERFACE_1, 6)
    """Whether the operation of the digital interface is controlled externally in slave mode
    (`False`) or operated internally in master mode (`True`).

    :default: `False`
    """

    _bit_depth: int = WOBits(2, _REG_AUDIO_INTERFACE_1, 2)

    @property
    def bit_depth(self) -> int:
        """The number of bits per sample. The values 16, 20, 24, and 32 are supported.

        :default: 32
        """
        value = self._bit_depth
        return 32 if value == 3 else 16 + 4 * value

    @bit_depth.setter
    def bit_depth(self, value: int) -> None:
        self._bit_depth = (min(value, 28) - 16) // 4

    word_select_invert: bool = WOBit(_REG_AUDIO_INTERFACE_1, 4)
    """The polarity of the LRCLK, either left-first (`False`) or right-first (`True`).

    :default: `False`
    """

    adc_channel_swap: bool = WOBit(_REG_AUDIO_INTERFACE_1, 8)
    """Whether or not to swap left and right ADC data.

    :default: `False`
    """

    _vref_output_disable: bool = WOBit(_REG_ADDITIONAL_CONTROL_3, 6)

    @property
    def vref_output(self) -> bool:
        """Whether or not VMID is sent to the output circuitry. If set to `False`, it will
        essentially disable the output of the device.

        :default `True`
        """
        return not self._vref_output_disable

    @vref_output.setter
    def vref_output(self, value: bool) -> None:
        self._vref_output_disable = not value

    _vsel: int = WOBits(2, _REG_ADDITIONAL_CONTROL_1, 6)

    @property
    def power_supply(self) -> float:
        """The incoming voltage of the AVDD power supply in volts. Setting this value appropriately
        will optimize bias current. Accepts a value of 2.7v or 3.3v.

        :default: 3.3
        """
        return (constrain(self._vsel, 1, 2) - 1) * 0.6 + 2.7

    @power_supply.setter
    def power_supply(self, value: float) -> None:
        self._vsel = (constrain(value, 2.7, 3.3) - 2.7) // 0.6 * 2 + 1

    ## GPIO

    gpio_output: bool = WOBit(_REG_AUDIO_INTERFACE_2, 6)
    """Whether or not to enable special GPIO operation modes on the ADCLRC/GPIO1 pin.

    :default: `False`
    """

    gpio_output_mode: int = WOBits(3, _REG_ADDITIONAL_CONTROL_4, 4)
    """The GPIO operation mode of ADCLRC/GPIO1 when :attr:`gpio_output` is set to `True`. Accepts a
    value between 0 and 7. See WM8960 datasheet for the operation of each function.

    :default: 0
    """

    gpio_output_invert: bool = WOBit(_REG_ADDITIONAL_CONTROL_4, 7)
    """Whether or not to invert the polarity of the output of ADCLRC/GPIO1 when :attr:`gpio_output`
    is set to `True`.

    :default: `False`
    """

    _gpio_clock_divider: int = WOBits(3, _REG_CLOCKING_2, 6)

    @property
    def gpio_clock_divider(self) -> float:
        """The divisor of the GPIO clock (when :attr:`gpio_output_mode` is set to 4) as SYSCLK /
        value. Accepts a value of 1.0, 2.0, 3.0, 4.0, 5.5, or 6.0.

        :default: 1.0
        """
        return _OPCLKDIV[min(self._gpio_clock_divider, len(_ADCDACDIV))]

    @gpio_clock_divider.setter
    def gpio_clock_divider(self, value: float) -> None:
        value = round(value * 2.0) / 2.0
        if value in _OPCLKDIV:
            self._gpio_clock_divider = _OPCLKDIV.index(value)

    @property
    def sample_rate(self) -> int:
        """The rate of the ADC/DAC processing of the device in samples per second used for I2S
        communication and internal digital processing. If this property has not been set to a valid
        value before being accessed, it will return `None`. The sample rates 8000, 11025, 12000,
        16000, 22050, 24000, 32000, 44100, and 48000 are supported.

        NOTE: This assumes that the master clock of the WM8960 is 24 MHz in order determine
        appropriate clock settings.

        :default: `None`
        """
        return self._sample_rate

    @sample_rate.setter
    def sample_rate(self, value: int) -> None:
        # MCLK = 24 MHz
        self.pll = True  # Needed for class-d amp clock
        self.clock_fractional_mode = True
        self.clock_from_pll = True

        self.pll_prescale_div2 = True
        self.system_clock_div2 = True
        self.base_clock_divider = 4.0
        self.amp_clock_divider = 16.0

        if value in {8000, 12000, 16000, 24000, 32000, 48000}:
            # SYSCLK = 12.288 MHz
            # DCLK = 768.0k_hz
            self.pll_n = 8
            self.pll_k = 0x3126E8
            self.adc_clock_divider = self.dac_clock_divider = 48000 / value

        elif value in {11025, 22050, 44100}:
            # SYSCLK = 11.2896 MHz
            # DCLK = 705.6k_hz
            self.pll_n = 7
            self.pll_k = 0x86C226
            self.adc_clock_divider = self.dac_clock_divider = 44100 / value

        else:
            raise ValueError("Invalid sample rate")

        self._sample_rate = value

    def __init__(self, i2c_bus: I2C, address: int = _DEFAULT_I2C_ADDR) -> None:
        """
        Initialize the WM8960 device.

        This function initialized the I2C device, performs a reset, turns on power, and sets vmid to
        :const:`Vmid_Mode.PLAYBACK`.

        :param i2c: The I2C bus.
        :param address: The I2C address of the device. Defaults to 0x1A.
        """
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._sample_rate = None

        self._registers = [0] * len(_REG_DEFAULTS)
        for i, reg in enumerate(self._registers):
            self._registers[i] = bytearray(reg.to_bytes(2, "big"))

        # Must be called before `reset` to ensure that _REG_RESET is addressed properly
        self._reset_registers()

        self.reset()

        # General setup
        self.power = True
        self.vmid = Vmid_Mode.PLAYBACK

    # Resets all registers to their default state
    _reset: bool = WOBit(_REG_RESET, 7)

    def _reset_registers(self) -> None:
        for i, reg in enumerate(self._registers):
            reg[:] = _REG_DEFAULTS[i].to_bytes(2, "big")
            reg[0] |= i << 1

    def reset(self) -> None:
        """Resets all parameters of the WM8960. All audio and digital functionality will be disabled
        after calling this function. In order to resume normal operation, :attr:`power` must be set
        to `True` and :attr:`vmid` should be set to :const:`Vmid_Mode.PLAYBACK`.
        """
        self._reset = True
        self._reset_registers()
