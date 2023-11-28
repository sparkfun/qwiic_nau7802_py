#-------------------------------------------------------------------------------
# qwiic_nau7802.py
#
# Python library for the SparkFun Qwiic Scale, available here:
# https://www.sparkfun.com/products/15242
#-------------------------------------------------------------------------------
# Written by SparkFun Electronics, November 2023
#
# This python library supports the SparkFun Electroncis Qwiic ecosystem
#
# More information on Qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#===============================================================================
# Copyright (c) 2023 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#===============================================================================
# This code was generated in part with ChatGPT, created by OpenAI. The code was
# reviewed and edited by the following human(s):
#
# Dryw Wade
#===============================================================================

"""
qwiic_nau7802
============
Python module for the [SparkFun Qwiic Scale](https://www.sparkfun.com/products/15242)
This is a port of the existing [Arduino Library](https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library)
This package can be used with the overall [SparkFun Qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)
New to Qwiic? Take a look at the entire [SparkFun Qwiic ecosystem](https://www.sparkfun.com/qwiic).
"""

# The Qwiic_I2C_Py platform driver is designed to work on almost any Python
# platform, check it out here: https://github.com/sparkfun/Qwiic_I2C_Py
import qwiic_i2c
import time

# Define the device name and I2C addresses. These are set in the class defintion
# as class variables, making them avilable without having to create a class
# instance. This allows higher level logic to rapidly create a index of Qwiic
# devices at runtine
_DEFAULT_NAME = "Qwiic NAU7802"

# Some devices have multiple available addresses - this is a list of these
# addresses. NOTE: The first address in this list is considered the default I2C
# address for the device.
_AVAILABLE_I2C_ADDRESS = [0x2A]

# Define the class that encapsulates the device being created. All information
# associated with this device is encapsulated by this class. The device class
# should be the only value exported from this module.
class QwiicNAU7802(object):
    # Set default name and I2C address(es)
    device_name         = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    # Register Map
    NAU7802_PU_CTRL = 0x00
    NAU7802_CTRL1 = 0x01
    NAU7802_CTRL2 = 0x02
    NAU7802_OCAL1_B2 = 0x03
    NAU7802_OCAL1_B1 = 0x04
    NAU7802_OCAL1_B0 = 0x05
    NAU7802_GCAL1_B3 = 0x06
    NAU7802_GCAL1_B2 = 0x07
    NAU7802_GCAL1_B1 = 0x08
    NAU7802_GCAL1_B0 = 0x09
    NAU7802_OCAL2_B2 = 0x0A
    NAU7802_OCAL2_B1 = 0x0B
    NAU7802_OCAL2_B0 = 0x0C
    NAU7802_GCAL2_B3 = 0x0D
    NAU7802_GCAL2_B2 = 0x0E
    NAU7802_GCAL2_B1 = 0x0F
    NAU7802_GCAL2_B0 = 0x10
    NAU7802_I2C_CONTROL = 0x11
    NAU7802_ADCO_B2 = 0x12
    NAU7802_ADCO_B1 = 0x13
    NAU7802_ADCO_B0 = 0x14
    NAU7802_ADC = 0x15
    NAU7802_OTP_B1 = 0x16
    NAU7802_OTP_B0 = 0x17
    NAU7802_PGA = 0x1B
    NAU7802_PGA_PWR = 0x1C
    NAU7802_DEVICE_REV = 0x1F

    # Bits within the PU_CTRL register
    NAU7802_PU_CTRL_RR = 0
    NAU7802_PU_CTRL_PUD = 1
    NAU7802_PU_CTRL_PUA = 2
    NAU7802_PU_CTRL_PUR = 3
    NAU7802_PU_CTRL_CS = 4
    NAU7802_PU_CTRL_CR = 5
    NAU7802_PU_CTRL_OSCS = 6
    NAU7802_PU_CTRL_AVDDS = 7

    # Bits within the CTRL1 register
    NAU7802_CTRL1_GAIN = 2
    NAU7802_CTRL1_VLDO = 5
    NAU7802_CTRL1_DRDY_SEL = 6
    NAU7802_CTRL1_CRP = 7

    # Bits within the CTRL2 register
    NAU7802_CTRL2_CALMOD = 0
    NAU7802_CTRL2_CALS = 2
    NAU7802_CTRL2_CAL_ERROR = 3
    NAU7802_CTRL2_CRS = 4
    NAU7802_CTRL2_CHS = 7

    # Bits within the PGA register
    NAU7802_PGA_CHP_DIS = 0
    NAU7802_PGA_INV = 3
    NAU7802_PGA_BYPASS_EN = 4
    NAU7802_PGA_OUT_EN = 5
    NAU7802_PGA_LDOMODE = 6
    NAU7802_PGA_RD_OTP_SEL = 7

    # Bits within the PGA PWR register
    NAU7802_PGA_PWR_PGA_CURR = 0
    NAU7802_PGA_PWR_ADC_CURR = 2
    NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4
    NAU7802_PGA_PWR_PGA_CAP_EN = 7

    # Allowed Low drop out regulator voltages
    NAU7802_LDO_2V4 = 0b111
    NAU7802_LDO_2V7 = 0b110
    NAU7802_LDO_3V0 = 0b101
    NAU7802_LDO_3V3 = 0b100
    NAU7802_LDO_3V6 = 0b011
    NAU7802_LDO_3V9 = 0b010
    NAU7802_LDO_4V2 = 0b001
    NAU7802_LDO_4V5 = 0b000

    # Allowed gains
    NAU7802_GAIN_128 = 0b111
    NAU7802_GAIN_64 = 0b110
    NAU7802_GAIN_32 = 0b101
    NAU7802_GAIN_16 = 0b100
    NAU7802_GAIN_8 = 0b011
    NAU7802_GAIN_4 = 0b010
    NAU7802_GAIN_2 = 0b001
    NAU7802_GAIN_1 = 0b000

    # Allowed samples per second
    NAU7802_SPS_320 = 0b111
    NAU7802_SPS_80 = 0b011
    NAU7802_SPS_40 = 0b010
    NAU7802_SPS_20 = 0b001
    NAU7802_SPS_10 = 0b000

    # Select between channel values
    NAU7802_CHANNEL_1 = 0
    NAU7802_CHANNEL_2 = 1

    # Calibration state
    NAU7802_CAL_SUCCESS = 0
    NAU7802_CAL_IN_PROGRESS = 1
    NAU7802_CAL_FAILURE = 2

    def __init__(self, address=None, i2c_driver=None):
        """
        Constructor

        :param address: The I2C address to use for the device
            If not provided, the default address is used
        :type address: int, optional
        :param i2c_driver: An existing i2c driver object
            If not provided, a driver object is created
        :type i2c_driver: I2CDriver, optional
        """

        # Use address if provided, otherwise pick the default
        self.address = self.available_addresses[0] if address is None else address

        # Load the I2C driver if one isn't provided
        if i2c_driver is None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c is None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver

        # Initialize other member variables
        self._zero_offset = 0
        self._calibration_factor = 1

    def is_connected(self):
        """
        Determines if this device is connected

        :return: `True` if connected, otherwise `False`
        :rtype: bool
        """
        # Check if connected by seeing if an ACK is received
        return qwiic_i2c.isDeviceConnected(self.address)

    connected = property(is_connected)

    def begin(self):
        """
        Initializes this device with default parameters

        :return: Returns `True` if successful, otherwise `False`
        :rtype: bool
        """
        # Confirm device is connected before doing anything
        if not self.is_connected():
            return False

        # Reset everything back to defaults
        self.reset()

        # Tell device to power up and confirm that worked
        result = self.power_up()
        if result == False:
            return False
        
        # Set default config values
        self.set_ldo(self.NAU7802_LDO_3V3)
        self.set_gain(self.NAU7802_GAIN_128)
        self.set_sample_rate(self.NAU7802_SPS_80)
        self.set_register(self.NAU7802_ADC, 0x30)
        self.set_bit(self.NAU7802_PGA_PWR_PGA_CAP_EN, self.NAU7802_PGA_PWR)
        
        # Tell device to calibrate with new config values
        return self.calibrate_afe()

    def available(self):
        """
        Gets whether data is available

        :return: `True` if data is available, otherwise `False`
        :rtype: bool
        """
        return self.get_bit(self.NAU7802_PU_CTRL_CR, self.NAU7802_PU_CTRL)

    def calibrate_afe(self):
        """
        Calibrate analog front end. Takes approximatly 344ms to calibrate, but
        wait up to 1000ms. It is recommended to recalibrate whenever the gain,
        sample rate, or channel number is changed

        :return: `True` if successful, otherwise `False`
        :rtype: bool
        """
        self.begin_calibrate_afe()
        return self.wait_for_calibrate_afe(1000)

    def begin_calibrate_afe(self):
        """
        Begin asycnhronous calibration of the analog front end. Poll for
        completion with calAFEStatus() or wait with waitForCalibrateAFE()
        """
        self.set_bit(self.NAU7802_CTRL2_CALS, self.NAU7802_CTRL2)

    def cal_afe_status(self):
        """
        Returns calibration status of analog front end

        :return: Calibration status. Can be NAU7802_CAL_IN_PROGRESS,
        NAU7802_CAL_SUCCESS, or NAU7802_CAL_FAILURE
        :rtype: int
        """
        if self.get_bit(self.NAU7802_CTRL2_CALS, self.NAU7802_CTRL2):
            return self.NAU7802_CAL_IN_PROGRESS

        if self.get_bit(self.NAU7802_CTRL2_CAL_ERROR, self.NAU7802_CTRL2):
            return self.NAU7802_CAL_FAILURE

        return self.NAU7802_CAL_SUCCESS

    def wait_for_calibrate_afe(self, timeout_ms=0):
        """
        Wait for asynchronous AFE calibration to complete with optional timeout.
        If timeout is not specified (or set to 0), then wait indefinitely.

        :param timeout_ms: Timeout in ms, defaults to 0
        :type timeout_ms: int, optional
        :return: `True` if successful, otherwise `False`
        :rtype: bool
        """
        begin = self.millis()
        cal_ready = self.cal_afe_status()

        while cal_ready == self.NAU7802_CAL_IN_PROGRESS:
            if timeout_ms > 0 and self.millis() - begin > timeout_ms:
                break
            time.sleep(0.001)
            cal_ready = self.cal_afe_status()

        if cal_ready == self.NAU7802_CAL_SUCCESS:
            return True
        return False

    def set_sample_rate(self, rate):
        """
        Set sample rate in Hz.

        :param rate: Sample rate in Hz. Can be NAU7802_SPS_10, NAU7802_SPS_20,
        NAU7802_SPS_40, NAU7802_SPS_80, or NAU7802_SPS_320
        :type rate: int
        """
        if rate > 0b111:
            rate = 0b111

        value = self.get_register(self.NAU7802_CTRL2)
        value &= 0b10001111
        value |= rate << 4

        self.set_register(self.NAU7802_CTRL2, value)

    def set_channel(self, channel_number):
        """
        Select between channel 1 and 2

        :param channel_number: Channel number. Can be NAU7802_CHANNEL_1 or
        NAU7802_CHANNEL_2
        :type channel_number: int
        """
        if channel_number == self.NAU7802_CHANNEL_1:
            self.clear_bit(self.NAU7802_CTRL2_CHS, self.NAU7802_CTRL2)
        else:
            self.set_bit(self.NAU7802_CTRL2_CHS, self.NAU7802_CTRL2)

    def power_up(self):
        """
        Power up digital and analog sections of scale

        :return: `True` if successful, otherwise `False`
        :rtype: bool
        """
        self.set_bit(self.NAU7802_PU_CTRL_PUD, self.NAU7802_PU_CTRL)
        self.set_bit(self.NAU7802_PU_CTRL_PUA, self.NAU7802_PU_CTRL)

        counter = 0
        while True:
            if self.get_bit(self.NAU7802_PU_CTRL_PUR, self.NAU7802_PU_CTRL):
                break
            time.sleep(0.001)
            counter += 1
            if counter > 100:
                return False

        return True

    def power_down(self):
        """
        Puts scale into low-power mode
        """
        self.clear_bit(self.NAU7802_PU_CTRL_PUD, self.NAU7802_PU_CTRL)
        self.clear_bit(self.NAU7802_PU_CTRL_PUA, self.NAU7802_PU_CTRL)

    def reset(self):
        """
        Resets all registers to power off defaults
        """
        self.set_bit(self.NAU7802_PU_CTRL_RR, self.NAU7802_PU_CTRL)
        time.sleep(0.001)
        self.clear_bit(self.NAU7802_PU_CTRL_RR, self.NAU7802_PU_CTRL)

    def set_ldo(self, ldo_value):
        """
        Sets the onboard low-drop-out voltage regulator to a given value

        :param ldo_value: LDO voltage. Can be NAU7802_LDO_2V4, NAU7802_LDO_2V7,
        NAU7802_LDO_3V0, NAU7802_LDO_3V3, NAU7802_LDO_3V6, NAU7802_LDO_3V9,
        NAU7802_LDO_4V2, or NAU7802_LDO_4V5
        :type ldo_value: int
        """
        if ldo_value > 0b111:
            ldo_value = 0b111

        value = self.get_register(self.NAU7802_CTRL1)
        value &= 0b11000111
        value |= ldo_value << 3
        self.set_register(self.NAU7802_CTRL1, value)

        self.set_bit(self.NAU7802_PU_CTRL_AVDDS, self.NAU7802_PU_CTRL)

    def set_gain(self, gain_value):
        """
        Sets the gain

        :param gain_value: Gain. Can be NAU7802_GAIN_1, NAU7802_GAIN_2,
        NAU7802_GAIN_4, NAU7802_GAIN_8, NAU7802_GAIN_16, NAU7802_GAIN_32,
        NAU7802_GAIN_64, or NAU7802_GAIN_128
        :type gain_value: int
        """
        if gain_value > 0b111:
            gain_value = 0b111

        value = self.get_register(self.NAU7802_CTRL1)
        value &= 0b11111000
        value |= gain_value

        self.set_register(self.NAU7802_CTRL1, value)

    def get_revision_code(self):
        """
        Gets the revision code of this IC. Should be 0x0F

        :return: Revision code
        :rtype: int
        """
        revision_code = self.get_register(self.NAU7802_DEVICE_REV)
        return revision_code & 0x0F

    def get_reading(self):
        """
        Returns 24-bit reading. Assumes available() has been checked and
        returned `True`

        :return: Raw 24-bit reading
        :rtype: int
        """
        raw_data = self._i2c.readBlock(self.address, self.NAU7802_ADCO_B2, 3)
        value = (raw_data[0] << 16) | (raw_data[1] << 8) | (raw_data[2])

        if value >= (1 << 23):
            value -= (1 << 24)
        
        return value

    def get_average(self, average_amount, timeout_ms=1000):
        """
        Return the average of a given number of readings. Gives up after timeout

        :param average_amount: Number of measurements to average
        :type average_amount: int
        :param timeout_ms: Timeout in milliseconds, defaults to 1000
        :type timeout_ms: int, optional
        :return: Average measurement value
        :rtype: float
        """
        total = 0
        samples_acquired = 0
        start_time = self.millis()

        while True:
            if self.available():
                total += self.get_reading()
                samples_acquired += 1
                if samples_acquired == average_amount:
                    break

            if self.millis() - start_time > timeout_ms:
                return 0
            time.sleep(0.001)

        total /= average_amount
        return total

    def calculate_zero_offset(self, average_amount):
        """
        Calculates and stores zero offset measurement. Must only be called when
        zero weight is on the scale

        :param average_amount: Number of measurements to average
        :type average_amount: int
        """
        self.set_zero_offset(self.get_average(average_amount))

    def set_zero_offset(self, new_zero_offset):
        """
        Sets zero offset mesaurement. Useful if the zero offset has already been
        calculated

        :param new_zero_offset: Zero offset
        :type new_zero_offset: float
        """
        self._zero_offset = new_zero_offset

    def get_zero_offset(self):
        """
        Gets the current zero offset

        :return: Current zero offset
        :rtype: float
        """
        return self._zero_offset

    def calculate_calibration_factor(self, weight_on_scale, average_amount):
        """
        Calculates and stores calibration factor. Must only be called after zero
        offset has been set, and when a known weight is on the scale.

        :param weight_on_scale: Weight currently on the scale, without units
        :type weight_on_scale: float
        :param average_amount: Number of measurements to average
        :type average_amount: int
        """
        on_scale = self.get_average(average_amount)
        new_cal_factor = (on_scale - self._zero_offset) / weight_on_scale
        self.set_calibration_factor(new_cal_factor)

    def set_calibration_factor(self, new_cal_factor):
        """
        Sets calibration factor. Useful if the calibration factor has already
        been calculated

        :param new_cal_factor: Calibration factor
        :type new_cal_factor: float
        """
        self._calibration_factor = new_cal_factor

    def get_calibration_factor(self):
        """
        Gets the current calibration factor

        :return: Current calibration factor
        :rtype: float
        """
        return self._calibration_factor

    def get_weight(self, allow_negative_weights = False, samples_to_take = 8):
        """
        Gets the weight on the scale using the zero offset and calibration
        factor, so those must be set before using this. Uses a simple linear
        calculation following y=mx+b

        :param allow_negative_weights: Whether negative weights are allowed,
        defaults to False
        :type allow_negative_weights: bool, optional
        :param samples_to_take: Number of measurements to average, defaults to 8
        :type samples_to_take: int, optional
        :return: Weight on the scale
        :rtype: float
        """
        on_scale = self.get_average(samples_to_take)

        if not allow_negative_weights and on_scale < self._zero_offset:
            on_scale = self._zero_offset

        weight = (on_scale - self._zero_offset) / self._calibration_factor
        return weight

    def set_int_polarity_high(self):
        """
        Set interrupt pin to be high when data is ready (default)
        """
        self.clear_bit(self.NAU7802_CTRL1_CRP, self.NAU7802_CTRL1)

    def set_int_polarity_low(self):
        """
        Set interrupt pin to be low when data is ready
        """
        self.set_bit(self.NAU7802_CTRL1_CRP, self.NAU7802_CTRL1)

    def set_bit(self, bit_number, register_address):
        """
        Mask & set a given bit within a register

        :param bit_number: Bit index to set
        :type bit_number: int
        :param register_address: Register address
        :type register_address: int
        """
        value = self.get_register(register_address)
        value |= (1 << bit_number)
        self.set_register(register_address, value)

    def clear_bit(self, bit_number, register_address):
        """
        Mask & clear a given bit within a register

        :param bit_number: Bit index to clear
        :type bit_number: int
        :param register_address: Register address
        :type register_address: int
        """
        value = self.get_register(register_address)
        value &= ~(1 << bit_number)
        self.set_register(register_address, value)

    def get_bit(self, bit_number, register_address):
        """
        Return a given bit within a register

        :param bit_number: Bit index to get
        :type bit_number: int
        :param register_address: Register address
        :type register_address: int
        :return: Bit value
        :rtype: bool
        """
        value = self.get_register(register_address)
        return bool(value & (1 << bit_number))

    def get_register(self, register_address):
        """
        Get contents of a register

        :param register_address: Register address
        :type register_address: int
        :return: Register value
        :rtype: int
        """
        return self._i2c.readByte(self.address, register_address)

    def set_register(self, register_address, value):
        """
        Send a given value to be written to given address

        :param register_address: Register address
        :type register_address: int
        :param value: Register value
        :type value: int
        """
        self._i2c.writeByte(self.address, register_address, value)
    
    def millis(self):
        """
        Get the current time in milliseconds

        :return: Current time in milliseconds
        :rtype: int
        """
        if hasattr(time, "ticks_ms"):
	        # MicroPython: time.time() gives an integer, instead use ticks_ms()
            return time.ticks_ms()
        else:
	        # Other platforms: time.time() gives a float
            return int(round(time.time() * 1000))
