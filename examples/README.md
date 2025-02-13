# Sparkfun NAU7802 Examples Reference
Below is a brief summary of each of the example programs included in this repository. To report a bug in any of these examples or to request a new feature or example [submit an issue in our GitHub issues.](https://github.com/sparkfun/qwiic_nau7802_py/issues). 

NOTE: Any numbering of examples is to retain consistency with the Arduino library from which this was ported. 

## Qwiic Nau7802 Ex1 Basic Readings
Demonstrates how to get basic data from the Qwiic Scale.

The key methods showcased by this example are:
- [get_reading()]()
- [available()]()

## Qwiic Nau7802 Ex2 Complete Scale
Demonstrates how to calibrate the Qwiic Scale to measure weight

The key methods showcased by this example are:
- [calculate_zero_offset]()
- [calculate_calibration_factor]()
- [set_zero_offset()]()
- [set_calibration_factor()]()
- [get_weight()]()

## Qwiic Nau7802 Ex3 Gain Sample Rate
Demonstrates how to set the gain and sample rate of the Qwiic Scale.

The key methods showcased by this example are:
- [set_gain()]()
- [set_sample_rate()]()
- [calibrate_afe()]()

## Qwiic Nau7802 Ex4 Low Power
Demonstrates how to set the Qwiic Scale into low power mode.

The key methods showcased by this example are:
- [power_down]()
- [power_up]()
- [millis]()


