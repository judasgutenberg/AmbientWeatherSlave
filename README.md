This sketch is designed to run on an ATTiny Atmega and communicate to a master over I2C.  It handles the details of communication over 477 MHz via a commodity wireless module to get data from AmbientWeather weather probes. It communicates by sending temperatures in four byte packets where the first byte is a one byte representation of the age of the data, which, depending on how large it is, indicates several possible lengths of time with less and less granularity.
Format of the data packet: [number_of_sensor]: [time since last data update from sensor] [one byte representation of temperature value in decidegrees with decimal] [humidity]%...

Most of the hard work in this was done by Ron C Lewis, see:
https://eclecticmusingsofachaoticmind.wordpress.com/2015/01/21/home-automation-temperature-sensors/
though I can't find his original code anywhere.
