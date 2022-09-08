# Sensor Repeater

[![Arduino Compile Sketches](https://github.com/Andy4495/Sensor-Repeater/actions/workflows/arduino-compile-sketches.yml/badge.svg)](https://github.com/Andy4495/Sensor-Repeater/actions/workflows/arduino-compile-sketches.yml)
[![Check Markdown Links](https://github.com/Andy4495/Sensor-Repeater/actions/workflows/CheckMarkdownLinks.yml/badge.svg)](https://github.com/Andy4495/Sensor-Repeater/actions/workflows/CheckMarkdownLinks.yml)

This sketch is a wireless repeater station running on an MSP430 controller to improve reception from a remote wireless sensor.

This sketch uses a TI CC110L BoosterPack to listen for packets from an [outdoor weather sensor][1]. Since the sensor is outdoors and the [receiver hub][2] is indoors, the signal from the weather sensor is often weak or garbled when it is received by the hub. So I created this repeater station and installed it about halfway between the weather sensor and the receiver hub.

The weather sensor transmits on a channel that is monitored by the repeater hub, and the repeater station then re-transmits the packet on the channel that the receiver hub is listening.

Since the repeater station is mounted in my garage, I also added some  monitoring of its own which it transmits separately:

- Garage temperature (using the MSP430 internal temp sensor)
- Garage door open/closed states (using a SparkFun [ZX Sensor][3])
- Vcc (battery level)

The repeater station supports an external OLED display using my [NewhavenOLED library][5] which is used to display the number of valid (no CRC errors) and invalid (CRC error) messages received, along with the last RSSI and LQI values and time since last message received. This information can be useful in determining if the outdoor weather sensor is placed in a location where its transmissions can be received by the repeater.

The repeater station also supports an external 2-digit seven segment LED to display the last outdoor temperature reading received from the outdoor weather sensor. It uses my [LED744511 library][9].

In my implementation, the sketch is installed on an MSP430FR4133 LaunchPad.  On the FR4133, SPI and I2C share the same pins. Since the CC110L uses SPI communication and the ZX Sensor uses I2C, there is a pin conflict if both sensors try to use the built-in hardware communication.  The sketch therefore uses my [software I2C library][6] to communicate with the ZX Sensor.

## External Libraries

- [MspTandV][4]
- [NewhavenOLED][5]
- [LED744511][9]
- [SWI2C][6]

## References

- [Outdoor Weather Sensor][1]
- [Sensor Receiver Hub][2]
- SparkFun [ZX Sensor][3]
  - Note that a [newer version][7] is available
- CC110L [BoosterPack][8]

## License

The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE`][101] in this repository.

[1]: https://github.com/Andy4495/Outdoor-Weather-Sensor
[2]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub
[3]: https://www.sparkfun.com/products/retired/12780
[4]: https://github.com/Andy4495/MspTandV
[5]: https://github.com/Andy4495/NewhavenOLED
[6]: https://github.com/Andy4495/SWI2C
[7]: https://www.sparkfun.com/products/13162
[8]: https://www.ti.com/lit/ml/swru312b/swru312b.pdf
[9]: https://github.com/Andy4495/LED744511
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE
[//]: # ([200]: https://github.com/Andy4495/Sensor-Repeater)

[//]: # (Old TI product link that is no longer active: http://www.ti.com/tool/430BOOST-CC110L)
