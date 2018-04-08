Sensor Repeater
===============

This is an Energia sketch for a wireless repeater station to improve signal quality on a remote sensor.

This sketch uses a TI CC110L BoosterPack to listen for packets from an [outdoor weather sensor][1]. Since the sensor is outdoors and the [receiver hub][2] is indoors, the signal is often weak or garbled when it is received by the hub. So I created this repeater station and installed it about halfway between the weather sensor and the receiver hub.

The weather sensor transmits on a channel that is monitored by the repeater hub, and the repeater station then re-transmits the packet on the channel that the receiver hub is listening.

Since the repeater station is mounted in my garage, I also added some  monitoring of its own which it transmits separately:
- Garage temperature (using the MSP430 internal temp sensor)
- Garage door open/closed states (using a SparkFun [ZX Sensor][3])
- Vcc (battery level)

The repeater station also supports an external OLED display which is used to display various sensor readings from both the outdoor weather sensor and the garage sensors on the repeater station.

In my implementation, the sketch is installed on an MSP430FR4133 LaunchPad.  On the FR4133, SPI and I2C share the same pins. Since the CC110L uses SPI communication and the ZX Sensor uses I2C, there is a pin conflict if both sensors try to use the built-in hardware communication.  The sketch therefore uses my [software I2C library][6] to communicate with the ZX Sensor.

External Libraries
------------------
- [MSPTandV][4]
- [NewhavenOLED][5]
- [SWI2C][6]



References
----------
+ [Outdoor Weather Sensor][1]
+ [Sensor Receiver Hub][2]
+ SparkFun [ZX Sensor][3]
  + Note that a [newer version][7] is available
+ [MSPTandV][4] library
+ [NewhavenOLED][5] library
+ [SWI2C][6] library
+ CC110L [BoosterPack][8]

[1]: https://gitlab.com/Andy4495/Outdoor-Weather-Sensor
[2]: https://gitlab.com/Andy4495/Sensor-Receiver
[3]: https://www.sparkfun.com/products/retired/12780
[4]: https://gitlab.com/Andy4495/mspTandV
[5]: https://gitlab.com/Andy4495/NewhavenOLED
[6]: https://gitlab.com/Andy4495/NewhavenOLED
[7]: https://www.sparkfun.com/products/13162
[8]: http://www.ti.com/tool/430BOOST-CC110L
