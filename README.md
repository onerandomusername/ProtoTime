# ProtoTime

- [ProtoTime](#prototime)
  - [About](#about)
  - [Building your own](#building-your-own)
    - [Required parts](#required-parts)
  - [Credits](#credits)
  - [See Also](#see-also)

> [!WARNING]
> This project is still in alpha and is not yet recommended for production use.

## About

A networked power-over-ethernet GPS stratum 1 clock[^1]. Uses both a RTC module and a GPS module to have a fallback in the event that the GPS signal is lost.

Ever since my first days of theatre, one of the people I work with, had, on multiple occassions, remarked how a real time clock server would suit our network nicely, but we could not afford an expensive professionally built module at that point in time. I had the idea from then on, that some day, I wanted to build my own time server, as it didn't seem like it could be too hard. (Spoiler alert, it really wasn't.)

## Building your own

### Required parts


- [T-Ethernet-POE](https://www.lilygo.cc/products/t-internet-poe) from LILYGO.
  - Make sure to get the one with a programmer as well, even if you have one. Their pinout is different from a typical programmer and has weird pin spacing, so its recommended to get a programmer. Do note that this module is *not* electrically isolated, so you should have it connected to both POE power and usb power simultaneously.
- GPS Module. Most any will do fine for this use case, I used a NEO-6M module.
- DS3231 RTC module.

ProtoTime would likely work with different microcontrollers options and ethernet drivers and chipsets, but may require some modifications. Future improvements are planned to decouple the Ethernet related code from the rest of the project.

There is not a wiring guide as of yet, but the pins are defined in the source file. Both of the GPS and RTC modules require 3.3V and GND, and we have exactly two pins of each on the T-Ethernet-POE! As for the rest of the wires, all pins should be connected to the controller, aside from the 32K clock from the RTC module.

While not many choices are afforded, there is one option regarding the RTC. You *can* opt to run it without a battery, but behaviour when there are no known-good time sources is currently unknown. This should be fixed in a future version.

## Credits

The first few commits and working models of this project were based  <https://lloydm.net/Demos/GPS-NTP.html>, which itself was based on <https://forum.arduino.cc/index.php?topic=197870.0>.

I've made several modifications in order to support both RTC and GPS pulses, along with adding millisecond support, among other bugfixes and quality of life improvements.

## See Also

- NTP v3 and v4 RFCS: <https://www.rfc-editor.org/rfc/rfc1305> and <https://www.rfc-editor.org/rfc/rfc5905>
- NMEA 0183 standard: <https://www.nmea.org/nmea-0183.html>
- DS3231 datasheet: <https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf>

[^1]: Accuracy is not yet to the level of a stratum 1 clock, so this project is still a work in progress while the bugs get ironed out. But for a network with no time clock, this is accurate enough.
