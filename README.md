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

ProtoTime is a networked power-over-ethernet GPS stratum 1 clock[^1]. ProtoTime uses both a RTC module and a GPS module to have a fallback for when the GPS signal is lost.

ProtoTime is the physical manifestation of one of the first things I ever wanted to build. Ever since my first days involved in technical theatre, one of my team consistently remarked how a real time clock server would suit our lighting network nicely, but we all knew we couldn't afford any of the time servers available on the market. Its been over 3 years since I worked at that place, but the idea never left my mind. It always seemed like a simple project, just a microcontroller and a GPS module, never seemed like it was too hard or outside of my abilities. (Spoiler alert, it really wasn't hard at all!)

## Building your own

### Required parts

- [T-Ethernet-POE](https://www.lilygo.cc/products/t-internet-poe) from LILYGO.
  - Make sure to get the one with a programmer as well, even if you have one. Their pinout is different from a typical programmer and has weird pin spacing, so its recommended to get a programmer. Do note that this module is *not* electrically isolated, so you should have it connected to both POE power and usb power simultaneously.
- GPS Module. Most any will do fine for this use case, I used a NEO-6M module.
- DS3231 RTC module.

ProtoTime would likely work with different microcontrollers options and ethernet drivers and chipsets, but may require some modifications. Future improvements are planned to decouple the Ethernet related code from the rest of the project.

There is not a wiring guide as of yet, but the pins are defined in the source file. Both of the GPS and RTC modules require 3.3V and GND, and we have exactly two pins of each on the T-Ethernet-POE! As for the rest of the wires, all pins should be connected to the controller, aside from the 32K clock from the RTC module.

## Credits

The first few commits and working models of this project were based  <https://lloydm.net/Demos/GPS-NTP.html>, which itself was based on <https://forum.arduino.cc/index.php?topic=197870.0>.

I've made several modifications in order to support both RTC and GPS pulses, along with adding millisecond support, among other bugfixes and quality of life improvements.

## See Also

- NTP v3 and v4 RFCS: <https://www.rfc-editor.org/rfc/rfc1305> and <https://www.rfc-editor.org/rfc/rfc5905>
- NMEA 0183 standard: <https://www.nmea.org/nmea-0183.html>
- DS3231 datasheet: <https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf>

[^1]: Accuracy is not yet to the level of a stratum 1 clock, so this project is still a work in progress while the bugs get ironed out. But for a network with no time clock, this is accurate enough.
