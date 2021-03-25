# MTB-UNI v2 AVR Firmware

This repository contains firmware for ATmega328p MCU for upgrading old
[MTB-UNI v2 module](https://mtb.kmz-brno.cz/uni). Old AT89C2051 processor
from UNI-UNI board is removed & replaced with [new board with ATmega328p
processor](https://github.com/kmzbrnoI/mtb-2-avr-pcb).

This allows old MTB-UNI board to work on new MTBbus with new features.

## Build & requirements

This firmware is developed in C language, compiled via `avr-gcc` with help
of `make`. You may also find tools like `avrdude` helpful.

Hex files are available in *Releases* section.

## Programming

Firmware could be programmed directly to new board (connect pinheaders to
pins & solder/hold reset wire on pad) via ISP.

Firmware could also be upgraded directly via MTBbus via bootloader.

This FW uses EEPROM, however no programming of EEPROM is required. There should
be just an empty EEPROM on fresh devices.

Flash main application, fuses & bootloader:

```bash
$ cd bootloader
$ make
$ cd ..
$ make
$ make fuses
$ make program
```

Flash fuses & bootloader only:

```bash
$ make fuses
$ cd bootloader
$ make
$ make program
```

## Author's toolkit

Text editor + `make`. No more, no less.

## See also

* [MTB-UNI v2 upgrade board schematics & pcb](https://github.com/kmzbrnoI/mtb-2-avr-pcb)
* [MTBbus protocol](https://github.com/kmzbrnoI/mtbbus-protocol)

## License

This application is released under the [Apache License v2.0
](https://www.apache.org/licenses/LICENSE-2.0).
