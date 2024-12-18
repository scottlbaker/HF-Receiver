
## Summary

A minimalist HF receiver


## Description

This is a direct conversion receiver that uses a quadrature sampling detector (AKA Tayloe detector) to mix the RF down to audio baseband. Demodulation and DSP operations are done by an Atmega328 processor and software development uses the Arduino IDE. The receiver is implemented on a single circuit board with dimensions of 75mm x 55mm (2.9" x 2.1").


## Arduino IDE support

This project uses the Atmega328PB processor. You may need to upgrade your Arduino IDE to add support for this processor. There are probably a few ways to do this. The way I did it was to install MiniCore (https://github.com/MCUdude/MiniCore).


## Contributors

* Scott L Baker

* If you like my designs and would like to support my work: https://buymeacoffee.com/scottlbaker


## Acknowledgement

The hardware for this project is based on the uSDX architecture.
The uSDX is a software defined designed by PE1NNZ and DL2MAN.
https://github.com/threeme3/usdx

The firmware for this project is based on code by PE1NNZ
https://github.com/threeme3/usdx-sketch
and is used by permission from the author

## License

See the **LICENSE** file in this repository


