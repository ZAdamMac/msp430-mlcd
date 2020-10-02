# Display POC for MSP430 and Sharp Memory LCD
This codebase is built to act as a simplified proof of concept for printing alphanumeric characters to a SHARP Memory LCD. Exact hardware specs can be found by consulting the documentation for the following devices:
- MSP430-EXPFR5994 (Texas Instruments' development board for the MSP430FR5994)
- BOOSTXL-SHARP128 (TI module for the above board).

A simplified pinout is provided in the header comment for `main.c`.

This code was ported, crudely, from older code. Energia has libraries for interfacing with this module that may be more user-friendly. Additionally, this code relies heavily on the MSP430 driver library.
