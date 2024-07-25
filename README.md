# pico-ips2550

Micropython library to interface Raspberry Pi Pico with IPS2550 through I2C.

Supports reading and writing registers through the programming interface and has some quality of life methods to R/W the most important settings.

It may also be used to measure the analog outputs (after setting it to single ended output mode) by connecting the rx1, rx2 and ref pins to the ADC channels of the Pico.
