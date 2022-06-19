#!/bin/bash

make
st-flash write ./build/UART_7SEGMENT_RS485.bin 0x08000000
st-flash reset