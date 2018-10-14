# Pinky (8-bit CPU) + Assembler

Pinky is a simple 8-bit register-based CPU written in Verilog. 

It comes with an Assembler written in Python 3 that outputs Verilog code (i.e., initialize an array called `rom` with binary data).

## Simulate

```
iverilog -Wall -g2005 -I . -s top_tb *.v
vvp -la.lst -n a.out -vcd
```
## Using The Assembler

```
$ python3 asm2rom_v.py -i led_chaser.asm 
rom[0] = 8'b01000000; // lda $0
rom[1] = 8'b01010001; // ldb $1
rom[2] = 8'b11110100; // out $1, $0
// _sleep_led_off:
rom[3] = 8'b01101100; // add a, b
rom[4] = 8'b11000111; // cmp a, $7
rom[5] = 8'b10001110; // jne _sleep_led_off
rom[6] = 8'b11110101; // out $1, $1
// _sleep_led_on:
rom[7] = 8'b01101101; // sub a, b
rom[8] = 8'b11000000; // cmp a, $0
rom[9] = 8'b10001110; // jne _sleep_led_on
rom[10] = 8'b10011000; // jmp $0
```

## Tested

I've successfully synthesized it on Arty A7 using Xilinx Vivado 2018.2

## Bugs?

Yeah, plenty. I'm still working on it :-)
