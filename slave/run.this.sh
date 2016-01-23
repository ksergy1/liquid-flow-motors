#!/bin/bash

clear
avra -m map.slave.map -l list.master.list -o out.master.hex ./slave-mcu.src.asm

