#!/bin/bash

tup $1 && st-flash write $1/osuar_control.bin 0x08000000

