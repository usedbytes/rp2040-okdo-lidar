#! /bin/bash

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -s tcl -c "program ${1} verify reset exit"
