#! /bin/bash

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -s tcl -c "adapter_khz 1000" -c "program ${1} verify reset exit"
