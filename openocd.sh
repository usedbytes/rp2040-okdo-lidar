#! /bin/bash

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -s tcl
