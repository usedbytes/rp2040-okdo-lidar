import time
import usb.core
import usb.util

dev = usb.core.find(idVendor=0x1209, idProduct=0x0001)
if dev is None:
    raise ValueError('device not found')

#if dev.is_kernel_driver_active(0):
#    dev.detach_kernel_driver(0)

dev.set_configuration()
cfg = dev.get_active_configuration()

intf = usb.util.find_descriptor(
        cfg,
        bInterfaceClass=0xff
        )

ep_in = usb.util.find_descriptor(intf,
    # match the first IN endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)

assert (ep_in is not None)

try:
    while True:
        print(ep_in.read(57))
finally:
    usb.util.dispose_resources(dev)
