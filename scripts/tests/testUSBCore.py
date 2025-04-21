import usb.core
import usb.util

VENDOR_ID = 0x16c0
PRODUCT_ID = 0x27dd

# Find the USB device
dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
if dev is None:
    raise ValueError("Device not found")

# Set the active configuration (usually configuration 1)
dev.set_configuration()

# Get the active configuration and interface (assume interface 0)
cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]

# Find the first IN endpoint (customize the match if needed)
ep_in = usb.util.find_descriptor(
    intf,
    custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
)
if ep_in is None:
    raise ValueError("IN endpoint not found")

print("Device configured. Beginning data read...")

# Loop to continuously read from the endpoint.
# Note: adjust the size and timeout parameters as needed.
try:
    while True:
        try:
            data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize, timeout=100)
            # Process your data here – data will be a list/array of byte values
            print("Received:", data)
        except usb.core.USBError as e:
            # A timeout error is expected if no data is available in the window.
            if e.errno == 110:  # timeout
                continue
            else:
                raise e
except KeyboardInterrupt:
    print("Exiting read loop.")

# Release the device (good practice)
usb.util.release_interface(dev, intf)
