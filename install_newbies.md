Install the required dependencies for openocd (usb drivers for Olimex):
`sudo apt update`
`sudo apt install build-essential libtool pkg-config libusb-1.0-0-dev libhidapi-dev`

If building from git, run the bootstrap:
`./bootstrap`

Now time to configure before building, the following line will configure support for ART, CMSIS-DAP and Olimex:
`./configure --enable-art --enable-cmsis-dap --enable-ftdi`

Finally, build openocd (-j will set # of threads):
`build -j 4`
