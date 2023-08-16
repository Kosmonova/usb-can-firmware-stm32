# usb-can-firmware-stm32
Firmware for stm32 used for USB-CAN transceiver. This program have some
limitation from original firmware, like it don't works with external eeprom,
that is on PCB. I'm not sure, how exactly is used this eeprom in original
program, maybe for storing CAN data in ofline mode?

## necessary dependencies
	STM32CubeMX - for compiling of project
	Demonstrator GUI - for flashing of stm32 trough serial port

## downloading and compiling of project
```
git clone https://github.com/Kosmonova/usb-can-firmware-stm32
cd usb-can-firmware-stm32
make
```

## flashing of binary file into stm32 using ST-LINK programmer
```
st-flash write ./.pio/build/bluepill_f103c6/firmware.bin  0x08000000
```

## flashing of binary file into stm32 without programmer
Stm32 have build in bootloader for flashing through uart interface. The same
uart interface is used also for transceivering of CAN bus data trought USB. This
good behavior enables flashing of stm32 without any programmer.

For setting st32 into boot mode is needed these steps:
1) turn off USB cable from PC
2) connect pads together on this picture bellow and hold them

![TOP_SITE_DPS_USB_CAN](./TOP_SITE_DPS_USB_CAN.jpg)

3) turn on USB cable into PC

Now is stm32 in boot mode and is waiting for flassing. For Flashing of stm
trough serial port is used program "Demonstrator GUI". This program is possible
free download from internet. Breafly description how is possible flashing of
stm is somewere on web sites.
