all:;
	$(MAKE) -C build -j

flash/relay_serial: build/app/relay_serial.elf
	openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program $(<) verify reset exit"

build/app/relay_serial.elf:
	$(MAKE) -C build/app/relay_serial.elf -j
