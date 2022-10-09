all:;
	$(MAKE) -C build -j

# The flash.* targets allow flashing using the Pico SWD instead of having to press the button and re-plug the USB

relay_serial: build/app/relay_serial.elf

flash/relay_serial: build/app/relay_serial.elf
	openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program $(<) verify exit"
	$(MAKE) reset

build/app: FORCE
	$(MAKE) -C build/app -j

# Do a proper multicore reset workaround
# https://forums.raspberrypi.com/viewtopic.php?p=2025479&hilit=multicore+printf#p2025479
reset:;
#	openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "init; reset; exit"
	openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg \
		-c "init ; reset halt ; rp2040.core1 arp_reset assert 0 ; rp2040.core0 arp_reset assert 0 ; exit"
 
wait_debugger:;
	openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg

debugger:;
	gdb-multiarch -ix gdbinit build/app/relay_serial.elf
