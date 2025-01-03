# Dodgey Installation

1. Copy `mqtt-gateway/pico_ook_serial_to_mqtt.py` to `/root/serial_to_mqtt.py`
2. Copy the files from `systemd/` to `/etc/systemd/system`
3. Setup: run `systemctl enable --now pico433_serial_mqtt.service`
