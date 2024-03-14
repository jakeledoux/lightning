poetry run python con_stream.py server & \
sleep 1 && poetry run python con_stream.py client 127.0.0.1 8080 -serial=/dev/cu.usbmodem11121301 \
&& fg
