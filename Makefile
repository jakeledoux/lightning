USER := jake
HOSTNAME := lightning

rpi:
	cross build \
		--release \
		--target aarch64-unknown-linux-gnu \
		--target-dir ./target

deploy: rpi
	scp \
		./target/aarch64-unknown-linux-gnu/release/lightning \
		$(USER)@$(HOSTNAME):/home/$(USER)


format: format-cpp

format-cpp:
	clang-format \
		-i \
		--style Microsoft \
		carduino/carduino.ino
		
