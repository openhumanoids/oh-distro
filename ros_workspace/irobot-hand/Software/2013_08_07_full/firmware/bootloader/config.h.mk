
config.h: config.mk
	@echo "Generating config.h for $(MCU)"
	@echo "// XBoot config header file" > config.h
	@echo "// MCU: $(MCU)" >> config.h
	@echo "// F_CPU: $(F_CPU)" >> config.h
	@echo >> config.h
	$(foreach v, \
	$(sort $(filter-out USE_CONFIG_H, $(filter \
				ATTACH_% \
				ENABLE_% \
				ENTER_% \
				FIFO_% \
				I2C_% \
				LED_% \
				UART_% \
				USE_% \
				WATCHDOG_%, $(.VARIABLES)))), \
	echo "#define $(v) $($(v))" >> config.h;) 
