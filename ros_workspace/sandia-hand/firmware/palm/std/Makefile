# input files
C_SOURCES := bootloader tactile io comms startup main pins imu 
#ASM_SOURCES := bootloader_startup
# random definitions
RS485_ADDRESS ?= 14
BOARD = PALM_REV1
OPTIMIZATION = -O2
OUTPUT = palm
#CFLAGS += -DBL_MANCHESTER
CFLAGS += -DBL_LED_PIO=PIOC -DBL_LED_PIN=19 -D$(BOARD)
CFLAGS += -DBL_RS485_DE_PIO=PIOC -DBL_RS485_DE_PIN=9
include ../libcommon-sam3s/common-sam3s.mk
include ../libcommon-sam3s/common-elf.mk
PALM_CLI=`rospack find sandia_hand`/bin/palm_cli /dev/ttyUSB0
OCD = openocd $(OCD_INIT)
fresh_board:
	$(OCD) -c "at91sam3 gpnvm set 1" -c shutdown
	$(OCD) -c "at91sam3 gpnvm show all" -c shutdown
	sleep 0.5
	$(OCD) -c "flash banks" -c "flash write_image $(OUTPUT).elf" -c reset -c shutdown
	#$(FINGER_CLI) dp_test

test:
	$(PALM_CLI) test
