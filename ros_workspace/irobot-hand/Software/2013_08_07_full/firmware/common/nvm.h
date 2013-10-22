/**
 * \file
 *
 * \brief Non Volatile Memory controller driver
 *
 * Copyright (c) 2010-2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#ifndef NVM_H
#define NVM_H

//#include <compiler.h>
#include "ccp.h"
#include <string.h>

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \defgroup nvm_group NVM driver
 *
 * See \ref xmega_nvm_quickstart
 *
 * \brief Low-level driver implementation for the AVR XMEGA Non Volatile
 *        Memory Controller (NVM).
 *
 * The XMEGA NVM controller interfaces the internal non-volatile memories
 * in the XMEGA devices. Program memory, EEPROM and signature row is can be
 * interfaced by the module. See the documentation of each sub-module for
 * more information.
 *
 * \note If using GCC and the flash sub-module, remember to configure
 *       the boot section in the make file. More information in the sub-module
 *       documentation.
 *
 * \section xmega_nvm_quickstart_section Quick Start Guide
 * See \ref xmega_nvm_quickstart
 */

/**
 * \defgroup nvm_generic_group NVM driver generic module handling
 * \ingroup nvm_group
 * \brief Support functions for the NVM driver.
 *
 * These functions are helper functions for the functions of the
 * \ref nvm_group "NVM driver".
 *
 * @{
 */

/**
 * \brief Wait for any NVM access to finish.
 *
 * This function is blocking and waits for any NVM access to finish.
 * Use this function before any NVM accesses, if you are not certain that
 * any previous operations are finished yet.
 */
static inline void nvm_wait_until_ready( void )
{
	do {
		// Block execution while waiting for the NVM to be ready
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}

/**
 * \brief Non-Volatile Memory Execute Command
 *
 * This function sets the CCP register before setting the CMDEX bit in the
 * NVM.CTRLA register.
 *
 * \note The correct NVM command must be set in the NVM.CMD register before
 *       calling this function.
 */
static inline void nvm_exec(void)
{
	ccp_write_io((uint8_t *)&NVM.CTRLA, NVM_CMDEX_bm);
}

/**
 * \brief Non-Volatile Memory Execute Specific Command
 *
 * This function sets a command in the NVM.CMD register, then performs an
 * execute command by writing the CMDEX bit to the NVM.CTRLA register.
 *
 * \note The function saves and restores the NVM.CMD register, but if this
 *       function is called from an interrupt, interrupts must be disabled
 *       before this function is called.
 *
 * \param nvm_command NVM Command to execute.
 */
static inline void nvm_issue_command(NVM_CMD_t nvm_command)
{
	uint8_t old_cmd;

	old_cmd = NVM.CMD;
	NVM.CMD = nvm_command;
	ccp_write_io((uint8_t *)&NVM.CTRLA, NVM_CMDEX_bm);
	NVM.CMD = old_cmd;
}

/**
 * \brief Read one byte using the LDI instruction
 * \internal
 *
 * This function sets the specified NVM_CMD, reads one byte using at the
 * specified byte address with the LPM instruction. NVM_CMD is restored after
 * use.
 *
 * \note Interrupts should be disabled before running this function
 *       if program memory/NVM controller is accessed from ISRs.
 *
 * \param nvm_cmd NVM command to load before running LPM
 * \param address Byte offset into the signature row
 */
uint8_t nvm_read_byte(uint8_t nvm_cmd, uint16_t address);


/**
 * \brief Perform SPM write
 * \internal
 *
 * This function sets the specified NVM_CMD, sets CCP and then runs the SPM
 * instruction to write to flash.
 *
 * \note Interrupts should be disabled before running this function
 *       if program memory/NVM controller is accessed from ISRs.
 *
 * \param addr Address to perform the SPM on.
 * \param nvm_cmd NVM command to use in the NVM_CMD register
 */
void nvm_common_spm(uint32_t addr, uint8_t nvm_cmd);

//! @}



/**
 * \defgroup nvm_fuse_lock_group NVM driver fuse and lock bits handling
 * \ingroup nvm_group
 * \brief Functions for reading fuses and writing lock bits.
 *
 * The Fuses are used to set important system functions and can only be written
 * from an external programming interface. The application software can read
 * the fuses. The fuses are used to configure reset sources such as Brown-out
 * Detector and Watchdog, Start-up configuration, JTAG enable and JTAG user ID.
 *
 * The Lock bits are used to set protection level on the different flash
 * sections. They are used to block read and/or write on the different flash
 * sections. Lock bits can be written from en external programmer and from the
 * application software to set a more strict protection level, but not to set a
 * less strict protection level. Chip erase is the only way to erase the lock
 * bits. The lock bits are erased after the rest of the flash memory is erased.
 * An unprogrammed fuse or lock bit will have the value one, while a programmed
 * fuse or lock bit will have the value zero.
 * Both fuses and lock bits are reprogrammable like the Flash Program memory.
 *
 * \note The functions in this module are modifying the NVM.CMD register.
 *       If the application are using program space access in interrupts
 *       (__flash pointers in IAR EW or pgm_read_byte in GCC) interrupts
 *       needs to be disabled when running EEPROM access functions. If not
 *       the program space reads will be corrupted.
 * @{
 */

// The different fuse bytes
enum fuse_byte_t {
	FUSEBYTE0 = 0,
	FUSEBYTE1 = 1,
	FUSEBYTE2 = 2,
	FUSEBYTE3 = 3, // not used on current devices
	FUSEBYTE4 = 4,
	FUSEBYTE5 = 5,
};

uint8_t nvm_fuses_read(enum fuse_byte_t fuse);

/**
 * \brief Program the lock bits.
 *
 * Program the lock bits to the given values. Lock bits can only be programmed
 * to a more secure setting than previously programmed. To clear lock bits, a
 * flash erase has to be issued.
 *
 * \param blbb_lock Boot loader section lock bits to program
 * \param blba_lock Application section lock bits to program
 * \param blbat_lock Application table section lock bits to program
 * \param lb_lock Flash/eeprom lock bits to program
 */
static inline void nvm_lock_bits_write(enum NVM_BLBB_enum blbb_lock,
	enum NVM_BLBA_enum blba_lock, enum NVM_BLBAT_enum blbat_lock,
	enum NVM_LB_enum lb_lock)
{
	nvm_wait_until_ready();
	NVM.DATA0 = (uint8_t)blbb_lock | (uint8_t)blba_lock | (uint8_t)blbat_lock |
		(uint8_t)lb_lock;
	nvm_issue_command(NVM_CMD_WRITE_LOCK_BITS_gc);
}

/**
 * \brief Program the BLBB lock bits.
 *
 * Program the lock bits for the boot loader section (BLBB). Other lock bits
 * (BLBA, BLBAT and LB) are not altered (ie. programmed to NOLOCK).
 *
 * \param blbb_lock Boot loader section lock bits to program
 */
static inline void nvm_blbb_lock_bits_write(enum NVM_BLBB_enum blbb_lock)
{
	nvm_lock_bits_write(blbb_lock, NVM_BLBA_NOLOCK_gc, NVM_BLBAT_NOLOCK_gc,
		NVM_LB_NOLOCK_gc);
}

/**
 * \brief Program the BLBA lock bits.
 *
 * Program the lock bits for the application section (BLBA). Other lock bits
 * (BLBB, BLBAT and LB) are not altered (ie. programmed to NOLOCK).
 *
 * \param blba_lock Application section lock bits to program
 */
static inline void nvm_blba_lock_bits_write(enum NVM_BLBA_enum blba_lock)
{
	nvm_lock_bits_write(NVM_BLBB_NOLOCK_gc, blba_lock, NVM_BLBAT_NOLOCK_gc,
		NVM_LB_NOLOCK_gc);
}

/**
 * \brief Program the BLBAT lock bits.
 *
 * Program the lock bits for the application table section (BLBAT). Other lock
 * bits (BLBB, BLBA and LB) are not altered (ie. programmed to NOLOCK).
 *
 * \param blbat_lock Application table section lock bits to program
 */
static inline void nvm_blbat_lock_bits_write(enum NVM_BLBAT_enum blbat_lock)
{
	nvm_lock_bits_write(NVM_BLBB_NOLOCK_gc, NVM_BLBA_NOLOCK_gc, blbat_lock,
		NVM_LB_NOLOCK_gc);
}

/**
 * \brief Program the LB lock bits.
 *
 * Program the lock bits for the flash and eeprom (LB). Other lock bits
 * (BLBB, BLBA and BLBAT) are not altered (ie. programmed to NOLOCK).
 *
 * \param lb_lock Flash/eeprom lock bits to program
 */
static inline void nvm_lb_lock_bits_write(enum NVM_LB_enum lb_lock)
{
	nvm_lock_bits_write(NVM_BLBB_NOLOCK_gc, NVM_BLBA_NOLOCK_gc,
		NVM_BLBAT_NOLOCK_gc, lb_lock);
}

//! @}

/**
 * \page xmega_nvm_quickstart Quick Start Guide for the XMEGA NVM Driver
 *
 * This is the quick start guide for the \ref nvm_group "NVM Driver", with
 * step-by-step instructions on how to configure and use the driver for
 * specific use cases.
 *
 * The section described below can be compiled into e.g. the main application
 * loop or any other function that will need to interface non-volatile memory.
 *
 * \section xmega_nvm_quickstart_basic Basic usage of the NVM driver
 * This section will present three use cases of the NVM driver. The first will
 * write a page to EEPROM and verify that it has been written, the second will
 * access the BOD-level fuse to verify that the level is correctly set, and the
 * third will read a chunk from the user signature row.
 *
 * \section xmega_nvm_quickstart_eeprom_case Use case 1: EEPROM
 *
 * The NVM driver has functions for interfacing many types of non-volatile
 * memory, including flash, EEPROM, fuses and lock bits. The example code
 * below will write a page to the internal EEPROM, and read it back to verify,
 * using memory mapped I/O.
 *
 * \section xmega_nvm_quickstart_eeprom_case_setup_steps Setup steps
 * There are no setup steps required for this use case.
 *
 * \subsection nvm_quickstart_eeprom_case_example_code Example code
 *
 * \code
 * #define EXAMPLE_PAGE 2
 * #define EXAMPLE_ADDR EXAMPLE_PAGE * EEPROM_PAGE_SIZE
 *
 * uint8_t write_page[EEPROM_PAGE_SIZE];
 * uint8_t read_page[EEPROM_PAGE_SIZE];
 *
 * fill_page_with_known_data(write_page);
 * fill_page_with_zeroes(read_page);
 *
 * nvm_eeprom_load_page_to_buffer(write_page);
 * nvm_eeprom_atomic_write_page(EXAMPLE_PAGE);
 *
 * nvm_eeprom_read_buffer(EXAMPLE_ADDR,
 *         read_page, EEPROM_PAGE_SIZE);
 *
 * check_if_pages_are_equal(write_page, read_page);
 * \endcode
 *
 * \subsection nvm_quickstart_eeprom_case_workflow Workflow
 *
 * -# We define where we would like to store our data, and we arbitrarily
 *    choose page 2 of EEPROM:
 *     - \code
 *       #define EXAMPLE_PAGE 2
 *       #define EXAMPLE_ADDR EXAMPLE_PAGE * EEPROM_PAGE_SIZE
 *       \endcode
 * -# Define two tables, one which contains the data which we will write,
 *    and one which we will read the data into:
 *     - \code
 *     uint8_t write_page[EEPROM_PAGE_SIZE];
 *     uint8_t read_page[EEPROM_PAGE_SIZE];
 *       \endcode
 * -# Fill the tables with our data, and zero out the read table:
 *     - \code
 *       fill_page_with_known_data(write_page);
 *       fill_page_with_zeroes(read_page);
 *       \endcode
 *     - \note These functions are undeclared, you should replace them with
 *             your own appropriate functions.
 * -# We load our page into a temporary EEPROM page buffer:
 *     - \code
 *       nvm_eeprom_load_page_to_buffer(write_page);
 *       \endcode
 *     - \attention The function used above will not work if memory mapping
 *                  is enabled.
 * -# Do an atomic write of the page from buffer into the specified page:
 *     - \code
 *       nvm_eeprom_atomic_write_page(EXAMPLE_PAGE);
 *       \endcode
 *     - \note The function \ref nvm_eeprom_atomic_write_page() erases the
 *             page before writing the new one. For non-atomic (split)
 *             writing without deleting, see \ref nvm_eeprom_split_write_page()
 * -# Read the page back into our read_page[] table:
 *     - \code
 *       nvm_eeprom_read_buffer(EXAMPLE_ADDR,
 *               read_page, EEPROM_PAGE_SIZE);
 *       \endcode
 * -# Verify that the page is equal to the one that was written earlier:
 *     - \code
 *       check_if_pages_are_equal(write_page, read_page);
 *       \endcode
 *     - \note This function is not declared, you should replace it with your
 *             own appropriate function.
 *
 * \section xmega_nvm_quickstart_fuse_case Use case 2: Fuses
 *
 * The NVM driver has functions for reading fuses.
 * See \ref nvm_fuse_lock_group.
 *
 * We would like to check whether the Brown-out Detection level is set to
 * 2.1V. This is set by programming the fuses when the chip is connected
 * to a suitable programmer. The fuse is a part of FUSEBYTE5. If the BODLVL
 * is correct, we turn on LED0.
 *
 * \section xmega_nvm_quickstart_fuse_case_setup_steps Setup steps
 * There are no setup steps required for this use case.
 *
 * \subsection nvm_quickstart_fuse_case_example_code Example code
 * \code
 * uint8_t fuse_value;
 * fuse_value = nvm_fuses_read(FUSEBYTE5);
 *
 * if ((fuse_value & NVM_FUSES_BODLVL_gm) == BODLVL_2V1_gc) {
 *     gpio_set_pin_low(LED0_GPIO);
 * }
 * \endcode
 *
 * \subsection nvm_quickstart_fuse_case_workflow Workflow
 *
 * -# Create a variable to store the fuse contents:
 *     - \code
 *       uint8_t fuse_value;
 *       \endcode
 * -# The fuse value we are interested in, BODLVL, is stored in FUSEBYTE5.
 *    We call the function \ref nvm_fuses_read() to read the fuse into our
 *    variable:
 *     - \code
 *       fuse_value = nvm_fuses_read(FUSEBYTE5);
 *       \endcode
 * -# This ends the reading portion, but we would like to see whether the
 *    BOD-level is correct, and if it is, light up an LED:
 *     - \code
 *       if ((fuse_value & NVM_FUSES_BODLVL_gm) == BODLVL_2V1_gc) {
 *           gpio_set_pin_low(LED0_GPIO);
 *       }
 *       \endcode
 *
 * \section xmega_nvm_quickstart_signature_case Use case 3: Signature row
 *
 * The NVM driver has functions for reading the signature row of the device.
 * Here we will simply read 16 bytes from the user signature row, assuming
 * we need what is stored there.
 *
 * \section xmega_nvm_quickstart_signature_row_setup_steps Setup steps
 * There are no setup steps required for this use case.
 *
 * \subsection xmega_nvm_quickstart_signature_row_example_code Example code
 *
 * \code
 * #define START_ADDR 0x10
 * #define DATA_LENGTH 16
 *
 * uint8_t values[LENGTH];
 * uint8_t i;
 *
 * for (i = 0; i < DATA_LENGTH; i++) {
 *     values[i] = nvm_read_user_signature_row(START_ADDR + i);
 * }
 * \endcode
 *
 * \subsection nvm_quickstart_signature_case_workflow Workflow
 *
 * -# Define starting address and length of data segment, and create
 *    variables needed to store and process the data:
 *     - \code
 *       #define START_ADDR 0x10
 *       #define DATA_LENGTH 16
 *
 *       uint8_t values[LENGTH];
 *       uint8_t i;
 *       \endcode
 * -# Iterate through the user signature row, and store our desired data:
 *     - \code
 *       for (i = 0; i < DATA_LENGTH; i++) {
 *           values[i] = nvm_read_user_signature_row(START_ADDR + i);
 *       }
 *       \endcode
 *
 */

#ifdef __cplusplus
}
#endif

#endif /* NVM_H */
