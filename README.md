rov_2200 development sandbox, for drivers, userspace program etc. 

its one place to store all thoughts and ideas and backup the project code files written.

also a place for documentation of requirements, uart, general principle of operation.etc.

rov_2200_spi.c: This is the main eeprom spi driver, it will function as a custom kernel module, and this needs to be loaded into petalinux.

rov_eeprom_flash.c: this is a flashing program for the eeprom, it will check for correct formatting. theres ideas about also asking the user for files here.

spi_card_test: this is a test program to test and validate the card select gpios, and after that a simple transmission, and read ability. was written before the flash utility.
