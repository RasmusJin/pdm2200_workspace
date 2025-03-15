SUMMARY = "ROV 2200 SPI EEPROM Driver"
DESCRIPTION = "Linux kernel driver for ROV 2200 SPI EEPROM communication"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

SRC_URI = "file://rov_2200_spi.c \
           file://Makefile \
          "

S = "${WORKDIR}"

RPROVIDES:${PN} += "kernel-module-rov-2200-spi"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the OpenEmbedded core build system.
