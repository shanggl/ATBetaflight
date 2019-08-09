H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

EXST = yes
# flash address 0x97CE0000 - code_ram address (0x24010000) = 0x73CD0000 
EXST_ADJUST_VMA = 0x73CD0000

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/barometer/barometer_bmp388.c \
