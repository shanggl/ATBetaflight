/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Chris Hockuba (https://github.com/conkerkh)
 */

#pragma once


#ifdef USE_HAL_DRIVER
#include "usbd_msc.h"
#elif defined(USE_ATBSP_DRIVER)
//Fixme: fix this before merge
#else
#include "usbd_msc_mem.h"
#include "usbd_msc_core.h"
#endif


/** @defgroup USBD_MEM_Exported_Defines
  * @{
  */
#define USBD_STD_INQUIRY_LENGTH     36
/**
  * @}
  */


/** @defgroup USBD_MEM_Exported_TypesDefinitions
  * @{
  */

typedef struct _USBD_STORAGE
{
  int8_t (* Init) (uint8_t lun);
  int8_t (* GetCapacity) (uint8_t lun, uint32_t *block_num, uint32_t *block_size);
  int8_t (* IsReady) (uint8_t lun);
  int8_t (* IsWriteProtected) (uint8_t lun);
  int8_t (* Read) (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;

} USBD_STORAGE_cb_TypeDef;

/** @defgroup USBD_MEM_Exported_FunctionsPrototype
  * @{
  */
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;


#include "common/time.h"

#ifdef USE_HAL_DRIVER
extern USBD_StorageTypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD_SDIO
extern USBD_StorageTypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_SDCARD_SPI
extern USBD_StorageTypeDef USBD_MSC_MICRO_SD_SPI_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_StorageTypeDef USBD_MSC_EMFAT_fops;
#endif
#elif defined(USE_ATBSP_DRIVER)
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD_SDIO
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_SDCARD_SPI
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SD_SPI_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_STORAGE_cb_TypeDef USBD_MSC_EMFAT_fops;
#endif
#else // USE_HAL_DRIVER
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD_SDIO
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_SDCARD_SPI
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SD_SPI_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_STORAGE_cb_TypeDef USBD_MSC_EMFAT_fops;
#endif
#endif // USE_HAL_DRIVER
