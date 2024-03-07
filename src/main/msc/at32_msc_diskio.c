/*
 * At32 MSC scsi interface
    *    先实现FLASH emfat 方式读写，后面再实现sd等其他存储读写
 *
 *
 *
 *
 */


#include <stdbool.h>

#include "at32_msc_diskio.h"
#include "platform.h"

#include "blackbox/blackbox.h"
#include "pg/sdcard.h"

#include "common/utils.h"
#include "msc_bot_scsi.h"
#include "drivers/usb_msc.h"

#include "msc/usbd_storage.h"

USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;


static USBD_STORAGE_cb_TypeDef *getDevice(){
  if(USBD_STORAGE_fops!=0){
    return USBD_STORAGE_fops;
  }
  switch (blackboxConfig()->device) {
  #ifdef USE_SDCARD
      case BLACKBOX_DEVICE_SDCARD:
          switch (sdcardConfig()->mode) {
  #ifdef USE_SDCARD_SDIO
          case SDCARD_MODE_SDIO:
              USBD_STORAGE_fops = &USBD_MSC_MICRO_SDIO_fops;
              break;
  #endif
  #ifdef USE_SDCARD_SPI
          case SDCARD_MODE_SPI:
              USBD_STORAGE_fops = &USBD_MSC_MICRO_SD_SPI_fops;
              break;
  #endif
          default:;
          }
          break;
  #endif

  #ifdef USE_FLASHFS
      case BLACKBOX_DEVICE_FLASH:
          USBD_STORAGE_fops = &USBD_MSC_EMFAT_fops;
          break;
  #endif
      default:;
      }
  if(USBD_STORAGE_fops){
    USBD_STORAGE_fops->Init(0);
  }
  return USBD_STORAGE_fops;
}



static uint32_t last_block_size=512;
usb_sts_type msc_disk_capacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    getDevice()->GetCapacity(lun, block_num, block_size);
    last_block_size = *block_size;
    return USB_OK;
}


usb_sts_type msc_disk_read(
    uint8_t lun,        // logical unit number
    uint32_t blk_addr,  // address of 1st block to be read
    uint8_t *buf,       // Pointer to the buffer to save data
    uint32_t blk_len)   // nmber of blocks to be read
{
    int ret=getDevice()->Read(
      lun,        // logical unit number
      buf ,       // Pointer to the buffer to save data
      blk_addr /last_block_size,  // address of 1st block to be read
      blk_len/last_block_size);   // nmber of blocks to be read
    return ret==0?USB_OK:USB_FAIL;
}

usb_sts_type msc_disk_write(uint8_t lun,
    uint32_t blk_addr,
    uint8_t *buf,
    uint32_t blk_len)
{
    int ret=getDevice()->Write(
      lun,        // logical unit number
      buf ,       // Pointer to the buffer to save data
      blk_addr / last_block_size,  // address of 1st block to be read
      blk_len / last_block_size);   // nmber of blocks to be read
    return ret==0?USB_OK:USB_FAIL;
}


/**
  * @brief  get disk inquiry
  * @param  lun: logical units number
  * @retval inquiry string
  */
uint8_t * get_inquiry(uint8_t lun)
{
  if(lun < MSC_SUPPORT_MAX_LUN)
    return (uint8_t *)getDevice()->pInquiry;
  else
    return NULL;
}
