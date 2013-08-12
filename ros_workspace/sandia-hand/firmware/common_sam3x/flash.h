#ifndef FLASH_H
#define FLASH_H

void flash_init();
// terminology: "page" = 256 bytes
void flash_read_page(const uint32_t page_num, uint8_t *page_data); 
void flash_write_page(const uint32_t page_num, const uint8_t *page_data);
void flash_erase_sector(const uint32_t page_num);

#endif

