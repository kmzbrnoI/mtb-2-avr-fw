#ifndef _COMMON_H_
#define _COMMON_H_

/* Common functions for all units.
 */

#define BOOTLOADER_ADDR 0xF000

#define MEMCPY_FROM_VAR(target, source) { memcpy((void*)target, (void*)&source, sizeof(source)); }

#endif
