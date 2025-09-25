#ifndef __RF_H
#define __RF_H

#include <stdint.h>
#include "main.h"   // for RF_DATA_Pin and RF_DATA_GPIO_Port

#ifdef __cplusplus
extern "C" {
#endif

// ---- Public API ----
void RF_Init(void);
void RF_SendCode(uint32_t code, uint8_t bits);

#ifdef __cplusplus
}
#endif

#endif /* __RF_H */
