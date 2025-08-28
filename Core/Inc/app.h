/*
 * app.h
 * Created on: Aug 12, 2025
 * Author: skylink
 */
#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Handle commands from USB CDC: list, sendfilename <name>, status */
void waitForCommand(void);

/** Basic E32 init (sets power to 21 dBm, prints replies over CDC) */
void e32_basic_init(void);

/** Send a zero-terminated string over UART1 */
void uart1_send_str(const char *s);

/** Send a single byte over UART1 (required by task) */
void sendByteUART(uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_H_ */
