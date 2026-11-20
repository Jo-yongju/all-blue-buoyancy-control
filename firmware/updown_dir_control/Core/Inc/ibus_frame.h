#ifndef IBUS_FRAME_H
#define IBUS_FRAME_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint8_t bytes[32]; unsigned used; uint32_t last_ms; } IbusFrame;
bool Ibus_Byte(IbusFrame *f, uint8_t byte, uint32_t now, uint16_t channels[14]);
#endif
