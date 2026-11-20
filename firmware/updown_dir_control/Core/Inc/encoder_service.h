#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void EncoderService_Tick(uint32_t now);
bool EncoderService_Command(const char *line,bool motors_busy,uint32_t now,char *reply,size_t size);
