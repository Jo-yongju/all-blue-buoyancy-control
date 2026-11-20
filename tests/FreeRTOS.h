#pragma once
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef struct { unsigned head,tail,used,length,size; uint8_t *data; } StaticQueue_t;
#define configASSERT(x) assert(x)
#define pdPASS 1
