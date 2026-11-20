#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Pure x4 quadrature decoder. IRQ owns it; snapshot with interrupts masked. */
typedef struct { uint32_t count, errors; uint8_t ab; } EncoderRaw;
void EncoderRaw_Init(volatile EncoderRaw *raw, unsigned ab);
void EncoderRaw_Edge(volatile EncoderRaw *raw, unsigned ab);
int32_t Encoder_Delta(uint32_t newer, uint32_t older);

/* Observer only: never authorizes or writes motor output. */
typedef struct {
    uint32_t last_count, last_errors, window_errors, sampled_ms, reference_count;
    float counts_per_mm, reference_mm, position_mm, counts_per_s;
    bool sampled, referenced, position_valid, speed_valid;
} EncoderObserver;
void EncoderObserver_Sample(EncoderObserver *o, EncoderRaw raw, uint32_t now);
bool EncoderObserver_Scale(EncoderObserver *o, float signed_counts_per_mm);
bool EncoderObserver_Reference(EncoderObserver *o, EncoderRaw raw, float known_mm);
