#include "encoder_observer.h"
#include <math.h>

void EncoderRaw_Init(volatile EncoderRaw *r, unsigned ab)
{ r->count=0; r->errors=0; r->ab=(uint8_t)(ab&3U); }

void EncoderRaw_Edge(volatile EncoderRaw *r, unsigned ab)
{
    /* Positive cycle: 00 -> 01 -> 11 -> 10 -> 00. Wiring decides direction. */
    static const int8_t step[16]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
    unsigned old=r->ab, next=ab&3U;
    if ((old^next)==3U) ++r->errors;
    else r->count+=(uint32_t)(int32_t)step[old*4U+next];
    r->ab=(uint8_t)next;
}

int32_t Encoder_Delta(uint32_t newer, uint32_t older)
{
    uint32_t d=newer-older;
    /* Defined modular subtraction, including the INT32_MIN boundary. */
    return d<=INT32_MAX ? (int32_t)d : -1-(int32_t)(UINT32_MAX-d);
}

void EncoderObserver_Sample(EncoderObserver *o, EncoderRaw r, uint32_t now)
{
    uint32_t dt=now-o->sampled_ms;
    if (r.errors!=o->last_errors) o->referenced=false;
    if (o->sampled && dt>=10U) {
        o->speed_valid=dt<=100U && r.errors==o->window_errors;
        o->counts_per_s=o->speed_valid ? (float)Encoder_Delta(r.count,o->last_count)*1000.0f/(float)dt : 0;
    }
    if (!o->sampled || dt>=10U) {
        o->last_count=r.count; o->sampled_ms=now; o->sampled=true; o->window_errors=r.errors;
    }
    if (r.errors!=o->last_errors) o->speed_valid=false;
    o->last_errors=r.errors;
    o->position_valid=o->referenced && fabsf(o->counts_per_mm)>=0.001f;
    if (o->position_valid) {
        o->position_mm=o->reference_mm+(float)Encoder_Delta(r.count,o->reference_count)/o->counts_per_mm;
        if (!isfinite(o->position_mm) || fabsf(o->position_mm)>1000) {
            o->position_valid=false; o->referenced=false;
        }
    }
}

bool EncoderObserver_Scale(EncoderObserver *o, float scale)
{
    if (!isfinite(scale) || fabsf(scale)<0.001f || fabsf(scale)>1000000) return false;
    o->counts_per_mm=scale; o->referenced=false; o->position_valid=false;
    return true;
}

bool EncoderObserver_Reference(EncoderObserver *o, EncoderRaw r, float mm)
{
    if (!isfinite(mm) || mm<0 || mm>58.48f || fabsf(o->counts_per_mm)<0.001f) return false;
    o->reference_count=r.count; o->reference_mm=mm; o->last_errors=r.errors;
    o->referenced=true; o->position_valid=true; o->position_mm=mm;
    return true;
}
