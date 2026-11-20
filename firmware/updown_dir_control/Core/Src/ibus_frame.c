#include "ibus_frame.h"

bool Ibus_Byte(IbusFrame *f,uint8_t byte,uint32_t now,uint16_t out[14])
{
    if (now-f->last_ms>10) f->used=0;
    f->last_ms=now;
    if (!f->used && byte!=0x20) return false;
    if (f->used==1 && byte!=0x40) {
        f->used=byte==0x20 ? 1 : 0;
        return false;
    }
    f->bytes[f->used++]=byte;
    if (f->used!=sizeof f->bytes) return false;
    f->used=0;
    uint16_t sum=0xffff;
    for (unsigned i=0;i<30;i++) sum-=f->bytes[i];
    if (sum!=(uint16_t)(f->bytes[30]|f->bytes[31]<<8)) return false;
    for (unsigned i=0;i<14;i++) {
        unsigned raw=f->bytes[2+2*i]|f->bytes[3+2*i]<<8;
        if (raw<900 || raw>2100) return false;
    }
    for (unsigned i=0;i<14;i++) out[i]=(uint16_t)(f->bytes[2+2*i]|f->bytes[3+2*i]<<8);
    return true;
}
