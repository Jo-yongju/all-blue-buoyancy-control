#include "encoder_service.h"
#include "encoder_port.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static EncoderObserver observer[2]; /* ControlTask is sole owner. */
static bool enabled;
void EncoderService_Tick(uint32_t now)
{
    if(enabled) for(unsigned i=0;i<2;i++)
        EncoderObserver_Sample(&observer[i],EncoderPort_Read(i),now);
}
bool EncoderService_Command(const char *line,bool busy,uint32_t now,char *reply,size_t size)
{
    if(strcmp(line,"ENC") && strncmp(line,"ENC ",4)) return false;
    if(!strcmp(line,"ENC") || !strcmp(line,"ENC HELP")) {
        snprintf(reply,size,"ENC ON/OFF | ENC STATUS 1/2 | ENC SCALE 1/2 signed_counts_per_mm | ENC REF 1/2 known_mm; OBSERVE ONLY\r\n"); return true;
    }
    if(!strncmp(line,"ENC STATUS ",11) && strlen(line)==12 && (line[11]=='1'||line[11]=='2')) {
        unsigned a=(unsigned)(line[11]-'1');
        EncoderService_Tick(now);
        EncoderRaw raw=EncoderPort_Read(a); EncoderObserver *o=&observer[a];
        /* Invalid values are deliberately zeroed; check valid flags, not value. */
        snprintf(reply,size,"ENC axis=%u on=%u count=%ld errors=%lu speed_ok=%u cps=%ld ref=%u pos_ok=%u pos_um=%ld OBSERVE_ONLY\r\n",
            a+1,enabled,(long)Encoder_Delta(raw.count,0),(unsigned long)raw.errors,
            enabled&&o->speed_valid,(long)(enabled&&o->speed_valid?fmaxf(-2147483000.0f,fminf(2147483000.0f,o->counts_per_s)):0),
            enabled&&o->referenced,enabled&&o->position_valid,(long)(enabled&&o->position_valid?o->position_mm*1000.0f:0));
        return true;
    }
    if(busy) {snprintf(reply,size,"ERR ENC configuration requires stopped motors\r\n");return true;}
    if(!strcmp(line,"ENC ON")) {
        EncoderPort_Start(); memset(observer,0,sizeof observer); enabled=true;
        EncoderService_Tick(now);
        snprintf(reply,size,"OK ENC observe only; PC0/PC1 front PC2/PC3 rear; scale/reference UNSET\r\n");return true;
    }
    if(!strcmp(line,"ENC OFF")) {
        EncoderPort_Stop(); enabled=false; memset(observer,0,sizeof observer);
        snprintf(reply,size,"OK ENC OFF; reference discarded\r\n");return true;
    }
    bool scale=!strncmp(line,"ENC SCALE ",10), ref=!strncmp(line,"ENC REF ",8);
    if(scale||ref) {
        const char *p=line+(scale?10:8); char *end;
        if((*p!='1'&&*p!='2')||p[1]!=' '||!p[2]) goto syntax;
        unsigned a=(unsigned)(*p-'1');
        float value=strtof(p+2,&end);
        if(end==p+2 || *end || !isfinite(value)) goto syntax;
        if(!enabled) {snprintf(reply,size,"ERR ENC ON first\r\n");return true;}
        bool ok=scale?EncoderObserver_Scale(&observer[a],value):EncoderObserver_Reference(&observer[a],EncoderPort_Read(a),value);
        snprintf(reply,size,ok?"OK ENC %s axis=%u; OBSERVE ONLY\r\n":"ERR ENC %s axis=%u; check scale/reference range\r\n",scale?"SCALE":"REF",a+1);
        return true;
    }
syntax:
    snprintf(reply,size,"ERR ENC syntax; ENC HELP\r\n");return true;
}
