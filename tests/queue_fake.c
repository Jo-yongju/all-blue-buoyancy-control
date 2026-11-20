#include "queue.h"
#include <string.h>
QueueHandle_t xQueueCreateStatic(UBaseType_t n,UBaseType_t size,uint8_t *bytes,StaticQueue_t *q){
    *q=(StaticQueue_t){.length=n,.size=size,.data=bytes};return q;
}
BaseType_t xQueueSend(QueueHandle_t q,const void *m,TickType_t wait){
    assert(wait==0);if(q->used==q->length)return 0;
    memcpy(q->data+q->head*q->size,m,q->size);q->head=(q->head+1)%q->length;++q->used;return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t q,void *m,TickType_t wait){
    assert(wait==0);if(!q->used)return 0;
    memcpy(m,q->data+q->tail*q->size,q->size);q->tail=(q->tail+1)%q->length;--q->used;return pdPASS;
}
