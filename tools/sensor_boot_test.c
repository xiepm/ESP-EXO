/* Real sensor init implementation; only I2C/clock/RTOS boundaries are fake. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_INVALID_RESPONSE 0x108
#include "../main/exo_sensors.c"
static uint8_t regs[256];
static int64_t test_us, reset_us;
static unsigned resets, probes, handles, removes;
static bool stuck_id, bad_readback;
static unsigned missing_probes;
const char *esp_err_to_name(esp_err_t err) { (void)err; return "test-error"; }
esp_err_t gpio_config(const gpio_config_t *c) { (void)c; abort(); }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { abort(); }
void vSemaphoreDelete(SemaphoreHandle_t s) { (void)s; abort(); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s,TickType_t t) { (void)s;(void)t; abort(); }
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { (void)s; abort(); }
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *c,i2c_master_bus_handle_t *b) { (void)c;(void)b; abort(); }
int xTaskCreate(void(*fn)(void*),const char*n,unsigned stack,void*a,unsigned p,TaskHandle_t*t) {
    (void)fn;(void)n;(void)stack;(void)a;(void)p;(void)t;abort();
}
TickType_t xTaskGetTickCount(void) { return (TickType_t)(test_us/1000); }
void vTaskDelayUntil(TickType_t *last,TickType_t t) { (void)last;(void)t;abort(); }
void exo_attitude_update(const exo_sensor_status_t *s) { (void)s;abort(); }
int64_t esp_timer_get_time(void) { return test_us; }
void vTaskDelay(TickType_t ticks) { test_us += (int64_t)ticks*1000; }
void esp_rom_delay_us(uint32_t us) { test_us+=us; }
esp_err_t i2c_master_probe(i2c_master_bus_handle_t b,uint16_t a,int ms) {
    (void)b;(void)a;(void)ms; ++probes;
    return probes<=missing_probes ? ESP_ERR_NOT_FOUND : ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t b,const i2c_device_config_t *c,i2c_master_dev_handle_t *d) {
    (void)b; assert(c->device_address==0x6b); assert(!handles); ++handles; *d=(void*)1; return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t d) {
    assert(d && handles); --handles; ++removes; return ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t d,const uint8_t *data,size_t n,int ms) {
    (void)d;(void)ms; assert(n==2);
    if(data[0]==0x60) {
        assert(data[1]==0xb0); ++resets; reset_us=test_us;
        memset(regs,0,sizeof(regs));regs[0]=stuck_id?0:5;regs[0x4d]=0x80;
    } else {
        regs[data[0]]=data[1];
        if(data[0]==0x0a) regs[0x2d]=data[1]?0x80:0;
    }
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t d,const uint8_t *reg,size_t tx,uint8_t *out,size_t n,int ms) {
    (void)d;(void)ms; assert(tx==1);
    if(*reg==0x4d && resets) assert(test_us-reset_us>=20000);
    memcpy(out,regs+*reg,n);
    if(bad_readback && *reg==0x03) *out=0;
    return ESP_OK;
}
static void reset_case(uint8_t id) {
    memset(&s_status,0,sizeof(s_status));memset(regs,0,sizeof(regs));regs[0]=id;
    s_qmi=NULL;s_bus=(void*)1;test_us=1000000;resets=probes=handles=removes=0;
    stuck_id=bad_readback=false;missing_probes=0;
}
int main(void) {
    reset_case(0); assert(init_sensor_locked(true)==ESP_OK);
    assert(resets==1 && s_status.qmi.configured && s_status.qmi.chip_id==5);
    assert(!s_status.qmi.valid && s_status.qmi.timestamp_us==0);
    puts("PASS zero boot ID recovers only after reset status, real ID and config readback");
    reset_case(0);missing_probes=4;assert(init_sensor_locked(true)==ESP_OK);
    assert(probes==5 && resets==1 && s_status.qmi.configured);
    puts("PASS initial NACK recovers after bounded delayed re-probe");
    reset_case(0);stuck_id=true;assert(init_sensor_locked(true)!=ESP_OK);
    assert(resets==3 && removes==2 && handles==1 && !s_status.qmi.configured && !s_status.qmi.valid);
    puts("PASS persistent zero ID stops after three attempts; no stale valid sample");
    reset_case(0x42);assert(init_sensor_locked(true)!=ESP_OK);assert(resets==0 && !s_status.qmi.configured);
    puts("PASS foreign nonzero ID is never reset or accepted");
    reset_case(5);bad_readback=true;assert(init_sensor_locked(true)!=ESP_OK);
    assert(resets==3 && !s_status.qmi.configured);
    puts("PASS wrong configuration readback cannot publish initialized state");
    return 0;
}
