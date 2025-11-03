#include "main.h"
#include <stdint.h>
//Gọi hàm này nó trả pointer về áp suất với nhiệt độ nè nhe bme_read_temp_pressure(double *t_C, double *p_hPa)
/* ====== Bit-bang I2C: PB6=SCL, PB7=SDA ====== */
#define PB6 6u
#define PB7 7u
#define BIT(n) (1u<<(n))

static inline void SDA_high(void){ GPIOB->BSRR = BIT(PB7); }
static inline void SDA_low (void){ GPIOB->BRR  = BIT(PB7); }
static inline void SCL_high(void){ GPIOB->BSRR = BIT(PB6); }
static inline void SCL_low (void){ GPIOB->BRR  = BIT(PB6); }
static inline int  SDA_read(void){ return (GPIOB->IDR>>PB7) & 1u; }
static inline int  SCL_read(void){ return (GPIOB->IDR>>PB6) & 1u; }

static inline void SDA_in (void){
  GPIOB->CRL &= ~(0xFu<<(PB7*4));
  GPIOB->CRL |=  (0x4u<<(PB7*4));   // input floating
}
static inline void SDA_out(void){
  GPIOB->CRL &= ~(0xFu<<(PB7*4));
  GPIOB->CRL |=  (0x5u<<(PB7*4));   // open-drain, 10MHz
}
static inline void SCL_od (void){
  GPIOB->CRL &= ~(0xFu<<(PB6*4));
  GPIOB->CRL |=  (0x5u<<(PB6*4));   // open-drain, 10MHz
}

static inline void i2c_delay(void){ for(volatile int i=0;i<1500;i++){} }

static void I2C_GPIO_Init(void){
  RCC->APB2ENR |= 1<<3;  // enable GPIOB
  SCL_od(); SDA_out(); SDA_high(); SCL_high(); i2c_delay();
}

static void I2C_start(void){
  SDA_out(); SDA_high(); i2c_delay();
  SCL_high(); while(!SCL_read()){} i2c_delay();
  SDA_low();  i2c_delay();
  SCL_low();  i2c_delay();
}
static void I2C_stop(void){
  SDA_out(); SDA_low();  i2c_delay();
  SCL_high(); while(!SCL_read()){} i2c_delay();
  SDA_high(); i2c_delay();
}
static int I2C_wait_ack(void){
  SDA_in(); SCL_high();
  for(int t=0;t<5000 && !SCL_read();++t){} i2c_delay();
  int nack = SDA_read(); SCL_low(); i2c_delay();
  SDA_out(); SDA_high();
  return nack; // 0=ACK
}
static void i2c_tx(uint8_t v){
  SDA_out();
  for(int i=0;i<8;i++){
    (v&0x80)? SDA_high(): SDA_low();
    i2c_delay(); SCL_high(); while(!SCL_read()){} i2c_delay();
    SCL_low(); i2c_delay(); v<<=1;
  }
  SDA_high(); // release
}
static uint8_t i2c_rx(int ack){
  uint8_t d=0; SDA_in();
  for(int i=0;i<8;i++){
    d<<=1; SCL_high(); while(!SCL_read()){} i2c_delay();
    if(SDA_read()) d|=1; SCL_low(); i2c_delay();
  }
  SDA_out(); (ack? SDA_low(): SDA_high()); i2c_delay();
  SCL_high(); while(!SCL_read()){} i2c_delay();
  SCL_low(); i2c_delay(); SDA_high();
  return d;
}

/* ====== BME280 driver (đơn giản) ====== */
static uint8_t g_addr = 0x76;             // mặc định 0x76 (SDO=GND)
#define BME_WR ((g_addr<<1)|0)
#define BME_RD ((g_addr<<1)|1)

/* calib lưu toàn cục sau khi init */
static uint16_t T1, P1;
static int16_t  T2,T3, P2,P3,P4,P5,P6,P7,P8,P9;

static int bme_write8(uint8_t reg, uint8_t val){
  I2C_start();
  i2c_tx(BME_WR); if(I2C_wait_ack()){ I2C_stop(); return 1; }
  i2c_tx(reg);    if(I2C_wait_ack()){ I2C_stop(); return 2; }
  i2c_tx(val);    if(I2C_wait_ack()){ I2C_stop(); return 3; }
  I2C_stop(); return 0;
}
static int bme_read8(uint8_t reg, uint8_t *out){
  if(!out) return -1;
  I2C_start();
  i2c_tx(BME_WR); if(I2C_wait_ack()){ I2C_stop(); return 1; }
  i2c_tx(reg);    if(I2C_wait_ack()){ I2C_stop(); return 2; }
  I2C_start();
  i2c_tx(BME_RD); if(I2C_wait_ack()){ I2C_stop(); return 3; }
  *out = i2c_rx(0); I2C_stop(); return 0;
}

static void delay_ms_soft(unsigned ms){
  for(unsigned i=0;i<ms;i++){ for(volatile unsigned k=0;k<8000;k++) __NOP(); }
}

static uint16_t read_u16_le(uint8_t addr){
  uint8_t lo=0, hi=0;
  bme_read8(addr,&lo); bme_read8(addr+1,&hi);
  return (uint16_t)(((uint16_t)hi<<8)|lo);
}
static int16_t read_s16_le(uint8_t addr){ return (int16_t)read_u16_le(addr); }

static int bme_config(void){
  if (bme_write8(0xE0, 0xB6)) return -1; // reset
  delay_ms_soft(15);
  if (bme_write8(0xF2, 0x00)) return -2; // humidity off
  if (bme_write8(0xF4, 0x27)) return -3; // T x1, P x1, normal
  if (bme_write8(0xF5, 0x00)) return -4; // filter off
  return 0;
}
static uint32_t bme_read_raw20(uint8_t base){
  uint8_t msb=0,lsb=0,xlsb=0;
  bme_read8(base+0,&msb); bme_read8(base+1,&lsb); bme_read8(base+2,&xlsb);
  return ((uint32_t)msb<<12)|((uint32_t)lsb<<4)|((uint32_t)xlsb>>4);
}
static int32_t bme_temp_centi_from_raw(uint32_t adc_T, int32_t *t_fine_out){
  int32_t var1 = ((((int32_t)adc_T>>3) - ((int32_t)T1<<1)) * (int32_t)T2) >> 11;
  int32_t var2 = (((((int32_t)adc_T>>4) - (int32_t)T1) * (((int32_t)adc_T>>4) - (int32_t)T1)) >> 12) * (int32_t)T3 >> 14;
  int32_t t_fine = var1 + var2;
  if (t_fine_out) *t_fine_out = t_fine;
  return (t_fine * 5 + 128) >> 8;  // centi-°C
}
static uint32_t bme_press_pa_from_raw(uint32_t adc_P, int32_t t_fine){
  int32_t var1 = (t_fine>>1) - 64000;
  int32_t var2 = (((var1>>2)*(var1>>2))>>11)*P6;
  var2 += ((var1*P5)<<1);
  var2 = (var2>>2) + ((int32_t)P4<<16);
  var1 = ( (((int32_t)P3 * (((var1>>2)*(var1>>2)) >> 13)) >> 3)
        + (((int32_t)P2 * var1) >> 1) ) >> 18;          // <<— đúng datasheet
  var1 = ((32768 + var1) * (int32_t)P1) >> 15;
  if (var1==0) return 0;
  uint32_t p = (((uint32_t)((int32_t)1048576 - (int32_t)adc_P) - (var2>>12)) * 3125u);
  if (p < 0x80000000u) p = (p<<1) / (uint32_t)var1; else p = (p/(uint32_t)var1) << 1;
  var1 = ((int32_t)P9 * (int32_t)(((p>>3)*(p>>3))>>13)) >> 12;
  var2 = ((int32_t)(p>>2) * (int32_t)P8) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + P7) >> 4));
  return p; // Pascal
}

/* ====== PUBLIC API ====== */
/* Khởi tạo: chọn địa chỉ 0x76 hoặc 0x77, init GPIO, config, đọc calib */
int bme_init(uint8_t addr){
  g_addr = addr & 0x7F;
  I2C_GPIO_Init();

  // optional: kiểm tra ID = 0x60
  uint8_t id=0; if (bme_read8(0xD0,&id) || id!=0x60) return -10;

  if (bme_config()!=0) return -11;

  // đọc hệ số bù và lưu global
  T1 = read_u16_le(0x88); T2 = read_s16_le(0x8A); T3 = read_s16_le(0x8C);
  P1 = read_u16_le(0x8E); P2 = read_s16_le(0x90); P3 = read_s16_le(0x92);
  P4 = read_s16_le(0x94); P5 = read_s16_le(0x96); P6 = read_s16_le(0x98);
  P7 = read_s16_le(0x9A); P8 = read_s16_le(0x9C); P9 = read_s16_le(0x9E);
  return 0;
}


int bme_read_temp_pressure(double *t_C, double *p_hPa){
  if (!t_C || !p_hPa) return -1;

  uint32_t t_raw = bme_read_raw20(0xFA);
  int32_t  t_fine = 0;
  int32_t  T_centi = bme_temp_centi_from_raw(t_raw, &t_fine);

  uint32_t p_raw = bme_read_raw20(0xF7);
  uint32_t P_pa  = bme_press_pa_from_raw(p_raw, t_fine);

  *t_C   = T_centi / 100.0;   // °C
  *p_hPa = P_pa   / 100.0;    // hPa
  return 0;
}

/* ====== Ví dụ dùng trong main ====== */
/*
int main(void){
  if (bme_init(0x77) != 0){ for(;;){} }  // đổi 0x76 nếu SDO=GND

  while(1){
    double tC, pHPa;
    if (bme_read_temp_pressure(&tC, &pHPa) == 0){
      // in ra tùy bạn: printf, SWV ITM, UART…
      // printf("T=%.2f C  P=%.2f hPa\r\n", tC, pHPa);
    }
    // delay_ms_soft(500);
  }
}
*/
