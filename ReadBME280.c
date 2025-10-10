#include "main.h"

static inline void SDA_high(void){ GPIOB->BSRR = (1u<<7); }
static inline void SDA_low (void){ GPIOB->BRR  = (1u<<7); }
static inline void SCL_high(void){ GPIOB->BSRR = (1u<<6); }
static inline void SCL_low (void){ GPIOB->BRR  = (1u<<6); }
static inline int  SDA_read(void){ return (GPIOB->IDR>>7) & 1u; }
static inline int  SCL_read(void){ return (GPIOB->IDR>>6) & 1u; }
static inline void SDA_in(void){  GPIOB->CRL &= ~(0xFu<<(7*4)); GPIOB->CRL |=  (0x4u<<(7*4)); }
static inline void SDA_out(void){ GPIOB->CRL &= ~(0xFu<<(7*4)); GPIOB->CRL |=  (0x5u<<(7*4)); }
static inline void SCL_od(void){  GPIOB->CRL &= ~(0xFu<<(6*4)); GPIOB->CRL |=  (0x5u<<(6*4)); }
static inline void i2c_delay(void){ for(volatile int i=0;i<1500;i++){} }
//Khởi động chân I2C 
static inline void I2C_GPIO_Init(void){
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  SCL_od();
  SDA_out(); 
  SDA_high(); 
  SCL_high(); 
  i2c_delay();
}
//Tín hiệu bắt đầu I2C phát từ master
static inline void I2C_start(void){
  SDA_out();
  SDA_high(); 
  SCL_high(); 
  i2c_delay(); 
  SDA_low(); 
  i2c_delay(); 
  SCL_low(); 
  i2c_delay();
}
//Tín hiệu kết thúc I2C từ master
static inline void I2C_stop(void){
  SDA_out(); 
  SDA_low(); 
  i2c_delay(); 
  SCL_high(); 
  while(!SCL_read()){} 
  i2c_delay(); 
  SDA_high(); 
  i2c_delay();
}
//Chờ ACK từ slave
static inline int I2C_wait_ack(void){
  SDA_in(); 
  SCL_high(); 
  for(volatile int t=0;t<30000 && !SCL_read();++t){} 
  i2c_delay();
  int nack = SDA_read(); 
  SCL_low(); 
  i2c_delay(); 
  SDA_out(); 
  SDA_high(); 
  return nack;
}
// Truyền 1byte từ master (MSB first)
static inline void I2C_write8_raw(uint8_t v){
  SDA_out();
  for(int i=0;i<8;i++)
  { 
    if(v&0x80) SDA_high(); else SDA_low(); 
    i2c_delay(); 
    SCL_high();
    while(!SCL_read()){} 
    i2c_delay(); 
    SCL_low(); 
    i2c_delay(); 
    v<<=1; 
  }
  SDA_high();
}
// Đọc 1 byte từ slave
static inline uint8_t I2C_read8_raw(int ack){
  uint8_t d=0;
  SDA_in();
  for(int i=0;i<8;i++)
  { 
    d<<=1; 
    SCL_high(); 
    while(!SCL_read()){} 
    i2c_delay(); 
    if(SDA_read()) d|=1; 
    SCL_low(); 
    i2c_delay(); 
  }
  SDA_out(); 
  if(ack) SDA_low(); else SDA_high(); 
  i2c_delay(); 
  SCL_high(); 
  while(!SCL_read()){} 
  i2c_delay(); 
  SCL_low();
  i2c_delay();
  SDA_high();
  return d;
}

#define BME_I2C_ADDR_WR  (0x77u<<1)
#define BME_I2C_ADDR_RD  (BME_I2C_ADDR_WR | 1u)

int bme_write8(uint8_t reg, uint8_t val){
  I2C_start(); 
  I2C_write8_raw(BME_I2C_ADDR_WR); 
  if(I2C_wait_ack())
  { 
    I2C_stop(); 
    return 1;
  }
  I2C_write8_raw(reg); 
  if(I2C_wait_ack())
  { 
    I2C_stop(); 
    return 2;
  }
  I2C_write8_raw(val); 
  if(I2C_wait_ack())
  { 
    I2C_stop();
    return 3; 
  }
  I2C_stop(); 
  return 0;
}
//Đọc 1 thanh ghi cảm biến BME
int bme_read8(uint8_t reg, uint8_t *out){
  if(!out) return -1;
  I2C_start(); 
  I2C_write8_raw(BME_I2C_ADDR_WR); 
  if(I2C_wait_ack())
  { 
    I2C_stop(); 
    return 1; 
  }
  I2C_write8_raw(reg); 
  if(I2C_wait_ack())
  { 
    I2C_stop(); 
    return 2; 
  }
  I2C_start(); 
  I2C_write8_raw(BME_I2C_ADDR_RD); 
  if(I2C_wait_ack())
  { 
    I2C_stop(); 
    return 3; 
  }
  *out = I2C_read8_raw(0); 
  I2C_stop(); 
  return 0;
}

static void delay_ms_soft(unsigned ms)
{ for(unsigned i=0;i<ms;i++)
  { 
    for(volatile unsigned k=0;k<8000;k++) __NOP(); 
  } 
}

static int bme_config_x1_nohum_normal(void){
  if (bme_write8(0xE0, 0xB6)) return -1; delay_ms_soft(3);
  if (bme_write8(0xF2, 0x00)) return -2;
  if (bme_write8(0xF4, 0x27)) return -3;
  if (bme_write8(0xF5, 0x00)) return -4;
  return 0;
}

static uint32_t bme_read_raw20(uint8_t msb_addr){
  uint8_t msb=0,lsb=0,xlsb=0;
  bme_read8(msb_addr+0,&msb);
  bme_read8(msb_addr+1,&lsb);
  bme_read8(msb_addr+2,&xlsb);
  return ((uint32_t)msb<<12)|((uint32_t)lsb<<4)|((uint32_t)xlsb>>4);
}

static void bme_read_calib_T(uint16_t* T1,int16_t* T2,int16_t* T3){
  uint8_t v;
  bme_read8(0x88,&v); uint16_t lo=v; bme_read8(0x89,&v); *T1=(v<<8)|lo;
  bme_read8(0x8A,&v); lo=v; bme_read8(0x8B,&v); *T2=(int16_t)((v<<8)|lo);
  bme_read8(0x8C,&v); lo=v; bme_read8(0x8D,&v); *T3=(int16_t)((v<<8)|lo);
}

static void bme_read_calib_P(uint16_t* P1,int16_t* P2,int16_t* P3,int16_t* P4,int16_t* P5,int16_t* P6,int16_t* P7,int16_t* P8,int16_t* P9){
  uint8_t v; uint16_t lo;
  bme_read8(0x8E,&v); lo=v; bme_read8(0x8F,&v); *P1=(v<<8)|lo;
  bme_read8(0x90,&v); lo=v; bme_read8(0x91,&v); *P2=(int16_t)((v<<8)|lo);
  bme_read8(0x92,&v); lo=v; bme_read8(0x93,&v); *P3=(int16_t)((v<<8)|lo);
  bme_read8(0x94,&v); lo=v; bme_read8(0x95,&v); *P4=(int16_t)((v<<8)|lo);
  bme_read8(0x96,&v); lo=v; bme_read8(0x97,&v); *P5=(int16_t)((v<<8)|lo);
  bme_read8(0x98,&v); lo=v; bme_read8(0x99,&v); *P6=(int16_t)((v<<8)|lo);
  bme_read8(0x9A,&v); lo=v; bme_read8(0x9B,&v); *P7=(int16_t)((v<<8)|lo);
  bme_read8(0x9C,&v); lo=v; bme_read8(0x9D,&v); *P8=(int16_t)((v<<8)|lo);
  bme_read8(0x9E,&v); lo=v; bme_read8(0x9F,&v); *P9=(int16_t)((v<<8)|lo);
}
//Bù nhiêt độ
static int32_t bme_temp_centi_from_raw(uint32_t adc_T, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3, int32_t* t_fine_out){
  int32_t var1 = ((((int32_t)adc_T>>3) - ((int32_t)dig_T1<<1)) * (int32_t)dig_T2) >> 11;
  int32_t var2 = (((((int32_t)adc_T>>4) - (int32_t)dig_T1) * (((int32_t)adc_T>>4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3 >> 14;
  int32_t t_fine = var1 + var2;
  if(t_fine_out) *t_fine_out = t_fine;
  return (t_fine * 5 + 128) >> 8;
}
// Bù áp suất (bme_write8(0xF4, 0x27)) return -3;
  if (bme_write8(0xF5, 0x00)) return -4;
  return 0;
}

static uint32_t bme_read_raw20(uint8_t msb_addr){
  uint8_t msb=0,lsb=0,xlsb=0;
  bme_read8(msb_addr+0,&msb);
  bme_read8(msb_addr+1,&lsb);
  bme_read8(msb_addr+2,&xlsb);
  return ((uint32_t)msb<<12)|((uint32_t)lsb<<4)|((uint32_t)xlsb>>4);
}

static void bme_read_calib_T(uint16_t* T1,int16_t* T2,int16_t* T3){
  uint8_t v;
  bme_read8(0x88,&v); uint16_t lo=v; bme_read8(0x89,&v); *T1=(v<<8)|lo;
  bme_read8(0x8A,&v); lo=v; bme_read8(0x8B,&v); *T2=(int16_t)((v<<8)|lo);
  bme_read8(0x8C,&v); lo=v; bme_read8(0x8D,&v); *T3=(int16_t)((v<<8)|lo);
}

static void bme_read_calib_P(uint16_t* P1,int16_t* P2,int16_t* P3,int16_t* P4,int16_t* P5,int16_t* P6,int16_t* P7,int16_t* P8,int16_t* P9){
  uint8_t v; uint16_t lo;
  bme_read8(0x8E,&v); lo=v; bme_read8(0x8F,&v); *P1=(v<<8)|lo;
  bme_read8(0x90,&v); lo=v; bme_read8(0x91,&v); *P2=(int16_t)((v<<8)|lo);
  bme_read8(0x92,&v); lo=v; bme_read8(0x93,&v); *P3=(int16_t)((v<<8)|lo);
  bme_read8(0x94,&v); lo=v; bme_read8(0x95,&v); *P4=(int16_t)((v<<8)|lo);
  bme_read8(0x96,&v); lo=v; bme_read8(0x97,&v); *P5=(int16_t)((v<<8)|lo);
  bme_read8(0x98,&v); lo=v; bme_read8(0x99,&v); *P6=(int16_t)((v<<8)|lo);
  bme_read8(0x9A,&v); lo=v; bme_read8(0x9B,&v); *P7=(int16_t)((v<<8)|lo);
  bme_read8(0x9C,&v); lo=v; bme_read8(0x9D,&v); *P8=(int16_t)((v<<8)|lo);
  bme_read8(0x9E,&v); lo=v; bme_read8(0x9F,&v); *P9=(int16_t)((v<<8)|lo);
}
//Bù nhiêt độ
static int32_t bme_temp_centi_from_raw(uint32_t adc_T, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3, int32_t* t_fine_out){
  int32_t var1 = ((((int32_t)adc_T>>3) - ((int32_t)dig_T1<<1)) * (int32_t)dig_T2) >> 11;
  int32_t var2 = (((((int32_t)adc_T>>4) - (int32_t)dig_T1) * (((int32_t)adc_T>>4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3 >> 14;
  int32_t t_fine = var1 + var2;
  if(t_fine_out) *t_fine_out = t_fine;
  return (t_fine * 5 + 128) >> 8;
}
// Bù áp suất
static uint32_t bme_press_pa_from_raw(uint32_t adc_P, int32_t t_fine,
                                      uint16_t P1,int16_t P2,int16_t P3,int16_t P4,int16_t P5,int16_t P6,int16_t P7,int16_t P8,int16_t P9){
  int32_t var1 = (t_fine>>1) - 64000;
  int32_t var2 = (((var1>>2)*(var1>>2))>>11)*P6;
  var2 = var2 + ((var1*P5)<<1);
  var2 = (var2>>2) + ((int32_t)P4<<16);
  var1 = (((int32_t)P3 * (((var1>>2)*(var1>>2))>>13))>>3) + (((int32_t)P2*var1)>>1);
  var1 = (((32768 + var1)) * (int32_t)P1) >> 15;
  if(var1==0) return 0;
  uint32_t p = (((uint32_t)((int32_t)1048576 - (int32_t)adc_P) - (var2>>12)) * 3125u);
  if(p < 0x80000000u) p = (p << 1) / (uint32_t)var1; else p = (p / (uint32_t)var1) << 1;
  var1 = ((int32_t)P9 * (int32_t)(((p>>3)*(p>>3))>>13)) >> 12;
  var2 = ((int32_t)(p>>2) * (int32_t)P8) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + P7) >> 4));
  return p;
}

int main(void){
  I2C_GPIO_Init(); 
  delay_ms_soft(5);
  if(bme_config_x1_nohum_normal()!=0) while(1){ __NOP(); }

  uint16_t dig_T1; int16_t dig_T2, dig_T3;
  bme_read_calib_T(&dig_T1,&dig_T2,&dig_T3);

  uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9;
  bme_read_calib_P(&P1,&P2,&P3,&P4,&P5,&P6,&P7,&P8,&P9);

  uint32_t t_raw = bme_read_raw20(0xFA);
  int32_t t_fine = 0;
  int32_t T_centi = bme_temp_centi_from_raw(t_raw, dig_T1, dig_T2, dig_T3, &t_fine);

  uint32_t p_raw = bme_read_raw20(0xF7);
  uint32_t P_pa = bme_press_pa_from_raw(p_raw, t_fine, P1,P2,P3,P4,P5,P6,P7,P8,P9);

  (void)T_centi; (void)P_pa;
  while(1){ __NOP(); }
}

