#include "main.h"

static inline void delay_cycles(volatile uint32_t n){ while(n--) __NOP(); }

#define VREF_MV   3000u          // 3.000 V
#define ADC_MAX   4095u          // 12-bit: 0..4095

static uint16_t ADC1_ReadChannel(uint8_t ch)
{
    if (ch <= 9)
    {
        uint8_t shift = ch * 3;
        ADC1->SMPR2 &= ~(0x7u << shift); //xóa SMPx
        ADC1->SMPR2 |=  (0x7u << shift); //Ghi 111 vào SMPx -> tốc độ 	239.5cycle
    }
    ADC1->SQR1 &= ~(0xFu << 20);					//  L[23:20] = 0, 1 conversion
    ADC1->SQR3  = (ch & 0x1Fu);						/// SQ1 = ch (bits [4:0]) : bật bit ch
    ADC1->CR2  |= ADC_CR2_EXTTRIG | ADC_CR2_SWSTART; //Bật SWSTART=1: bắn đầu chuyển, EXTTRIG: cho phép trigger bên ngoài
    while(!(ADC1->SR & ADC_SR_EOC)){} // chờ SR(ADC_SR_EOC=1: kết thúc chuyển đổi)
    return (uint16_t)ADC1->DR;
}

static void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN; //IOPAEN=portA clock enable,ADC1EN=1: cho phép ADC1
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;		//clear prescaler
    RCC->CFGR |=  RCC_CFGR_ADCPRE_DIV6; //prescale=6
    GPIOA->CRL &= ~(0xFu << (1*4)); // PA1 analog
    GPIOA->CRL &= ~(0xFu << (2*4)); // PA2 analog
    ADC1->CR2 |= ADC_CR2_ADON; // bật adc
    delay_cycles(1000);
    ADC1->CR2 |= ADC_CR2_RSTCAL; //bật hiệu chuẩn
    while(ADC1->CR2 & ADC_CR2_RSTCAL){}
    ADC1->CR2 |= ADC_CR2_CAL;   // cho phép hiệu chuẩn
    while(ADC1->CR2 & ADC_CR2_CAL){}
    ADC1->CR2 |= ADC_CR2_ADON; //bật ADC

    // đảm bảo dữ liệu canh phải (12-bit right aligned)
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
}

// giả lập coi output thôi
static void GPIOB_Init_Output(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // bật clock PortB

    // PB0..PB7: output push-pull 50MHz (MODE=11, CNF=00 → 0x3)
    GPIOB->CRL = 0x33333333;

    // PB8..PB11: cũng output push-pull 50MHz
    // Mỗi pin chiếm 4 bit, nên ta set 4 nhóm = 0x3333 cho 8..11
    GPIOB->CRH &= ~(0xFFFFu);      // clear bit [15:0] (PB8..PB11)
    GPIOB->CRH |=  (0x3333u);      // set output push-pull 50MHz
}


volatile uint16_t mq4_raw = 0;
volatile uint16_t mq7_raw = 0;

// mV quy đổi từ raw (theo Vref=3.000V)
volatile uint16_t mq4_mv  = 0;
volatile uint16_t mq7_mv  = 0;

int main(void)
{
    ADC1_Init();
    GPIOB_Init_Output();

    while (1)
    {
        mq4_raw = ADC1_ReadChannel(1);   // PA1
        mq7_raw = ADC1_ReadChannel(2);   // PA2

        // đổi sang mV: làm tròn (raw*3000 + 2047) / 4095
        mq4_mv  = (uint16_t)(((uint32_t)mq4_raw * VREF_MV + (ADC_MAX/2)) / ADC_MAX);
        mq7_mv  = (uint16_t)(((uint32_t)mq7_raw * VREF_MV + (ADC_MAX/2)) / ADC_MAX);

        GPIOB->ODR = mq7_mv;

    }
}
