#include "main.h"

static inline void delay_cycles(volatile uint32_t n){ while(n--) __NOP(); }

#define VREF_MV   3300u      // Vref = 3.300 V (VREF+ = 3.3V)
#define ADC_MAX   4095u      // 12-bit: 0..4095

// --- Đọc một kênh ADC1 (single conversion, right-aligned) ---
static uint16_t ADC1_ReadChannel(uint8_t ch)
{
    // Cấu hình thời gian lấy mẫu dài để ổn định nguồn đo
    if (ch <= 9) {
        uint8_t shift = ch * 3;
        ADC1->SMPR2 &= ~(0x7u << shift); // clear SMPx
        ADC1->SMPR2 |=  (0x7u << shift); // SMPx=111 -> 239.5 cycles
    } else {
        uint8_t shift = (ch - 10u) * 3u;
        ADC1->SMPR1 &= ~(0x7u << shift);
        ADC1->SMPR1 |=  (0x7u << shift);
    }

    // 1 lần chuyển đổi, kênh duy nhất
    ADC1->SQR1 &= ~(0xFu << 20);          // L=0 -> 1 conversion
    ADC1->SQR3  = (ch & 0x1Fu);           // SQ1 = ch

    // Bắt đầu chuyển đổi bằng SWSTART
    ADC1->CR2  |= ADC_CR2_EXTTRIG | ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC)){}     // đợi xong
    return (uint16_t)ADC1->DR;
}

// --- Khởi tạo ADC1: bật clock, analog mode PA1/PA2, hiệu chuẩn ---
static void ADC1_Init(void)
{
    // Bật clock GPIOA và ADC1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // Chia clock ADC = PCLK2/6 (đảm bảo trong dải cho ADC)
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |=  RCC_CFGR_ADCPRE_DIV6;

    // PA1 (ADC1_IN1), PA2 (ADC1_IN2) -> analog input
    GPIOA->CRL &= ~(0xFu << (1*4));   // PA1 analog
    GPIOA->CRL &= ~(0xFu << (2*4));   // PA2 analog

    // Bật ADC và hiệu chuẩn
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_cycles(1000);

    ADC1->CR2 |= ADC_CR2_RSTCAL;      // reset calibration
    while(ADC1->CR2 & ADC_CR2_RSTCAL){}
    ADC1->CR2 |= ADC_CR2_CAL;         // start calibration
    while(ADC1->CR2 & ADC_CR2_CAL){}
    ADC1->CR2 |= ADC_CR2_ADON;        // bật lại ADC

    // Kết quả canh phải 12-bit
    ADC1->CR2 &= ~ADC_CR2_ALIGN;

    // (Tuỳ chọn) Bật buffer Vrefint nếu bạn muốn đo/giám sát nội bộ:
    // ADC1->CR2 |= ADC_CR2_TSVREFE; // cho phép kênh nhiệt độ/Vrefint (F1 có tuỳ biến thể)
}

// --- (Tùy chọn) Xuất thử ra PortB để quan sát nhanh ---
static void GPIOB_Init_Output(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL = 0x33333333;                // PB0..PB7: push-pull 50 MHz
    GPIOB->CRH &= ~(0xFFFFu);               // PB8..PB11: clear
    GPIOB->CRH |=  (0x3333u);               // PB8..PB11: push-pull 50 MHz
}

volatile uint16_t mq4_raw = 0, mq7_raw = 0; // giá trị thô ADC
volatile uint16_t mq4_mv  = 0, mq7_mv  = 0; // Vout (mV) sau quy đổi

int main(void)
{
    ADC1_Init();
    GPIOB_Init_Output(); // demo quan sát nhanh

    while (1)
    {
        // MQ-4 -> PA1 (ADC1_IN1), MQ-7 -> PA2 (ADC1_IN2)
        mq4_raw = ADC1_ReadChannel(1);
        mq7_raw = ADC1_ReadChannel(2);

        // Quy đổi sang mV theo VREF = 3.300 V:
        // làm tròn: (raw * 3300 + 2047) / 4095
        mq4_mv  = (uint16_t)(((uint32_t)mq4_raw * VREF_MV + (ADC_MAX/2)) / ADC_MAX);
        mq7_mv  = (uint16_t)(((uint32_t)mq7_raw * VREF_MV + (ADC_MAX/2)) / ADC_MAX);

        // Ví dụ: xuất mq7_mv ra ODR để nhìn nhanh bằng LA/LED (demo)
        GPIOB->ODR = mq7_mv;

        // Ghi chú: Đây là Vout (mV) từ cảm biến.
        // Chưa phải nồng độ ppm. Muốn ra ppm -> cần hiệu chuẩn Rs/R0.
    }
}
