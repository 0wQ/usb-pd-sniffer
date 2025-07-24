#include "led_strip.h"

static struct {
#if LED_STRIP_ORDER == LED_STRIP_RGB
    uint8_t red;
    uint8_t green;
    uint8_t blue;
#elif LED_STRIP_ORDER == LED_STRIP_GRB
    uint8_t green;
    uint8_t red;
    uint8_t blue;
#elif LED_STRIP_ORDER == LED_STRIP_BRG
    uint8_t blue;
    uint8_t red;
    uint8_t green;
#else
#error "Invalid LED_STRIP_ORDER"
#endif
} led_buffer[LED_STRIP_LENGTH];

static bool is_spi_initialized = false;

static void spi_init(void) {
    is_spi_initialized = true;

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef SPI_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  // MOSI
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_SSOutputCmd(SPI1, ENABLE);
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
}

static void led_strip_send_byte(uint8_t data) {
    if (!is_spi_initialized) return;

    uint8_t i;
    for (i = 8; i; i--, data <<= 1) {            // 8 bits, MSB first
        while (!(SPI1->STATR & SPI_STATR_TXE));  // wait for transmit buffer empty

        if (data & 0x80) {
            SPI1->DATAR = 0x7c;  // 833us high for "1"-bit
        } else {
            SPI1->DATAR = 0x60;  // 333us high for "0"-bit
        }
    }
}

void led_strip_init(void) {
    if (!is_spi_initialized) {
        spi_init();
    }

    for (uint8_t i = 0; i < 100; i++) {
        while (!(SPI1->STATR & SPI_STATR_TXE));
        SPI1->DATAR = 0x00;
    }

    led_strip_clear();
    led_strip_refresh();
}

void led_strip_clear(void) {
    memset(led_buffer, 0, sizeof(led_buffer));
}

void led_strip_refresh(void) {
    for (uint16_t i = 0; i < LED_STRIP_LENGTH; i++) {
        led_strip_send_byte(((uint8_t*)&led_buffer[i])[0]);
        led_strip_send_byte(((uint8_t*)&led_buffer[i])[1]);
        led_strip_send_byte(((uint8_t*)&led_buffer[i])[2]);
    }

    for (int i = 0; i < 10; i++) {
        while (!(SPI1->STATR & SPI_STATR_TXE));
        SPI1->DATAR = 0x00;
    }

    Delay_Us(85);
}

void led_strip_set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (index >= LED_STRIP_LENGTH) return;

    red = (uint8_t)((float)red * 0.6f);
    green = (uint8_t)((float)green * 0.4f);
    blue = (uint8_t)((float)blue * 1.0f);

#if LED_STRIP_ORDER == LED_STRIP_RGB
    led_buffer[index].red = red;
    led_buffer[index].green = green;
    led_buffer[index].blue = blue;
#elif LED_STRIP_ORDER == LED_STRIP_GRB
    led_buffer[index].green = green;
    led_buffer[index].red = red;
    led_buffer[index].blue = blue;
#elif LED_STRIP_ORDER == LED_STRIP_BRG
    led_buffer[index].blue = blue;
    led_buffer[index].red = red;
    led_buffer[index].green = green;
#endif
}

void led_strip_set_pixel_with_refresh(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    led_strip_set_pixel(index, red, green, blue);
    led_strip_refresh();
}

void led_strip_set_pixel_hsv(uint16_t index, uint16_t hue, uint8_t saturation, uint8_t value) {
    // HSV: hue(0-359), saturation(0-255), value(0-255)
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;

    uint32_t rgb_max = value;
    uint32_t rgb_min = rgb_max * (255 - saturation) / 255.0f;

    uint32_t i = hue / 60;
    uint32_t diff = hue % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
        case 0:
            red = rgb_max;
            green = rgb_min + rgb_adj;
            blue = rgb_min;
            break;
        case 1:
            red = rgb_max - rgb_adj;
            green = rgb_max;
            blue = rgb_min;
            break;
        case 2:
            red = rgb_min;
            green = rgb_max;
            blue = rgb_min + rgb_adj;
            break;
        case 3:
            red = rgb_min;
            green = rgb_max - rgb_adj;
            blue = rgb_max;
            break;
        case 4:
            red = rgb_min + rgb_adj;
            green = rgb_min;
            blue = rgb_max;
            break;
        default:
            red = rgb_max;
            green = rgb_min;
            blue = rgb_max - rgb_adj;
            break;
    }

    led_strip_set_pixel(index, red, green, blue);
}