#include "ws2812.h"

// WS2812 通过 SPI 方式“编码后吐波形”
#define WS2812_LowLevel  0xC0
#define WS2812_HighLevel 0xF0

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;

    // GRB 顺序：每个 bit 用 1 字节的波形近似来生成 T0H/T1H
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }

    // 发送一个空字节（保持线性/对齐）
    HAL_SPI_Transmit(&hspi6, &res, 0, 0xFFFF);

    // 发送 24 bytes：G(8)+R(8)+B(8)
    HAL_SPI_Transmit(&hspi6, txbuf, 24, 0xFFFF);

    // 复位/延时：持续低电平
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&hspi6, &res, 1, 0xFFFF);
    }
}

