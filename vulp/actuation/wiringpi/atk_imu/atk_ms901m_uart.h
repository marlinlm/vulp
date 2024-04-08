#pragma once

#include <cstdint>

#include "wiring_uart.h"

#ifndef __ATK_MS901M_UART_H
#define __ATK_MS901M_UART_H

#define ATK_MS901M_UART_TX_GPIO_PORT            GPIOB
#define ATK_MS901M_UART_TX_GPIO_PIN             GPIO_PIN_10
#define ATK_MS901M_UART_TX_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)     

#define ATK_MS901M_UART_RX_GPIO_PORT            GPIOB
#define ATK_MS901M_UART_RX_GPIO_PIN             GPIO_PIN_11
#define ATK_MS901M_UART_RX_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    

#define ATK_MS901M_UART_INTERFACE               USART3
#define ATK_MS901M_UART_IRQn                    USART3_IRQn
#define ATK_MS901M_UART_IRQHandler              USART3_IRQHandler
#define ATK_MS901M_UART_CLK_ENABLE()            do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)   

#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128

namespace vulp::actuation::wiringpi {

struct Buffer
{
    uint8_t buf[ATK_MS901M_UART_RX_FIFO_BUF_SIZE];  
    uint16_t size;                                  
    uint16_t reader;                                
    uint16_t writer;                                
};

class BufferedUartAdapter{
public:
    BufferedUartAdapter(string dev, int baud):
        uart_adapter_(new UartAdapter(dev, baud, [&](uint8_t* dat, uint16_t len){
            atk_ms901m_uart_rx_fifo_write(dat, len);
        }))
    {
        buffer_.size = ATK_MS901M_UART_RX_FIFO_BUF_SIZE;         
        buffer_.reader = 0;                                      
        buffer_.writer = 0;            
        uart_adapter_->reset();
    };
    ~BufferedUartAdapter() = default;
    uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);  
    uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);  
    void atk_ms901m_rx_fifo_flush(void);                                
    void atk_ms901m_uart_send(uint8_t *dat, uint16_t len);               
private:
    
    UartAdapter* uart_adapter_;
    Buffer buffer_;                     
};

}

#endif