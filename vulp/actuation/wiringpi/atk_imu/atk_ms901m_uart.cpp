#include "atk_ms901m_uart.h"
#include "wiring_uart.h"
#include <spdlog/spdlog.h>

namespace vulp::actuation::wiringpi {

uint8_t BufferedUartAdapter::atk_ms901m_uart_rx_fifo_write(uint8_t* dat, uint16_t len)
{
    for (uint16_t i=0; i<len; i++)
    {
        buffer_.buf[buffer_.writer] = dat[i];
        buffer_.writer = (buffer_.writer + 1) % buffer_.size;
    }
    
    return 0;
}

uint16_t BufferedUartAdapter::atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len)
{
    uint16_t fifo_usage;
    

    if (buffer_.writer >= buffer_.reader)
    {
        fifo_usage = buffer_.writer - buffer_.reader;
    }
    else
    {
        fifo_usage = buffer_.size - buffer_.reader + buffer_.writer;
    }
    
    if (len > fifo_usage)
    {
        len = fifo_usage;
    }
    
    for (uint16_t i=0; i<len; i++)
    {
        dat[i] = buffer_.buf[buffer_.reader];
        buffer_.reader = (buffer_.reader + 1) % buffer_.size;
    }
    
    return len;
}


void BufferedUartAdapter::atk_ms901m_rx_fifo_flush(void)
{
    buffer_.writer = buffer_.reader;
}


void BufferedUartAdapter::atk_ms901m_uart_send(uint8_t *dat, uint16_t len)
{
    for(int i = 0; i<len; i++){
        uart_adapter_->send_char(dat[i]);
    }
}


}