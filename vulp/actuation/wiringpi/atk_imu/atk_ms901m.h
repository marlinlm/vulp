#pragma once

// #ifndef __ATM_MS901M_H
// #define __ATM_MS901M_H

#include <cstdint>

#include "wiring_uart.h"

#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28

#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01    
#define ATK_MS901M_FRAME_ID_QUAT            0x02    
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03    
#define ATK_MS901M_FRAME_ID_MAG             0x04    
#define ATK_MS901M_FRAME_ID_BARO            0x05    
#define ATK_MS901M_FRAME_ID_PORT            0x06    

#define ATK_MS901M_FRAME_ID_REG_SAVE        0x00    
#define ATK_MS901M_FRAME_ID_REG_SENCAL      0x01    
#define ATK_MS901M_FRAME_ID_REG_SENSTA      0x02    
#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03    
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04    
#define ATK_MS901M_FRAME_ID_REG_GYROBW      0x05    
#define ATK_MS901M_FRAME_ID_REG_ACCBW       0x06    
#define ATK_MS901M_FRAME_ID_REG_BAUD        0x07    
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08    
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2  0x09    
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A    
#define ATK_MS901M_FRAME_ID_REG_ALG         0x0B    
#define ATK_MS901M_FRAME_ID_REG_ASM         0x0C    
#define ATK_MS901M_FRAME_ID_REG_GAUCAL      0x0D    
#define ATK_MS901M_FRAME_ID_REG_BAUCAL      0x0E    
#define ATK_MS901M_FRAME_ID_REG_LEDOFF      0x0F    
#define ATK_MS901M_FRAME_ID_REG_D0MODE      0x10    
#define ATK_MS901M_FRAME_ID_REG_D1MODE      0x11    
#define ATK_MS901M_FRAME_ID_REG_D2MODE      0x12    
#define ATK_MS901M_FRAME_ID_REG_D3MODE      0x13    
#define ATK_MS901M_FRAME_ID_REG_D1PULSE     0x16    
#define ATK_MS901M_FRAME_ID_REG_D3PULSE     0x1A    
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD    0x1F    
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD    0x23    
#define ATK_MS901M_FRAME_ID_REG_RESET       0x7F    

#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD     0       
#define ATK_MS901M_FRAME_ID_TYPE_ACK        1       


namespace vulp::actuation::wiringpi {

typedef struct
{
    float roll;                                     
    float pitch;                                    
    float yaw;                                      
} atk_ms901m_attitude_data_t;

typedef struct
{
    float q0;                                       /* Q0 */
    float q1;                                       /* Q1 */
    float q2;                                       /* Q2 */
    float q3;                                       /* Q3 */
} atk_ms901m_quaternion_data_t;

typedef struct
{
    struct
    {
        int16_t x;                                  
        int16_t y;                                  
        int16_t z;                                  
    } raw;
    float x;                                        
    float y;                                        
    float z;                                        
} atk_ms901m_gyro_data_t;

typedef struct
{
    struct
    {
        int16_t x;                                  
        int16_t y;                                  
        int16_t z;                                  
    } raw;
    float x;                                        
    float y;                                        
    float z;                                        
} atk_ms901m_accelerometer_data_t;

typedef struct
{
    int16_t x;                                      
    int16_t y;                                      
    int16_t z;                                      
    float temperature;                              
} atk_ms901m_magnetometer_data_t;

typedef struct
{
    int32_t pressure;                               
    int32_t altitude;                               
    float temperature;                              
} atk_ms901m_barometer_data_t;

typedef struct
{
    uint16_t d0;                                    
    uint16_t d1;                                    
    uint16_t d2;                                    
    uint16_t d3;                                    
} atk_ms901m_port_data_t;

typedef enum
{
    ATK_MS901M_LED_STATE_ON  = 0x00,                
    ATK_MS901M_LED_STATE_OFF = 0x01,                
} atk_ms901m_led_state_t;

typedef enum
{
    ATK_MS901M_PORT_D0 = 0x00,                      
    ATK_MS901M_PORT_D1 = 0x01,                      
    ATK_MS901M_PORT_D2 = 0x02,                      
    ATK_MS901M_PORT_D3 = 0x03,                      
} atk_ms901m_port_t;

typedef enum
{
    ATK_MS901M_PORT_MODE_ANALOG_INPUT   = 0x00,     
    ATK_MS901M_PORT_MODE_INPUT          = 0x01,     
    ATK_MS901M_PORT_MODE_OUTPUT_HIGH    = 0x02,     
    ATK_MS901M_PORT_MODE_OUTPUT_LOW     = 0x03,     
    ATK_MS901M_PORT_MODE_OUTPUT_PWM     = 0x04,     
} atk_ms901m_port_mode_t;


typedef struct
{
    uint8_t head_l;                                 
    uint8_t head_h;                                 
    uint8_t id;                                     
    uint8_t len;                                    
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];     
    uint8_t check_sum;                              
} atk_ms901m_frame_t;                               

typedef enum
{
    wait_for_head_l = 0x00,                         
    wait_for_head_h = 0x01,                         
    wait_for_id     = 0x02,                         
    wait_for_len    = 0x04,                         
    wait_for_dat    = 0x08,                         
    wait_for_sum    = 0x16,                         
} atk_ms901m_handle_state_t;                        

static const uint16_t g_atk_ms901m_gyro_fsr_table[4] = {250, 500, 1000, 2000};
static const uint8_t g_atk_ms901m_accelerometer_fsr_table[4] = {2, 4, 8, 16};

static struct
{
    uint8_t gyro;                                   
    uint8_t accelerometer;                          
} g_atk_ms901m_fsr;                                 


struct read_reg_lock{
    uint8_t id=0;
    uint8_t dat[64];
    uint8_t len=0;
};


#define ATK_MS901M_EOK      0                       
#define ATK_MS901M_ERROR    1                       
#define ATK_MS901M_EINVAL   2                       
#define ATK_MS901M_ETIMEOUT 3                       
#define ATK_MS901M_EWAIT    4

class AtkImu901{
private:
    UartAdapter* uart_;
    int uart_status_;

    atk_ms901m_attitude_data_t* attitude_data_;
    atk_ms901m_quaternion_data_t* quaternion_data_;
    atk_ms901m_gyro_data_t* gyro_data_;
    atk_ms901m_accelerometer_data_t* accelerometer_data_;
    atk_ms901m_magnetometer_data_t* magnetometer_data_;
    atk_ms901m_barometer_data_t* barometer_data_;

    uint8_t dat_;
    atk_ms901m_handle_state_t handle_state_ = wait_for_head_l;
    uint8_t dat_index_ = 0;
    uint8_t id_type_ = 0;
    atk_ms901m_frame_t *frame_;
    
    read_reg_lock* reg_read_lock_;

    uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat);                                                                      
    uint8_t atk_ms901m_get_attitude(atk_ms901m_frame_t *frame);                                                        
    uint8_t atk_ms901m_get_quaternion(atk_ms901m_frame_t *frame);                                                  
    uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_frame_t *frame);  
    uint8_t atk_ms901m_get_magnetometer(atk_ms901m_frame_t *frame);                                            
    uint8_t atk_ms901m_get_barometer(atk_ms901m_frame_t *frame);                                                     
    void atk_ms901m_send(uint8_t *dat, uint8_t len);
    void on_frame(atk_ms901m_frame_t *frame);
    uint8_t on_message(uint8_t dat);

public:
    AtkImu901(string dev, int baud):
        attitude_data_(new atk_ms901m_attitude_data_t()),
        quaternion_data_(new atk_ms901m_quaternion_data_t()),
        gyro_data_(new atk_ms901m_gyro_data_t()),
        accelerometer_data_(new atk_ms901m_accelerometer_data_t()),
        magnetometer_data_(new atk_ms901m_magnetometer_data_t()),
        barometer_data_(new atk_ms901m_barometer_data_t()),
        
        frame_(new atk_ms901m_frame_t()),
        reg_read_lock_(NULL)
    {   
        uart_ = new UartAdapter(dev, baud, [&](uint8_t* dat, uint16_t len){
            for (uint16_t i=0; i<len; i++){
                uint8_t resp = on_message(dat[i]);
                switch(resp){
                    case ATK_MS901M_EOK:
                    {
                        atk_ms901m_frame_t *out_frame = frame_;
                        frame_ = new atk_ms901m_frame_t();
                        on_frame(out_frame);
                        delete out_frame;
                        break;
                    }
                    case ATK_MS901M_EINVAL:
                    {
                        delete frame_;
                        frame_ = new atk_ms901m_frame_t();
                        break;
                    }
                    case ATK_MS901M_EWAIT:
                    {
                        break;
                    }
                    default:
                    {
                        throw std::logic_error("Unexpacted uart message handler response. " + std::to_string(resp));
                    }
                }
            }
        });

        
        uart_->reset();

        uint8_t ret = atk_ms901m_read_reg_by_id(ATK_MS901M_FRAME_ID_REG_GYROFSR, &g_atk_ms901m_fsr.gyro);
        if (ret == 0)
        {
            uart_status_ = ATK_MS901M_ERROR;
            spdlog::error("error getting gyro fsr");
            throw std::logic_error("error getting gyro fsr");
        }
        
        ret = atk_ms901m_read_reg_by_id(ATK_MS901M_FRAME_ID_REG_ACCFSR, &g_atk_ms901m_fsr.accelerometer);
        if (ret == 0)
        {
            uart_status_ = ATK_MS901M_ERROR;
            spdlog::error("error getting accelerometer fsr");
            throw std::logic_error("error getting accelerometer fsr");
        }
        
        uart_status_ = ATK_MS901M_EOK;
        spdlog::info("initialzed atk imu901 uart {}", uart_status_);
    }

    atk_ms901m_attitude_data_t* atk_ms901m_get_attitude_data(){return attitude_data_;};                                                        
    atk_ms901m_quaternion_data_t* atk_ms901m_get_quaternion_data(){return quaternion_data_;};
    atk_ms901m_gyro_data_t* atk_ms901m_get_gyrometer_data(){return gyro_data_;};
    atk_ms901m_accelerometer_data_t* atk_ms901m_get_accelerometer_data(){return accelerometer_data_;};  
    atk_ms901m_magnetometer_data_t* atk_ms901m_get_magnetometer_data(){return magnetometer_data_;};                                            
    atk_ms901m_barometer_data_t* atk_ms901m_get_barometer_data(){return barometer_data_;}; 

};
}
// #endif
