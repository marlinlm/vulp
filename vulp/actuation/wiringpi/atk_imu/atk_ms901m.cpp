#include <string>
#include <cstddef>
#include <spdlog/spdlog.h>

#include "atk_ms901m.h"
#include "vulp/actuation/wiringpi/wiringPi.h"

#define ATK_MS901M_FRAME_HEAD_L             0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H      0x55    
#define ATK_MS901M_FRAME_HEAD_ACK_H         0xAF    

#define ATK_MS901M_READ_REG_ID(id)         (id | 0x80)
#define ATK_MS901M_WRITE_REG_ID(id)        (id)


namespace vulp::actuation::wiringpi {

void AtkImu901::atk_ms901m_send(uint8_t *dat, uint8_t len)
{
    for(int i = 0; i<len; i++){
        uart_->send_char(dat[i]);
    }
}

void AtkImu901::on_frame(atk_ms901m_frame_t *frame){
    if(id_type_ == ATK_MS901M_FRAME_ID_TYPE_UPLOAD){
        switch(frame->id){
            case ATK_MS901M_FRAME_ID_ATTITUDE:{
                atk_ms901m_get_attitude(frame);
                break;
            }
            case ATK_MS901M_FRAME_ID_QUAT:{
                atk_ms901m_get_quaternion(frame);
                break;
            }
            case ATK_MS901M_FRAME_ID_GYRO_ACCE:{
                atk_ms901m_get_gyro_accelerometer(frame);
                break;
            }
            case ATK_MS901M_FRAME_ID_MAG:{
                atk_ms901m_get_magnetometer(frame);
                break;
            }
            case ATK_MS901M_FRAME_ID_BARO:{
                atk_ms901m_get_barometer(frame);
                break;
            }
            default:{
                // spdlog::warn("unsupported frame id {}", frame_->id);
                break;
            }
        }
    }else if(id_type_ == ATK_MS901M_FRAME_ID_TYPE_ACK){
        spdlog::info("read reg ack");
        if(reg_read_lock_ != NULL ){
            if(reg_read_lock_->id == frame->id){
                for(uint8_t i = 0; i< frame->len; i++){
                    reg_read_lock_->dat[i] = frame->dat[i];
                }
                reg_read_lock_->len = frame->len;
                spdlog::info("copied {} bytes to reg lock", frame->len);
            }else{
                spdlog::warn("frame id not match (lock id:frame id) {}:{}", reg_read_lock_->id, frame->id);
            }
        }else{
            spdlog::warn("no reg read lock available.");
        }
    }else{
        spdlog::info("unsupported frame type {}", id_type_);
    }
}

uint8_t AtkImu901::on_message(uint8_t message)
{
    dat_ = message;
    
    switch (handle_state_)
    {
        case wait_for_head_l:
        {
            if (dat_ == ATK_MS901M_FRAME_HEAD_L)
            {
                frame_->head_l = dat_;
                frame_->check_sum = frame_->head_l;
                handle_state_ = wait_for_head_h;
            }
            else
            {
                handle_state_ = wait_for_head_l;
            }
            break;
        }
        case wait_for_head_h:
        {
            switch (dat_)
            {
                case ATK_MS901M_FRAME_HEAD_UPLOAD_H:
                {
                    id_type_ = ATK_MS901M_FRAME_ID_TYPE_UPLOAD;
                    frame_->head_h = dat_;
                    frame_->check_sum += frame_->head_h;
                    handle_state_ = wait_for_id;
                    break;
                }
                case ATK_MS901M_FRAME_HEAD_ACK_H:
                {
                    id_type_ = ATK_MS901M_FRAME_ID_TYPE_ACK;
                    frame_->head_h = dat_;
                    frame_->check_sum += frame_->head_h;
                    handle_state_ = wait_for_id;
                    break;
                }
                default:
                {
                    handle_state_ = wait_for_head_l;
                    return ATK_MS901M_EINVAL;
                }
            }
            break;
        }
        case wait_for_id:
        {
            frame_->id = dat_;
            frame_->check_sum += frame_->id;
            handle_state_ = wait_for_len;
            break;
        }
        case wait_for_len:
        {
            if (dat_ > ATK_MS901M_FRAME_DAT_MAX_SIZE)
            {
                handle_state_ = wait_for_head_l;
                return ATK_MS901M_EINVAL;
            }
            else
            {
                frame_->len = dat_;
                frame_->check_sum += frame_->len;
                if (frame_->len == 0)
                {
                    handle_state_ = wait_for_sum;
                }
                else
                {
                    handle_state_ = wait_for_dat;
                }
            }
            break;
        }
        case wait_for_dat:
        {
            frame_->dat[dat_index_] = dat_;
            frame_->check_sum += frame_->dat[dat_index_];
            dat_index_++;
            if (dat_index_ == frame_->len)
            {
                dat_index_ = 0;
                handle_state_ = wait_for_sum;
            }
            break;
        }
        case wait_for_sum:
        {
            if (dat_ == frame_->check_sum)
            {
                handle_state_ = wait_for_head_l;
                return ATK_MS901M_EOK;
            }
            handle_state_ = wait_for_head_l;
            return ATK_MS901M_EINVAL;
        }
        default:
        {
            handle_state_ = wait_for_head_l;
            return ATK_MS901M_EINVAL;;
        }
    }
    
    return ATK_MS901M_EWAIT;
}


uint8_t AtkImu901::atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat)
{
    uint8_t buf[7];
    uint16_t time_out = 10000;
    
    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];

    // not thread safe, this method can not be invoded multi-threadly.
    if(reg_read_lock_ != NULL){
        delete reg_read_lock_;
    }
    reg_read_lock_ = new read_reg_lock();
    reg_read_lock_->id = id;
    atk_ms901m_send(buf, 6);
    while(1){
        if(time_out-- <= 0){
            return 0;
        }
        if(reg_read_lock_->len > 0){
            break;
        }
        delayMicroseconds(1);
    }

    for (uint8_t dat_index=0; dat_index<reg_read_lock_->len; dat_index++)
    {
        dat[dat_index] = reg_read_lock_->dat[dat_index];
    }
    
    return reg_read_lock_->len;
}

uint8_t AtkImu901::atk_ms901m_get_attitude(atk_ms901m_frame_t *frame)
{
    attitude_data_->roll = (float)((int16_t)(frame->dat[1] << 8) | frame->dat[0]) / 32768 * 180;
    attitude_data_->pitch = (float)((int16_t)(frame->dat[3] << 8) | frame->dat[2]) / 32768 * 180;
    attitude_data_->yaw = (float)((int16_t)(frame->dat[5] << 8) | frame->dat[4]) / 32768 * 180;
    spdlog::debug("attitude data: {}, {}, {}",  attitude_data_->roll, attitude_data_->pitch, attitude_data_->yaw);
    return ATK_MS901M_EOK;
}

uint8_t AtkImu901::atk_ms901m_get_quaternion(atk_ms901m_frame_t *frame)
{
    quaternion_data_->q0 = (float)((int16_t)(frame->dat[1] << 8) | frame->dat[0]) / 32768;
    quaternion_data_->q1 = (float)((int16_t)(frame->dat[3] << 8) | frame->dat[2]) / 32768;
    quaternion_data_->q2 = (float)((int16_t)(frame->dat[5] << 8) | frame->dat[4]) / 32768;
    quaternion_data_->q3 = (float)((int16_t)(frame->dat[7] << 8) | frame->dat[6]) / 32768;
    
    spdlog::debug("quaternion data: {}, {}, {}, {}",  quaternion_data_->q0, quaternion_data_->q1, quaternion_data_->q2, quaternion_data_->q3);
    return ATK_MS901M_EOK;
}

uint8_t AtkImu901::atk_ms901m_get_gyro_accelerometer(atk_ms901m_frame_t *frame)
{
    gyro_data_->raw.x = (int16_t)(frame->dat[7] << 8) | frame->dat[6];
    gyro_data_->raw.y = (int16_t)(frame->dat[9] << 8) | frame->dat[8];
    gyro_data_->raw.z = (int16_t)(frame->dat[11] << 8) | frame->dat[10];
    gyro_data_->x = (float)gyro_data_->raw.x / 32768 * g_atk_ms901m_gyro_fsr_table[g_atk_ms901m_fsr.gyro];
    gyro_data_->y = (float)gyro_data_->raw.y / 32768 * g_atk_ms901m_gyro_fsr_table[g_atk_ms901m_fsr.gyro];
    gyro_data_->z = (float)gyro_data_->raw.z / 32768 * g_atk_ms901m_gyro_fsr_table[g_atk_ms901m_fsr.gyro];

    accelerometer_data_->raw.x = (int16_t)(frame->dat[1] << 8) | frame->dat[0];
    accelerometer_data_->raw.y = (int16_t)(frame->dat[3] << 8) | frame->dat[2];
    accelerometer_data_->raw.z = (int16_t)(frame->dat[5] << 8) | frame->dat[4];
    accelerometer_data_->x = (float)accelerometer_data_->raw.x / 32768 * g_atk_ms901m_accelerometer_fsr_table[g_atk_ms901m_fsr.accelerometer];
    accelerometer_data_->y = (float)accelerometer_data_->raw.y / 32768 * g_atk_ms901m_accelerometer_fsr_table[g_atk_ms901m_fsr.accelerometer];
    accelerometer_data_->z = (float)accelerometer_data_->raw.z / 32768 * g_atk_ms901m_accelerometer_fsr_table[g_atk_ms901m_fsr.accelerometer];
    
    
    spdlog::debug("gyro data: {}, {}, {}",  gyro_data_->x, gyro_data_->y, gyro_data_->z);
    spdlog::debug("accelerometer data: {}, {}, {}",  accelerometer_data_->x, accelerometer_data_->y, accelerometer_data_->z);
    return ATK_MS901M_EOK;
}

uint8_t AtkImu901::atk_ms901m_get_magnetometer(atk_ms901m_frame_t *frame)
{
    magnetometer_data_->x = (int16_t)(frame->dat[1] << 8) | frame->dat[0];
    magnetometer_data_->y = (int16_t)(frame->dat[3] << 8) | frame->dat[2];
    magnetometer_data_->z = (int16_t)(frame->dat[5] << 8) | frame->dat[4];
    magnetometer_data_->temperature = (float)((int16_t)(frame->dat[7] << 8) | frame->dat[6]) / 100;
    spdlog::debug("magnetometer data: {}, {}, {}, {}",  magnetometer_data_->x, magnetometer_data_->y, magnetometer_data_->z, magnetometer_data_->temperature);
    return ATK_MS901M_EOK;
}

uint8_t AtkImu901::atk_ms901m_get_barometer(atk_ms901m_frame_t *frame)
{
    barometer_data_->pressure = (int32_t)(frame->dat[3] << 24) | (frame->dat[2] << 16) | (frame->dat[1] << 8) | frame->dat[0];
    barometer_data_->altitude = (int32_t)(frame->dat[7] << 24) | (frame->dat[6] << 16) | (frame->dat[5] << 8) | frame->dat[4];
    barometer_data_->temperature = (float)((int16_t)(frame->dat[9] << 8) | frame->dat[8]) / 100;
    spdlog::debug("barometer data: {}, {}, {}",  barometer_data_->pressure, barometer_data_->altitude, barometer_data_->temperature);
    return ATK_MS901M_EOK;
}


}