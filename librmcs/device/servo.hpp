#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>

namespace librmcs::device {

class Servo {
public:
    enum class  Type : uint8_t{DG995,MG995,DG995_ANGLE};
    struct Servo_enable {
        explicit Servo_enable(Type servo_type) {
            this->servo_zero_point=0.0;
            this->servo_type = servo_type;
            this->reversed = false;
            switch (servo_type) {
            case Type::DG995:
            case Type::DG995_ANGLE: max_angle = 180.0; break;
            case Type::MG995: max_angle=360.0;break;
            }
    };
        Servo_enable& set_angle(double value) { return   max_angle= value, *this; }
        Servo_enable& set_servo_zero_point(double value){ return servo_zero_point = value,*this;}
        Servo_enable& set_reversed_angle(double value){return max_angle=value,*this;}
        Type servo_type;
        double servo_zero_point;
        double max_angle;
        bool reversed;
        double angle;
};
    Servo()
    : set_zero_point_(150.0)
    , angle_(0.0)
    , velocity_(0.0)
    ,palus_to_angle_coefficient(0.0)
   ,sign_(1.0)

    {}
    explicit Servo(const Servo_enable& servo_enable)
    : angle_(0.0)
    , velocity_(0.0){
    configure(servo_enable);
    }
// 禁止拷贝和赋值，保持资源的独立性
    Servo(const Servo&) = delete;
    Servo& operator= (const Servo&)=delete;
/**
     * @brief 核心配置函数：推导绝对零位和转换系数
     */
    void configure(const Servo_enable& servo_enable){
        // 0度对应50， 180度对应250
        // 1. 确定正反向符号
        sign_ = servo_enable.reversed ?-1.0:1.0;
        // 2. 计算 弧度 到 PWM脉宽（50~250）的转换系数
        // 系数 = 总脉宽变化量 / 舵机最大物理弧度
        palus_to_angle_coefficient =palus_distance/servo_enable.max_angle;
         if (servo_enable.reversed) {
            // 反向
            set_zero_point_ =250.0 -(servo_enable.servo_zero_point*palus_to_angle_coefficient);
         }
         else {
            // 正向
         set_zero_point_ =50.0+(servo_enable.servo_zero_point*palus_to_angle_coefficient);
         }   
         set_zero_point_ = std::clamp(set_zero_point_, min_palus_, max_palus_);
    }
/**
     * @brief
     */
    template <typename Func>
    void send_data(double target_relative_angle, Func&& pwm_write_func) {
        // 1. 安全检查
        if (std::isnan(target_relative_angle)) {
            pwm_write_func(static_cast<uint16_t>(set_zero_point_));
            return;
        }

        // 2. 原来的数学核心计算
        double target_pulse = set_zero_point_ + (target_relative_angle *palus_to_angle_coefficient * sign_);
        target_pulse = std::clamp(target_pulse, min_palus_, max_palus_);

        // 3. 更新状态
        angle_ = target_relative_angle;
        velocity_ = 0.0; 

        // 4. 执行输出
        pwm_write_func(static_cast<uint16_t>(std::round(target_pulse)));
    }
    /**
     * @brief 接收 CAN 信号（4字节 float）并联动触发 PWM 发送
     */
    template <typename Func>
    void handle_switch_cmd(uint8_t can_rx_byte, Func&& pwm_write_func) {
        double target_degree = 0.0;
        
        if (can_rx_byte == 0x01) {
            target_degree = 180.0; // 对应下位机转动到 180 度
        } else if (can_rx_byte == 0x00) {
            target_degree = 0.0;   // 对应下位机转动到 0 度
        } else {
            return; // 过滤非法指令，直接拦截
        }

        // 调用内部角度控制函数
        send_data(target_degree, std::forward<Func>(pwm_write_func));
    }
        double angle() const{return angle_;}
        double max_palus() const{return max_palus_;}
        double min_palus() const{return min_palus_;}
private:

    double set_zero_point_;
    double angle_;
    double velocity_;
    double palus_distance=200.0;
    double palus_to_angle_coefficient;
    double sign_;
    double max_palus_ =250.0;
    double min_palus_ =50.0;
};


}
