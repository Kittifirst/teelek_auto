#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>

#include <esp32_Encoder.h> 

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
    #define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
    #define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)
    
    //------------------------------ < Define > -------------------------------------//
#ifdef teelek_karake
    rcl_publisher_t debug_cmd_vel_publisher;
    rcl_subscription_t cmd_vel_subscriber;

    geometry_msgs__msg__Twist debug_wheel_motor_msg;
    geometry_msgs__msg__Twist cmd_vel_msg;

    rcl_publisher_t debug_encoder_wheels_publisher;
    std_msgs__msg__Float32MultiArray debug_encoder_wheels_msg;

    rcl_subscription_t cmd_resetencoder_subscriber;
    geometry_msgs__msg__Twist cmd_resetencoder_msg;
#elif teelek_katsu
 
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
static unsigned long last_sync = 0;

// Encoder wheels
float wheel_ticks[4] = {0};   // LF, LB, RF, RB
long lastLF = 0;
long lastLB = 0;
long lastRF = 0;
long lastRB = 0;

long offsetLF = 0, offsetLB = 0, offsetRF = 0, offsetRB = 0;
long totalLF = 0, totalLB = 0, totalRF = 0, totalRB = 0; // ✅ tick สะสม

// PIDF(Kp, Ki, Kd, Kf, dt, i_min, i_max, min_val, max_val)

PIDF pidLF(2.0, 0.1, 0.01, 0, 0.05, -500, 500, -1023, 1023);
PIDF pidRF(2.0, 0.1, 0.01, 0, 0.05, -500, 500, -1023, 1023);
PIDF pidLB(2.0, 0.1, 0.01, 0, 0.05, -500, 500, -1023, 1023);
PIDF pidRB(2.0, 0.1, 0.01, 0, 0.05, -500, 500, -1023, 1023);

float targetSpeedLF = 0, targetSpeedRF = 0, targetSpeedLB = 0, targetSpeedRB = 0;
long prevLF = 0, prevLB = 0, prevRF = 0, prevRB = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

#ifdef teelek_karake
    Controller motor_LF(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_LF_INV, MOTOR_LF_BRAKE, MOTOR_LF_PWM, MOTOR_LF_IN_A, MOTOR_LF_IN_B);
    Controller motor_RF(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_RF_INV, MOTOR_RF_BRAKE, MOTOR_RF_PWM, MOTOR_RF_IN_A, MOTOR_RF_IN_B);
    Controller motor_LB(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_LB_INV, MOTOR_LB_BRAKE, MOTOR_LB_PWM, MOTOR_LB_IN_A, MOTOR_LB_IN_B);
    Controller motor_RB(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_RB_INV, MOTOR_RB_BRAKE, MOTOR_RB_PWM, MOTOR_RB_IN_A, MOTOR_RB_IN_B);

    // Encoder 4 wheels
    esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
    esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
    esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
    esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);
#elif teelek_katsu
    
#endif

//------------------------------ < Fuction Prototype > ------------------------------//
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

void cmd_vel_callback(const void *);
void cmd_reset_encoder_callback(const void *msgin);

void publishData();
void getEncoderWheelsTick(); 
void MovePower(int, int, int, int);

//------------------------------ < Main > -------------------------------------//

#ifdef teelek_karake
    // ---------------- Encoder Left ----------------
    #define CLK_L 20
    #define DT_L  21
    volatile long encoderL_pos = 0;
    volatile uint8_t lastStateL = 0;

    void IRAM_ATTR handleEncoderL() {
        uint8_t state = (digitalRead(CLK_L) << 1) | digitalRead(DT_L);
        int8_t change = 0;
        switch (lastStateL) {
            case 0b00: if (state == 0b01) change = 1; else if (state == 0b10) change = -1; break;
            case 0b01: if (state == 0b11) change = 1; else if (state == 0b00) change = -1; break;
            case 0b11: if (state == 0b10) change = 1; else if (state == 0b01) change = -1; break;
            case 0b10: if (state == 0b00) change = 1; else if (state == 0b11) change = -1; break;
        }
        encoderL_pos += change;
        lastStateL = state;
    }

    long readEncoderL() {
        noInterrupts();
        long v = encoderL_pos;
        interrupts();
        return v;
    }

    // ---------------- Encoder Right ----------------
    #define CLK_R 47
    #define DT_R  48
    volatile long encoderR_pos = 0;
    volatile uint8_t lastStateR = 0;

    void IRAM_ATTR handleEncoderR() {
        uint8_t state = (digitalRead(CLK_R) << 1) | digitalRead(DT_R);
        int8_t change = 0;
        switch (lastStateR) {
            case 0b00: if (state == 0b01) change = 1; else if (state == 0b10) change = -1; break;
            case 0b01: if (state == 0b11) change = 1; else if (state == 0b00) change = -1; break;
            case 0b11: if (state == 0b10) change = 1; else if (state == 0b01) change = -1; break;
            case 0b10: if (state == 0b00) change = 1; else if (state == 0b11) change = -1; break;
        }
        encoderR_pos += change;
        lastStateR = state;
    }

    long readEncoderR() {
        noInterrupts();
        long v = encoderR_pos;
        interrupts();
        return v;
    }
#elif teelek_katsu
#endif


#ifdef teelek_karake
    void setup()
    {
        // Encoder Left
        pinMode(CLK_L, INPUT_PULLUP);
        pinMode(DT_L, INPUT_PULLUP);
        lastStateL = (digitalRead(CLK_L) << 1) | digitalRead(DT_L);
        attachInterrupt(digitalPinToInterrupt(CLK_L), handleEncoderL, CHANGE);
        attachInterrupt(digitalPinToInterrupt(DT_L), handleEncoderL, CHANGE);

        // Encoder Right
        pinMode(CLK_R, INPUT_PULLUP);
        pinMode(DT_R, INPUT_PULLUP);
        lastStateR = (digitalRead(CLK_R) << 1) | digitalRead(DT_R);
        attachInterrupt(digitalPinToInterrupt(CLK_R), handleEncoderR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(DT_R), handleEncoderR, CHANGE);
        
        Serial.begin(115200);
        #ifdef MICROROS_WIFI
            
            IPAddress agent_ip(AGENT_IP);
            uint16_t agent_port = AGENT_PORT;
            set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
        #else
            set_microros_serial_transports(Serial);
        #endif
    }
#elif teelek_katsu
    void setup() {
        Serial.begin(115200);
        #ifdef MICROROS_WIFI
            IPAddress agent_ip(AGENT_IP);
            uint16_t agent_port = AGENT_PORT;
            set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
        #else
            set_microros_serial_transports(Serial);
        #endif

        state = WAITING_AGENT;
        last_sync = millis();
    }
#endif

#ifdef teelek_karake
    void loop()
    {   
        if (millis() - last_sync > 10000) {
            syncTime();
            last_sync = millis();
        }
        switch (state)
        {
        case WAITING_AGENT:
            // EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 4)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(250, state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(150));
            }
            break;
        case AGENT_DISCONNECTED:
            MovePower(0, 0, 0, 0);
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
        }
    }
#elif teelek_katsu
    void loop() {
        // Sync time every 60 seconds
        if (millis() - last_sync > 60000) {
            syncTime();
            last_sync = millis();
        }

        switch (state) {
            case WAITING_AGENT:
                // Ping agent every 1 second
                EXECUTE_EVERY_N_MS(1000, {
                    if (RMW_RET_OK == rmw_uros_ping_agent(1000, 1)) {
                        state = AGENT_AVAILABLE;
                    }
                });
                break;

            case AGENT_AVAILABLE:
                static unsigned long last_attempt = 0;
                if (millis() - last_attempt > 2000) {
                    last_attempt = millis();
                    if (createEntities()) {
                        state = AGENT_CONNECTED;
                    }
                }
                break;

            case AGENT_CONNECTED:
                // Spin executor
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

                // Check agent connection every 2 seconds
                EXECUTE_EVERY_N_MS(2000, {
                    if (!(RMW_RET_OK == rmw_uros_ping_agent(1000, 1))) {
                        state = AGENT_DISCONNECTED;
                    }
                });
                break;

            case AGENT_DISCONNECTED:
                destroyEntities();
                state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }
#endif

//------------------------------ < Fuction > -------------------------------------//

#ifdef teelek_karake
    // Motor Move
    void MovePower(int Motor_LFSpeed, int Motor_RFSpeed, int Motor_LBSpeed, int Motor_RBSpeed)
    {
        Motor_LFSpeed = constrain(Motor_LFSpeed, PWM_Min, PWM_Max);
        Motor_RFSpeed = constrain(Motor_RFSpeed, PWM_Min, PWM_Max);
        Motor_LBSpeed = constrain(Motor_LBSpeed, PWM_Min, PWM_Max);
        Motor_RBSpeed = constrain(Motor_RBSpeed, PWM_Min, PWM_Max);

        motor_LF.spin(Motor_LFSpeed);
        motor_RF.spin(Motor_RFSpeed);
        motor_LB.spin(Motor_LBSpeed);
        motor_RB.spin(Motor_RBSpeed);
    }

    void cmd_vel_callback(const void *msgin) 
    {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        float V_x = msg->linear.x;
        float W_z = msg->angular.z;
        float d = max(abs(V_x) + abs(W_z), (float) PWM_Max);
        int fl = (V_x - W_z)/d * (float) PWM_Max;
        int fr = (V_x + W_z)/d * (float) PWM_Max;
        int bl = (V_x - W_z)/d * (float) PWM_Max;
        int br = (V_x + W_z)/d * (float) PWM_Max;
        MovePower(fl, fr,
                  bl, br);
    }

    void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
            getEncoderWheelsTick();
            publishData(); 
        }
    }

    void getEncoderWheelsTick() {
        long curLF = encLF.read();
        long curLB = encLB.read();
        long curRF = encRF.read();
        long curRB = encRB.read();

        debug_encoder_wheels_msg.data.data[0] = curLF - offsetLF;
        debug_encoder_wheels_msg.data.data[1] = curLB - offsetLB;
        debug_encoder_wheels_msg.data.data[2] = curRF - offsetRF;
        debug_encoder_wheels_msg.data.data[3] = curRB - offsetRB;
    }

    void resetEncoderOffset()
    {
        offsetLF = encLF.read();
        offsetLB = encLB.read();
        offsetRF = encRF.read();
        offsetRB = encRB.read();

        // Set current message to zero
        debug_encoder_wheels_msg.data.data[0] = 0;
        debug_encoder_wheels_msg.data.data[1] = 0;
        debug_encoder_wheels_msg.data.data[2] = 0;
        debug_encoder_wheels_msg.data.data[3] = 0;
    }
    void cmd_reset_encoder_callback(const void *msgin)
    {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        if (msg->linear.x > 0.5)
        {
            resetEncoderOffset();
        }
    }

#elif teelek_katsu
    void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
            publishData(); 
            // if (servo_detach_pending) {
            //     if (millis() - servo_detach_time >= 5000) {
            //         servo_dig.detach();
            //         servo_attached = false;
            //         servo_detach_pending = false;
            //     } else if (servo_attached) {
            //         // ระหว่างรอ 5 วิ ให้ส่ง PWM เดิมไว้ก่อน
            //         servo_dig.writeMicroseconds(servo_pulse);
            //     }
            // } else if (servo_attached) {
            //     // refresh PWM ป้องกัน servo ย้วย
            //     servo_dig.writeMicroseconds(servo_pulse);
            // }
        }
    }

    // -------------------- Callbacks --------------------
    // void cmd_loadleft_callback(const void *msgin) {
    //     const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    //     cmd_loadleft_msg = *msg;
    //     loadleft(msg->linear.x);
    // }

    // void cmd_loadright_callback(const void *msgin) {
    //     const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    //     cmd_loadright_msg = *msg;
    //     loadright(msg->linear.x);
    // }

    // void cmd_servo_callback(const void *msgin) {
    //     const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    //     cmd_servo_msg = *msg;
    //     controlServo(msg->angular.x);
    // }
#endif

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    // init options
    init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
    rcl_init_options_set_domain_id(&init_options, 10);

    // support
    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;

    // executor init first
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 10, &allocator) != RCL_RET_OK) return false;

    // create node
    #ifdef teelek_karake
        if (rclc_node_init_default(&node, "teelek_karake", "", &support) != RCL_RET_OK) return false;
    #elif teelek_katsu
        if (rclc_node_init_default(&node, "teelek_katsu", "", &support) != RCL_RET_OK) return false;
    #endif

    // -------------------- Publishers --------------------
    #ifdef teelek_karake
        if (rclc_publisher_init_default(
                &debug_cmd_vel_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "debug/wheel/cmd_vel") != RCL_RET_OK) return false;

        if (rclc_publisher_init_default(
                &debug_encoder_wheels_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                "teelek/debug/encoder_wheels") != RCL_RET_OK) return false;

        // init float sequence 4 elements
        rosidl_runtime_c__float__Sequence__init(&debug_encoder_wheels_msg.data, 4);

    #elif teelek_katsu
        // if (rclc_publisher_init_default(
        //         &debug_servo_publisher,
        //         &node,
        //         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        //         "/teelek/debug/servo") != RCL_RET_OK) return false;
    #endif

    // -------------------- Subscriptions --------------------
    #ifdef teelek_karake
        if (rclc_subscription_init_default(
                &cmd_vel_subscriber,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "/teelek/cmd_move") != RCL_RET_OK) return false;

        if (rclc_subscription_init_default(
                &cmd_resetencoder_subscriber,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "/teelek/cmd_resetencoder") != RCL_RET_OK) return false;

        // add subscription to executor
        if (rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
            &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

        if (rclc_executor_add_subscription(&executor, &cmd_resetencoder_subscriber, &cmd_resetencoder_msg,
            &cmd_reset_encoder_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    #elif teelek_katsu
        // if (rclc_subscription_init_default(
        //         &cmd_servo_subscriber,
        //         &node,
        //         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        //         "/teelek/cmd_servo") != RCL_RET_OK) return false;

        // if (rclc_executor_add_subscription(&executor, &cmd_servo_subscriber, &cmd_servo_msg,
        //     &cmd_servo_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
    #endif

    // -------------------- Timer --------------------
    const unsigned int control_timeout = 50;
    if (rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(control_timeout), &controlCallback) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) return false;

    // sync time with agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    #ifdef teelek_karake
        rcl_subscription_fini(&cmd_vel_subscriber, &node);
        rcl_subscription_fini(&cmd_resetencoder_subscriber, &node);
        rcl_publisher_fini(&debug_cmd_vel_publisher, &node);
        rcl_publisher_fini(&debug_encoder_wheels_publisher, &node);
    #elif teelek_katsu
        // rcl_subscription_fini(&cmd_servo_subscriber, &node);
        // rcl_publisher_fini(&debug_servo_publisher, &node);
    #endif

    rcl_timer_fini(&control_timer);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void publishData()
{
    #ifdef teelek_karake
        debug_wheel_motor_msg.linear.x = cmd_vel_msg.linear.x;
        debug_wheel_motor_msg.linear.y = cmd_vel_msg.linear.y;
        debug_wheel_motor_msg.angular.z = cmd_vel_msg.angular.z;

        rcl_publish(&debug_cmd_vel_publisher, &debug_wheel_motor_msg, NULL);
        rcl_publish(&debug_encoder_wheels_publisher, &debug_encoder_wheels_msg, NULL);
    #elif teelek_katsu
        // debug_servo_msg.angular.x = cmd_servo_msg.angular.x;

        // rcl_publish(&debug_servo_publisher, &debug_servo_msg, NULL);
    #endif
}

void syncTime()
{
    // get the current time from the agent
        unsigned long now = millis();
        RCCHECK(rmw_uros_sync_session(10));
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
        tp.tv_sec = now / 1000;
        tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop()
{
    ESP.restart();
    // while (true)
    // {
    //     delay(1000);
    // }
}