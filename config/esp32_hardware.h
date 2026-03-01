#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

    //define your robot' specs here
    #define MOTOR_MAX_RPM 500                                               // motor's max RPM          
    #define MAX_RPM_RATIO 0.85                                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
    #define MOTOR_OPERATING_VOLTAGE 12                                      // motor's operating voltage (used to calculate max RPM)
    #define MOTOR_POWER_MAX_VOLTAGE 12                                      // max voltage of the motor's power source (used to calculate max RPM)
    #define MOTOR_POWER_MEASURED_VOLTAGE 12                                 // current voltage reading of the power connected to the motor (used for calibration)

    // #define LR_WHEELS_DISTANCE 0.335                                        // distance between left and right wheels
    #define PWM_BITS 10                                                     // PWM Resolution of the microcontroller
    #define PWM_FREQUENCY 20000                                             // PWM Frequency
    #define PWM_Max 1023
    #define PWM_Min PWM_Max * -1
    // #define GEAR_Ratio 1.575                                               // Midpoint of the PWM signal
    #define WHEEL_DIAMETER 0.13                                           // wheel's diameter in meters 

    // INVERT MOTOR DIRECTIONS  
    #define MOTOR_LF_INV false
    #define MOTOR_RF_INV true
    #define MOTOR_LB_INV false
    #define MOTOR_RB_INV true

    //  Motor Brake
    #define MOTOR_LF_BRAKE true
    #define MOTOR_RF_BRAKE true
    #define MOTOR_LB_BRAKE true
    #define MOTOR_RB_BRAKE true
    
    #ifdef teelek_karake

        // Motor 1 Parameters
        #define MOTOR_LF_PWM  -1
        #define MOTOR_LF_IN_A 36
        #define MOTOR_LF_IN_B 35

        // Motor 2 Parameters
        #define MOTOR_RF_PWM  -1
        #define MOTOR_RF_IN_A 4
        #define MOTOR_RF_IN_B 5 
        

        // Motor 3 Parameters
        #define MOTOR_LB_PWM  -1
        #define MOTOR_LB_IN_A 15
        #define MOTOR_LB_IN_B 16

        // Motor 4 Parameters
        #define MOTOR_RB_PWM  -1
        #define MOTOR_RB_IN_A 1
        #define MOTOR_RB_IN_B 2
        
        // ---------------- Encoder 4 wheels ----------------

        // Motor 1
        #define Encoder_LF_A 39
        #define Encoder_LF_B 40

        // Motor 2
        #define Encoder_RF_A 13
        #define Encoder_RF_B 14
        
        // Motor 3
        #define Encoder_LB_A 41
        #define Encoder_LB_B 42

        // Motor 4
        #define Encoder_RB_A 37
        #define Encoder_RB_B 38

        #define ENCODER_TICKS 11
        #define GEAR_RATIO 534.0168 
        #define COUNTS_PER_REV ENCODER_TICKS * GEAR_RATIO * 4       // encoder resolution
        #define ENCODER_INV_LF false
        #define ENCODER_INV_RF false
        #define ENCODER_INV_LB true
        #define ENCODER_INV_RB false
    #endif

    #ifdef teelek_katsu

        // Motor loadleft config
        #define MOTORLOAD_LEFT_PWM  -1
        #define MOTORLOAD_LEFT_IN_A  15
        #define MOTORLOAD_LEFT_IN_B  16

        // Motor loadleft config
        #define MOTORLOAD_RIGHT_PWM  -1
        #define MOTORLOAD_RIGHT_IN_A  17
        #define MOTORLOAD_RIGHT_IN_B  18

        // Servo Parameter
        #define SERVO_PIN 39
        #define SERVO_MIN_PULSE 800 
        #define SERVO_MAX_PULSE 3200

        //Load Zone
        //Stepmotor
        #define ENA 13
        #define DIR 12
        #define PUL 11

        //Plant Zone
        //define your robot' specs here
        #define MOTOR_RPM 400.0f                                            // motor's max RPM          
        #define MAX_RPM_RATIO 0.85f                                             // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
        #define MOTOR_MAX_RPM  MOTOR_RPM * MAX_RPM_RATIO
        #define MOTOR_OPERATING_VOLTAGE 12                                      // motor's operating voltage (used to calculate max RPM)
        #define MOTOR_POWER_MAX_VOLTAGE 12                                      // max voltage of the motor's power source (used to calculate max RPM)
        #define MOTOR_POWER_MEASURED_VOLTAGE 12                                 // current voltage reading of the power connected to the motor (used for calibration)
        #define ENCODER_PULSES_PER_REVOLUTION 16                                // encoder 1 pulse

        #define ENCODER_TICKS 4                                                 // encoder ticks
        #define ENCODER_COUNTS_PER_REV ENCODER_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel1 encoder's no of ticks per rev
        #define WHEEL_DIAMETER 1.0f                                           // wheel's diameter in meters
        #define ODOM_TICKS_SIGN (-1.0f)                                          // flip odom if forward travel reports negative distance
        #define PWM_BITS 10                                                     // PWM Resolution of the microcontroller
        #define PWM_FREQUENCY 20000                                             // PWM Frequency
        #define PWM_Max 1023
        #define PWM_Min PWM_Max * -1          
        #define PWM_SPIN_Max (int)(PWM_Max * 1.0f)                          // max PWM value used in spin control                             
        #define PWM_STEER_Max (int)(PWM_Max * 0.65)                          // max PWM value used in spin control                             


        // INVERT MOTOR DIRECTIONS
        #define MOTOR_INV false

        // INVERT ENCODER DIRECTIONS
        #define MOTOR_ENCODER_INV true

        //  Motor Brake
        #define MOTOR_BRAKE true

        // Motor 1 Parameters
        #define MOTOR_PWM  -1
        #define MOTOR_IN_A 38
        #define MOTOR_IN_B 37

        // Encoder 1 Parameter
        #define MOTOR_ENCODER_INCRIMENT -1
        #define MOTOR_ENCODER_PIN_A 36
        #define MOTOR_ENCODER_PIN_B 35
        #define MOTOR_ENCODER_GEAR 22.0f
        #define MOTOR_ENCODER_PULLEY 41.0f
        #define MOTOR_ENCODER_RATIO (MOTOR_ENCODER_PULLEY / MOTOR_ENCODER_GEAR)

        // Servo Parameters
        #define SERVO_PIN 39
        #define SERVO_MIN_PULSE_WIDTH 500
        #define SERVO_MAX_PULSE_WIDTH 2500

        // Lead screw Parameters
        #define LEAD_SCREW_PITCH_MM_PER_REV 2.0f  // Lead screw pitch in mm/rev
        #define LEAD_SCREW_MOTOR_GEAR 10.0f
        #define LEAD_SCREW_PULLEY_GEAR 15.0f
        #define LEAD_SCREW_GEAR_RATIO (LEAD_SCREW_PULLEY_GEAR / LEAD_SCREW_MOTOR_GEAR)

        // I2C communication
        #define SCL_PIN 22
        #define SDA_PIN 21

        // Limit SW
        #define LIMIT_SWITCH_PIN 15 

    #endif

    // INVERT ENCODER DIRECTIONS (ใช้เฉพาะ 2 ข้าง)
    #define MOTOR_LF_ENCODER_INV false
    #define MOTOR_RF_ENCODER_INV false

    // I2C communication
    #define SCL_PIN 8
    #define SDA_PIN 9

#endif
