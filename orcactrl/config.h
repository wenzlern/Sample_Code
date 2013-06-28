/****************************************************************************
 * apps/orcactrl/config.h
 *
 *   Copyright (C) 2012 Nils Wenzler. All rights reserved.
 *   Author: Nils Wenzler <wenzlern@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H 


/****************************************************************************
 * Preprocessor directives
 ****************************************************************************/
/*** Files ***/
/* The file names here are dictated by the drivers. */
/* ADC */
#define ORCACTRL_ADC_FILE "/dev/adc0"

/* PWM */
#define ORCACTRL_PWM_FILE "/dev/pwm_servo"

/* Accelerometer */
#define ORCACTRL_ACCEL_FILE "/dev/lis331"

/* Gyroscope */
#define ORCACTRL_GYRO_FILE "/dev/l3gd20"

/* bluetooth uart file to write to */
#define LMX9830_UART_FILE  "/dev/ttyS1"



/*** debugging settings ***/
/* 1 means: Print values every time they are read.
 * 0 means: Do not print values.
 */
#define PRINT_ADCVALUES 0   /* Print the values read by the ADC */
#define PRINT_GYROVALUES 0  /* Print the values read by the gyro */
#define PRINT_ACCELVALUES 0 /* Print the values read by the accelerometer */
#define PRINT_PWMVALUES 0   /* Print the values that are set for PWM */
#define PRINT_CONTROLLER 0  /* Print the values of the steering controller */
#define PRINT_SETVALS 0     /* Print the setvalues for steering and throttle */


/*** Bluetooth ***/
/** Module Settings **/
/* Server/Slave */
/* Should the module connect by itself or wait for incoming connections.
 * SELFCONNECT 1 needs the user to run the connect routines.
 */
#define SELFCONNECT 0

/* Use packet mode or transparent */
#define PACKET_MODE 0

/* The timeout to use when receiving (in cycles) */
#define RECV_TIMEOUT 50

/* Maximum Packet Length */
#define LMX9830_PACK_MAXSIZE 300

/* Bluetooth buffer size (1 byte more for the rfcomm_channel byte)*/
#define BT_BUFFER_REC_SIZE LMX9830_PACK_MAXSIZE + 1

/* RFCOMM Ports to use for the Bluetooth connections */
#define REALTIME_RFC 1

/** Parser Settings */
/* B&R or Phone */
/* Only select one for the time beeing! The parsing has not been coded in order
 * to support both packet types simultaniously. Most likely it would work,
 * but nothing can be guaranteed
 */
#define BR 1  /* Enable the parsing for the B&R Protcol */
#define PHONE 0 /* Enable parsing for Bluetooth steering wheel
                   and controller tuning on the Phone*/

/** Protocol **/
/* What should be sent to the B&R */ 
/* FULL_PROTOC0L 1: Battery, Accelerometer, Gyro 
 * FULL_PROTOCOL 0: Battery */
#define FULL_PROTOCOL 0

/* The frame start character */
#define STARTBYTE '#'
#define ENDBYTE '\r'

/* B&R => Embedded Packet */
enum packet_recv{
    RECV_START = 0,
    RECV_THROTTLE,
    RECV_STEERING,
    RECV_END,

    /* Leave this one at the bottom, it determines the size of the payload */
    RECV_PACKET_LENGTH
};

/* Embedded => B&R Packet */
enum packet_send{
    SEND_START,
    SEND_BATTERY,
#if FULL_PROTOCOL
    /*The accelerometer and gyro are 16bit, so we have a high and low register */
    SEND_ACCEL_X_L,
    SEND_ACCEL_X_H,
    SEND_ACCEL_Y_L,
    SEND_ACCEL_Y_H,
    SEND_GYRO_X_L,
    SEND_GYRO_X_H,
    SEND_GYRO_Y_L,
    SEND_GYRO_Y_H,
#endif
    SEND_END,
    /*Leave this one at the bottom, it determines the size of the payload */
    SEND_PACKET_LENGTH
};




/*** Scaling ***/
/** ADC **/
/* Steering */
/* We want a value range of +-1f (float) (1 Byte) 
 * on both controls. The use of floats in between +-1 is due to the px4 system
 * that uses this convention everywhere.
 */
/* Steering scale and mid default value for the ADC readout of the poti */
#define STEERING_ADC_MIN 1300
#define STEERING_ADC_MAX 2600
/* The 2 comes from our desired range of +-1f */
#define STEERING_ADC_MID (STEERING_ADC_MAX + STEERING_ADC_MIN)/2
#define STEERING_ADC_SCALE (STEERING_ADC_MAX - STEERING_ADC_MIN)/2

/* Throttle */
/*The Scale that gets applied to the shunt readout at the adc*/
/*TODO: Determine this value */
#define MOTOR_ADC_SCALE 0

/* Battery */
/*TODO: Check this value!*/
#define BATTERY_ADC_SCALE 1.725f


/** PWM **/
/* This has to be adjusted if the PWM update frequency in src/up_pwm_servo.c
 * gets changed. The value is +/- (1/pwm_update_rate) [microseconds]
 */
#define PWM_MIN -20
#define PWM_MAX 20
/* The 2 comes from our desired range of +-1f */
#define PWM_SCALE (PWM_MAX - PWM_MIN)/2




/*** Controller constants ***/
/** Steering **/
/* P coefficient for PID controller. These values are +/- Ziegler-Nichols */
#define STEERING_KP  0.672f//0.334f//0.21 //0.0990f
/* I coefficient for PID controller */
#define STEERING_KI  0.300f//0.06f//2.165//3.97070f
/* D coefficient for PID controller */
#define STEERING_KD  0.014f//0.0038f//0.0051//0.00200f

/* Lowpass */
/* Order of the lowpass filter for the steering , if set to 1, the 
 * ADC will not get filtered.
 */
#define FILTER_ORDER 1


/** Throttle **/
/* Wheter to use the kalman filter/estimator for the throttle or just set it.
 * This controller has NOT been configured or tested. Please refer to the
 * documentation for further information.
 */
#define ESTIMATE_RPM 0


/** Wait Times **/
/* ADC Wait for adc_read_current() */
#define ADC_WAIT 650 /* [us] */
/* Main loop speed */
#define LOOP_TUNING 650 /* [us] */ 


/** Timeout for BT connection **/
/* Change this setting to the desired value [ms]*/
#define TIMEOUT 250 /* [ms] */
/* Calculate the cycles from that (Assume Runtime = ADC_WAIT+LOOP_TUNING) */
#define TIMEOUT_CYCLES (TIMEOUT*1e3) / (ADC_WAIT + LOOP_TUNING)

/****************************************************************************
 * Setting Variables 
 *
 * Export the variables that get used at some place else.
 * Change/define them in orcactrl.c
 *
 ****************************************************************************/

/* The bluetooth address of the embedded board */
extern const uint8_t const_lmx9830_btaddr[];

/* The bluetooth address of the B&R */
extern const uint8_t const_realtime_btaddr[];

#endif /*CONFIG_H*/
