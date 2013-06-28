/**************************************************************************** 
 * apps/orcactrl.c
 *
 *   Copyright (C) 2012 Sandro Merkli, Nils Wenzler. All rights reserved.
 *   Author: Sandro Merkli <smerkli@ee.ethz.ch>
 *           Nils Wenzler  <wenzlern@ee.ethz.ch>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

#include "systemlib/systemlib.h"
#include "systemlib/pid/pid.h"

#include <arch/board/board.h>
#include <arch/board/drv_pwm_servo.h>
#include <arch/board/drv_lis331.h>

#include <nuttx/analog/adc.h>

/* Our custom high-level Bluetooth driver */
#include "bt_lmx9830.h"

/* Our config file */
#include "config.h"


/****************************************************************************
 * Setting Variables 
 *****************************************************************************/

/** Bluetooth settings **/
/* The bluetooth address of the embedded board */
const uint8_t const_lmx9830_btaddr[] = 
{ 
  0x11, 0x34, 0x56, 0x78, 0x9A, 0xBC 
};

/* The bluetooth address of the B&R */
const uint8_t const_realtime_btaddr[] = 
{ 
  0x71, 0x44, 0x43, 0x66, 0x06, 0x00 
};


/****************************************************************************
 * Internal variables 
 *
 * The variables declared here are local to this file and 
 * together form the internal state of the app.
 *
 ****************************************************************************/
/** General variables **/
static int16_t result; /* generic function return value holder */


/** Bluetooth **/
/* Bluetooth reception and send buffers */
static uint8_t bt_buffer_recv[  BT_BUFFER_REC_SIZE ]; /**<Receive Buffer*/
static uint8_t bt_buffer_send[ BT_BUFFER_REC_SIZE ];  /**<Send Buffer*/

/* Bluetooth parsing */
int8_t x, y, z; /**< Receive the bluetooth steering wheel apps output */

uint8_t window_buffer[4]; /**< Needed atm since the sparkfun module is in
                               transparent mode and I need to assemble the
                               window */
/* Timeout timer. Used to stop the car if no data received ([Cycles])*/
int16_t timeout;


/** ADC **/
static int adc_fd; /**< ADC file descriptor */

/** ADC readout data: 
 * - adc_data[0].am_data is the battery voltage
 * - adc_data[1].am_data is the shunt monitor
 * - adc_data[2].am_data is the poti wiper
 * rest is not used by us.
 */
struct adc_msg_s adc_data[4]; 

/** Struct to store the scaled ADC values as well as the scaling parameters */
struct adc_scaled {
  float battery_voltage; /**< Voltage in X.x float format (eg. 3.7)*/
  float motor_current;   /**< The motor current in mA (eg. 10mA, float */
  float poti_wiper;      /**< The scaled value for the potentiometer, +-1f */ 
  float scale_bat;       /**< The scaling value for the battery */ 
  uint16_t scale_mot;    /**< The scaling value for the motor current */ 
  uint16_t scale_pot;    /**< The scaling value for the poti redaout */
  uint16_t mid_pot;      /**< The middle value for the poti wiper (shift) */ 
};

struct adc_scaled adc_values = {0.0f, 0.0f, 0.0f, BATTERY_ADC_SCALE, MOTOR_ADC_SCALE,
  STEERING_ADC_SCALE, STEERING_ADC_MID};


/**Lipo discharge LUT
 * 
 * Since Lipos have a flat (nonlinear) discharge curve, we need a look up
 * table to more or less map a voltage to a percentage. This may have to be
 * improved. (measure one of our batteries?)
 */
const float lipo_discharge[51] = { 
  4.2000f, 4.1858f, 4.1717f, 4.1575f, 4.1433f, 4.1292f, 4.1150f, 4.1008f, 4.0867f, 
  4.0725f, 4.0583f, 4.0442f, 4.0300f, 4.0158f, 4.0017f, 3.9875f, 3.9733f, 3.9592f, 
  3.9450f, 3.9308f, 3.9167f, 3.9025f, 3.8883f, 3.8742f, 3.8600f, 3.8540f, 3.8480f, 
  3.8420f, 3.8360f, 3.8300f, 3.8233f, 3.8167f, 3.8100f, 3.8033f, 3.7967f, 3.7900f, 
  3.7805f, 3.7711f, 3.7616f, 3.7521f, 3.7426f, 3.7332f, 3.7237f, 3.7142f, 3.7047f, 
  3.6973f, 3.6918f, 3.6864f, 3.6809f, 3.6755f, 3.6700f
};



/** Accelerometer **/
static int accel_fd; /**< Accelerometer file descriptor */

/**Accelerometer readout data. */
/* We actually only need 3 slots
 * but the driver insists on having 
 * 12. Oh well.
 */
static int16_t accel_data[12]; 

/** Gyro **/
static int gyro_fd; /**< Gyroscope file descriptor */
static int16_t gyro_data[3]; /**< Gyroscope readout data. */

/** PWM **/
static int pwm_fd; /**< PWM file descriptor */

/* Servo position data (PWM value data)
 * The value of this is the actual pulse 
 * width in microseconds. */
static servo_position_t pwm_data[4]; 



/** Controller variables **/
/* Steering */
/**Struct that contains the setval and the scaling value of an actuator */
struct actuator {
  float setval;    /**< Setval, range +-1f */
  uint16_t scale;  /**< The scaling value for the actuator */
};

struct actuator steering = {0.0f, PWM_SCALE};

/** Used for computing the PID delta time for the integral */
uint64_t last_run = 0;

/** PID Struct for the steering controller */
PID_t steering_controller; 

/* Lowpass */
uint8_t filter_ord = FILTER_ORDER;   /**< The order of the Lowpass */
float steering_filter[FILTER_ORDER]; /**< The old values */ 
float steering_position;             /**< The filtered positon */ 


/* Throttle */
struct actuator throttle = {0.0f, PWM_SCALE};
/* Return value holder for the estimator/controler */
float throotle_act[3];



/****************************************************************************
 * IO helper functions 
 *
 * The helper functions in this section provide easy access to the
 * functionality we need so that the main control loop is easily readable.
 * The actual work is done by the drivers in nuttx/configs/orcav1/src/.
 *
 ****************************************************************************/
 
/** Bluetooth **/

/** Highlevel bluetooth polling function
 * 
 * This function will be the point where control data from bluetooth is 
 * parsed and action is taken by modifying the state of the app.
 * Since this app does not have threads and execution is purely sequential,
 * communication between the functions over a central state is clean and
 * elegant to implement.
 */
static void 
helper_poll_bluetooth(void)
{
  /* call lmx9830_receive() and see if we got a packet. */
  memset( bt_buffer_recv, 0x0, BT_BUFFER_REC_SIZE ); /* reset recv buffer */

#if PACKET_MODE
  result = lmx9830_receive( bt_buffer_recv ); /* get a packet, if any */
#else
  result = lmx9830_receive_windowing( bt_buffer_recv ); /* get a packet, if any */
#endif /* PACKET_MODE */

  /* if yes (more than 0 bytes received), parse packet and take action */
  if( result > 0 )
  {
  
  /* Reset timeout timer */
  timeout = TIMEOUT_CYCLES;

#if PHONE
    
    /* If the packet is 4 bytes (plus the rfcomm_chan) long, we assume
     * we have a packet from the Bluetooth Steering wheel app
     */
    if ((bt_buffer_recv[1] == 'c' ) && ( result == 5 ))
    {
      /* The app sends a 'c' char when in driving mode, followed by
       * the 3 axes in int8
       */
      x = (int8_t) bt_buffer_recv[2]; 
      y = (int8_t) bt_buffer_recv[3]; 
      z = (int8_t) bt_buffer_recv[4]; 
    }
    /* This is for tuning the steering controller */
    else if(bt_buffer_recv[1] == 'k')
    {
      /* Reduce kp by 5% */
      steering_controller.kp -= steering_controller.kp/20;
    }
    else if(bt_buffer_recv[1] == 'j')
    {
      /* Increase kp by 5% */
      steering_controller.kp += steering_controller.kp/20;
    }
    else if(bt_buffer_recv[1] == 'o')
    {
      /* Reduce ki by 5% */
      steering_controller.ki -= steering_controller.ki/20;
    }
    else if(bt_buffer_recv[1] == 'i')
    {
      /* Increase ki by 5% */
      steering_controller.ki += steering_controller.ki/20;
    }
     else if(bt_buffer_recv[1] == 'f')
    {
      /* Reduce kd by 5% */
      steering_controller.kd -= steering_controller.kd/20;
    }
    else if(bt_buffer_recv[1] == 'd')
    {
      /* Increase kd by 5% */
      steering_controller.kd += steering_controller.kd/20;
    }
    else if(bt_buffer_recv[1] == 'n')
    {
      /* Reduce filter order by 1 */
      if(filter_ord > 0)
        filter_ord -= 1;
    }
    else if(bt_buffer_recv[1] == 'm')
    {
      /* Increase filter order by 1 */
      if(filter_ord < FILTER_ORDER)
        filter_ord += 1;
    }
    else if(bt_buffer_recv[1] == 's')
    {
      /* Step input positiv */
      y = 100;
    }
    else if(bt_buffer_recv[1] == 'a')
    {
      /* Step input negativ */
      y = -100;
    }
    else if(bt_buffer_recv[1] == 't')
    {
      /* Step input negativ */
      y = 127;
    }
    else if(bt_buffer_recv[1] == 'y')
    {
      /* Step input negativ */
      y = -127;
    }
    else if(bt_buffer_recv[1] == 'r')
    {
      /* Neutral */
      y = 0;
    }
 
    /*Map the int8_t values to a range of +-1 float */
    steering.setval = -1.0f*((float)y)/127;
    throttle.setval = -1.0f*((float)x)/127;

  #if PRINT_SETVALS
    printf("x: %i, y: %i, z: %i \t steering: %.2f, throttle: %.2f, Poti: %0.2f\n", x, y, z,
        steering.setval, throttle.setval, adc_values.poti_wiper);
  #endif
  #if PRINT_CONTROLLER
    printf("kp: %0.5f, ki: %0.5f, kd: %0.5f, FILTER_ORDER: %i\n", 
        steering_controller.kp, steering_controller.ki, steering_controller.kd,
        filter_ord);
  #endif /*PRINT_CONTROLLER*/
#endif /*PHONE*/

#if BR
#if PACKET_MODE
    /* The packet should be 4 + rfcomm channel long, so result=5)
     * The routine adapts by itself when the protocol (defined in config.h)
     * changes. You only have to change it if you want to parse some changed
     * (additional) packet data.
     */
    if (result == (RECV_PACKET_LENGTH + 1) && (char)bt_buffer_recv[RECV_START + 1] 
        == STARTBYTE )
    {
      /* Ugly one liner so we don't have to use an extra buffer.
       * First cast the uint8_t bt_buffer_recv to int8_t to make it +/-,
       * then cast this to float, so we can change the range to +/-1.
       */
      throttle.setval = ((((float)bt_buffer_recv[RECV_THROTTLE-1]))-127)
        /127;
      steering.setval = ((((float)bt_buffer_recv[RECV_STEERING-1]))-127)
        /127;

      printf("Received: Start: %i, Throttle: %i, Steering: %i, End: %i\n", bt_buffer_recv[RECV_START + 1],
        bt_buffer_recv[RECV_THROTTLE + 1], bt_buffer_recv[RECV_STEERING + 1], bt_buffer_recv[4]);
 
 
 

    }
    else
    {
      /* Bad packet */
      printf("Packet has length %i instead of 4\n", result );
    }

#if PRINT_SETVALS
    printf("steering: %.2f, throttle: %.2f\n",steering.setval,throttle.setval);
#endif
#else
    /* We only get the information back */
    if(result == 2)
    {
      throttle.setval = ((((float)bt_buffer_recv[RECV_THROTTLE-1]))-127)
        /127;
      steering.setval = ((((float)bt_buffer_recv[RECV_STEERING-1]))-127)
        /127;
#if PRINT_SETVALS
      printf("steering: %.2f, throttle: %.2f\n",steering.setval,throttle.setval);
#endif /* PRINT_SETVALS */
    }
#endif /* PACKET_MODE */
#endif /* BR */
  }
  else
  {
    /* Check for timeout, when reached stop the car */
    if(timeout > 0 )
      timeout--;
    else
      throttle.setval = 0;
      steering.setval = 0;
  }
}


/**Send to B&R routine
 *
 * Setting up a packet to send to the B&R computer.
 *@param battery The battery voltage in percent
 *@param accel_x The accel x-axis value
 *@param accel_y The accel y-axis value
 *@param gyro_x  The x value of the gyro 
 *@param gyro_y  The y value of the gyro  
 */
static void
helper_send_packet(uint8_t battery, int16_t accel_x, int16_t accel_y, 
    int16_t gyro_x, int16_t gyro_y) 
{
  /* Set the first byte to our Startbyte */
  bt_buffer_send[SEND_START] = STARTBYTE;

  bt_buffer_send[SEND_BATTERY] = battery;

#if FULL_PROTOCOL
  /* The accel and gyro use 2 bytes */
  /* The accel needs x and y direction */
  bt_buffer_send[SEND_ACCEL_X_L] = (uint8_t) accel_x;
  bt_buffer_send[SEND_ACCEL_X_H] = (uint8_t) (accel_x >> 8 );

  bt_buffer_send[SEND_ACCEL_Y_L] = (uint8_t) accel_y;
  bt_buffer_send[SEND_ACCEL_Y_H] = (uint8_t) (accel_y >> 8 );

  bt_buffer_send[SEND_GYRO_X_L] = (uint8_t) gyro_x;
  bt_buffer_send[SEND_GYRO_X_H] = (uint8_t) (gyro_x >> 8 );
 
  bt_buffer_send[SEND_GYRO_Y_L] = (uint8_t) gyro_y;
  bt_buffer_send[SEND_GYRO_Y_H] = (uint8_t) (gyro_y >> 8 );
#endif

  /* Now send the packet, The packet length is measured in a enum in config.h*/
  lmx9830_send( REALTIME_RFC, bt_buffer_send, SEND_PACKET_LENGTH );
}

/** RTT measurment
 *
 * Routine to measure the RountTripTime of the bluetoot connection.
 */
static void
bt_ping(void)
{ 
  uint8_t recbuf[5] = {0};
  uint8_t retval = 0;
  uint64_t curr_time = 0; /* starttime */
  uint64_t rtt = 0;       /* RountTripTime */

  /* First we echo a byte (random value 93)*/
  /* Log the time */
  curr_time = hrt_absolute_time();
  uint8_t ping=93;
  lmx9830_send(1, &ping, 1); 
  /* Start to receive until we get the echo back, timeout after 10000 cycles*/
  int i;
  for(i=0; i < 10000; i++)
  {
    retval = lmx9830_receive( recbuf ); 
    if(recbuf[1] == 93)
    {
      rtt = hrt_absolute_time() - curr_time;
      printf("Received Byte Ping: Cycles: %i, RTT: %i\n", i+1, rtt);
    }
    else
      printf("Timed out waiting for a respone");
  }
}


/** ADC **/

/** Highlevel ADC init function
 *
 *  Open the filedescriptor^^ 
 */
static void 
helper_init_adc(void)
{
  /* open the ADC file */
  adc_fd = open( ORCACTRL_ADC_FILE, O_RDONLY | O_NONBLOCK );
  if( adc_fd < 0 )  /* fd < 0 means error opening the file */
  { /* no other error handling here at the moment */
    printf( "failed opening " ORCACTRL_ADC_FILE "\n" );
  }

  /* wait a while to make sure it is started properly */
  up_udelay(10000); /* numerical value taken from ADC test app, 
                     * most propably empirical */

}

/** Highlevel ADC polling function
 *
 *  Reads out the adc, performs some error-checking and writes the scaled
 *  values to the adc_values struct.
 */
static void 
helper_read_adc(void)
{
  memset( adc_data, 0x0, 4*sizeof(struct adc_msg_s) ); /* reset data buffer */

  /* read out ADC values by reading from the adc file */
  result = read( adc_fd, adc_data, 4*sizeof(struct adc_msg_s) );

  /* see if we read as many bytes as we expected */
  if( result != (4 * sizeof(struct adc_msg_s)) ) 
    /* no, report an error */
    ;//printf("failed bulk-reading adc channel values\n");
  else
  {
    /* Write the scaled values to the adc_values struct */
    adc_values.battery_voltage = (float)adc_data[0].am_data*adc_values.scale_bat;
    adc_values.motor_current   = (float)adc_data[1].am_data*adc_values.scale_mot;
    adc_values.poti_wiper      = (float)(adc_data[2].am_data - adc_values.mid_pot)
                                  /adc_values.scale_pot;
  }

  /* debug printing */
  #if PRINT_ADCVALUES
    printf("ADC: BAT: %f, MOT: %f, POT: %0.2f, POT_UNSCALED: %i, POT_SCALE: %i, POT_MID: %i\n", 
      adc_values.battery_voltage, adc_values.motor_current,
      adc_values.poti_wiper,adc_data[2].am_data, adc_values.scale_pot, adc_values.mid_pot);
  #endif
}

/** ADC readout function without delayed read
 *
 * The adc has a delay of one cycle on the buffer. This is not a problem if
 * it is run continously with a high frequency. In our single loop architecture
 * however it is a problem, since the steering pid controller always receives
 * an old value, causing it to oscillate. Also we need it for the
 * calibration routine, where we want the values at a defined point in time.
 */
static void 
helper_read_adc_current(void)
{
  /* Read out the ADC twice with some sleep inbetween */
  helper_read_adc();
  usleep(ADC_WAIT);
  helper_read_adc();
}



/** Sensors **/ 

/** Highlevel accelerometer init function
 * 
 *  Open the filedescriptor and set the appropriate options
 */
static void 
helper_init_accel(void)
{
  /* the code here is taken and adapted from the test sensors app */
  /* open the accelerometer file */
  accel_fd = open(ORCACTRL_ACCEL_FILE, O_RDONLY);
  if (accel_fd < 0) 
  { /* ouptput message */
    printf("Failed opening " ORCACTRL_ACCEL_FILE "\n");

    /* TODO proper error handling needs to be added here sometime */
  }

  /* set options, these might not be optimal yet for us */
  if( ioctl(accel_fd, LIS331_SETRATE, LIS331_RATE_100Hz) ||
      ioctl(accel_fd, LIS331_SETRANGE, LIS331_RANGE_4G) )
  { /* if the ioctl functions fail, print to the user */
    printf("LIS331: ioctl fail\n");

    /* TODO also here, report over bluetooth or something since
     * in the final implementation, there won't be a NuttShell to
     * print to, at least none the user will see directly.
     */
  }

  /* reset data storage */
  memset( accel_data, 0x0, sizeof(accel_data) );

  /* wait for sensor to be ready. Test app says it should be after 20ms. */
  usleep(100000);

}

/** Highlevel accelerometer readout function
 *
 * Reads out the accelerometer
 *
 * @return -1 for error, 0 for no new samples and 1 for new sample
 */
static int8_t 
helper_read_accel(void)
{
  /* attempt to read */
  result = read(accel_fd, accel_data, sizeof(accel_data));

  /* see if it worked and return a value according to that */
  switch( result )
  { /* first case: all went well, we got a sample */
    case 12:
      #if PRINT_ACCELVALUES /* only print if debug printing enabled */ 
        printf("Accel X: %5.1d, Y: %5.1d, Z: %5.1d\n", accel_data[0],
          accel_data[1], accel_data[2] );
      #endif
      return 1;
      break;

    /* no sample, no error */
    case 0:
      return 0;
      break;

    /* some kind of error happened */
    default: 
      /* TEST */
      #if PRINT_ACCELVALUES /* only print if debug printing enabled */ 
      printf( "accel fail %d\n", result );
      #endif
      /* END TEST */
      /* TODO error handling already here? */
      return -1;
      break;
  }
}



/** Highlevel Gyro init function 
 *  Open the filedescriptor  
 */
static void
helper_init_gyro(void)
{
  /* the code here is largely taken and adapted from the test sensors app */
  /* open the gyro file */
  gyro_fd = open(ORCACTRL_GYRO_FILE, O_RDONLY | O_NONBLOCK);
  if (gyro_fd < 0) 
  { /* ouptput message */
    printf("Failed opening " ORCACTRL_GYRO_FILE "\n");

    /* TODO proper error handling needs to be added here sometime */
  }

  /* not setting any options, currently. See the test sensors
   * app for examples on how one can set settings.
   */
  /* XXX */

  /* reset data storage */
  memset( gyro_data, 0x0, sizeof(gyro_data) );

  /* wait for a couple of milliseconds */
  usleep(2000); /* ... choosing 2ms (arbitrarily) */
}

/** Highlevel gyro readout function
 *
 * Reads out the gyro
 * @return -1 for error, 0 for no new samples and 1 for new sample
 */
static int8_t 
helper_read_gyro(void)
{
  /* attempt to read */
  result = read(gyro_fd, gyro_data, sizeof(gyro_data));

  /* see if result is lower than 0 */
  if( result < 0 )
  { 
    #if PRINT_GYROVALUES /* only print if debug printing on */
    printf( "Gyro fail %d\n", result );
    #endif
    return -1;
  }

  /* see if it worked and return a value according to that */
  switch( result )
  { /* first case: all went well, we got a sample */
    case 1:
    case 2:
      /* not enough samples, return error */
      #if PRINT_GYROVALUES /* only print if debug printing on */
      printf( "Gyro: partial sample: %d points\n", result );
      #endif
      return -1;
      break;

    /* no sample, no error */
    case 0:
      return 0;
      break;

    /* 3 or more samples are there */
    default: 
      #if PRINT_GYROVALUES /* only print if debug printing on */
        printf("Gyro  X: %5.1d, Y: %5.1d, Z: %5.1d\n\n", 
          gyro_data[0], gyro_data[1], gyro_data[2] );
      #endif
      return 1;
      break;
  }
}





/** Servo IO **/
/** PWM servo init function
 *
 *  Initializes and arms the pwm servo outputs
 */
static void 
helper_init_pwm(void)
{
  /* start pwm  */
  up_pwm_servo_init(0xf);
  up_pwm_servo_arm( true );

}

/** PWM read function
 *
 * Read out all channels and store in pwm_data, if someone ever wants to 
 * do this for some reason
 */
static void 
helper_read_pwmdata(void)
{
  /* read out servo data from the servo IO file */
  result = read(pwm_fd, pwm_data, sizeof(pwm_data));

  /* see if we got the number of bytes we expected */
  if (result != sizeof(pwm_data)) {
    printf("PWM: failed bulk-reading channel values\n");
  }
}


/** Set pwm channel
 *
 *  Set the pwm channels. For breaking, set the motor PWMs both to high. 
 *  This is not tested yet 
 *@param channel
 *            - Channel 0 is Servo FIN (Probably left? :D Don't know yet)
 *            - Channel 1 is Servo RIN (See above :D)
 *            - Channel 2 is Motor FIN (forward)
 *            - Channel 3 is Motor RIN (backwards)
 *@param value Pulse high duration in microseconds.
               Period is specified in up_nsh.c 
               as the update_rate part of the pwm_servo_config 
               (1/update_rate = period, hence 0 = no pulse, full period = high)
 */
inline static void 
subhelper_set_pwmchan( uint8_t channel, uint16_t value )
{
  up_pwm_servo_set( channel, value );
}

/** Helper so set the pwm channels
 *  This is used to transfer the values stored in pwm_data to the servos
 */
static void 
helper_set_pwm( void )
{
  /* Saturate values to PWM boundaries */
  int i;
  for(i=0; i < 4; i++)
  {
    if(pwm_data[i] > PWM_MAX)
    {
      pwm_data[i] = PWM_MAX;
    }
  }
  if((pwm_data[0] == 0) && (pwm_data[1] == 0))
  {
    pwm_data[0] = PWM_MAX;  
    pwm_data[1] = PWM_MAX;  
  }

  if((pwm_data[2] == 0) && (pwm_data[3] == 0))
  {
    pwm_data[2] = PWM_MAX;  
    pwm_data[3] = PWM_MAX;  
  }

  /* set all channels to the values stored in pwm_data. */
  subhelper_set_pwmchan( 0, pwm_data[0] ); 
  subhelper_set_pwmchan( 1, pwm_data[1] );
  subhelper_set_pwmchan( 2, pwm_data[2] );
  subhelper_set_pwmchan( 3, pwm_data[3] );
}

void ctrl_thread();

/** Calibrate the poti  
 * 
 * Gets the steering boundaries (change with input voltage...) and intializes
 * the PID controller for the steering
 */
inline static void
helper_init_steering(void)
{
  /* intermediate storage */
  int32_t adc_boundaries[2]= {0};
  /* Set it to one side, wait a moment to settle,
   * then readout the ADC 
   */
  pwm_data[0] = PWM_MAX;
  pwm_data[1] = 0;
  helper_set_pwm();
  usleep(2000000);

  helper_read_adc_current();
  adc_boundaries[0] = adc_data[2].am_data;

  /* Set it to the other side, wait a moment to settle,
   * then readout the ADC 
   */
  pwm_data[1] = PWM_MAX;
  pwm_data[0] = 0;
  helper_set_pwm();
  usleep(2000000);

  helper_read_adc_current();
  adc_boundaries[1] = adc_data[2].am_data;

  /*Reset to 0 */ 
  pwm_data[1] = 0;
  helper_set_pwm();


  printf("ADC_BOUNDARIES: 0: %i, 1: %i, POT_SCALE: %i, POT_MID: %i\n",
      adc_boundaries[0], adc_boundaries[1], adc_values.scale_pot, adc_values.mid_pot);
  /* Find the minimum and maximum and write them to the
   * Appropriate value in adc_values.
   */
  if( adc_boundaries[0] > adc_boundaries[1] )
    adc_values.scale_pot = (uint16_t)(adc_boundaries[0] - adc_boundaries[1])/2;
  else
    adc_values.scale_pot = (uint16_t)(adc_boundaries[1] - adc_boundaries[0])/2;

  adc_values.mid_pot = (uint16_t)(adc_boundaries[0] + adc_boundaries[1])/2;


  /*Disable the anti-windup for the moment, use satuaration
   * The last value seems to be for disabling or enabling plotting.
   * We disable it, since it is not used in the function anyway...
   * Set mode to 0 to disable the use of a gyro in the function and enable
   * that the derivativ is computed internally
   */
  pid_init( &steering_controller, STEERING_KP, STEERING_KI, STEERING_KD, 0.0f, 0, 0);

}



/****************************************************************************
 * Runtime helper functions for highlevel abstract functionality
 ****************************************************************************/

/** Battery status calculation
 * 
 * Gives back the battery level from 0-100%
 */
static uint8_t
helper_calc_battery_status(void)
{
  uint8_t i; 
  result=0;
  for(i = 0; i < 51; i++)
  {
    if(adc_values.battery_voltage > lipo_discharge[i])
    {
      /* The array begins with the highest value */
      result = 100 - i;
      break;
    }
  }
  return result;
}

/** Lowpass for the steering
 *
 * We need to lowpass filter the ADC input since we have less play on the 
 * ADC than in the gears.
 */
inline static void
sub_sub_helper_calc_steering(void)
{
  /** Shift the new value in the first spot of the array **/
  /* Move'em back */
  uint8_t i;
  for(i = 0; i < FILTER_ORDER; i++)
    steering_filter[i+1] = steering_filter[i];
  /* And put the new fellow at the now empty spot */
  steering_filter[0] = adc_values.poti_wiper;

  /** Now we average over the array **/
  for(i=0; i < FILTER_ORDER; i++)
    steering_position += steering_filter[i];
  /* Scale it back with 1/FILTER_ORDER */
  steering_position = steering_position/((float)FILTER_ORDER);
}




/** Steering servo update routine
 *
 *  This function updates the steering and writes the appropriate pwm values
 */
inline static void
sub_helper_calc_steering( void )
{

  /* Calculate the filtered steering_position */
  //sub_sub_helper_calc_steering();

  /* We use the PID controller provided in systemlib/pid */
  float offset;

  /* calculate delta T for the integral */
  const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
  last_run = hrt_absolute_time();

  /* Use 0 for derivativ (dot_val), so the routine calculates it by itself */
  offset = pid_calculate( &steering_controller, steering.setval,
      adc_values.poti_wiper, 0, deltaT);


  /* update PWM */
  if( offset < 0 ) 
  { /* go into this direction*/
    pwm_data[0] = (int16_t) (-1.0f*offset*(float)steering.scale); 

    /* set other direction to 0 */
    pwm_data[1] = 0;
  }
  else
  { /* go into the other direction */
    pwm_data[1] = (int16_t) (offset*(float)steering.scale); 

    /* set other direction to 0 */
    pwm_data[0] = 0;
  }
#if PRINT_PWMVALUES == 1
  printf("PWM: Offset: %0.2f, MID: %i, Chan0: %i, Chan1: %i\n", offset,
      adc_values.mid_pot, pwm_data[0], pwm_data[1]);
#endif

}


/** Throttle controller 
 *
 *  The routine that updates the throttle PWM. 
 */
inline static void
sub_helper_calc_throttle( void )
{  

#if ESTIMATE_RPM
   throttle_act = control_motor( adc_values.motor_current, throttle.setval );
   throttle.setval = throttle_act[0];
#endif /* ESTIMATE_RPM */

  /* update PWM */
  if( throttle.setval < 0 ) 
  { /* go into one direction  */
    pwm_data[2] = 0;//(int16_t) (-1.0f*throttle.setval*(float)throttle.scale);

    /* set other direction to 0 */
    pwm_data[3] = 0;
  }
  else
  { /* go into the other direction */
    pwm_data[3] = (int16_t) (throttle.setval*throttle.scale); 

    /* set other direction to 0 */
    pwm_data[2] = 0;
  }
  #if PRINT_PWMVALUES == 1
    printf("PWM: Scale: %i, Chan2: %i, Chan3: %i\n", throttle.scale, pwm_data[2], pwm_data[3]);
  #endif

}

/** State update
 *
 *  This routine calls all the state update functions 
 */
static void 
helper_calc_states()
{
  /* steering */
  sub_helper_calc_steering();

  /* throttle */
  sub_helper_calc_throttle();
}


/****************************************************************************
 * Main runtime function
 *************************************************************************/
int 
orcactrl_main(int argc, char *argv[])
{
  /* initialization */
  printf( "Initializing peripherial access...\n" );
  lmx9830_init(); /* init bluetooth */
  /* Initialize the timeout timer */
  timeout = TIMEOUT_CYCLES;

#if SELFCONNECT
  /* Connect Bluetooth */
  /* First we connect to the realtime system. This is vital, so we only
   * proceed on succes.
   * ATM we use the same rfc (rfcomm_port) for local and remote.
   */
  printf("Trying to connect...");
  while( lmx9830_connect(REALTIME_RFC, REALTIME_RFC,
        const_realtime_btaddr));
#endif

  /* Low level init */
  helper_init_adc(); /* init adc */
  helper_init_pwm(); /* init pwm */
  helper_init_accel(); /* init accelerometer */
  helper_init_gyro(); /* init gyro */

  /*High level init */
  helper_init_steering(); /*Init the steering */

#if PHONE
  /*tmp for sending on bt when using the phone*/
  uint8_t tmp;
#endif

  /* main loop */
  printf( "Starting loop...\n" );
  while( 1 /* maybe some exit condition here at some point? */ )
  { 
    /***** Stage 1: Receive information *****/

    /* poll bluetooth and execute commands received on it */
    helper_poll_bluetooth();


#if PHONE
    /* The Bt steering app awaits the char 'r' to answer ^^*/
    /* We put it here to increase changes that the reply is ready
     * once we come to helper_poll_bluetooth() again.
     */
    tmp = (uint8_t) 'r';
    lmx9830_send(REALTIME_RFC, &tmp, 1);
#endif

    /***** Stage 2: Sensor readouts *****/
    /* update ADC values (stored in adc_data) */
    helper_read_adc_current();

    /* update Accelerometer readings TODO error handling */
    helper_read_accel();

    /* update gyro readings TODO error handling */
    helper_read_gyro();


    /***** Stage 3: State update calculation *****/
    /* update states */
    helper_calc_states();


    /***** Stage 4: Actuator updates *****/
    /* update PWM outputs */
    helper_set_pwm();


    /***** Stage 5: Send information *****/
#if BR
#if FULL_PROTOCOL
    helper_send_packet(helper_calc_battery_status(), adc_data[0], adc_data[1],
        gyro_data[0], gyro_data[1]);
#endif /* Full Protocol */
    helper_send_packet(helper_calc_battery_status(), 0, 0, 0, 0);
#endif /* B&R */
    
    /* Print the Battery status */
    //printf("Battery:%i\n", helper_calc_battery_status());
//    printf("Battery:%0.2f\n", adc_values.battery_voltage);


    /***** Stage 6: Loop Tuning and prescaler Updates *****/
    /* For functions that run with lower speed */
    //loop_prescaler++;

    /* Use this to tune the loop speed. Keep in mind, the ADC read routine
     * has an additional usleep(600) to compensate a bug in the Software.
     */
    usleep(LOOP_TUNING);


  }
  /* not really reached as of now. */
  return 0;

}


