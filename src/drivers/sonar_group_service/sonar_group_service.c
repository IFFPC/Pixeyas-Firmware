/**
 * @file sonar_group_service.c
 *
 * SRF01 ultrasonic rangefinder
 *
 * @ author: Lingyu Yang <yanglingyu@buaa.edu.cn>
 *	     Xiaoke Yang <das.xiaoke@hotmail.com>
 * @ last modified: 2016-05-05
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>

#include <stdlib.h>
#include <poll.h>
#include <string.h>

#include "sonar_group_topic.h"

#include <termios.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>

/* Define UART8_TX as an output pin, this is no longer used.
 * #define GPIO_UART8_TX_BREAK       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)
*/



/*  address of the 1st SFR01 */
#define SRF01_SONAR1_ADDRESS		    0x01            
/*  address of the 2nd SFR01 */
#define SRF01_SONAR2_ADDRESS		    0x02            
/*  address of the 3rd SFR01 */
#define SRF01_SONAR3_ADDRESS		    0x03            
/*  address of the 4th SRF01 */
#define SRF01_SONAR4_ADDRESS		    0x04            
/*  address of the 5th SRF01 */
#define SRF01_SONAR5_ADDRESS		    0x05            

/* Real Ranging Mode - result in inches */
#define SRF01_REAL_RANGE_INCH		    0x50		
/* Real Ranging Mode - result in centimeters */
#define SRF01_REAL_RANGE_CM		    0x51		
/* Real Ranging Mode - result in inches, automatically Tx range back 
 * to the controller as soon as ranging is complete.  */
#define SRF01_AUTO_REAL_RANGE_INCH	    0x53		
/* Real Ranging Mode - Result in centimeters, automatically Tx range back 
 * to the controller as soon as ranging is complete. */
#define SRF01_AUTO_REAL_RANGE_CM	    0x54		

/* Fake Ranging Mode - Result in inches */
#define SRF01_FAKE_RANGE_INCH		    0x56		
/* Fake Ranging Mode - Result in centimeters */
#define SRF01_FAKE_RANGE_CM		    0x57		
/* Fake Ranging Mode - Result in inches, automatically Tx range back 
 * to the controller as soon as ranging is complete. */
#define SRF01_AUTO_FAKE_RANGE_INCH	    0x59		
/* Fake Ranging Mode - Result in centimeters, automatically Tx range back 
 * to the controller as soon as ranging is complete. */
#define SRF01_AUTO_FAKE_RANGE_CM	    0x5A            

/* Transmit an 8 cycle 40khz burst - no ranging takes place */
#define SRF01_SEND_BURST		    0x5C		
/* Get software version - sends a single byte back to the controller */
#define SRF01_GET_SOFTWARE_VERSION	    0x5D            
/* Get Range, returns two bytes (higher byte first) from the most recent 
 * ranging. */
#define SRF01_GET_RANGE			    0x5E		
/* Get Status, returns a single byte. Bit 0 indicates "Transducer locked". 
 * Bit 1 indicates "Advanced Mode" */
#define SRF01_GET_STATUS		    0x5F		
/* Sleep, shuts down the SRF01 so that it uses very low power (55uA). */
#define SRF01_SLEEP			    0x60		
/* Unlock. Causes the SRF01 to release and re-acquire its "Transducer lock". 
 * Used by our factory test routines. */
#define SRF01_UNLOCK			    0x61		
/* Set Advanced Mode (Factory default) - When locked, SRF01 will range 
 * down to zero. */
#define SRF01_SET_ADVANCED_MODE		    0x62		
/* Clear Advanced Mode - SRF01 will range down to approx. 12cm/5in, */
#define SRF01_CLEAR_ADVANCED_MODE	    0x63		
/* Changes the baud rate to 19200 */
#define SRF01_SET_BAUDRATE_19200	    0x64		
/* Changes the baud rate to 38400 */
#define SRF01_SET_BAUDRATE_38400	    0x65		

/* 1st in sequence to change I2C address */
#define SRF01_SET_ADDRESS_FIRST_BYTE	    0xA0            
/* 3rd in sequence to change I2C address */
#define SRF01_SET_ADDRESS_THIRD_BYTE 	    0xA5            
/* 2nd in sequence to change I2C address */
#define SRF01_SET_ADDRESS_SECOND_BYTE 	    0xAA            


/* invalid value for range measurement, -1979 is a magic number */
#define SRF01_INVALID_RANGE_VALUE	    -1979
/* the device path of the sonar group */
#define SONAR_GROUP_DEVICE_PATH		    "/dev/ttyS6"

ORB_DEFINE(SonarGroupDistance, struct SonarGroupDistance_s);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

/* the entry point */ 
__EXPORT int sonar_group_service_main(int argc, char *argv[]);
/* the main thread */
int sonar_group_service_thread_main(int argc, char *argv[]);
/* set baudrate */ 
static bool set_uart_baudrate(const int fd, unsigned int baud);
/* send a command to srf01 */ 
static bool srf01_send_command(int fd,unsigned char address, unsigned char cmd);
/* get range measurements */ 
static int srf01_get_range(int fd,unsigned char address);
/* print usage messages */
static void usage(const char *reason);
/* set singlewire mode for the uart */
static bool set_uart_singlewire_mode(const int fd);


/****************************************************************************
Name: usage 

   Description:
     This function display a warning message or the usage of the command 
   Input Parameters:
     reason - warning message to be displayed 
   Returned Value:
     None 
****************************************************************************/
static void usage(const char *reason)
{
    if (reason) {
        warnx("%s", reason);
    }
    errx(1, "usage: sonar_group_service {start|stop|status|test}");
}

/****************************************************************************
Name: sonar_group_service_main 

   Description:
     This function is the entry point of the servie, i.e. the main function 
   Input Parameters:
     argc - number of arguments
     argv - list of arguments 
   Returned Value:
     0 if successful, nonzero if error occurs 
 ****************************************************************************/
int sonar_group_service_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("already running");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("sonar_group_service",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_MAX - 5,
                                         2000,
                                         sonar_group_service_thread_main,
                                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return(0);
    }
    /* stop the service */
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return(0);
    }
    /* query status of the process */
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");
        } else {
            warnx("stopped");
        }
        return(0);
    }
    /* perform a ranging test */
    if (!strcmp(argv[1], "test")) {
        return(0);
    }
    /* otherwise display an error message */
    usage("unrecognized command");
    /* return the error status */
    return(1);
}



/****************************************************************************
Name: sonar_group_service_thread_main 

   Description:
     This function is the main thread when the service is started
   Input Parameters:
     argc - number of arguments
     argv - list of arguments 
   Returned Value:
     0 if successful, nonzero if error occurs 
 ****************************************************************************/
int sonar_group_service_thread_main (int argc, char *argv[])
{
    /* open the uart , the options are O_RDWR - read and write
     * O_NOCTTY - not controlling tty 
     * O_NONBLOCK - non-blocking IO */
    int fd = open(SONAR_GROUP_DEVICE_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
    /* return if error occurs */
    if (fd < 0) {
        errx(1, "failed to open port: %s", SONAR_GROUP_DEVICE_PATH);
	return -1;
    }
    /* set the baudrate to be 9600, the default one for SRF01 when powered up */
    if(set_uart_baudrate(fd,9600) == false){
        warnx("set uart baudrate failed");
        return -1;
    }
    /* set singlewire mode for the uart */
    if(set_uart_singlewire_mode(fd) == false){
        warnx("set uart single wire mode failed");
        return -1;
    }
    /* PX4_INFO won't add the name of the process before the message, thus
     * [SONAR GROUP] is added to identify the source of the message */
    PX4_INFO("[SONAR GROUP] uart initialisation successful");
    thread_running = true;

    //初始化数据结构体
    struct SonarGroupDistance_s sonardata;
    memset(&sonardata, 0, sizeof(sonardata));
    //公告主题
    orb_advert_t SonarGroupDistance_pub = orb_advertise(ORB_ID(SonarGroupDistance), &sonardata);

    PX4_INFO("[SONAR GROUP] service started ");

    //获取所有超声波距离 Sonar_group GetRangeAll
    while(!thread_should_exit) {
	/* send a ranging request to all the rangefinders connected */
        if(srf01_send_command(fd,0,SRF01_REAL_RANGE_CM)==false){
	    errx(1, "srf01_send_command failed.");
	    return 1;
	}
        /* wait 70ms for responses */
        usleep(70000);
        for (int i=1; i<=5; i++){
            int range=srf01_get_range(fd,i);
            if (range!=SRF01_INVALID_RANGE_VALUE){
                sonardata.sonar_group_distance[i-1]=range;
                sonardata.sonar_group_status[i-1]=1;
            }else{
                sonardata.sonar_group_status[i-1]=0;
            }
        }
        orb_publish(ORB_ID(SonarGroupDistance), SonarGroupDistance_pub, &sonardata);
    }

    /* close the uart */
    warnx("exiting.");
    close(fd);
    thread_running = false;
    fflush(stdout);  /* why flush stdout? */
    return 0;
}


/****************************************************************************
Name: srf01_send_command

   Description:
     This function sends a command to the srf01 rangefinder specified by 
	the address provided. 
   Input Parameters:
     fd - file descriptor for the uart
     address - address of the target srf01 rangefinder 
     command - command to be sent
   Returned Value:
     true if command is sent successfully, false otherwise
 ****************************************************************************/
static bool srf01_send_command(int fd, unsigned char address, unsigned char command)
{
	/* The following code sends a break frame on TX pin using GPIO mode
	 * on TX pin. This is no longer used due to its portability.
	 */
	/**
	 * // Configure the UART8_TX as an general purpose output pin.
	 * stm32_configgpio(GPIO_UART8_TX_BREAK);
	 * // Send a 2ms break (0s) to begin communications with the SRF01
	 * stm32_gpiowrite(GPIO_UART8_TX_BREAK,0);
	 * usleep(2000);
	 * stm32_gpiowrite(GPIO_UART8_TX_BREAK,1);
	 * // Configure the UART8_TX as an UART
	 * stm32_configgpio(GPIO_UART8_TX);
	 */
	/* start sending break frames on TX*/
	if(ioctl(fd, TIOCSBRK,0)<0){
	    return false;
	}
	/* wait for 2ms */
	usleep(2000);
	/* stop sending break frames */
	if(ioctl(fd, TIOCCBRK,0)){
	    return false;
	}
	/* send the address byte */
	if(write(fd, &address, sizeof(address))<=0){
	    return false;
	}
	/* send the command byte */
	if(write(fd, &command, sizeof(command))<=0){
	    return false;
	}

	/* In single-wire operation, the RX and TX connected internally. 
	 * Thus the data sent out will be received and have to be discarded.
	 * There has not been effective ways to clear/flush the receive buffer.
	 * We have to work around by performing read action instead. 
	 * This read action suffers from problem when using BLOCKING IOs. Thus
	 * when opening the UART, the option O_NONBLOCK has to be used.
	 * Not quite sure whether there will be side effects. 
	 */
	/* Not quite sure why we need to wait, but there will be problems 
	 * if we do not. */
	usleep(3000);
	unsigned char dummy = 0;
	/* SRF01 commands are nonzero. */
	while(dummy != command){
		/* read until all the sent data are received. */
		if(read(fd, &dummy, 1)<=0){
		    return false;
		}
	}
	return true;
}

/****************************************************************************
Name: srf01_get_range

   Description:
     This function returns the range measurement from the srf01 rangefinder 
	specified by the address provided. 
   Input Parameters:
     fd - file descriptor for the uart
     address - address of the target srf01 rangefinder 
   Returned Value:
     range measurement or SRF01_INVALID_RANGE_VALUE if error occurs
 ****************************************************************************/
static int srf01_get_range(int fd,unsigned char address)
{
    /* send a command to the SRF01 sensor to get its range measurement */
    if (srf01_send_command(fd,address,SRF01_GET_RANGE) == false){
	errx(1, "error in sending command to srf01 ");
        return SRF01_INVALID_RANGE_VALUE;
    }
    /* wait 1ms */
    usleep(1000);
    /* there are two bytes to be returned */
    unsigned char higher_byte,lower_byte;
    /* the status of the two read actions */
    int higher_byte_status, lower_byte_status;
    /* read the two types in a row */
    higher_byte_status = read(fd, &higher_byte, 1);
    lower_byte_status = read(fd, &lower_byte, 1);
    /* if any one of the two bytes are not correctly received*/
    if ((higher_byte_status!=1) || (lower_byte_status!=1)){
        return SRF01_INVALID_RANGE_VALUE;
    }else{
	/* combine the two bytes into an integer */
        int range = ((higher_byte<<8)+lower_byte);
	/* return the result */
        return range;
    }
}

/****************************************************************************
Name: set_uart_baudrate 

   Description:
     This function sets the baudrate of the uart specified by fd 
   Input Parameters:
     fd - file descriptor for the uart
     baud - the target baudrate 
   Returned Value:
     true if successful, and false otherwise 
 ****************************************************************************/
static bool set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;
    switch (baud) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
        warnx("error in setting baudrate to be: %d\n", baud);
        return false;
    }
    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("error setting cfsetispeed: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("error setting crsetospeed: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("error setting tcsetattr: %d (tcsetattr)\n", termios_state);
        return false;
    }
    return true;
}
/****************************************************************************
Name: set_uart_singlewire_mode 

   Description:
     This function sets the uart specified by fd to work in singlewire mode
   Input Parameters:
     fd - file descriptor for the uart
   Returned Value:
     true if successful, and false otherwise 
 ****************************************************************************/
static bool set_uart_singlewire_mode(const int fd)
{
    /* activate single wire mode */
    int ret = ioctl(fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
    /* display the error message if ioctl failed */
    if (ret < 0){
	warnx("setting singlewire mode failed: %d", errno);
	return false;
    }
    return true;
}
