#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>


//根据“Pixhawk-透过串口方式添加一个自定义传感器（超声波为例）”文件添加
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include "stm32_gpio.h"
//添加结束

//#define GPIO_UART8_RX           (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN0)
//#define GPIO_UART8_TX           (GPIO_ALT|GPIO_AF8|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN1)

////下面这两行还需要改
#define GPIO_UART8_TX_BREAK       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)
//#define SRF01_READ              (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_SPEED_50MHz|GPIO_PORTE|GPIO_PIN0)


#define SONAR01                 0x01            // Address of the SFR01
#define SONAR02                 0x02            // Address of the SFR02
#define SONAR03                 0x03            // Address of the SFR03
#define SONAR04                 0x04            // Address of the SFR04
#define SONAR05                 0x05            // Address of the SFR05

#define RANGE_INCH 		0x50		//Real Ranging Mode - Result in inches
#define RANGE_CM		0x51		//Real Ranging Mode - Result in centimeters
#define QRANGE_INCH		0x53		//Real Ranging Mode - Result in inches, automatically Tx range back to controller as soon as ranging is complete.
#define QFASTRANGE_CM           0x54		//Real Ranging Mode - Result in centimeters, automatically Tx range back to controller as soon as ranging is complete.

#define FAKERANGE_INCH          0x56		//Fake Ranging Mode - Result in inches
#define FAKERANGE_CM            0x57		//Fake Ranging Mode - Result in centimeters
#define QFAKERANGE_INCH 	0x59		//Fake Ranging Mode - Result in inches, automatically Tx range back to controller as soon as ranging is complete.
#define QFAKERANGE_INCM         0x5A            //Fake Ranging Mode - Result in centimeters, automatically Tx range back to controller as soon as ranging is complete.

#define SENDBURST		0x5C		//Transmit an 8 cycle 40khz burst - no ranging takes place
#define GETVERSION 		0x5D            //Get software version - sends a single byte back to the controller
#define GETRANGE 		0x5E		//Get Range, returns two bytes (high byte first) from the most recent ranging.
#define GETSTATUS		0x5F		//Get Status, returns a single byte. Bit 0 indicates "Transducer locked", Bit 1 indicates "Advanced Mode"
#define SLEEP		 	0x60		//Sleep, shuts down the SRF01 so that it uses very low power (55uA).
#define UNLOCK			0x61		//Unlock. Causes the SRF01 to release and re-acquire its "Transducer lock". Used by our factory test routines.
#define ADVANCEDMODE            0x62		//Set Advanced Mode (Factory default) - When locked, SRF01 will range down to zero.
#define CLEARADVANCED           0x63		//Clear Advanced Mode - SRF01 will range down to approx. 12cm/5in,
#define BAUD19200		0x64		//Changes the baud rate to 19200
#define BAUD38400		0x65		//Changes the baud rate to 38400

#define FIRSTI2C		0xA0            //1st in sequence to change I2C address
#define THIRDI2C 		0xA5            //3rd in sequence to change I2C address
#define SECONDI2C 		0xAA            //2nd in sequence to change I2C address




//声纳主函数
__EXPORT int sonar_group_main(int argc, char *argv[]);

//UART初始化，默认选择UART7
static int uart_init(char * uart_name);
//设置波特率
static int set_uart_baudrate(const int fd, unsigned int baud);

//发送SRF指令
void SRF01_Cmd(int fd_UART,unsigned char Address, unsigned char cmd);
//读取测量距离
int GetRange(int fd_UART,unsigned char Address);
// 获取声纳工作状态
int GetStatus(int fd_UART,unsigned char Address);
// 获取版本号
int GetVersion(int fd_UART,unsigned char Address);


int sonar_group_main(int argc, char *argv[])
{
    if (argc!=2)
    {
        printf("Please try Sonar_group RANGE|OFF\n");
        return -1;
    }

    /*初始化GPIO*/
    //stm32_configgpio(SRF01_WRITE);
    //printf("stm32_configgpio set!\n");


    /*初始化UARt8*/
    int uart_fd = uart_init("/dev/ttyS6");
    if(false == uart_fd)return -1;
    if(false == set_uart_baudrate(uart_fd,9600)){
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[YCM]uart init is successful\n");


    if(!strcmp(argv[1],"test"))  {
        char sample_test_uart[25];
        char bufftest[25];
        char bufftest2[25];
        char bufftest3[25];
        int n,j,k,l;


        n = sprintf(sample_test_uart, "SAMPLE without GPIO,%d", 1);
        write(uart_fd, sample_test_uart, n);
        printf("sample_test_uart=%s，n=%d\n",sample_test_uart,n);
        usleep(3000);
        j=read(uart_fd,&bufftest,n);
        printf("recive=%d,bufftest=%s\n",j,bufftest);

        stm32_configgpio(GPIO_UART8_TX_BREAK);
        stm32_gpiowrite(GPIO_UART8_TX_BREAK, 0);
        stm32_configgpio(GPIO_UART8_TX);

        n = sprintf(sample_test_uart, "SAMPLE with    GPIO,%d", 2);
        write(uart_fd, sample_test_uart, n);
        printf("sample_test_uart=%s,n=%d\n",sample_test_uart,n);
        usleep(3000);
        k=read(uart_fd,&bufftest2,n);
        printf("recive=%d,bufftest2=%s\n",k,bufftest2);


        if(false == set_uart_baudrate(uart_fd,9600)){
            printf("[YCM]set_uart_baudrate is failed\n");
            return -1;
        }
        n = sprintf(sample_test_uart, "SAMPLE without GPIO,%d", 3);
        write(uart_fd, sample_test_uart, n);
        printf("sample_test_uart=%s,n=%d\n",sample_test_uart,n);
        usleep(3000);
        l=read(uart_fd,&bufftest3,n);
        printf("recive=%d,bufftest2=%s\n",l,bufftest3);
        return 1;


    }


    if(!strcmp(argv[1],"RANGE"))  {
        SRF01_Cmd(uart_fd,SONAR01,RANGE_CM);
        //wait 70ms to recive the respons
        usleep(70000);
        GetRange(uart_fd,SONAR01);
        return 1;
    }

    if(!strcmp(argv[1],"OFF"))
    {
        close(uart_fd);
        return 1;
    }

    printf("Wrong Input\n");
    printf("Please try Sonar_group RANGE|OFF");
    return -1;

}


// Function to send commands to the SRF01
void SRF01_Cmd(int fd_UART,unsigned char Address,unsigned char cmd)
{
    char addtemp[10];
    char cmdtemp[10];
    sprintf(addtemp,"%d",Address);
    sprintf(cmdtemp,"%d",cmd);

    //配置成ＧＰＩＯ模式，发送高低电平“break"
    stm32_configgpio(GPIO_UART8_TX_BREAK);
    // Send a 2ms break to begin communications with the SRF01
    stm32_gpiowrite(GPIO_UART8_TX_BREAK,0);
    usleep(20000);
    stm32_gpiowrite(GPIO_UART8_TX_BREAK,1);
    usleep(10);

    //配置成URAT模式，发送指令
    stm32_configgpio(GPIO_UART8_TX);
    write(fd_UART,&Address,sizeof(Address));
    write(fd_UART,&cmd,sizeof(cmd));


    // As RX and TX are the same pin it will have recieved the data we just sent out, as we dont want this we read it back and ignore it as junk before waiting for useful data to arrive

    usleep(3000);
    unsigned char buff2[20];
    int readlength2=0;
    readlength2=read(fd_UART,&buff2,20);
    printf("readlength=%d\n",readlength2);
    printf("buff2=%d\n",buff2);
    printf("ADD=%s\n",addtemp);
    printf("cmd=%s\n",cmdtemp);



}

int GetRange(int fd_UART,unsigned char Address)
{
    SRF01_Cmd(fd_UART,Address,GETRANGE);
    unsigned char hByte,lByte;
    int readH=read(fd_UART,&hByte,1);
    int readL=read(fd_UART,&lByte,1);
    int range = ((hByte<<8)+lByte);
    printf("readH,readL=%d,%d",readH,readL);
    printf("Range=%d\n",range);
    return range;
}

// 获取声纳工作状态
int GetStatus(int fd_UART,unsigned char Address)
{
    SRF01_Cmd(fd_UART,Address,GETSTATUS);
    unsigned char Status;
    read(fd_UART,&Status,1);
    if (Status=='0')
    {
        printf("Transducer locked");
    }
    if (Status=='1')
    {
        printf("Advanced Mode");
    }
    return Status;
}

// 获取版本号
int GetVersion(int fd_UART,unsigned char Address)
{
    SRF01_Cmd(fd_UART,Address,GETVERSION);
    unsigned char Version;
    read(fd_UART,&Version,1);
    printf("Version=%u",Version);
    return Version;
}





int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
        warnx("ERR: baudrate: %d\n", baud);
        return -EINVAL;
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
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}

int uart_init(char * uart_name)
{
    // 可读写|如果欲打开的文件为终端机设备时，则不会将该终端机当成进程控制终端机|非阻塞
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}
