#define version 1810031
#include <stdio.h>
#include <unistd.h>   //Used for UART
#include <fcntl.h>   //Used for UART
#include <termios.h>   //Used for UART
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <dirent.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include <syslog.h>

#define cTIDlen   8
#define cMaxNoTID  1024
#define cRecordLen  26//20											//XX XX XX XX  XX XX XX XX(8 bytes tap id) XX(1 byte status) XX(1 byte RSSI) ??(unuse) XX XX XX XX (4 bytes last arriving time) XX XX XX XX(4 bytes first Arriving time) XX(Exist?)

#define LED 21
#define BCM2708_PERI_BASE        0x3F000000 //0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE  (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

#define uart_buffer_len 2048
#define reuse_uart_buffer_len 512
#define TT 1
#define PT 0
volatile unsigned char g_rx_buffer[uart_buffer_len];
volatile unsigned char * w_ptr = &g_rx_buffer[0];
volatile unsigned char * r_ptr = &g_rx_buffer[0];
volatile unsigned char g_reuse_buffer[reuse_uart_buffer_len];
//volatile int g_no_uart=0;
volatile int g_count_reuse_buffer = 0;
volatile unsigned char g_no_uart_flag = 0;

int gUart0_filestream = -1;
time_t gCurrent_time;
int gNoTap = 0;
#define max_record_instead 1024     //lung 2015/5/20
int no_record_instead = 0; //lung 2015/5/20

unsigned char gStart[] = {0x02, 0x06, 0x07, 0x01, 0x02, 0x03}; //start command for the tap reciever
unsigned char gStop[] = {0x02, 0x06, 0x07, 0x00, 0x03, 0x03};
unsigned char gGetSN[] = {0x02, 0x05, 0x51, 0x56, 0x03};
unsigned char gGetStatus[] = {0x02, 0x05, 0x53, 0x54, 0x03};
unsigned char gSaveTIDlen = 10; //length of a tap ID record	XX XX XX XX  XX XX XX XX(8 bytes tap id) XX(1 byte status) XX(1 byte RSSI)
volatile unsigned char gTap[cMaxNoTID][cRecordLen]; //current tap records

unsigned char noUnitByte(unsigned char pFirByte); //how many bytes to identify same tap id
void transmit(unsigned char *p_tx_buffer, int number); //data, length of date //tramsmin to tap reciever via usb uart
unsigned char receive(volatile unsigned char * p); //XX XX XX XX  XX XX XX XX(8 bytes tap id) XX(1 byte status) XX(1 byte RSSI), p is the saving adress
void btoa(volatile unsigned char *p_rx_buffer, char *p_rx_char, int length);
void get_time();
void matching(volatile unsigned char *p_tap); //compara to gTap, save new or updata the last arriving time
void leave(unsigned int min); //identify the tap leaved and write into txt file
unsigned char filter(volatile unsigned char * pDataBuf); //filt out unused tap id
void endOfHour();
unsigned int gTimeSpa = 60; //min. departure time (sec)
unsigned int gTimePeriod = 1; //Time period log (h)
unsigned int minRSSI = 80;
//20151005
unsigned char gApp = 0;
unsigned char clearRecord = 0;
int sensorInterval = 60;
#define Attendance 0
#define TemperatureSensor 1
#define TimeRecorder    2
//end
unsigned char SN[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
unsigned char gEndHour = 0x00;
#define rx_len  256
unsigned char l_rx_char[rx_len];
unsigned char led_status = 0;
unsigned char led1_status = 0;

char g_logLoca[30] = "/home/pi/log/";
char g_logfile[50];
char g_debug_logfile[50] = "/home/pi/";
unsigned char tcp = 0;
int total = 0;
int er = 0;

void write_uart_buffer(unsigned char input_data) {
    *w_ptr = input_data;
    if (w_ptr >= (g_rx_buffer + uart_buffer_len - 1))
        w_ptr = g_rx_buffer;
    else
        w_ptr++;
}

unsigned char read_uart_buffer_with_clear() {
    unsigned char data;

    int n;
    if (g_count_reuse_buffer > 0) {
        data = g_reuse_buffer[0];
        g_count_reuse_buffer--;
        for (n = 0; n < g_count_reuse_buffer; n++) {
            g_reuse_buffer[n] = g_reuse_buffer[n + 1];
        }
    } else {
        data = *r_ptr;
        if (r_ptr >= (g_rx_buffer + uart_buffer_len - 1))
            r_ptr = g_rx_buffer;
        else
            r_ptr++;
    }
    return data;
}

int len_uart_buffer() {
    if (w_ptr >= r_ptr)
        return (w_ptr - r_ptr + g_count_reuse_buffer);
    else
        return (w_ptr + uart_buffer_len - r_ptr + g_count_reuse_buffer);

}

int len_uart_buffer_w() {
    if (r_ptr > w_ptr)
        return (r_ptr - w_ptr);
    else
        return (r_ptr + uart_buffer_len - w_ptr);
}

void readPgConf() {
    FILE * pFile;
    pFile = fopen("/home/pi/uart.conf", "r");

    char buff[20];
    int k;
    if (pFile != NULL) {
        fscanf(pFile, "%s%*[^\n]", buff); //minRSSI
        minRSSI = atoi(buff);
        printf("%i\n", minRSSI);

        fscanf(pFile, "%s%*[^\n]", buff); //min departure time(sec)

        gTimeSpa = atoi(buff);
        printf("%i\n", gTimeSpa);

        fscanf(pFile, "%s%*[^\n]", buff); //Time period log (h)
        gTimePeriod = atoi(buff);
        printf("%i\n", gTimePeriod);

        fscanf(pFile, "%s%*[^\n]", buff); //log file location

        printf("%s\n", buff);
        //printf("%i\n",strlen (buff));
        sprintf(g_logLoca, "%s", buff);
        if (mkdir(g_logLoca, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) {
            printf("The directory does not exist and create directory\n");
        } else {
            printf("The directory exists\n");
        }
        //20151005
        fscanf(pFile, "%s%*[^\n]", buff); //application 
        gApp = atoi(buff);
        printf("%s\n", buff);
        fscanf(pFile, "%s%*[^\n]", buff); //sensorInterval 
        sensorInterval = atoi(buff);
        printf("%s\n", buff);
        //end
        fclose(pFile);
    }

    printf("minRSSI: %i;departure time: %i; period: %i; location: %s;\n", minRSSI, gTimeSpa, gTimePeriod, g_logLoca);
}

void readerConf() {
    FILE * pFile;
    pFile = fopen("/home/pi/em02.conf", "r");
    char buff[30];
    unsigned char outPcomm[20];
    int k = 0;

    if (pFile != NULL) {


        while (1) {
            k = 0;

            if (fscanf(pFile, "%s", buff) == EOF)
                break;

            outPcomm[k++] = strtol(buff, NULL, 16);
            printf("%02x", outPcomm[k - 1]);

            if (fscanf(pFile, "%s", buff) == EOF)
                break;
            outPcomm[k++] = strtol(buff, NULL, 16);
            printf("%02x", outPcomm[k - 1]);

            for (k; k < outPcomm[1]; k++) {
                if (fscanf(pFile, "%s", buff) == EOF)
                    break;
                outPcomm[k] = strtol(buff, NULL, 16);
                printf("%02x", outPcomm[k]);
            }
            if (fscanf(pFile, "%[^\n]", buff) == EOF)
                break;

            printf("	%s\n", buff);
            transmit(&outPcomm[0], outPcomm[1]);
            sleep(1);
        }
        fclose(pFile);
    }
}

void getCom() {

    struct dirent **namelist;
    const char* sysdir = "/dev/";

    int n = scandir(sysdir, &namelist, NULL, NULL);
    int i;
    if (n < 0)
        perror("scandir");
    else {
        for (i = 0; i < n; i++) {
            if (strncmp(namelist[i]->d_name, "ttyUSB", 6) == 0) {

                char devicedir[20];
                strcpy(devicedir, sysdir);
                strcat(devicedir, namelist[i]->d_name);
                printf("%s\n", devicedir);
                gUart0_filestream = open(devicedir, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
                if (gUart0_filestream == -1) {
                    //ERROR - CAN'T OPEN SERIAL PORT
                    printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
                    exit(-1);

                }

                struct termios options;
                struct termios termios;



                tcgetattr(gUart0_filestream, &options);
                options.c_cflag = B38400 | CS8 | CLOCAL | CREAD; //<Set baud rate 
                options.c_iflag = IGNPAR;
                options.c_oflag = 0;
                options.c_lflag = 0;
                // termios.c_lflag &= ~ICANON; /* Set non-canonical mode */
                // termios.c_cc[VTIME] = 1; /* Set timeout of 10.0 seconds */
                tcflush(gUart0_filestream, TCIFLUSH);
                tcsetattr(gUart0_filestream, TCSANOW, &options);
                syslog(LOG_INFO, "Com Port Connection Success");
                return;
            }
        }
    }
}

char *IP = "192.168.11.83";

void getTCP() {

    int sd, rc, length = sizeof (int);
    struct sockaddr_in serveraddr;
    char server[255];
    char temp;
    int totalcnt = 0;
    struct hostent *hostp;
    char data[100] = "This is a test string from client lol!!! ";


    /* get a socket descriptor */
    if ((gUart0_filestream = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Client-socket() error");
        exit(-1);
    }

    //timeout
    struct timeval tv;

    tv.tv_sec = 1; /* 30 Secs Timeout */
    tv.tv_usec = 0; // Not init'ing this can cause strange errors

    setsockopt(gUart0_filestream, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof (struct timeval));
    //setsockopt(gUart0_filestream, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv,sizeof(struct timeval));

    //server = "192.168.12.81";
    memset(&serveraddr, 0x00, sizeof (struct sockaddr_in));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(5000);
    //serveraddr.sin_addr.s_addr = inet_addr("192.168.12.190");
    serveraddr.sin_addr.s_addr = inet_addr(IP);
    signal(SIGPIPE, SIG_IGN);
    if ((rc = connect(gUart0_filestream, (struct sockaddr *) &serveraddr, sizeof (serveraddr))) < 0) {
        perror("Client-connect() error");
        close(sd);
        //exit(-1);
    } else {
        printf("Connection established...\n");
        syslog(LOG_INFO, "TCP Connection Success");
    }

    sleep(1);


}

void write_overflow_log() {
    FILE * pFile_debug;
    char datetime_str[50];

    get_time();
    pFile_debug = fopen("/home/pi/debug_log.txt", "a+");
    strftime(datetime_str, 50, "Overflow %F_%T \r\n", localtime(&gCurrent_time));
    fprintf(pFile_debug, datetime_str);
    fclose(pFile_debug);

}

void thread_function() {
    unsigned char thread_uart_buffer[256];
    int thread_uart_buffer_len;
    int thread_i;

    while (1) {
        if (gUart0_filestream != -1) {
            thread_uart_buffer_len = read(gUart0_filestream, thread_uart_buffer, sizeof (thread_uart_buffer));
            if (thread_uart_buffer_len == 0)
                usleep(1);
            // thread_uart_buffer_len = 0x13;//read(gUart0_filestream, thread_uart_buffer, sizeof(thread_uart_buffer));
            // thread_uart_buffer[0]=0x02;
            // thread_uart_buffer[1]=0x13;
            // thread_uart_buffer[2]=0x06;
            // thread_uart_buffer[3]=0x11;
            // thread_uart_buffer[4]=0x22;
            // thread_uart_buffer[5]=0x33;
            // thread_uart_buffer[6]=0x44;
            // thread_uart_buffer[7]=0x55;
            // thread_uart_buffer[8]=0x66;
            // thread_uart_buffer[9]=0x77;
            // thread_uart_buffer[10]=0x88;
            // thread_uart_buffer[11]=0x99;
            // thread_uart_buffer[12]=0x00;
            // thread_uart_buffer[13]=0x11;
            // thread_uart_buffer[14]=0x22;
            // thread_uart_buffer[15]=0x33;
            // thread_uart_buffer[16]=0x44;
            // thread_uart_buffer[17]=0x42;
            // thread_uart_buffer[18]=0x03;


            for (thread_i = 0; thread_i < thread_uart_buffer_len; thread_i++) {
                while (len_uart_buffer_w() <= 1) {
                    usleep(1);
                    // printf("over buff\n");
                    // write_overflow_log();
                }
                write_uart_buffer(thread_uart_buffer[thread_i]);
                //printf("%02X ",thread_uart_buffer[thread_i]);
                // FILE * pFile_in=fopen("/var/tmp/tmp.txt","a+");
                // fprintf(pFile_in,"%02X ",thread_uart_buffer[thread_i]);
                // fclose(pFile_in);
            }

        }

        // printf("No uart buffer:%i\n",g_no_uart);
    }

}

unsigned char get_led_status() {
    return led_status;
}

set_led_on() {
    //system("echo 1 >/sys/class/leds/led0/brightness");
    led_status = 1;
}

set_led_off() {
    //system("echo 0 >/sys/class/leds/led0/brightness");
    led_status = 0;
}

unsigned char get_led1_status() {
    return led1_status;
}

set_led1_on() {
    //system("echo 1 >/sys/class/leds/led1/brightness");
    led1_status = 1;
}

set_led1_off() {
    //system("echo 0 >/sys/class/leds/led1/brightness");
    led1_status = 0;
}

void update_logfile() {
    sprintf(g_logfile, "%s%02X%02X%02X%02X%02X%02X%02X%02X", g_logLoca, SN[0], SN[1], SN[2], SN[3], SN[4], SN[5], SN[6], SN[7]);
    strftime(&g_logfile[strlen(g_logfile)], 50, "_%F_%H_%M.txt", localtime(&gCurrent_time));
    printf("log file:%s\n", g_logfile);
    
    FILE * file=fopen(g_logfile, "a+");
    fprintf(file,"");
    fclose(file);
    
}

float toTemperature(unsigned char type, unsigned char dataH, unsigned char dataL) {

    short temp;
    switch (type) {
        case PT:
            temp = (signed short) dataH << 8 | dataL;
            return (float) temp * 0.03125;

            //int temp;
            /*if(dataH>=0x80){
                    dataL--;
                    dataL=~dataL;
                    dataH=~dataH;
                    temp=dataH<<8|dataL;
                    return (float)-0.03125*temp;
            }else{
                    temp=dataH<<8|dataL;
                    return (float)0.03125*temp;
            }*/
            break;

        case TT:
            temp = (signed short) dataH << 8 | dataL;
            if (temp >= 0x0800)
                temp |= 0xF800;
            return (float) temp * 0.0625;
            break;
        default:
            return (float) 0.0;
    }
}

void takeData(int interval) {
    if (interval % sensorInterval == 0) {
        char aa[50];

        double l_diff_time;
        int n, i, k;
        FILE * l_out;
        char l_out_char[120];
        char l_out_file[80];
        //struct tm * timeinfo =localtime (&gCurrent_time);
        //printf("print record\n");
        // sprintf(l_out_file,"%s%02X%02X%02X%02X%02X%02X%02X%02X",g_logLoca,SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6],SN[7]);
        // strftime(&l_out_file[strlen(l_out_file)],53,"_%F_%H_%M.txt",localtime(&gCurrent_time));
        for (n = 0; n < cMaxNoTID; n++) {
            if (gTap[n][cRecordLen - 1] != 0x00) {



                l_out = fopen(g_logfile, "a+");

                memset(l_out_char, 0x00, sizeof (l_out_char));
                btoa(&gTap[n][0], &l_out_char[0], noUnitByte(gTap[n][0]));
                i = 3 * noUnitByte(gTap[n][0]) - 1;
                for (k = cTIDlen; k < gSaveTIDlen; k++) {
                    l_out_char[i] = '	';
                    i++;
                    btoa(&gTap[n][k], &l_out_char[i], 1);
                    i += 2;
                }
                l_out_char[i++] = '	';
                unsigned char type = 0xff;
                if (gTap[n][1] == 0x60)
                    type = PT;
                else if ((gTap[n][1]&0xF0) == 0x00)
                    type = TT;
                sprintf(&l_out_char[i], "%.1f	%.1f	%.1f", toTemperature(type, gTap[n][cRecordLen - 15], gTap[n][cRecordLen - 14]),
                        toTemperature(type, gTap[n][cRecordLen - 13], gTap[n][cRecordLen - 12]), toTemperature(type, gTap[n][cRecordLen - 11], gTap[n][cRecordLen - 10]));
                strftime(&l_out_char[strlen(l_out_char)], 21, "	%F %T", localtime((time_t *) & gTap[n][cRecordLen - 9]));
                fprintf(l_out, "%s\n", l_out_char);
                fclose(l_out);




            }
        }
        clearRecord = 1;

    }
}

void clearSensorRecord() {
    if (gApp == TemperatureSensor)
        if (clearRecord == 1) {
            memset(gTap, 0x00, sizeof (gTap));
            gNoTap = 0;
            clearRecord = 0;
        }
}

void thread_function2() {
    int n;
    //system("echo none >/sys/class/leds/led1/trigger");
    get_time();
    time_t l_prevTime;
    struct tm *timeinfo;
    l_prevTime = gCurrent_time;
    timeinfo = localtime(&l_prevTime);
    timeinfo->tm_sec = 0;
    timeinfo->tm_min = 0;
    l_prevTime = mktime(timeinfo);
    update_logfile();
    while (1) {
        get_time();
        if (localtime(&gCurrent_time)->tm_year < 2015 - 1900) {
            // printf("year Blink:%i\n",	localtime (&gCurrent_time)->tm_year);
            if (get_led1_status() == 0)
                set_led1_on();
            else
                set_led1_off();

        } else {
            // printf("year ON:%i\n",	localtime (&gCurrent_time)->tm_year);
            if (get_led1_status() == 0) {
                set_led1_on();
                //system("echo mmc0 >/sys/class/leds/led1/trigger");
            }
        }

        //20151005
        if (gApp == Attendance)
            leave(gTimeSpa);
        else if (gApp == TemperatureSensor)
            takeData(n);
        //

        if (n % 30 == 0) {
            printf("No. taps per min: %i\n", gNoTap);
            transmit(&gGetStatus[0], 5);
        }
        if (n % 60 == 0) {
            transmit(&gGetSN[0], 5);
        }

        if (difftime(gCurrent_time, l_prevTime) >= gTimePeriod * 60) {
            //20151005
            if (gApp == Attendance)
                endOfHour();
            transmit(&gStart[0], 6);
            printf("end of hour\n");
            update_logfile();
            l_prevTime = gCurrent_time;
            if (localtime(&l_prevTime)->tm_sec != 0) {
                timeinfo = localtime(&l_prevTime);
                timeinfo->tm_sec = 0;
                l_prevTime = mktime(timeinfo);
            }
            syslog(LOG_INFO, "End of Each Period");
        }
        if (n % 60 == 0) {
            if (total > 0) {
                //printf("total:%d, error:%d, rate:%f\n",total,er,(float)er/total);
                total = 0;
                er = 0;
            }
        }
        sleep(1);
        n++;
        n %= 60 * 60;

    }

}

write_debuf_log() {
    FILE * pFile_debug;
    char datetime_str[50];

    get_time();
    pFile_debug = fopen("/home/pi/debug_log.txt", "a+");
    strftime(datetime_str, 50, "%F_%T \r\n", localtime(&gCurrent_time));
    fprintf(pFile_debug, datetime_str);
    fclose(pFile_debug);

}

/*---------------lung 2015/5/20---------*/
char* date_toString(volatile unsigned char* lTap, char* l_out_char) {
    unsigned char k, i;
    btoa(&lTap[0], &l_out_char[0], cTIDlen);
    i = 3 * cTIDlen - 1;
    for (k = cTIDlen; k < gSaveTIDlen; k++) {
        l_out_char[i] = '	';
        i++;
        btoa(&lTap[k], &l_out_char[i], 1);
        i += 2;

    }

    strftime(&l_out_char[i], 21, "	%F %T", localtime(&gCurrent_time)); // lung 2015/5/29
    //	strftime(&l_out_char[i],21,"	%F %T",localtime((time_t *)&gTap[n][cRecordLen-5]));
    //	i+=20;
    //	strftime(&l_out_char[i],21,"	%F %T",localtime((time_t *)&lTap[cRecordLen-9]));

    return (char*) l_out_char;

}
//20160815

char* write_timeRecorder(volatile unsigned char* lTap, char* l_out_char) {
    unsigned char k, i;
    btoa(&lTap[0], &l_out_char[0], cTIDlen);
    i = 3 * cTIDlen - 1;
    for (k = cTIDlen; k < gSaveTIDlen; k++) {
        l_out_char[i] = '	';
        i++;
        btoa(&lTap[k], &l_out_char[i], 1);
        i += 2;

    }
    l_out_char[i] = '	';
    i++;
    struct timespec start;
    if (clock_gettime(CLOCK_REALTIME, &start) == -1) {
        perror("clock gettime");
        exit(EXIT_FAILURE);
    }
    time_t time = (time_t) start.tv_sec;
    char timeBuff[21];
    strftime(timeBuff, 21, "%F %T", localtime(&time));
    sprintf(&l_out_char[i], "%s.%03d", timeBuff, start.tv_nsec / 1000000);

    return (char*) l_out_char;

}

write_instant_data(volatile unsigned char* lTap) {
    FILE * pFile_instead;
    char l_out_char[120];
    char l_out_file_name[50];
    sprintf(l_out_file_name, "/var/tmp/%02X%02X%02X%02X%02X%02X%02X%02X.txt", SN[0], SN[1], SN[2], SN[3], SN[4], SN[5], SN[6], SN[7]);
    if (no_record_instead <= max_record_instead) {
        pFile_instead = fopen(l_out_file_name, "a+");
    } else {
        pFile_instead = fopen(l_out_file_name, "w+");
        no_record_instead = 0;
    }
    if (gApp == TimeRecorder) {
        fprintf(pFile_instead, "%s\n", write_timeRecorder(lTap, l_out_char));
    } else {
        fprintf(pFile_instead, "%s\n", date_toString(lTap, l_out_char));
    }


    fclose(pFile_instead);

    if (gApp == TimeRecorder) {
        pFile_instead = fopen(g_logfile, "a+");
        fprintf(pFile_instead, "%s\n", write_timeRecorder(lTap, l_out_char));
        fclose(pFile_instead);
    }
    no_record_instead++;
}

/*---------------\\lung 2015/5/20---------*/
main(int argc, char *argv[]) {
    openlog("RFIDLogger", LOG_PID | LOG_CONS, LOG_USER);
    syslog(LOG_INFO, "RFIDLogger start");
    if (argc > 2) {
        if (strcmp(argv[1], "TCP") == 0) {
            IP = argv[2];
            tcp = 1;
        }
    }
    printf("version No.: %i\n", version);
    readPgConf();

    volatile unsigned char lTap_temp[50];
    int k;
    for (k = 0; k < cMaxNoTID; k++) {
        gTap[k][0] = 0x00;
    }

    /*
    memset(&lTap_temp,0x00,sizeof(lTap_temp));
    memset(&gTap,0x00,sizeof(gTap));
     */
    //system("echo none >/sys/class/leds/led0/trigger");
    write_debuf_log();

    if (tcp == 1) {
        getTCP();
        printf("TCP verion. IP:%s\n", IP);
    } else {
        getCom();
        printf("COM port version\n");
    }
    transmit(&gStop[0], 6);
    sleep(1);
    pthread_t th;
    pthread_create(&th, NULL, (void *) &thread_function, NULL);
    transmit(&gGetSN[0], 5);
    sleep(1);
    readerConf();
    sleep(1);
    transmit(&gStart[0], 6);
    sleep(1);
    pthread_t th2;
    pthread_create(&th2, NULL, (void *) &thread_function2, NULL);

    //set first log file 
    update_logfile();

    while (1) {

        if (receive(&lTap_temp[0]) == 1) {
            matching(&lTap_temp[0]);
            if (get_led_status() == 0) {
                set_led_on();
            } else {
                set_led_off();
            }

        }

        clearSensorRecord();
        if (len_uart_buffer() <= 0)
            usleep(1);

    }
}

void transmit(unsigned char *p_tx_buffer, int number) {
    if (gUart0_filestream != -1) {
        int count = write(gUart0_filestream, p_tx_buffer, number); //Filestream, bytes to write, number of bytes to write
        if (count < 0) {
            printf("UART TX error\n");
            //printf("%s\n",strerror(errno));
            syslog(LOG_INFO, "Connection Error");

            close(gUart0_filestream);
            if (tcp == 1)
                getTCP();
            else
                getCom();


        }
        return;
    } else {

        close(gUart0_filestream);
        if (tcp == 1)
            getTCP();
        else
            getCom();
    }
}

void write_reuse_log(volatile unsigned char * pointer, int length) {

    FILE * pFile_reuse;
    char datetime_str[50];
    int n;
    get_time();
    pFile_reuse = fopen("/home/pi/reuse_log.txt", "a+");
    strftime(datetime_str, 50, "reuse %F_%T \r\n", localtime(&gCurrent_time));
    fprintf(pFile_reuse, datetime_str);
    for (n = 0; n < length; n++) {
        fprintf(pFile_reuse, "%02X ", *pointer);
        pointer++;
    }
    fprintf(pFile_reuse, "\n");
    fclose(pFile_reuse);
}

unsigned char shift_byte() {
    int n, i, len;
    er++;
    for (n = 1; n < l_rx_char[1]; n++) {
        if (l_rx_char[n] == 0x02) {
            len = l_rx_char[1] - n;
            if (g_count_reuse_buffer > 0) {
                // for(i=0;i<g_count_reuse_buffer;i++)
                // {
                // g_reuse_buffer[len+i]=g_reuse_buffer[i];
                // }
                for (i = g_count_reuse_buffer - 1; i >= 0; i--)
                    g_reuse_buffer[len + i] = g_reuse_buffer[i];
                g_count_reuse_buffer = g_count_reuse_buffer + len;
            } else
                g_count_reuse_buffer = len;

            for (i = 0; i < len; i++) {
                g_reuse_buffer[i] = l_rx_char[n + i];
            }
            // printf("len:%i;\n",g_count_reuse_buffer);
            // printf("buf start  : ");
            // for(n=0;n<(int)l_rx_char[1];n++)
            // printf("%02X ",l_rx_char[n]);
            // printf("end\n");
            // write_reuse_log(&g_reuse_buffer[0],g_count_reuse_buffer);	//check reuse
            return 0xFF;
        }
    }
    // memset(&l_rx_char[0],0x00,rx_len);
    return 0xFF;
}

unsigned char receive(volatile unsigned char * pDataBuf) {
    volatile int l_data_length;
    unsigned char l_checksum = 0x00;
    unsigned char * lp_rx_buffer = (unsigned char *) l_rx_char;
    int n;
    int no_block = 0;
    unsigned char decode_state = 0;
    if (gUart0_filestream != -1) {
        while (1) {
            switch (decode_state) {
                case 0: //start byte
                    if (len_uart_buffer() > 0) {
                        *lp_rx_buffer = read_uart_buffer_with_clear();
                        lp_rx_buffer++;

                    } else
                        return 0;

                    //check first byte 02
                    if (l_rx_char[0] != 0x02) {
                        return 0;
                    }
                    decode_state = 1;
                    total++;
                    break;

                case 1: //len byte

                    if (len_uart_buffer() > 0) {
                        *lp_rx_buffer = read_uart_buffer_with_clear();
                        //*lp_rx_buffer=read_uart_buffer(k++);
                        lp_rx_buffer++;
                        // printf("len  : %d\n",l_rx_char[1]);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

                        //get len
                        l_data_length = l_rx_char[1] - 2;
                        decode_state = 2;
                    } else {
                        usleep(1);
                    }



                    break;

                case 2: //date
                    // get following packet

                    if (len_uart_buffer() > 0) {
                        *lp_rx_buffer = read_uart_buffer_with_clear();
                        //*lp_rx_buffer=read_uart_buffer(k++);
                        l_data_length--;
                        lp_rx_buffer++;
                    } else {
                        usleep(1);
                    }


                    if (l_data_length <= 0) {
                        // printf("buf start  : ");
                        // for(n=0;n<(int)l_rx_char[1];n++)
                        // printf("%02X ",l_rx_char[n]);
                        // printf("end\n");			
                        decode_state = 3;
                    }
                    break;

                case 3: //check end byte
                    //check end byte == 03
                    if (l_rx_char[l_rx_char[1] - 1] != 0x03)
                        decode_state = shift_byte();
                    else
                        decode_state = 4;
                    break;

                case 4: //check sum
                    //calc check sum
                    for (n = 0; n < (int) l_rx_char[1] - 2; n++) {
                        l_checksum = l_checksum ^l_rx_char[n];
                    }

                    //check if checksum correct
                    if (l_checksum != l_rx_char[l_rx_char[1] - 2])
                        decode_state = shift_byte();
                    else
                        decode_state = 5;
                    break;

                case 5: //filter
                    // do filter 
                    if (filter(pDataBuf) == 0) {
                        return 0;
                    }
                    // for(n=0;n<20;n++){
                    // printf("%02X",pDataBuf[n]);}
                    // printf("\n");	
                    return 1;
                    break;

                default:
                    return 0;
                    break;
            }
        }

    }
    return 0;
}

void btoa(volatile unsigned char *p_rx_buffer, char *p_rx_char, int length) {
    unsigned char l_temp;
    while (length > 0) {
        l_temp = *p_rx_buffer & 0xF0;
        l_temp = l_temp >> 4;
        if (l_temp < 10)
            *p_rx_char = l_temp + 0x30;
        else
            *p_rx_char = l_temp + 0x37;
        ;
        p_rx_char++;

        l_temp = *p_rx_buffer & 0x0F;
        if (l_temp < 10)
            *p_rx_char = l_temp + 0x30;
        else
            *p_rx_char = l_temp + 0x37;
        p_rx_char++;
        *p_rx_char = ' ';

        p_rx_char++;
        *p_rx_buffer++;
        length--;
    }

    p_rx_char--;
    *p_rx_char = '\0';
}

void get_time() {
    gCurrent_time = time(&gCurrent_time);
}

void updateSensor(unsigned char type, unsigned char dataH, unsigned char dataL, int record) {
    int max = (int) gTap[record][cRecordLen - 10] | gTap[record][cRecordLen - 11] << 8;
    int min = (int) gTap[record][cRecordLen - 12] | gTap[record][cRecordLen - 13] << 8;
    int mean = (int) gTap[record][cRecordLen - 14] | gTap[record][cRecordLen - 15] << 8;
    int counter = (int) gTap[record][cRecordLen - 16];
    int data = (int) dataL | dataH << 8;

    switch (type) {
        case PT:
            if (max >= 0x8000)
                max |= 0xFFFF0000;
            if (min >= 0x8000)
                min |= 0xFFFF0000;
            if (mean >= 0x8000)
                mean |= 0xFFFF0000;
            if (data >= 0x8000)
                data |= 0xFFFF0000;
            break;
        case TT:
            if (max >= 0x0800)
                max |= 0xFFFFF800;
            if (min >= 0x0800)
                min |= 0xFFFFF800;
            if (mean >= 0x0800)
                mean |= 0xFFFFF800;
            if (data >= 0x0800)
                data |= 0xFFFFF800;
            break;
        default:
            break;
    }

    int newMean;
    if (data > max) {
        //if(!(data&0x8000==0x8000 && max&0x8000!=0x8000))
        //{
        gTap[record][cRecordLen - 11] = dataH;
        gTap[record][cRecordLen - 10] = dataL;
        //}
    }

    if (data < min) {
        gTap[record][cRecordLen - 13] = dataH;
        gTap[record][cRecordLen - 12] = dataL;
    }
    if (counter >= 255)
        newMean = (mean * counter + data) / (counter);
    else {
        newMean = (mean * counter + data) / (counter + 1);
        gTap[record][cRecordLen - 16]++;
    }
    gTap[record][cRecordLen - 15] = (newMean & 0xFF00) >> 8;
    gTap[record][cRecordLen - 14] = newMean & 0x00FF;
}

void initSersorData(int record) {
    unsigned char dataH = gTap[record][noUnitByte(gTap[record][0])];
    unsigned char dataL = gTap[record][noUnitByte(gTap[record][0]) + 1];
    gTap[record][cRecordLen - 10] = dataL;
    gTap[record][cRecordLen - 11] = dataH;
    gTap[record][cRecordLen - 12] = dataL;
    gTap[record][cRecordLen - 13] = dataH;
    gTap[record][cRecordLen - 14] = dataL;
    gTap[record][cRecordLen - 15] = dataH;
    gTap[record][cRecordLen - 16] = 1;
}

void matching(volatile unsigned char *p_tap) {

    int n, i, k;
    volatile unsigned char *l_tap_temp;
    unsigned char type = 0xFF;

    if (gApp == TimeRecorder) {
        return;
    }

    for (n = 0; n < cMaxNoTID; n++) {
        l_tap_temp = p_tap;
        if (gTap[n][cRecordLen - 1] != 0x00) {
            for (i = 0; i < noUnitByte(*p_tap); i++) {
                if (gTap[n][i] == *l_tap_temp) {
                    l_tap_temp++;

                    if (i == noUnitByte(*p_tap) - 1) //update record
                    { //printf("update\n");
                        if (gApp == TemperatureSensor) {
                            if (gTap[n][1] == 0x60)
                                type = PT;
                            else if ((gTap[n][1]&0xF0) == 0x00)
                                type = TT;
                            updateSensor(type, *(l_tap_temp), *(l_tap_temp + 1), n);
                        }
                        for (k = noUnitByte(*p_tap); k < gSaveTIDlen; k++) {
                            if (k == gSaveTIDlen - 1) //RSSI
                            {
                                if (gTap[n][k]>*l_tap_temp) //save the max RSSI	
                                {
                                    gTap[n][k] = *l_tap_temp;
                                }
                            } else {
                                gTap[n][k] = *l_tap_temp++;
                            }
                        }

                        gTap[n][cRecordLen - 9] = (gCurrent_time & 0x000000FF);
                        gTap[n][cRecordLen - 8] = (gCurrent_time & 0x0000FF00) >> 8;
                        gTap[n][cRecordLen - 7] = (gCurrent_time & 0x00FF0000) >> 16;
                        gTap[n][cRecordLen - 6] = (gCurrent_time & 0xFF000000) >> 24;

                        return;
                    }
                } else {
                    break;
                }
            }
        }
    }
    for (n = 0; n < cMaxNoTID; n++) //write new record
    {
        l_tap_temp = p_tap;
        if (gTap[n][cRecordLen - 1] == 0x00) {
            for (i = 0; i < gSaveTIDlen; i++) {
                gTap[n][i] = *l_tap_temp;
                l_tap_temp++;
            }
            if (gApp == TemperatureSensor)
                initSersorData(n);
            gTap[n][cRecordLen - 9] = (gCurrent_time & 0x000000FF);
            gTap[n][cRecordLen - 8] = (gCurrent_time & 0x0000FF00) >> 8;
            gTap[n][cRecordLen - 7] = (gCurrent_time & 0x00FF0000) >> 16;
            gTap[n][cRecordLen - 6] = (gCurrent_time & 0xFF000000) >> 24;
            gTap[n][cRecordLen - 5] = (gCurrent_time & 0x000000FF);
            gTap[n][cRecordLen - 4] = (gCurrent_time & 0x0000FF00) >> 8;
            gTap[n][cRecordLen - 3] = (gCurrent_time & 0x00FF0000) >> 16;
            gTap[n][cRecordLen - 2] = (gCurrent_time & 0xFF000000) >> 24;

            gTap[n][cRecordLen - 1] = 0xFF;
            gNoTap++;

            return;
        }
    }
}

void endOfHour() {
    int n, i, k;
    FILE * l_out;
    char l_out_char[120];
    // sprintf(l_out_char,"%s%02X%02X%02X%02X%02X%02X%02X%02X",g_logLoca,SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6],SN[7]);
    // strftime(&l_out_char[25],50,"Appear_%F_%H%M.txt",localtime(&gCurrent_time));
    // l_out=fopen(l_out_char,"a+");
    // fprintf(l_out,"");
    l_out = fopen(g_logfile, "a+");
    fprintf(l_out, "End of this hour, tags appear on zone\n");
    for (n = 0; n < cMaxNoTID; n++) {
        if (gTap[n][cRecordLen - 1] != 0x00) {


            btoa(&gTap[n][0], &l_out_char[0], cTIDlen);
            i = 3 * cTIDlen - 1;
            for (k = cTIDlen; k < gSaveTIDlen; k++) {
                l_out_char[i] = '	';
                i++;
                btoa(&gTap[n][k], &l_out_char[i], 1);
                i += 2;

            }

            //i=3*gSaveTIDlen-1;
            strftime(&l_out_char[i], 21, "	%F %T", localtime((time_t *) & gTap[n][cRecordLen - 5]));
            i += 20;
            strftime(&l_out_char[i], 21, "	%F %T", localtime((time_t *) & gTap[n][cRecordLen - 9]));
            //i+=20;
            //strftime(&l_out_char[i],21,"	%F %T",localtime(&gCurrent_time));
            fprintf(l_out, "%s\n", l_out_char);


        }
    }
    fclose(l_out);
}

void leave(volatile unsigned int p_min) {
    char aa[50];

    double l_diff_time;
    int n, i, k;
    FILE * l_out;
    char l_out_char[120];
    //struct tm * timeinfo =localtime (&gCurrent_time);
    for (n = 0; n < cMaxNoTID; n++) {
        if (gTap[n][cRecordLen - 1] != 0x00) {
            l_diff_time = difftime(gCurrent_time, *(time_t *) & gTap[n][cRecordLen - 9]);
            //l_diff_time = gCurrent_time - (*(time_t *)&gTap[n][cRecordLen-9]);




            if (l_diff_time > p_min) {

                //printf("print record\n");
                //sprintf(l_out_char,"/home/pi/%02X%02X%02X%02X%02X%02X%02X%02X",SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6],SN[7]);

                //strftime(&l_out_char[25],50,"_%F_%H.txt",localtime(&gCurrent_time));
                //l_out=fopen(l_out_char,"a+");

                l_out = fopen(g_logfile, "a+");

                btoa(&gTap[n][0], &l_out_char[0], cTIDlen);
                i = 3 * cTIDlen - 1;
                for (k = cTIDlen; k < gSaveTIDlen; k++) {
                    l_out_char[i] = '	';
                    i++;
                    btoa(&gTap[n][k], &l_out_char[i], 1);
                    i += 2;

                }

                //i=3*gSaveTIDlen-1;
                strftime(&l_out_char[i], 21, "	%F %T", localtime((time_t *) & gTap[n][cRecordLen - 5]));
                i += 20;
                strftime(&l_out_char[i], 21, "	%F %T", localtime((time_t *) & gTap[n][cRecordLen - 9]));
                //i+=20;
                //strftime(&l_out_char[i],21,"	%F %T",localtime(&gCurrent_time));
                fprintf(l_out, "%s\n", l_out_char);

                fclose(l_out);

                for (i = 0; i < cRecordLen; i++) //clear
                    gTap[n][i] = 0x00;

                gNoTap--;


            }


        }
    }
}

unsigned char noUnitByte(unsigned char pFirByte) {
    switch (pFirByte & 0xF0) {
        case 0x80:
            return 6;
            break;

        default:
            return 8;
            break;
    }
}

unsigned char filter(volatile unsigned char * pDataBuf) {
    unsigned char n = 0;
    char temp[16];
    //char *
    switch (l_rx_char[2]) {
        case 0x06: //Tap ID

            if (l_rx_char[1] > 0x06) {
                if (gApp == TemperatureSensor) //20151006 find temperature tag
                {
                    if (!((l_rx_char[3] == 0x83 && l_rx_char[4] == 0x60) || (l_rx_char[3] == 0x83 && (l_rx_char[4]&0xF0) == 0x00)))
                        return 0;
                }
                write_instant_data(&l_rx_char[3]); //lung 2015/5/29
                if (l_rx_char[12] > minRSSI)
                    return 0;

                memset((void *) pDataBuf, 0x00, sizeof (pDataBuf));
                //printf("sizeof(pDataBuf) : %d\n",sizeof(pDataBuf));

                for (n = 3; n < l_rx_char[1] - 2; n++) {
                    *pDataBuf = l_rx_char[n];
                    pDataBuf++;
                }
                // memset(&l_rx_char[0],0x00,l_rx_char[1]);
                return 1;
            }
            break;
        case 0x51: //EM02 reader SN

            if (l_rx_char[1] == 0x0D) {
                for (n = 0; n < 8; n++) {
                    SN[n] = l_rx_char[n + 3];
                }
                sprintf(&temp[0], "%02X%02X%02X%02X%02X%02X%02X%02X", SN[0], SN[1], SN[2], SN[3], SN[4], SN[5], SN[6], SN[7]);
                memcpy(&g_logfile[strlen(g_logLoca)], &temp[0], 16);
                //snprintf(&g_logfile[strlen(g_logLoca)],16,"%02X%02X%02X%02X%02X%02X%02X%02X",SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6],SN[7]);
                printf("log file:%s\n", g_logfile);
                //printf("%02X%02X%02X%02X%02X%02X%02X%02X\n",SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6],SN[7]);
                //gEndHour=0x00;
                //transmit(&gStart[0],6);
                //printf("start read\n");
                char l_out_file_name[50];
                sprintf(l_out_file_name, "/var/tmp/%02X%02X%02X%02X%02X%02X%02X%02X.txt", SN[0], SN[1], SN[2], SN[3], SN[4], SN[5], SN[6], SN[7]);
                FILE * file=fopen(l_out_file_name,"a+");
                fprintf(file,"");
                fclose(file);
            }
            break;

        case 0x53:

            if (l_rx_char[1] == 0x19) {
                if (l_rx_char[3 + 5] == 0x00) {
                    transmit(&gStart[0], 6);
                }
            }
            break;

        default:
            break;
    }
    // memset(&l_rx_char[0],0x00,l_rx_char[1]);
    return 0;
}
