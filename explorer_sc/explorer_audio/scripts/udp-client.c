//udp-dsp-client.c
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <linux/soundcard.h>

//录音频率
#define RATE 88200
//量化位数
#define SIZE 16
//声道数目
#define CHANNELS 2
//缓冲区大小
#define RSIZE 2048
//保存录取的音频数据
unsigned char buf[RSIZE];

int main(int argc, char *argv[]) {
    int fd, sockfd;
    int status;
    int arg;
    struct sockaddr_in s_addr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        exit(errno);
    } else {
        printf("creat sockfd success!.\n\r");
    }

    s_addr.sin_family = AF_INET;
    
    int port=0,i;
    for (i=0;argv[2][i]!='\0';++i)
    {
        port*=10;
        port+=argv[2][i]-'0';
    }
    s_addr.sin_port = htons(port);

    if (argc != 3) {
        printf("用法：padsp ./client <IP地址> <端口>\n");
        return 1;
    } else {
        s_addr.sin_addr.s_addr = inet_addr(argv[1]);
    }

    bzero(& (s_addr.sin_zero), 8);

    /*************读方式打开音频设备********************************************************************************/
    fd = open("/dev/dsp", O_RDONLY, 0777);

    if (fd < 0) {
        perror("Cannot open /dev/dsp device");
        return 1;
    }

    //设置采样的量化位数
    arg = SIZE;

    status = ioctl(fd, SOUND_PCM_WRITE_BITS, &arg);

    if (status == -1) {

        perror("Connet set SOUND_PCM_WRITE_BITS ");
        return -1;
    }

    //设置采样时的声道数目
    arg = CHANNELS;
    status = ioctl(fd, SOUND_PCM_WRITE_CHANNELS, &arg);

    if (status == -1) {

        perror("Connet set SOUND_PCM_WRITE_CHANNELS ");
        return -1;
    }

    //设置采样时的频率
    arg = RATE;
    status = ioctl(fd, SOUND_PCM_WRITE_RATE, &arg);

    if (status == -1) {

        perror("Connet set SOUND_PCM_WRITE_RATE ");
        return -1;
    }

    int readNum, sendNum;

    while (1) {
        //从声卡读语音数据
        readNum = read(fd, buf, RSIZE);

        if (readNum == -1) {
            perror("read wrong number of bytes\n");
        }

        //发送语音数据
        sendNum = sendto(sockfd, buf, readNum, 0, (struct sockaddr *)& s_addr, sizeof(struct sockaddr));

        if (sendNum == -1) {
            printf("sendto error\n");
            break;
        }
    }
}

