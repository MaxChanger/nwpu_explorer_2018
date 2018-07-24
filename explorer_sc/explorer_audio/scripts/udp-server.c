/*************************************************************************
    > File Name: dsp-.c
    > Author: tangshojie
    > Mail: tangshaojie@sina.cn
    > Created Time: 2014年09月18日 星期四 14时19分44秒
 ************************************************************************/

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
    //声卡描述符
    int fd;
    int arg;
    int status;

    //socket
    struct sockaddr_in s_addr;
    struct sockaddr_in c_addr;
    int sock;
    socklen_t addr_len;
    int len;

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) { //使用UDP方式
        perror("socket");
        exit(errno);
    } else {
        printf("creat socket success.\n\r");
    }

    memset(&s_addr, 0, sizeof(struct sockaddr_in));
    s_addr.sin_family = AF_INET; //协议设为AF_INET
    if (argc!=2)
    {
        printf("用法：padsp ./server <端口>\n");
        return 1;
    }
    int port=0,i;
    for (i=0;argv[1][i]!='\0';++i)
    {
        port*=10;
        port+=argv[1][i]-'0';
    }
    s_addr.sin_port = htons(port); //接受端口为7000
    s_addr.sin_addr.s_addr = INADDR_ANY; //本地任意IP

    if ((bind(sock, (struct sockaddr *) &s_addr, sizeof(s_addr))) == -1) {
        perror("bind");
        exit(errno);
    } else {
        printf("bind address to socket.\n\r");
    }

    addr_len = sizeof(c_addr);
    fd = open("/dev/dsp", O_WRONLY);

    if (fd < 0) {
        perror("Connot open /dev/dsp device");
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

    //一直接受播放，直到按下ctrl+c为止
    while (1) {
        len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&c_addr, &addr_len);
        status = write(fd, buf, len);

        if (status != len) {
            perror("wrote wrong number of bytes");
        }
    }

    close(fd);
    return 0;
}

