#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#define BUF_SIZE 1024

void error_handling(char *message);
int main()
{
    char send_socket[BUF_SIZE] = {
        0,
    };
    int sock;
    char message[BUF_SIZE] = "test";
    struct sockaddr_in serv_adr;
    

    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (-1 == sock)
        error_handling("socket() error");

    memset(&serv_adr, 0, sizeof(serv_adr));
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = inet_addr(192.168.0.9);
    serv_adr.sin_port = htons(9000);

    if (-1 == connect(sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)))
    {
        printf("접속 실패\n");
        exit(1);
    }

    while (1)
    {
            
            write(sock, message, sizeof(message) - 1); // 서버로 message 전송
        }
    }
    close(sock);
    return 0;
}

void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}