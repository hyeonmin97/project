#include <stdio.h>
#include <unistd.h> //Used for UART
#include <fcntl.h>  //Used for UART

#include <termios.h> //Used for UART
int uart0_filestream = -1;
void send(){
    unsigned char tx_buffer[20];
    unsigned char *p_tx_buffer;

    p_tx_buffer = "hello";
    //    p_tx_buffer = &tx_buffer[0];
    //    *p_tx_buffer++ = 'H';
    //    *p_tx_buffer++ = 'e';
    //    *p_tx_buffer++ = 'l';
    //    *p_tx_buffer++ = 'l';
    //    *p_tx_buffer++ = 'o';
    //
    if (uart0_filestream != -1)
    {
        int count = write(uart0_filestream, p_tx_buffer, sizeof(p_tx_buffer)); //Filestream, bytes to write, number of bytes to write
        if (count < 0)
        {
            printf("UART TX error\n");
        }
    }
}

void reads(){
    if (uart0_filestream != -1)
    {
        // Read up to 255 characters from the port if they are there
        unsigned char rx_buffer[256];
        int rx_length = read(uart0_filestream, (void *)rx_buffer, 255); //Filestream, buffer to store in, number of bytes to read (max)
        if (rx_length < 0)
        {
            //An error occured (will occur if there are no bytes)
        }
        else if (rx_length == 0)
        {
            //No data waiting
        }
        else
        {
            //Bytes received
            rx_buffer[rx_length] = '\0';
            printf("%i bytes read : %s\n", rx_length, rx_buffer);
        }
    }
}
int main(){
    
    uart0_filestream = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY | O_NDELAY);
    printf("open\n");
    if (uart0_filestream == -1)
    { // ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART. Ensure it is not in use by another application\n");
    }

    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);

    tcsetattr(uart0_filestream, TCSANOW, &options);
    while(1){
        //send();
        reads();
    }
}