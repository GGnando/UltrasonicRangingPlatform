#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>

#define LINE_BUF 256

static void raw_mode(int on)
{
    static struct termios save;
    if(on)
    { 
        tcgetattr(STDIN_FILENO,&save);
        struct termios t=save; t.c_lflag&=~(ICANON|ECHO);
        tcsetattr(STDIN_FILENO,TCSANOW,&t);
    }
    else
    {
        tcsetattr(STDIN_FILENO,TCSANOW,&save);
    }
}

int main(int c,char*argv[])
{
    if(c!=2)
    {
        fprintf(stderr,"usage: %s <ip>\n",argv[0]);
        return 1;
    }
    int fd = socket(AF_INET,SOCK_STREAM,0);
    struct sockaddr_in a={0};
    a.sin_family=AF_INET;
    a.sin_port=htons(5000);
    inet_pton(AF_INET,argv[1],&a.sin_addr);

    if(connect(fd,(struct sockaddr*)&a,sizeof(a)))
    {
        perror("connect");
        return 1;
    }

    raw_mode(1);
    fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);
    // tcp buffer
    char buf[LINE_BUF];
    size_t tail=0;

    while(1)
    {
        // check user input from terminal
        // convert user input to command to server
        char ch;
        if(read(STDIN_FILENO,&ch,1)==1){
            const char*cmd=NULL;
            if(ch=='1')
            {
                cmd="CFG1_START\n";
            }
            else if(ch=='!')
            {
                cmd="CFG1_STOP\n";
            }
            else if(ch=='2')
            {
                cmd="CFG2_START\n";
            } 
            else if(ch=='@')
            {
                cmd="CFG2_STOP\n";
            }
            else if(ch=='q')
            {
                write(fd,"QUIT\n",5);
                break;
            }
            if(cmd)
            {
                write(fd,cmd,strlen(cmd));
            }
        }
        // read telemetry from server
        ssize_t k=read(fd,buf+tail,LINE_BUF-tail);
        if(k>0)
        {
            // parse potential multiple messages from server
            tail+=k; size_t start=0;
            for(size_t i=0;i<tail;i++)
            {
                // telemetry messages are deliminated by a new line
                if(buf[i]=='\n')
                {
                    buf[i]='\0';
                    char *line=buf+start;
                    // telemtry messages are prefixed with 'T'
                    if(line[0]=='T')
                    {
                        // format telemtry data into a pretty print format
                        double d1,d2,d3,ax,ay,az,gx,gy,gz,s1,s2;
                        int c1,c2;
                        sscanf(line+1,
                          "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d",
                           &d1,&d2,&d3,&ax,&ay,&az,&gx,&gy,&gz,&s1,&s2,&c1,&c2);
                        printf("\rUS: %5.1f %5.1f %5.1f cm | "
                               "ACC: %6.3f %6.3f %6.3f | "
                               "GYR: %6.1f %6.1f %6.1f | "
                               "SER: %5.1f %5.1f [%c %c] ",
                               d1,d2,d3, ax,ay,az, gx,gy,gz,
                               s1,s2, c1?'C':'-',c2?'C':'-');
                        fflush(stdout);
                    }
                    start=i+1;
                }
            }
            // handle buffer locations to not overflow
            if(start<tail)
            { 
                memmove(buf,buf+start,tail-start);
                tail-=start;
            }
            else
            {
                tail=0;
            }
        }
        usleep(10000);
    }
    raw_mode(0);
    puts("\nExiting");
    return 0;
}
