#include <18F4431.h>
#use delay(clock = 20M)
#use RS232(BAUD = 9600,XMIT = PIN_C6 , RCV = PIN_C7)

#FUSES NOWDT                    
#FUSES HS                       
#FUSES NOPROTECT                
#FUSES NOBROWNOUT             
#use delay(clock=20000000)
#byte QEICON   =0xfb6
#byte DFLTCON  =0xf60
#byte POSCNTH  =0xf67
#byte POSCNTL  =0xf66
#byte MAXCNTH  =0xf65
#byte MAXCNTL  =0xf64


void QEI_init()
{
   QEICON   =  0b10111000;   
   DFLTCON  =  0b00010110;
   POSCNTH  =  0xEF;
   POSCNTL  =  0xff;
   MAXCNTH  =  0xff;
   MAXCNTL  =  0xff;
}

void main()
{  
   QEI_init();
   int16 xung;
   xung = 0;
   while(1)
   {
     xung = (make16(POSCNTH,POSCNTL))/4;
     printf("%ld\r\n",xung);
     delay_ms(1000);
   }
   }
