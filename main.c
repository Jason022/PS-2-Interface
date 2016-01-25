/*
 * Projeto_Jerome.c
 *
 * Created: 1/5/2016 7:37:37 PM
 *  Author: Bruno
 */ 

#define  F_CPU    8000000           // Clock Speed

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

//------[ Commands ]---------------------------------------------------------------------
// Useful
#define  CMD_RESET                     0xFF
#define  CMD_RESEND                    0xFE
#define  CMD_SET_DEFAULTS              0xF6
#define  CMD_DISABLE_DATA_REPORTING    0xF5
#define  CMD_ENABLE_DATA_REPORTING     0xF4
#define  CMD_SET_SAMPLE_RATE           0xF3
#define  CMD_GET_DEVICE_ID             0xF2
#define  CMD_READ_DATA                 0xEB
#define  CMD_STATUS_REQUEST            0xE9
#define  CMD_SET_RESOLUTION            0xE8
#define  ACKNOWLEDGE                   0xFA           // This is not a command. It is the acknowledge of any data
// Useless
#define  CMD_SET_REMOTE_MODE           0xF0
#define  CMD_SET_WRAP_MODE             0xEE
#define  CMD_RESET_WRAP_MODE           0xEC
#define  CMD_SET_STREAM_MODE           0xEA
#define  CMD_SET_SCALING2TO1           0xE7
#define  CMD_SET_SCALING1TO1           0xE6

typedef  unsigned char        BOOL;
#ifndef NULL
   #define NULL               0
#endif
#define  FALSE                0
#define  TRUE                 1
#define  RECEIVE_BUFFER_LEN   16

#define  LED1On()                         do{ PORTC |=  (1<<PC5); } while(0)
#define  LED1Off()                        do{ PORTC &= ~(1<<PC5); } while(0)

#define  LED2On()                         do{ PORTC |=  (1<<PC4); } while(0)
#define  LED2Off()                        do{ PORTC &= ~(1<<PC4); } while(0)

#define  SetDeviceClockPinHigh()          do { DDRD &= ~(1 << PD2); PORTD &= ~(1 << PD2); } while(0)     // Set as input and leave in high state
#define  SetDeviceClockPinLow()           do { DDRD |= (1 << PD2);  PORTD &= ~(1 << PD2); } while(0)     // Set as output and output zero
#define  ReleaseDeviceClockPin()          SetDeviceClockPinHigh()                                        // Same as leaving high
#define  SetDeviceDataPinHigh()           do { DDRD &= ~(1 << PD1); PORTD &= ~(1 << PD1); } while(0)     // Set as input and leave in high state
#define  SetDeviceDataPinLow()            do { DDRD |= (1 << PD1);  PORTD &= ~(1 << PD1); } while(0)     // Set as output and output zero
#define  ReleaseDeviceDataPin()           SetDeviceDataPinHigh()                                         // Same as leaving high

#define  SetHostClockPinHigh()            do { DDRD &= ~(1 << PD3); PORTD |=  (1 << PD3); } while(0)     // Set as input and pull-up
#define  SetHostClockPinLow()             do { DDRD |= (1 << PD3);  PORTD &= ~(1 << PD3); } while(0)     // Set as output and output zero
#define  ReleaseHostClockPin()            SetHostClockPinHigh()                                          // Same as leaving high
#define  SetHostDataPinHigh()             do { DDRD &= ~(1 << PD0); PORTD |=  (1 << PD0); } while(0)     // Set as input and leave in high state
#define  SetHostDataPinLow()              do { DDRD |= (1 << PD0);  PORTD &= ~(1 << PD0); } while(0)     // Set as output and output zero
#define  HostDeviceDataPin()              SetHostDataPinHigh()                                           // Same as leaving high

#define  WAIT_SETUP_TIME()                do { _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); \
                                               _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); \
                                               _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); \
                                               _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); } while(0)     // 6,250us

volatile unsigned char  g_ucDeviceClkCount      = 0;
volatile unsigned char  g_ucIsSendingDeviceData = FALSE;
volatile unsigned char  g_ucDataToSendToDevice  = 0;
volatile unsigned char  g_ucDeviceQueueHead     = 0;
volatile unsigned char  g_ucDeviceQueueTail     = 0;
volatile unsigned char  g_aucDeviceQueueBuffer[RECEIVE_BUFFER_LEN];

volatile unsigned char  g_ucHostClkCount           = 0;
volatile unsigned char  g_ucIsSendingHostData      = FALSE;
volatile unsigned char  g_ucIsHostRequestingToSend = FALSE;
volatile unsigned char  g_ucDataToSendToHost       = 0;
volatile unsigned char  g_ucDataSending            = 0;
volatile unsigned char  g_ucHostQueueHead          = 0;
volatile unsigned char  g_ucHostQueueTail          = 0;
volatile unsigned char  g_aucHostQueueBuffer[RECEIVE_BUFFER_LEN];

ISR( INT0_vect )
{
   static unsigned char ucRecvData   = 0;
   static unsigned char ucParityCalc = 0;
   static unsigned char ucParityRecv = 0;
   
   if( TRUE == g_ucIsSendingDeviceData )
   {
      if( (0 == g_ucDataToSendToDevice) && (0 == g_ucDeviceClkCount) )
      {  // Still sending the request-to-send
         ucParityCalc = 0;
      }
      else if( 8 > g_ucDeviceClkCount )
      {  // Send the data
         if( g_ucDataToSendToDevice & 0x01 )
         {
            SetDeviceDataPinHigh();
            ucParityCalc ^= 1;
         }
         else
         {
            SetDeviceDataPinLow();
         }
         g_ucDataToSendToDevice = g_ucDataToSendToDevice >> 1;                   // Get ready to send the next bit
         g_ucDeviceClkCount++;
      }
      else if( 8 == g_ucDeviceClkCount )
      {  // Send the parity (the parity is the inverse of what has been calculated)
         if( 0 == ucParityCalc )
         {
            SetDeviceDataPinHigh();
         }
         else
         {
            SetDeviceDataPinLow();
         }
         g_ucDeviceClkCount++;
      }
      else if( 9 == g_ucDeviceClkCount )
      {  // Time to send the stop bit. Just release the data line
         ReleaseDeviceDataPin();
         g_ucDeviceClkCount++;
      }
      else if( 10 == g_ucDeviceClkCount )
      {  // Receive the ACK
         /*if( PIND & (1<<PD1) )
         {  // NACK
            g_ucDataToSendToDevice = 0;
         }
         else
         {  // ACK
            g_ucDataToSendToDevice = 1;
         }*/
         g_ucIsSendingDeviceData = FALSE;                     // End of transmission
         g_ucDeviceClkCount = 0;
      }
   }
   else
   {  //------[ Receive data ]------
      if( 0 == g_ucDeviceClkCount )
      {  // Start bit
         TCNT0 = 0;
         ucParityCalc = 0;
      }
      else if( (TCNT0 < 56) || (TCNT0 > 104) )                 // Clock between around 17857 and 10416
      {  // Too long or too soon since last clock, start over
         TCNT0 = 0;
         g_ucDeviceClkCount = 0;
         ucParityCalc = 0;
      }
      else
      {
         if( g_ucDeviceClkCount < 9 )                          // Data bits
         {
            ucRecvData = ucRecvData >> 1;                      // Open space on the left for the received bit
            if( 0 == (PIND & (1<<PD1)) )
            {
               ucRecvData = ucRecvData & 0x7F;                 // Clear the most left bit
            }
            else
            {
               ucRecvData = ucRecvData | 0x80;                 // Set the most left bit
               ucParityCalc ^= 1;
            }
         }
         else if( 9 == g_ucDeviceClkCount)
         {  // Parity bit
            ucParityRecv = ( PIND & (1<<PD1) ) ? 1 : 0;
         }
         else if( 10 == g_ucDeviceClkCount )
         {  // Stop bit
            g_ucDeviceClkCount = 0;                                  // Start over for the next byte
            // Add data to the buffer, if valid
            if( ucParityRecv != ucParityCalc )
            {  // Valid data
               g_aucDeviceQueueBuffer[g_ucDeviceQueueHead] = ucRecvData;
               g_ucDeviceQueueHead++;
               if( RECEIVE_BUFFER_LEN == g_ucDeviceQueueHead )
               {  // End of the queue. Start back from the beginning
                  g_ucDeviceQueueHead = 0;
               }
               
               if( g_ucDeviceQueueHead == g_ucDeviceQueueTail )
               {  // Queue is full. Discard the oldest data received
                  g_ucDeviceQueueTail++;
                  if( RECEIVE_BUFFER_LEN == g_ucDeviceQueueTail )
                  {  // End of the queue. Start back from the beginning
                     g_ucDeviceQueueTail = 0;
                  }
               }
            }
         }
         
         TCNT0 = 0;                                            // Bit arrived in a valid time
      }
      
      ++g_ucDeviceClkCount;
   }
}

ISR( TIMER2_COMP_vect )
{
   static unsigned char ucParityCalc = 0;
   static unsigned char ucChekingHostLineFree = 0;
   static unsigned char ucRecvData = 0;
   static unsigned char ucParityRecv = 0;
   
   TCNT2 = 0;     // Reset the timer, so the generated wave is symmetric
   
   if( (FALSE == g_ucIsSendingHostData) && (FALSE == g_ucIsHostRequestingToSend) )
   {  // Do nothing
      SetHostClockPinHigh();
      ucChekingHostLineFree = 0;
      g_ucHostClkCount = 0;
   }
   else if( TRUE == g_ucIsHostRequestingToSend)
   {  // Host is sending data to me
      if( 0 == (PIND & (1<<PD3)) )
      {  // Just clock the signal. Host is changing the data line for the next bit
         SetHostClockPinHigh();
         
         if( 10 < g_ucHostClkCount )
         {
            WAIT_SETUP_TIME();
            SetHostDataPinHigh();
            g_ucIsHostRequestingToSend = FALSE;
            ucChekingHostLineFree = 0;
            
            // Add the received byte to the buffer, if valid
            if( ucParityRecv != ucParityCalc )
            {  // Valid data
               g_aucHostQueueBuffer[g_ucHostQueueHead] = ucRecvData;
               g_ucHostQueueHead++;
               if( RECEIVE_BUFFER_LEN == g_ucHostQueueHead )
               {  // End of the queue. Start back from the beginning
                  g_ucHostQueueHead = 0;
               }
               
               if( g_ucHostQueueHead == g_ucHostQueueTail )
               {  // Queue is full. Discard the oldest data received
                  g_ucHostQueueTail++;
                  if( RECEIVE_BUFFER_LEN == g_ucHostQueueTail )
                  {  // End of the queue. Start back from the beginning
                     g_ucHostQueueTail = 0;
                  }
               }
            }
         }
      }
      else
      {  // Time to read the data line
         WAIT_SETUP_TIME();
         
         if( 0 == g_ucHostClkCount )
         {  // Read the start bit
            ucRecvData = 0;
            ucParityCalc = 0;
            if( 0 != (PIND & (1<<PD0)) )
            {  // This is not a star bit! Something went wrong. I should not be here
               SetHostClockPinHigh();
               SetHostDataPinHigh();
               g_ucIsHostRequestingToSend = FALSE;
               ucChekingHostLineFree = 0;
               g_ucHostClkCount = 0;
            }
         }
         else if( (1 <= g_ucHostClkCount) && (8 >= g_ucHostClkCount) )
         {  // Read the data bits
            ucRecvData = ucRecvData >> 1;
            if( 0 == (PIND & (1<<PD0)) )
            {
               ucRecvData = ucRecvData & 0x7F;                 // Clear the most left bit
            }
            else
            {
               ucRecvData = ucRecvData | 0x80;                 // Set the most left bit
               ucParityCalc ^= 1;
            }
         }
         else if( 9 == g_ucHostClkCount )
         {  // Read the Parity bit
            ucParityRecv = ( PIND & (1<<PD0) ) ? 1 : 0;
            // Do not add the received data to the buffer, otherwise the main loop will start process before finishing the packet
         }
         else if( 10 == g_ucHostClkCount )
         {  // Read the stop bit and set the ACK
            SetHostDataPinLow();
         }
         
         if( g_ucIsHostRequestingToSend )
         {
            g_ucHostClkCount++;
            SetHostClockPinLow();
         }
      }
   }
   else if( TRUE == g_ucIsSendingHostData )
   {  // I am sending data to the host
      if( 4 > ucChekingHostLineFree )
      {
         SetHostClockPinHigh();
         _NOP();                       // 7 NOPs: 5 for the rise time (650ns), 1 for the input register latch and 1 as bonus
         _NOP();
         _NOP();
         _NOP();
         _NOP();
         _NOP();
         _NOP();
         if ( 0 == (PIND & (1<<PD3)) )
         {  // Clock pin is not high. Host is inhibiting communication
            ucChekingHostLineFree = 0;
         }
         else
         {
            ucChekingHostLineFree++;
            g_ucHostClkCount = 0;
         }
      }
      else if( 0 == (PIND & (1<<PD3)) )
      {
         SetHostClockPinHigh();
         WAIT_SETUP_TIME();
         
         if ( (0 == (PIND & (1<<PD3))) && (10 >= g_ucHostClkCount)  )      // The host can pull low right after the last clock high, so only
                                                                           // consider that the host is inhibiting if it is not the last bit
         {  // Clock pin did not go high. Host is inhibiting communication. Restart the comm
            ucChekingHostLineFree = 0;
            SetHostDataPinHigh();
         }
         else if( 8 >= g_ucHostClkCount )
         {  // Send Data
            if( 0 == (g_ucDataSending & 0x01) )
            {
               SetHostDataPinLow();
            }
            else
            {
               SetHostDataPinHigh();
               ucParityCalc ^= 1;
            }
            g_ucDataSending = g_ucDataSending >> 1;
         }
         else if( 9 == g_ucHostClkCount )
         {  // Send parity (inverse of what we calculated)
            if( 0 == ucParityCalc )
            {
               SetHostDataPinHigh();
            }
            else
            {
               SetHostDataPinLow();
            }
         }
         else if( 10 == g_ucHostClkCount )
         {  // Stop bit
            SetHostDataPinHigh();
         }
         else
         {  // End
            g_ucIsSendingHostData = FALSE;
            ucChekingHostLineFree = 0;
            g_ucHostClkCount = 0;
         }
      }
      else
      {
         if( 0 == g_ucHostClkCount )
         {  // Start bit
            SetHostDataPinLow();
            ucParityCalc = 0;
            g_ucDataSending = g_ucDataToSendToHost;
         }
         
         WAIT_SETUP_TIME();
         SetHostClockPinLow();
         g_ucHostClkCount++;
      }
   }
}

ISR( INT1_vect )
{
   if( (FALSE == g_ucIsSendingHostData) && (FALSE == g_ucIsHostRequestingToSend) && (0 == (PIND & (1<<PD3))) )
   {  // If not sending, there is no clock. It must be the host requesting to send data
      g_ucHostClkCount = 0;
      g_ucIsHostRequestingToSend = TRUE;
   }
   else if( (TRUE == g_ucIsHostRequestingToSend) && (0 != (PIND & (1<<PD3))) && ( 0 == g_ucHostClkCount ) )
   {  // Host just release the clock line.
      if ( 0 == (PIND & (1<<PD0)) )
      {  // The data line is low. Host wants to send
         TCNT2 = 0;              // Reset the clock counter so that we guarantee the minimum period on the first pulse
         TIFR |= (1<<OCF2);      // Clear any pending interruption already present
      }
      else
      {  // The data line is high. Host was just inhibiting the communication
         g_ucIsHostRequestingToSend = FALSE;
         g_ucHostClkCount = 0;
      }
   }
}

//---------------------------------------------------------------------------------------
//------[ Device Communication Functions ]-----------------------------------------------
//---------------------------------------------------------------------------------------

void SendDataToDevice( unsigned char ucData, BOOL bWaitFinish )
{
   g_ucIsSendingDeviceData = TRUE;
   
   //------[ Send a Request-to-send ]------
   
   // Inhibit communication by pulling Clock low for at least 100 microseconds
   SetDeviceClockPinLow();
   _delay_us( 110 );
   
   // Apply "Request-to-send" by pulling Data low, then release Clock
   SetDeviceDataPinLow();
   ReleaseDeviceClockPin();
   
   //------[ Send the data ]------
   g_ucDeviceClkCount = 0;
   g_ucDataToSendToDevice = ucData;
   
   if( bWaitFinish )
   {
      while( TRUE == g_ucIsSendingDeviceData );     // Wait for finish sending
      
      //------[ The End ]-------
      g_ucDataToSendToDevice = 0;
      g_ucDeviceClkCount = 0;
   }
}

BOOL IsSendingToDevice()
{
   return g_ucIsSendingDeviceData;
}

void ReceiveDataFromDevice( unsigned char* pucData )
{
   while( g_ucDeviceQueueTail == g_ucDeviceQueueHead );              // Wait for some data on the buffer
   
   if( NULL != pucData)
   {
      *pucData = g_aucDeviceQueueBuffer[g_ucDeviceQueueTail];
   }
   
   g_ucDeviceQueueTail++;
   if( RECEIVE_BUFFER_LEN == g_ucDeviceQueueTail )
   {  // End of the queue. Start back from the beginning
      g_ucDeviceQueueTail = 0;
   }
}

void ClearDeviceReceiveBuffer()
{
   g_ucDeviceQueueHead = 0;
   g_ucDeviceQueueTail = 0;
}

int HasDataFromDevice()
{
   if( g_ucDeviceQueueTail <= g_ucDeviceQueueHead )
   {
      return g_ucDeviceQueueHead - g_ucDeviceQueueTail;
   }
   else
   {
      return (RECEIVE_BUFFER_LEN - g_ucDeviceQueueTail) + g_ucDeviceQueueHead;
   }
}

//---------------------------------------------------------------------------------------
//------[ Host Communication Functions ]-------------------------------------------------
//---------------------------------------------------------------------------------------
void SendDataToHost( unsigned char ucData, BOOL bWaitFinish )
{
   while( TRUE == g_ucIsHostRequestingToSend );       // If the host is transmitting, do not send!
   
   g_ucHostClkCount = 0;
   g_ucDataToSendToHost = ucData;
   
   g_ucIsSendingHostData = TRUE;
   
   if( bWaitFinish )
   {
      while( TRUE == g_ucIsSendingHostData );
   }
}

BOOL IsSendingToHost()
{
   return g_ucIsSendingHostData;
}

void ReceiveDataFromHost( unsigned char* pucData )
{
   while( g_ucHostQueueTail == g_ucHostQueueHead );              // Wait for some data on the buffer
   
   if( NULL != pucData)
   {
      *pucData = g_aucHostQueueBuffer[g_ucHostQueueTail];
   }
   
   g_ucHostQueueTail++;
   if( RECEIVE_BUFFER_LEN == g_ucHostQueueTail )
   {  // End of the queue. Start back from the beginning
      g_ucHostQueueTail = 0;
   }
}

void ClearHostReceiveBuffer()
{
   g_ucHostQueueHead = 0;
   g_ucHostQueueTail = 0;
}

int HasDataFromHost()
{
   if( g_ucHostQueueTail <= g_ucHostQueueHead )
   {
      return g_ucHostQueueHead - g_ucHostQueueTail;
   }
   else
   {
      return (RECEIVE_BUFFER_LEN - g_ucHostQueueTail) + g_ucHostQueueHead;
   }
}

#define REPORT_LENGTH      4
volatile BOOL  g_IsDataReportingEnabled   = FALSE;

void ProcessCommand()
{
   static unsigned char ucDeviceID = 0x00;
   unsigned char  ucData;
   #if (REPORT_LENGTH == 4)
      static unsigned char ucMicrosoftModeStep = 0;
   #endif
   
   if( 0 == HasDataFromHost() )
   {  // No data yet
      return;
   }
   
   ReceiveDataFromHost( &ucData );
   switch( ucData )
   {
      case CMD_RESET:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         g_IsDataReportingEnabled = FALSE;
         #if (REPORT_LENGTH == 4)
            ucMicrosoftModeStep = 0;
         #endif
         ucDeviceID = 0x00;
         LED2Off();
         SendDataToHost( 0xAA, TRUE );    // Self-test passed
         SendDataToHost( 0x00, TRUE );    // Mouse ID
         break;
      }
         
      case CMD_SET_SAMPLE_RATE:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         // Receive the value
         ReceiveDataFromHost( &ucData );
         SendDataToHost( ACKNOWLEDGE, TRUE );
         #if (REPORT_LENGTH == 4)
            if( (200 == ucData) && (0 == ucMicrosoftModeStep) )
            {  // Received the first step to enter Microsoft mode
               ucMicrosoftModeStep = 1;
            }
            else if( (100 == ucData) && (1 == ucMicrosoftModeStep) )
            {  // Received the second step. Getting closer...
               ucMicrosoftModeStep = 2;
            }
            else if( (80 == ucData) && (2 == ucMicrosoftModeStep) )
            {  // Third and final step. I'm in Microsoft Intellimouse mode
               ucDeviceID = 0x03;
               ucMicrosoftModeStep = 0;
            }
            else
            {  // Just ruined the sequence
               ucMicrosoftModeStep = 0;
            }
         #endif
         break;
      }
         
      case CMD_SET_RESOLUTION:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         // Receive the value
         ReceiveDataFromHost( &ucData );           // Discard it, for now
         SendDataToHost( ACKNOWLEDGE, TRUE );
         break;
      }
      
      case CMD_GET_DEVICE_ID:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         SendDataToHost( ucDeviceID, TRUE );
         break;
      }
      
      case CMD_ENABLE_DATA_REPORTING:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         g_IsDataReportingEnabled = TRUE;
         LED2On();
         break;
      }
      
      case CMD_DISABLE_DATA_REPORTING:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         g_IsDataReportingEnabled = FALSE;
         LED2Off();
         break;
      }
      
      case CMD_SET_DEFAULTS:
      {
         SendDataToHost( ACKNOWLEDGE, TRUE );
         g_IsDataReportingEnabled = FALSE;
         LED2Off();
         break;
      }
      
      case CMD_RESEND:
      case CMD_READ_DATA:
      case CMD_STATUS_REQUEST:
      {  // This ones need more data after the ACK. It is not going to work
         SendDataToHost( ACKNOWLEDGE, TRUE );
         break;
      }
      
      default:
      {  // Unknown command. Ignore
         break;
      }
   }
}

#define  ADC_CHANNEL    0     // ADC0

unsigned int ADC_Read()
{
   ADCSRA |= (1<<ADSC);

   while ( ADCSRA & (1<<ADSC) );

   return (ADCL) | (ADCH<<8);
}

void SeedRNG()
{
   unsigned char  i;
   unsigned int   uiSeed;                       // It will be used uninitialized, to get extra randomness
   
   for( i = 0; i < sizeof(uiSeed); i++)
   {
      uiSeed ^= ((ADC_Read() & 0x01) << i);     // Use the least significant bit of the reading to XOR with the initialized variable
   }
   
   srand( uiSeed );
}

int main(void)
{
   unsigned char  ucData0;
   unsigned char  ucData1;
   unsigned char  ucData2;
   #if (REPORT_LENGTH == 4)
      unsigned char  ucData3;
   #endif
   int   iRandom;
   int   i;

   SFIOR &= ~(1 << PUD);               // Enable the internal pull-ups on input pins
   
   DDRC = (1 << PC4) | (1 << PC5);     // Just some LEDs
   DDRD = (0 << PD1) | (0 << PD2) |    // Data and clock for the device
          (0 << PD0) | (0 << PD3);     // Data and clock for the host
   
   //PORTD = 0;                          // When the pin is output, pulls it low, and when it is input, leave it in tri-state
   
   // Set host side lines to high
   SetHostClockPinHigh();
   SetHostDataPinHigh();
   
   ADMUX  = (1<<REFS0) | (ADC_CHANNEL<<MUX0);            // AVcc as reference
   ADCSRA = (1<<ADEN) | (0<<ADPS0);                      // Start as fast as possible, so we get more entropy
   
   // Start timer 0 to measure device clock
   TCNT0 = 0;                                            // Zero current value
   TCCR0 = (0 << CS02) | (1 << CS01) | (0 << CS00);      // Start counting with F=clk/8 (count from 0 to 255 in 255us)
   
   // Start timer 2 to generate host clock
   TCNT2 = 0;
   TCCR2 = (0 << CS22) | (0 << CS21) | (1 << CS20);      // Start counting with F=clk/1 (count from 0 to 255 in 31.875us).
   OCR2  = 248;                                          // Set the compare match value: 248 = 31us. Every compare match will be and edge
   TIMSK = (1 << OCIE2);                                 // Output Compare Match Interrupt Enable
   
   // Configure interrupts
   MCUCR = (1 << ISC01) | (0 << ISC00) |                 // Falling edge of INT0 generates an interrupt
           (0 << ISC11) | (1 << ISC10);                  // Any logical change on INT1 generates an interrupt
   GICR  = (1 << INT0)  | (1 << INT1);                   // Enable INT0 and INT1 interruptions
   sei();                                                // Enable interruptions globally
   
   PORTC = 0;
   g_ucIsSendingDeviceData = FALSE;
   ClearDeviceReceiveBuffer();
   ClearHostReceiveBuffer();
   
   SeedRNG();
   
   //--- [ Initialize the mouse ]---
   for( i = 0; i < 800; i++ )    // Usually it takes 700ms to receive the first Self-Tested
   {
      _delay_ms( 1 );
      if( 2 <= HasDataFromDevice() )
      {
         break;
      }
   }
   
   ClearDeviceReceiveBuffer();
   
   SendDataToDevice( CMD_RESET, TRUE );
   ReceiveDataFromDevice( NULL );                  // Receive ACKNOWLEDGE
   ReceiveDataFromDevice( NULL );                  // Receive Self-Test Passed
   //ReceiveDataFromDevice( &ucData );               // Receive Device ID
   
   // Microsoft Intellimouse sequence
   #if (REPORT_LENGTH == 4)
      SendDataToDevice( CMD_SET_SAMPLE_RATE, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
      SendDataToDevice( 200, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
      SendDataToDevice( CMD_SET_SAMPLE_RATE, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
      SendDataToDevice( 100, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
      SendDataToDevice( CMD_SET_SAMPLE_RATE, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
      SendDataToDevice( 80, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
      SendDataToDevice( CMD_GET_DEVICE_ID, TRUE );
      ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
      ReceiveDataFromDevice( NULL );               // Receive Device ID
   #endif
   
   SendDataToDevice( CMD_SET_RESOLUTION, TRUE );
   ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   SendDataToDevice( 0x03, TRUE );              // 8 count/mm
   ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
   SendDataToDevice( CMD_SET_SCALING1TO1, TRUE );
   ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
   SendDataToDevice( CMD_SET_SAMPLE_RATE, TRUE );
   ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   SendDataToDevice( 40, TRUE );
   ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
   SendDataToDevice( CMD_ENABLE_DATA_REPORTING, TRUE );
   ReceiveDataFromDevice( NULL );               // Receive ACKNOWLEDGE
   
   LED1On();
   
   // Start the Host
   SendDataToHost( 0xAA, TRUE );                   // Self-test passed
   SendDataToHost( 0x00, TRUE );                   // Mouse ID
   
   ClearDeviceReceiveBuffer();
   i = 0;
   
   while(1)
   {
      ProcessCommand();
      
      if( REPORT_LENGTH < HasDataFromDevice() )
      {  // Has more than one report packet. Discard the older
         ReceiveDataFromDevice( NULL );
         ReceiveDataFromDevice( NULL );
         ReceiveDataFromDevice( NULL );
         #if (REPORT_LENGTH == 4)
            ReceiveDataFromDevice( NULL );
         #endif
      }
      
      if( 0 == HasDataFromDevice() )
      {  // No data
         i = 0;
      }
      else if( (0 < HasDataFromDevice()) && (REPORT_LENGTH > HasDataFromDevice()) )
      {  // Not enough data
         i++;
         if( 500 <= i ) // 5ms
         {  // It has been too long since we did not received a packet. We probably lost sync. Clear the buffer
            ClearDeviceReceiveBuffer();
            i = 0;
         }
      }
      else if( REPORT_LENGTH == HasDataFromDevice() )
      {
         i = 0;
         
         if( TRUE == g_IsDataReportingEnabled )
         {
            ReceiveDataFromDevice( &ucData0 );
            ReceiveDataFromDevice( &ucData1 );
            ReceiveDataFromDevice( &ucData2 );
             #if (REPORT_LENGTH == 4)
               ReceiveDataFromDevice( &ucData3 );
             #endif
             
             // Mess with the data
            iRandom = rand();
            if( (RAND_MAX>>5) > iRandom )
            {  // Invert the buttons
               if( (ucData0 & 0x02) && !(ucData0 & 0x01) )
               {  // Right click only
                  ucData0 &= 0xFC;
                  ucData0 |= 0x01;
               }
               else if( (ucData0 & 0x01) && !(ucData0 & 0x02) )
               {  // Left Click only
                  ucData0 &= 0xFC;
                  ucData0 |= 0x02;
               }
               // else, both clicks. Do nothing
            }
            else if( (RAND_MAX>>4) > iRandom )
            {  // Invert the X, Y and Z movement
               if( 0 != ucData1 )
               {
                  ucData0 ^= 0x10;                 // Invert the X signal
                  ucData1 = ~ucData1;              // Invert the X
               }
               
               if( 0 != ucData2 )
               {
                  ucData0 ^= 0x20;                 // Invert the Y signal
                  ucData2 = ~ucData2;              // Invert the Y
               }
               
               #if (REPORT_LENGTH == 4)
                  if( 0 != ucData3)
                  {
                     ucData3 = (~ucData3) + 1;           // Invert the Z
                  }
               #endif
            }
            
            SendDataToHost( ucData0, TRUE );
            SendDataToHost( ucData1, TRUE );
            SendDataToHost( ucData2, TRUE );
            #if (REPORT_LENGTH == 4)
               SendDataToHost( ucData3, TRUE );
            #endif
         }
      }
      
      _delay_us( 10 );
   }
}