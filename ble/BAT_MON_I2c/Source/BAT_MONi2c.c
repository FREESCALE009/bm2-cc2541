/**************************************************************************************************
  Filename:       BAT_MONi2c.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Broadcaster sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <stdio.h>
#include <stdlib.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include  "hal_adc.h"

#include "hci.h"
#include "gap.h"

#include "devinfoservice.h"
#include "broadcaster.h"
#include "BAT_MONi2c.h"

#define HAL_ADC_EOC         0x80    /* End of Conversion bit */
#define HAL_ADC_START       0x40    /* Starts Conversion */


#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */


#define SBP_PERIODIC_EVT_PERIOD                   1000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)

#define DEFAULT_ADVERTISING_INTERVAL          1600

// Length of bd addr as a string

#define B_ADDR_STR_LEN                        15


uint8 adcRef;

int sensorID = 0;
uint16 bdcount = 0;


uint8 BAT_MONi2c_TaskID;   // Task ID for internal task/event processing
uint16 bdcount;
 
// GAP - SCAN RSP data (max size = 31 bytes)

uint8 hexval[]       = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',' '};
uint8 scanRspData1[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'B','A','T','M','O','N','1','0','0','6','B','5',0x02,GAP_ADTYPE_POWER_LEVEL,0};
uint8 baddr0[]       = {0xb5,0x06,0x10,0xcb,0xeb,0x80};
uint8 baddr1[]       = {0xb5,0x06,0x10,0xcb,0xeb,0x80};
uint8 aData1[]       = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x80,0xeb,0xcb,0x10,0x06,0xb5,0x00,0x00,0x00,0x00,0x00,0x09,0x00,0x00,0x38,0x00,0x00};

uint8 LedBlink = 0;

#include "ioCC2530.h"
#include "zcomdef.h"



#define MEMS_RETRY_CNT  3

#define OCM_CLK_PORT  0
#define OCM_DATA_PORT 0
#define OCM_CLK_PIN   4
#define OCM_DATA_PIN  5

// General I/O definitions
#define IO_GIO  0  // General purpose I/O
#define IO_PER  1  // Peripheral function
#define IO_IN   0  // Input pin
#define IO_OUT  1  // Output pin
#define IO_PUD  0  // Pullup/pulldn input
#define IO_TRI  1  // Tri-state input
#define IO_PUP  0  // Pull-up input pin
#define IO_PDN  1  // Pull-down input pin

#define SMB_ACK      (0)
#define SMB_NAK      (1)
#define SEND_STOP    (0)
#define NOSEND_STOP  (1)
#define SEND_START   (0)
#define NOSEND_START (1)


// *************************   MACROS   ************************************

/* I/O PORT CONFIGURATION */
#define CAT1(x,y) x##y  // Concatenates 2 strings
#define CAT2(x,y) CAT1(x,y)  // Forces evaluation of CAT1

// OCM port I/O defintions

// Builds I/O port name: PNAME(1,INP) ==> P1INP
#define PNAME(y,z) CAT2(P,CAT2(y,z))

// Builds I/O bit name: BNAME(1,2) ==> P1_2
#define BNAME(port,pin) CAT2(CAT2(P,port),CAT2(_,pin))

// OCM port I/O defintions
#define OCM_SCL BNAME(OCM_CLK_PORT, OCM_CLK_PIN)
#define OCM_SDA BNAME(OCM_DATA_PORT, OCM_DATA_PIN)

#define IO_DIR_PORT_PIN(port, pin, dir) \
{\
  if ( dir == IO_OUT ) \
    PNAME(port,DIR) |= (1<<(pin)); \
  else \
    PNAME(port,DIR) &= ~(1<<(pin)); \
}

#define OCM_DATA_HIGH()\
{ \
  IO_DIR_PORT_PIN(OCM_DATA_PORT, OCM_DATA_PIN, IO_IN); \
}

#define OCM_DATA_HIGHW()\
{ \
  IO_DIR_PORT_PIN(OCM_DATA_PORT, OCM_DATA_PIN, IO_OUT); \
  OCM_SDA = 1;\
}

#define OCM_DATA_LOW() \
{ \
  IO_DIR_PORT_PIN(OCM_DATA_PORT, OCM_DATA_PIN, IO_OUT); \
  OCM_SDA = 0;\
}

#define IO_FUNC_PORT_PIN(port, pin, func) \
{ \
  if( port < 2 ) \
  { \
    if ( func == IO_PER ) \
      PNAME(port,SEL) |= (1<<(pin)); \
    else \
      PNAME(port,SEL) &= ~(1<<(pin)); \
  } \
  else \
  { \
    if ( func == IO_PER ) \
      P2SEL |= (1<<(pin>>1)); \
    else \
      P2SEL &= ~(1<<(pin>>1)); \
  } \
}

#define IO_IMODE_PORT_PIN(port, pin, mode) \
{ \
  if ( mode == IO_TRI ) \
    PNAME(port,INP) |= (1<<(pin)); \
  else \
    PNAME(port,INP) &= ~(1<<(pin)); \
}

#define IO_PUD_PORT(port, dir) \
{ \
  if ( dir == IO_PDN ) \
    P2INP |= (1<<(port+5)); \
  else \
    P2INP &= ~(1<<(port+5)); \
}

void I2CInit( void )
{

    // Set port pins as inputs
    IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_IN );
    IO_DIR_PORT_PIN( OCM_DATA_PORT, OCM_DATA_PIN, IO_IN );

    // Set for general I/O operation
    IO_FUNC_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_GIO );
    IO_FUNC_PORT_PIN( OCM_DATA_PORT, OCM_DATA_PIN, IO_GIO );

    // Set I/O mode for pull-up/pull-down
    IO_IMODE_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_PUD );
    IO_IMODE_PORT_PIN( OCM_DATA_PORT, OCM_DATA_PIN, IO_PUD );

    // Set pins to pull-up
     IO_PUD_PORT( OCM_CLK_PORT, IO_PUP );
    IO_PUD_PORT( OCM_DATA_PORT, IO_PUP );

}


void WaitUs2(uint16 microSecs)
{
 return;
}

void memsWait( int count )
{

  int i ;
  i = count;
  do
  { 
	 i--;
  }while ( i > 0 );
}

void memsClock( bool dir )
{
  if ( dir )
  {
    IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_IN );
  }
  else
  {
    IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_OUT );
    OCM_SCL = 0;
  }
  memsWait(1);
}

void memsWrite( bool dBit )
{
  memsClock( 0 );
  if ( dBit )
  {
    OCM_DATA_HIGHW();
  }
  else
  {
    OCM_DATA_LOW();
  }
  memsClock( 1 );
}

bool memsSendByte( uint8 dByte )
{
  uint8 i;
  bool akk;

  for ( i = 0; i < 8; i++ )
  {
    memsWrite( dByte & 0x80 );
    dByte <<= 1;
  }
  memsClock( 0 );
  memsWait(2);
  OCM_DATA_HIGH();
  memsClock( 1 );
  akk = !OCM_SDA;
  memsWait(5);
  IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_OUT );
  OCM_SCL = 0;
  memsWait(1);
  return ( akk );
}


void memsStart( void )
{
   uint8 retry = MEMS_RETRY_CNT;

  memsClock(1);

  do {

    if (OCM_SCL)
    {
      break;
    }
    memsWait(1);
  } while ( --retry );

  OCM_DATA_LOW();
  memsWait(1);
  memsClock( 0 );
}

void memsStop( void )
{
  memsClock(0);
  memsWait(1);
  OCM_DATA_LOW();
  memsWait(1);
  memsClock( 1 );
  OCM_DATA_HIGH(); 
  memsWait(1);
}

bool memsRead( void )
{
  memsClock( 0 );
  memsClock( 1 );
  return OCM_SDA;
}

uint8 memsReceiveByte(bool nak)
{
  int8 i, rval = 0;
  OCM_DATA_HIGH();

  for (i=7; i>=0; --i)  {
    if (memsRead())  {
      rval |= 1<<i;
    }
  }

  memsClock( 0 );
  memsClock( 1 );
  if (nak)
  {
    OCM_DATA_HIGH();
  }
  else
  {
    OCM_DATA_LOW();
  }
  memsClock( 1 );
  memsWait(1);
  memsClock( 0 );

  return rval;
}

void memsSendDeviceAddress(uint8 address)
{
  uint8 retry = MEMS_RETRY_CNT;

  do {
    memsStart();
    if (memsSendByte(address))  // do ack polling...
    {
      break;
    }
  } while ( --retry );
}

void init_mems()
{
  
    // Initialize the BOSCH  BMA250E  MEMS
  
       memsSendDeviceAddress(0x30);
       memsSendByte(0x00);              // Chip ID Register
       memsClock( 0 );
       memsWait(3);
       OCM_DATA_HIGH();
       memsSendDeviceAddress(0x31);
       OCM_DATA_HIGH();
       memsWait(5);
       memsReceiveByte(1);              // Just get the ID
       memsClock( 1 );
       memsWait(5);
       memsStop();
       memsWait(5);
	
   
       memsSendDeviceAddress(0x30);
       memsWait(5);
       memsSendByte(0x0f);              // Accelerometer G-Range          
       memsClock( 0 );
       memsWait(5);
       memsSendByte(0x05);             // "0101 -- +/- 4g Range"
       memsWait(5);
       memsStop();
       memsWait(5);

       memsSendDeviceAddress(0x30);
       memsWait(5);
       memsSendByte(0x10);             // Accelerometer Filter Bandwidth
       memsClock( 0 );
       memsWait(5);
       memsSendByte(0x0d);             // "1101 - -- 250Hz "
       memsWait(5);
       memsStop();
       memsWait(5);

       memsSendDeviceAddress(0x30);
       memsWait(5);
       memsSendByte(0x11);              // Accelerometer Power Mode 
       memsClock( 0 );
       memsWait(5);
       memsSendByte(0x00);              // "00000000 --- NORMAL MODE" 
       memsWait(5);
       memsStop();
       memsWait(5);
       
}

uint8 read_mems_reg(uint8 memsreg)
{
     
  uint8 readval ;
  
       // Read the BMA250E Registers
   
       memsSendDeviceAddress(0x30);
       memsSendByte(memsreg);
       memsClock( 0 );
       memsWait(3);
       OCM_DATA_HIGH();
       memsSendDeviceAddress(0x31);
       OCM_DATA_HIGH();
       memsWait(5);
       readval = memsReceiveByte(1);;
       memsStop();
       return readval;
}

void WaitUs(uint16 microSecs)
{
 do
  {
    /* 32 NOPs == 1 usecs */
  asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  } while(microSecs--);
}


// GAP Role Callbacks
gapRolesCBs_t BAT_MONi2c_BroadcasterCBs =
{
  NULL,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};


uint16 AdcRead (uint8 channel, uint8 resolution)
{
  int16  reading = 0;


  uint8   i, resbits;
  uint8  adcChannel = 1;

  int16 waitcount;
  
  if (channel <= HAL_ADC_CHANNEL_7)
  {
    for (i=0; i < channel; i++)
    {
      adcChannel <<= 1;
    }

    /* Enable channel */
    ADCCFG |= adcChannel;
  }

  /* Convert resolution to decimation rate */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      resbits = HAL_ADC_DEC_064;
      break;
    case HAL_ADC_RESOLUTION_10:
      resbits = HAL_ADC_DEC_128;
      break;
    case HAL_ADC_RESOLUTION_12:
      resbits = HAL_ADC_DEC_256;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      resbits = HAL_ADC_DEC_512;
      break;
  }
  
  /* writing to this register starts the extra conversion */
  ADCCON3 = channel | resbits | adcRef;

  waitcount = 0;
  
  /* Wait for the conversion to be done */
  while (!(ADCCON1 & HAL_ADC_EOC))
  {
    WaitUs(1000);         // 1 Millsecond (?)
    waitcount++;
    if (waitcount > 500) /*Half a second wait */
      return ((uint16) 0xeffe);
  }

  /* Disable channel after done conversion */
  if (channel <= HAL_ADC_CHANNEL_7)
    ADCCFG &= (adcChannel ^ 0xFF);

  /* Read the result */
  reading = (int16) (ADCL);
  reading |= (int16) (ADCH << 8);

  /* Treat small negative as 0 */
  if (reading < 0)
    reading = 0;

  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      reading >>= 8;
      break;
    case HAL_ADC_RESOLUTION_10:
      reading >>= 6;
      break;
    case HAL_ADC_RESOLUTION_12:
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      reading >>= 2;
    break;
  }

  return ((uint16)reading);
}

void BAT_MONi2c_Init( uint8 task_id )
{
  BAT_MONi2c_TaskID = task_id;
  uint8 new_adv_enabled_status; //randthird;
  uint16 adc;
  
  P0SEL=0x00;
  P0DIR=0x40;
  adcRef = 0x80;
  P0 = 0x00;
  
  I2CInit();
  init_mems();
  
  bdcount = 0;
  
  adc = AdcRead(HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14);

  srand(adc);
  //randthird = rand() & 0xff;
  baddr1[0] = adc & 0xff;
  baddr1[1] = (adc >> 8) & 0xff;
  //baddr1[2] = randthird;
  
  baddr1[2] = 0x01;
  
  scanRspData1[13] = hexval[baddr1[0] & 0xf];
  scanRspData1[12] = hexval[(baddr1[0] >> 4) & 0xf];

  scanRspData1[11] = hexval[baddr1[1] & 0xf];
  scanRspData1[10] = hexval[(baddr1[1] >> 4) & 0xf];	

  scanRspData1[9] = hexval[baddr1[2] & 0xf];
  scanRspData1[8] = hexval[(baddr1[2] >> 4) & 0xf];
  
  aData1[16] = baddr1[0];
  aData1[15] = baddr1[1];
  aData1[14] = baddr1[2];
  
  HCI_EXT_SetBDADDRCmd(baddr1);
  // Setup the GAP Broadcaster Role Profile
  {
    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 100;

    //GAP_ConfigDeviceAddr(1,baddr);
    //uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;   // use non-connectable advertisements
    uint8 advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable unidirected advertisements
    new_adv_enabled_status = TRUE;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData1 ), scanRspData1 );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( aData1 ), aData1 );
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
	
  }

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  osal_set_event( BAT_MONi2c_TaskID, SBP_START_DEVICE_EVT );
  
}




void performPeriodicTask( void )
{

        bdcount++;
  	LedBlink++;


	if (LedBlink & 1) 
		P0 = 0x40;
	else
		P0 = 0x00;
  

	uint16 adc = AdcRead(HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14); //P0.7 

        // Just read the Accelerometer Data and Send it on the advertisement body
        
        aData1[19] = read_mems_reg(0x02);       // Bit7 Bit6 (lsb X) Bit 0 - New X Data Flag
        aData1[20] = read_mems_reg(0x03);       // Bit7-Bit0 (msb X)
        aData1[21] = read_mems_reg(0x04);       // Bit7 Bit6 (lsb Y) Bit 0 - New Y Data Flag
        aData1[22] = read_mems_reg(0x05);       // Bit7-Bit0 (msb Y)
        aData1[23] = read_mems_reg(0x06);       // Bit7 Bit6 (lsb Z) Bit 0 - New Z Data Flag
        aData1[24] = read_mems_reg(0x07);       // Bit7-Bit0 (msb Z)
        aData1[25] = read_mems_reg(0x08) + 23;  // Two S-Compliment Chip temperature
        
        // Send also the voltage on the Advertisement Boyd
        
        aData1[17] = adc & 0xff;
	aData1[18] = (adc >> 8) & 0xff;

	GAP_UpdateAdvertisingData( BAT_MONi2c_TaskID,TRUE,sizeof ( aData1 ),aData1);
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData1 ), scanRspData1 );
	
	//P0 = 0;
	
}



/*********************************************************************
 * @fn      BAT_MONi2c_ProcessEvent
 *
 * @brief   Simple BLE Broadcaster Application Task event processor. This
 *          function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BAT_MONi2c_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
    return (events ^ SYS_EVENT_MSG);

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Set timer for first periodic event

    osal_start_timerEx( BAT_MONi2c_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    // Start the Device
    VOID GAPRole_StartDevice( &BAT_MONi2c_BroadcasterCBs );
   
    //VOID GAPRole_GetParameter(GAPROLE_BD_ADDR, &baddr0);    

    return ( events ^ SBP_START_DEVICE_EVT );
  }
  
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
      osal_start_timerEx( BAT_MONi2c_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    // Perform periodic application task
    performPeriodicTask();
    return (events ^ SBP_PERIODIC_EVT);
  }  
  // Discard unknown events
  return 0;
}




