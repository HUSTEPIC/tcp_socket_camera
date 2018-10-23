//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - TCP Socket
// Application Overview - This particular application illustrates how this
//                        device can be used as a client or server for TCP
//                        communication.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_TCP_Socket_Application
// or
// docs\examples\CC32xx_TCP_Socket_Application.pdf
//
//*****************************************************************************


//****************************************************************************
//
//! \addtogroup tcp_socket
//! @{
//
//****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// simplelink includes 
#include "simplelink.h"
#include "wlan.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"

// common interface includes 
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if_amo.h"
#endif

#include "pinmux.h"
#include "camera_app.h"

#include "gpio.h"

#include "timer.h"
#include "timer_if.h"

#include "button_if_amo.h"

//#if defined(HAL_OLED)  
#include "hal_lcd.h"
//#endif

#if defined(HAL_LED)  
#include "gpio_if.h"
#endif

#if defined(HAL_KEY)
#include "Board_key.h"    
#endif

#define APPLICATION_NAME        "TCP Socket Camera"
#define APPLICATION_VERSION     "1.1.1"

//#define IP_ADDR             0xc0a80109/* 192.168.1.9 */
//#define IP_ADDR            0xc0a8032F /* 192.168.3.47 */    
//#define IP_ADDR                         0xc0a80064 /* 192.168.0.101 */
#define IP_ADDR            0xc0a82B70 /* 192.168.43.112 */

#define PORT_NUM            5001
#define BUF_SIZE            (1400-2)
#define TCP_PACKET_COUNT    1000 

#define USE_CAMERA_DATA_EN              1       // 是否使用摄像头   

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int BsdTcpClient(unsigned short usPort);
int BsdTcpServer(unsigned short usPort);
static long WlanConnect();
static void DisplayBanner();
static void BoardInit();
static void InitializeAppVariables();


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned long  g_ulDestinationIp = IP_ADDR;
unsigned int   g_uiPortNum = PORT_NUM;
volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned char  g_ucConnectionStatus = 0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
unsigned char g_cBsdBuf[BUF_SIZE+2];
unsigned short g_index = 0;

unsigned char g_camera_init_ok = 0;                     // 摄像头是否初始化 ok ， 1= ok
unsigned char g_camera_send_flag = 0;


#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
        return;
    }

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
                        " BSSID: %x:%x:%x:%x:%x:%x\r\n",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \r\n",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                            "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \r\n",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\r\n",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(!pNetAppEvent)
    {
        return;
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_ulIpAddr = pEventData->ip;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                        "Gateway=%d.%d.%d.%d\r\n",
                            SL_IPV4_BYTE(g_ulIpAddr,3),
                            SL_IPV4_BYTE(g_ulIpAddr,2),
                            SL_IPV4_BYTE(g_ulIpAddr,1),
                            SL_IPV4_BYTE(g_ulIpAddr,0),
                            SL_IPV4_BYTE(g_ulGatewayIP,3),
                            SL_IPV4_BYTE(g_ulGatewayIP,2),
                            SL_IPV4_BYTE(g_ulGatewayIP,1),
                            SL_IPV4_BYTE(g_ulGatewayIP,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \r\n",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(!pSock)
    {
        return;
    }

    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
        	UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }

}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************



//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param[in]    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_ulDestinationIp = IP_ADDR;
    g_uiPortNum = PORT_NUM;
    g_ulPacketCount = TCP_PACKET_COUNT;
}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\r\n",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\r\n",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}



//****************************************************************************
//
//!    \brief Parse the input IP address from the user
//!
//!    \param[in]                     ucCMD (char pointer to input string)
//!
//!    \return                        0 : if correct IP, -1 : incorrect IP
//
//****************************************************************************
int IpAddressParser(char *ucCMD)
{
    volatile int i=0;
    unsigned int uiUserInputData;
    unsigned long ulUserIpAddress = 0;
    char *ucInpString;
    ucInpString = strtok(ucCMD, ".");
    uiUserInputData = (int)strtoul(ucInpString,0,10);
    while(i<4)
    {
        //
       // Check Whether IP is valid
       //
       if((ucInpString != NULL) && (uiUserInputData < 256))
       {
           ulUserIpAddress |= uiUserInputData;
           if(i < 3)
               ulUserIpAddress = ulUserIpAddress << 8;
           ucInpString=strtok(NULL,".");
           uiUserInputData = (int)strtoul(ucInpString,0,10);
           i++;
       }
       else
       {
           return -1;
       }
    }
    g_ulDestinationIp = ulUserIpAddress;
    return SUCCESS;
}
//*****************************************************************************
//
//! UserInput
//!
//! This function
//!        1. Function for reading the user input for UDP RX/TX
//!
//!  \return 0 : Success, -ve : failure
//
//*****************************************************************************
long UserInput()
{
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    int iRightInput = 0;
    unsigned long ulUserInputData = 0;

    UART_PRINT("Default settings: SSID Name: %s, PORT = %d, Packet Count = %d, "
                    "Destination IP: %d.%d.%d.%d\r\n",
                    SSID_NAME, g_uiPortNum, g_ulPacketCount,
                    SL_IPV4_BYTE(g_ulDestinationIp,3),
                    SL_IPV4_BYTE(g_ulDestinationIp,2),
                    SL_IPV4_BYTE(g_ulDestinationIp,1),
                    SL_IPV4_BYTE(g_ulDestinationIp,0));

    do
    {
        UART_PRINT("\r\nChoose Options:\r\n1. Send TCP packets.\r\n2. Receive "
                    "TCP packets.\r\n3. Settings.\r\n4. Exit\r\n");
        UART_PRINT("Enter the option to use: ");
        lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
        if(lRetVal == 0)
        {
          //
          // No input. Just an enter pressed probably. Display a prompt.
          //
          UART_PRINT("\n\r\nEnter Valid Input.");
        }
        else
        {
            iInput  = (int)strtoul(acCmdStore,0,10);
          if(iInput  == 1)
          {
              UART_PRINT("Run iperf command \"iperf.exe -s -i 1 -t 100\" and press "
                            "Enter\r\n");
                //
                // Wait to receive a character over UART
                //
                MAP_UARTCharGet(CONSOLE);
                UART_PRINT("Sending TCP packets...\r\n");

              // Before proceeding, please make sure to have a server waiting on
              // PORT_NUM
              lRetVal = BsdTcpClient(g_uiPortNum);
          }
          else if(iInput  == 2)
          {
              UART_PRINT("Press Enter and run iperf command \"iperf.exe -c "
                            "%d.%d.%d.%d -i 1 -t 100000\" \r\n",
                            SL_IPV4_BYTE(g_ulIpAddr,3),
                            SL_IPV4_BYTE(g_ulIpAddr,2),
                            SL_IPV4_BYTE(g_ulIpAddr,1),
                            SL_IPV4_BYTE(g_ulIpAddr,0));
                //
                // Wait to receive a character over UART
                //
                MAP_UARTCharGet(CONSOLE);
                UART_PRINT("Receiving TCP packets...\r\n");
                // After calling this function, you can start sending data to 
                // CC3200 IP address on PORT_NUM
                  lRetVal = BsdTcpServer(g_uiPortNum);
                
          }
          else if(iInput  == 3)
          {
              iRightInput = 0;
              do
              {
                  // get input for PORT/ IP/ Packet count
              UART_PRINT("\r\nSetting Options:\r\n1. PORT\r\n2. Packet Count\r\n"
                          "3. Destination IP\r\n4. Main Menu\r\n");
              UART_PRINT("Enter the option to use: ");
              lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
              if(lRetVal == 0)
              {
                //
                // No input. Just an enter pressed probably. Display a prompt.
                //
                UART_PRINT("\n\r\nEnter Valid Input.");
              }
              else
              {

                      iInput  = (int)strtoul(acCmdStore,0,10);
                //SettingInput(iInput);
                switch(iInput)
                {
                    case 1:
                        do
                        {
                            UART_PRINT("Enter new Port: ");
                            lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
                            if(lRetVal == 0)
                            {
                              //
                              // No input. Just an enter pressed probably. 
                              // Display a prompt.
                              //
                              UART_PRINT("\r\nEnter Valid Input.");
                              iRightInput = 0;
                            }
                            else
                            {
                                ulUserInputData = (int)strtoul(acCmdStore,0,10);
                              if(ulUserInputData <= 0 || ulUserInputData > 65535)
                              {
                                UART_PRINT("\r\nWrong Input");
                                iRightInput = 0;
                              }
                              else
                              {
                                  g_uiPortNum = ulUserInputData;
                                iRightInput = 1;
                              }
                            }

                            UART_PRINT("\r\n");
                        }while(!iRightInput);

                        iRightInput = 0;
                        break;
                    case 2:
                        do
                        {
                            UART_PRINT("Enter Packet Count: ");
                            lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
                            if(lRetVal == 0)
                            {
                              //
                              // No input. Just an enter pressed probably. 
                              // Display a prompt.
                              //
                              UART_PRINT("\r\nEnter Valid Input.");
                              iRightInput = 0;
                            }
                            else
                            {
                                ulUserInputData = (int)strtoul(acCmdStore,0,10);
                              if(ulUserInputData <= 0 || ulUserInputData > 9999999)
                              {
                                UART_PRINT("\r\nWrong Input");
                                iRightInput = 0;
                              }
                              else
                              {
                                  g_ulPacketCount = ulUserInputData;
                                iRightInput = 1;
                              }
                            }

                            UART_PRINT("\r\n");
                        }while(!iRightInput);
                        iRightInput = 0;
                        break;
                    case 3:
                        do
                        {
                            UART_PRINT("Enter Destination IP: ");
                            lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
                            if(lRetVal == 0)
                            {
                              //
                              // No input. Just an enter pressed probably. 
                              // Display a prompt.
                              //
                              UART_PRINT("\r\nEnter Valid Input.");
                              iRightInput = 0;
                            }
                            else
                            {
                            if(IpAddressParser(acCmdStore) < 0)
                              {
                                UART_PRINT("\r\nWrong Input");
                                iRightInput = 0;
                              }
                              else
                              {
                                iRightInput = 1;
                              }
                            }

                            UART_PRINT("\r\n");
                        }while(!iRightInput);
                        iRightInput = 0;
                        break;
                    case 4:
                        iRightInput = 1;
                        break;


                }

              }
          }while(!iRightInput);

          }
          else if(iInput == 4)
          {
              break;
          }
          else
          {
            UART_PRINT("\n\r\nWrong Input");
          }
        }
        UART_PRINT("\r\n");
    }while(1);

    return SUCCESS;

}


//****************************************************************************
//
//! \brief Opening a TCP client side socket and sending data
//!
//! This function opens a TCP socket and tries to connect to a Server IP_ADDR
//!    waiting on port PORT_NUM.
//!    If the socket connection is successful then the function will send 1000
//! TCP packets to the server.
//!
//! \param[in]      port number on which the server will be listening on
//!
//! \return    0 on success, -1 on Error.
//
//****************************************************************************
int BsdTcpClient(unsigned short usPort)
{
    int             iCounter;
    short           sTestBufLen;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    long            lLoopCount = 0;

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    sTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    // connecting to TCP server
    iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);       
        ASSERT_ON_ERROR(CONNECT_ERROR);
    }

    // sending multiple packets to the TCP server
    while (lLoopCount < g_ulPacketCount)
    {
        // sending packet
        iStatus = sl_Send(iSockID, g_cBsdBuf, sTestBufLen, 0 );
        if( iStatus < 0 )
        {
            // error
            sl_Close(iSockID);
            ASSERT_ON_ERROR(SEND_ERROR);
        }
        lLoopCount++;
    }

    Report("Sent %u packets successfully\r\n",g_ulPacketCount);

    iStatus = sl_Close(iSockID);
    //closing the socket after sending 1000 packets
    ASSERT_ON_ERROR(iStatus);

    return SUCCESS;
}


int tcp_Send_data(int iSockID, unsigned char *buff, long buff_lengh)
{
    short           sTestBufLen;
    int             iStatus;
    long            lest_bytes = buff_lengh; //剩余字节


    while (lest_bytes > 0)
    {
        if(lest_bytes > BUF_SIZE)
        {
            sTestBufLen = BUF_SIZE;
        }
        else if(lest_bytes > 0)
        {
            sTestBufLen = lest_bytes;
        }
        else
        {
            break;
        }

        lest_bytes -= sTestBufLen;

        g_index++;
        g_cBsdBuf[0] = ((g_index>>8) & 0xff);
        g_cBsdBuf[1] = (g_index & 0xff);
        
        memcpy(g_cBsdBuf+2, buff, sTestBufLen);
        buff += sTestBufLen;

        // sending packet
        iStatus = sl_Send(iSockID, g_cBsdBuf, sTestBufLen+2, 0 );
        if( iStatus < 0 )
        {
            // error
            sl_Close(iSockID);
            ASSERT_ON_ERROR(SEND_ERROR);
        }
        //UART_PRINT("%d, ", sTestBufLen+2);
        MAP_UtilsDelay(2000);
    }


    return SUCCESS;
}

int BsdTcpClient_camera(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             ret;


    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    
    // connecting to TCP server
    iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);       
        ASSERT_ON_ERROR(CONNECT_ERROR);
    }

    int frames = 0;
    int frames_error = 0;

    UART_PRINT("camera tcp Client run at port: 5001 >>> \r\n");
    g_camera_send_flag  = 1;

    while(1)
    {
        long frame_lenght = 0;
        unsigned char *pFrameBuffer = NULL;
        long head_lenght = 0;
        unsigned char *pHeadBuffer = NULL;

        extern unsigned long g_image_buffer[];
        extern char g_header[];        
        
#if (USE_CAMERA_DATA_EN > 0)
        if(g_camera_init_ok == 1)
        {
            frame_lenght = 0;
            pFrameBuffer = amo_camera_get_one_frame(&frame_lenght);
            head_lenght = 0;
            pHeadBuffer = amo_camera_get_jpg_head(&head_lenght);
        }
        else
        {
            head_lenght = 625;
            frame_lenght = 42000;
            pFrameBuffer = (unsigned char *)g_image_buffer;
            pHeadBuffer = (unsigned char *)g_header;   
        }
#else
        head_lenght = 625;
        rame_lenght = 42000;
        pFrameBuffer = (unsigned char *)g_image_buffer;
        pHeadBuffer = (unsigned char *)g_header;   
#endif
        //数据出错--- 出错时  frame_lenght == IMAGE_BUF_SIZE， 因此这里处理出错后恢复
        if(frame_lenght >= IMAGE_BUF_SIZE )
        {
            frames = 0;            
            UART_PRINT("\r\n");
            UART_PRINT("error frame_lenght = %d : [%d]\r\n", frame_lenght, frames_error);
            UART_PRINT("\r\n");               
            break;
        }
            
        ret = tcp_Send_data(iSockID, pHeadBuffer, head_lenght);
        ret = tcp_Send_data(iSockID, pFrameBuffer, frame_lenght);

        if(ret == SUCCESS)
        {
            if(g_camera_init_ok == 1)
            {
                UART_PRINT("f[%d], [%d]\r\n", frames++, head_lenght + frame_lenght);
            }
            else
            {
                frames++;
                
#if defined(HAL_OLED)    
                HalLcdWriteString("Camera ...", HAL_LCD_LINE_2);
                UtilsDelay(1000000);
                HalLcdWriteString("Camera Init Fail", HAL_LCD_LINE_2);
                UtilsDelay(1000000);
#endif  
                UART_PRINT("f[%d], [%d][Error : Camera init Failure]\r\n", frames++, head_lenght + frame_lenght);
            }

// 显示发送出去的帧数 (不一定是正确的视频帧，如果是摄像头初始化不过的，也发送的....)
#if defined(HAL_OLED)    
            HalLcdWriteStringValue("Frame : ", frames, 10, HAL_LCD_LINE_8);
#endif  
            
#if defined(HAL_LED)  
            GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
#endif
        }
        else
        {
            break;
        }
    }

    //Report("Sent %u packets successfully\r\n",g_ulPacketCount);

    iStatus = sl_Close(iSockID);
    //closing the socket after sending 1000 packets
    ASSERT_ON_ERROR(iStatus);

    UART_PRINT("\r\nexit tcp...\r\r\n\n");

#if defined(HAL_LED)  
    GPIO_IF_LedOff(MCU_ALL_LED_IND);
#endif

    return SUCCESS;
}


//****************************************************************************
//
//! \brief Opening a TCP server side socket and receiving data
//!
//! This function opens a TCP socket in Listen mode and waits for an incoming
//!    TCP connection.
//! If a socket connection is established then the function will try to read
//!    1000 TCP packets from the connected client.
//!
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//
//****************************************************************************
int BsdTcpServer(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             iNewSockID;
    long            lLoopCount = 0;
    long            lNonBlocking = 1;
    int             iTestBufLen;

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    iTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        // error
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    iAddrSize = sizeof(SlSockAddrIn_t);

    // binding the TCP socket to the TCP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        ASSERT_ON_ERROR(BIND_ERROR);
    }

    // putting the socket for listening to the incoming TCP connection
    iStatus = sl_Listen(iSockID, 0);
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(LISTEN_ERROR);
    }

    // setting socket option to make the socket as non blocking
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, 
                            &lNonBlocking, sizeof(lNonBlocking));
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }
    iNewSockID = SL_EAGAIN;

    // waiting for an incoming TCP connection
    while( iNewSockID < 0 )
    {
        // accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
        iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr, 
                                (SlSocklen_t*)&iAddrSize);
        if( iNewSockID == SL_EAGAIN )
        {
           MAP_UtilsDelay(10000);
        }
        else if( iNewSockID < 0 )
        {
            // error
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
    }

    // waits for 1000 packets from the connected TCP client
    while (lLoopCount < g_ulPacketCount)
    {
        iStatus = sl_Recv(iNewSockID, g_cBsdBuf, iTestBufLen, 0);
        if( iStatus <= 0 )
        {
          // error
          sl_Close(iNewSockID);
          sl_Close(iSockID);
          ASSERT_ON_ERROR(RECV_ERROR);
        }

        lLoopCount++;
    }

    Report("Recieved %u packets successfully\r\n",g_ulPacketCount);
    
    // close the connected socket after receiving from connected TCP client
    iStatus = sl_Close(iNewSockID);    
    ASSERT_ON_ERROR(iStatus);
    // close the listening socket
    iStatus = sl_Close(iSockID);
    ASSERT_ON_ERROR(iStatus);   

    return SUCCESS;
}

//****************************************************************************
//
//!  \brief Connecting to a WLAN Accesspoint
//!
//!   This function connects to the required AP (SSID_NAME) with Security
//!   parameters specified in te form of macros at the top of this file
//!
//!   \param[in]              None
//!
//!   \return     Status value
//!
//!   \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    /* Wait */
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\r\n");
    Report("\t\t *************************************************\r\n");
    Report("\t\t      CC3200 %s Application       \r\n", AppName);
    Report("\t\t *************************************************\r\n");
    Report("\n\n\r\n");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs) || defined (gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

#if defined(HAL_KEY)
// sw2 被按下时 keys = SW2， 松开时 keys=0x00
// sw2 被按下时 keys = SW3， 松开时 keys=0x00
static void KeyPressedCB(uint8 keys)
{
    static uint8 sw2 = 0;
    static uint8 sw3 = 0;
    char str[32];
    
    if(keys & SW2) 
    {
        sw2++;
    }

    if(keys & SW3)
    {
        sw3++;
    }

    if( ( keys & SW2)  || ( keys & SW3) ) 
    {
        // 这个是中断回调函数，理论上是不可以在这里做显示的，因此一旦视频处于发送阶段，就不允许再显示按键值了， 
        // 避免冲突，目前不是带操作系统的， 没有边界函数可用， 因此只能这个弄...  amomcu...
        if(g_camera_send_flag == 0)
        {
            sprintf(str , "SW2:%3d ; SW3:%3d", sw2, sw3);
#if defined(HAL_OLED)    
            HalLcdWriteString(str, HAL_LCD_LINE_8);
#endif  
        }
    }
}

#endif

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
    long lRetVal = -1;
    int i;
    char str[32];
    
    // 
    // Board Initialization
    //
    BoardInit();

    //
    // Initialize the uDMA
    //
    UDMAInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig_for_Uart1();  

    //
    // Configuring UART
    //
    InitTerm();

    //
    // Display banner
    //
    DisplayBanner(APPLICATION_NAME);
    InitializeAppVariables();


#if defined(HAL_OLED)    
    PinMuxConfig_for_LCD();

    HalLcd_HW_Init();    
    Draw_AmoMcu_Logo();    
    UtilsDelay(8000000);
    
    LCD_CLS();
    
    HalLcdWriteString("CC3200 Camera demo", HAL_LCD_LINE_1);
#endif
    
#if defined(HAL_KEY)
    Board_initKeys(KeyPressedCB);
    //
    // Register Push Button Handlers
    //
//    while(1)
//    {
//        
//    }
#endif    

#if defined(HAL_LED)  
    PinMuxConfig_for_LED();

    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    for(i = 0; i < 3; i++)
    {
        GPIO_IF_LedOn(MCU_ALL_LED_IND);
        MAP_UtilsDelay(2000000);
        GPIO_IF_LedOff(MCU_ALL_LED_IND);
        MAP_UtilsDelay(2000000);
    }
    
    for(i = 0; i < 3; i++)
    {
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        MAP_UtilsDelay(2000000);
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);

        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
        MAP_UtilsDelay(2000000);
        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        MAP_UtilsDelay(2000000);
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
    }
#endif

    // sw3 用的是gpio13，这个gpio13 在摄像头中也用了， 所以这里只能先屏蔽 sw3
    Button_IF_DisableInterrupt(SW3);


    PinMuxConfig_for_Uart1();  

    // 注意 LED灯 D5 用的是gpio11，这个gpio11 在摄像头中也用了， 所以后面该led 灯会失效
    // 注意 LED灯 D6 用的是gpio10，这个gpio10 在本例中作为uart1的TXD 了， 所以后面该led 灯会失效
    // LED灯 D7 全程有效， 所有之后我们尽可以操作 D7 这个 led灯，也就是 红色灯， 例如点亮该灯的函数为 GPIO_IF_LedOn(MCU_RED_LED_GPIO); 

#if (USE_CAMERA_DATA_EN > 0)

#if defined(HAL_OLED)    
    HalLcdWriteString("Camera Init...", HAL_LCD_LINE_2);
#endif

    PinMuxConfig_for_Camera();

    UART_PRINT("amo_camera_init start... \r\n", __LINE__);   
    if(SUCCESS == amo_camera_init())
    {
        g_camera_init_ok = 1;
        UART_PRINT("amo_camera_init SUCCESS!!! \r\n");   
    }
    else
    {
        g_camera_init_ok = 0;    
        UART_PRINT("amo_camera_init FAILURE!!! \r\n");   
    }
    UART_PRINT("amo_camera_init finish... \r\n", __LINE__);   
#endif

#if defined(HAL_OLED)    
    HalLcdWriteString(g_camera_init_ok ? "Camera Init Success" : "Camera Init Fail", HAL_LCD_LINE_2);

    sprintf(str, "IP: %d.%d.%d.%d",
                    SL_IPV4_BYTE(g_ulDestinationIp,3),
                    SL_IPV4_BYTE(g_ulDestinationIp,2),
                    SL_IPV4_BYTE(g_ulDestinationIp,1),
                    SL_IPV4_BYTE(g_ulDestinationIp,0));
    HalLcdWriteString(str, HAL_LCD_LINE_3);

    sprintf(str, "PORT: %d",g_uiPortNum);
    HalLcdWriteString(str, HAL_LCD_LINE_4);
#endif

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();

    if(lRetVal < 0)
    {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
         UART_PRINT("Failed to configure the device in its default state \r\n");

      LOOP_FOREVER();
    }

    UART_PRINT("Device is configured in default state \r\n");

    //
    // Asumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \r\n");
        LOOP_FOREVER();
    }

    UART_PRINT("Device started as STATION \r\n");


#if defined(HAL_OLED)    
    HalLcdWriteString("Connecting...", HAL_LCD_LINE_5);
#endif

    UART_PRINT("Connecting to AP: %s ...\r\n",SSID_NAME);

    // Connecting to WLAN AP - Set with static parameters defined at common.h
    // After this call we will be connected and have IP address
    lRetVal = WlanConnect();
    if(lRetVal < 0)
    {
        UART_PRINT("Connection to AP failed \r\n");
        LOOP_FOREVER();
    }


    UART_PRINT("Connected to AP: %s \r\n",SSID_NAME);

    UART_PRINT("Device IP: %d.%d.%d.%d\r\n\r\n",
                      SL_IPV4_BYTE(g_ulIpAddr,3),
                      SL_IPV4_BYTE(g_ulIpAddr,2),
                      SL_IPV4_BYTE(g_ulIpAddr,1),
                      SL_IPV4_BYTE(g_ulIpAddr,0));

#if defined(HAL_OLED)    
    HalLcdWriteString("Connected", HAL_LCD_LINE_5);

    sprintf(str, "%s", SSID_NAME);
    HalLcdWriteString(str, HAL_LCD_LINE_6);

    sprintf(str, "IP: %d.%d.%d.%d",
                    SL_IPV4_BYTE(g_ulIpAddr,3),
                    SL_IPV4_BYTE(g_ulIpAddr,2),
                    SL_IPV4_BYTE(g_ulIpAddr,1),
                    SL_IPV4_BYTE(g_ulIpAddr,0));
    HalLcdWriteString(str, HAL_LCD_LINE_7);
#endif

#ifdef USER_INPUT_ENABLE
    lRetVal = UserInput();
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
#else

    while(1)
    {
        lRetVal = BsdTcpClient_camera(PORT_NUM);
        if(lRetVal < 0)
        {
            UART_PRINT("TCP Client failed\r\n");
            //LOOP_FOREVER();
        }

        MAP_UtilsDelay(1000 * 1000);
    }
#endif

    UART_PRINT("Exiting Application ...\r\n");

    //
    // power of the Network processor
    //
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    while (1)
    {
        _SlNonOsMainLoopTask();
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
