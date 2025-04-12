/******************************************************************************
* File Name: main.c
*
* Version: 1.10
*
* Description: This is the source code for the ADC and UART code example.
*
* Related Document: CE195277_ADC_and_UART.pdf
*
* Hardware Dependency: See CE195277_ADC_and_UART.pdf
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/

#include <project.h>
#include "stdio.h"

/* Project Defines */
#define FALSE  0
#define TRUE   1
#define TRANSMIT_BUFFER_SIZE  16
int32 Output;



/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  main() performs following functions:
*  1: Starts the ADC and UART components.
*  2: Checks for ADC end of conversion.  Stores latest result in output
*     if conversion complete.
*  3: Checks for UART input.
*     On 'C' or 'c' received: transmits the last sample via the UART.
*     On 'S' or 's' received: continuously transmits samples as they are completed.
*     On 'X' or 'x' received: stops continuously transmitting samples.
*     On 'E' or 'e' received: transmits a dummy byte of data.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
int main()
{
    /* Variable to store ADC result */
    
    /* Variable to store UART received character */
    uint8 Ch;
    /* Variable used to send emulated data */
    uint8 EmulatedData;
    /* Flags used to store transmit data commands */
    uint8 ContinuouslySendData;
    uint8 SendSingleByte;
    uint8 SendEmulatedData;
    uint8 txData[2];      // transmit
    uint8 rxData[2];      // receive
  
    /* Transmit Buffer */
    char TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    char TransmitBuffer2[TRANSMIT_BUFFER_SIZE];
    
    CyGlobalIntEnable;
   
    UART_1_Start();
    I2C_1_Start();
    SPIM_1_Start();
    
    
    
    /* Initialize Variables */
    ContinuouslySendData = FALSE;
    SendSingleByte = FALSE;
    SendEmulatedData = FALSE;
    EmulatedData = 0;
    
    uint16 y;
    char buffer[32];
    uint32 Output2;

    
    /* Send message to verify COM port is connected properly */
    UART_1_PutString("COM Port Open");
  
    
    for(;;)
    {        
        /* Non-blocking call to get the latest data recieved  */
        Ch = UART_1_GetChar();
        
        /* Set flags based on UART command */
        switch(Ch)
        {
            case 0:
                /* No new data was recieved */
                break;
            case 'C':
            case 'c':
                SendSingleByte = TRUE;
                break;
            case 'S':
            case 's':
                ContinuouslySendData = TRUE;
                break;
            case 'X':
            case 'x':
                ContinuouslySendData = FALSE;
                break;
            case 'E':
            case 'e':
                SendEmulatedData = TRUE;
                break;
            default:
                /* Place error handling code here */
                break;    
        }
           
            
            /* Send data based on last UART command */
            if((SendSingleByte || ContinuouslySendData))
            {
                /*I2C**************************************************************************************************/
                I2C_1_MasterSendStart(0x4A,I2C_1_WRITE_XFER_MODE);
                I2C_1_MasterWriteByte(0x00);
                I2C_1_MasterSendRestart(0x4A, I2C_1_READ_XFER_MODE);
                Output =I2C_1_MasterReadByte(I2C_1_NAK_DATA);
                I2C_1_MasterSendStop();
                
               /*SPI***************************************************************************************************/
                SPIM_1_ClearFIFO();
                
                SPIM_1_WriteTxData((uint16)0x0000);
                Output2 = SPIM_1_ReadRxData();
                Output2 = (Output2>>1)/2.08-0.96; 
                                            
                /*OUTPUT**************************************************************************************************/
                sprintf(TransmitBuffer2, "{\"TC74A2\": %lu",Output);
                UART_1_PutString(TransmitBuffer2);
                 sprintf(buffer, ", \"LM35\": %lu",Output2);
                UART_1_PutString(buffer);
                UART_1_PutString("}\r\n");
                
                SendSingleByte = FALSE;
                
                /*****************************************************************************************************/
            }
            
            else if(SendEmulatedData)
            {
                /* Format ADC result for transmition */
                sprintf(TransmitBuffer, "Emulated Data: %x \r\n", EmulatedData);
                /* Send out the data */
                UART_1_PutString(TransmitBuffer);
                EmulatedData++;
                /* Reset the send once flag */
                SendEmulatedData = FALSE;   
            }
        }
    }



/* [] END OF FILE */
