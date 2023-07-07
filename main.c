/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 MCU Multiple GPIO Interrupts
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
/* CY ASSERT failure */
#define CY_ASSERT_FAILED          (0u)

/* MACRO to enable printing UART messages to terminal */
#define DEBUG_PRINT               (0u)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* GPIO Port Interrupt Configuration */
const cy_stc_sysint_t GPIO_Port_intr_config =
{
    .intrSrc = CYBSP_PIN_0_IRQ,         /* Source of interrupt signal */
    .intrPriority = 3u,                 /* Interrupt priority */
};

/* Flags used to indicate PIN Interrupt event.
 * 0: No PIN interrupt triggered
 * 1: PIN interrupt triggered
 */
volatile bool PIN_0_Flag = 0;
volatile bool PIN_1_Flag = 0;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
/* GPIO ISR */
void GPIO_Port_Interrupt_Handler(void);

/* Function to blink LED */
void LED_blink(void);

#if DEBUG_PRINT
cy_stc_scb_uart_context_t UART_context;

/* Variable used for tracking the print status */
volatile bool ENTER_LOOP = true;

/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the error message.
*
* Parameters:
*  error_msg - message to print if any error encountered.
*  status - status obtained after evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    char error_msg[50];

    sprintf(error_msg, "Error Code: 0x%08" PRIX32 "\n", status);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\nFAIL: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, error_msg);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
}
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize GPIO Port Interrupt
*  - check if GPIO interrupt has occurred
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_sysint_status_t intr_result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if DEBUG_PRINT
    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Sequence to clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

    /* Print "Program Start" */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "***************** ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PMG1 MCU: Multiple GPIO Interrupts ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "***************** \r\n\n");
#endif

    /* Initialize GPIO Port interrupt */
    intr_result = Cy_SysInt_Init(&GPIO_Port_intr_config, &GPIO_Port_Interrupt_Handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_SysInt_Init failed with error code", intr_result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Clear any pending interrupt and enable the GPIO Port Interrupt */
    NVIC_ClearPendingIRQ(GPIO_Port_intr_config.intrSrc);
    NVIC_EnableIRQ(GPIO_Port_intr_config.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    for(;;)
    {
        /* Check if PIN_0 triggered an interrupt */
        if(PIN_0_Flag)
        {
            /* Clear PIN_0 Interrupt Flag */
            PIN_0_Flag = 0;
            /* Blink the LED */
            LED_blink();
#if DEBUG_PRINT
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "PIN_0 interrupt(falling edge) detected.\r\n");
#endif
        }
        /* Check if PIN_1 triggered an interrupt */
        if(PIN_1_Flag)
        {
            /* Clear PIN_1 Interrupt Flag */
            PIN_1_Flag = 0;
            /* Blink the LED */
            LED_blink();
#if DEBUG_PRINT
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "PIN_1 interrupt(rising edge) detected.\r\n\n");
#endif
        }
#if DEBUG_PRINT
        if (ENTER_LOOP)
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop\r\n");
            ENTER_LOOP = false;
        }
#endif
    }
}


/*******************************************************************************
* Function Name: GPIO_Port_Interrupt_Handler
********************************************************************************
*
* Summary:
*  This function is executed when an interrupt is triggered on the GPIO port.
*
*******************************************************************************/
void GPIO_Port_Interrupt_Handler(void)
{
    /* Variable to hold the interrupt status register of GPIO Port */
    uint32_t GPIO_PORT_intr_status = 0x0;

    /* Read GPIO Port Interrupt status register */
    GPIO_PORT_intr_status = CYBSP_PIN_0_PORT->INTR;

    /* Check if PIN_0 interrupt is triggered */
    if(GPIO_PORT_intr_status & (1 << CYBSP_PIN_0_PIN))
    {
        /* Set PIN_0 flag */
        PIN_0_Flag = 1;
        /* Clear the Interrupt for the PIN_0 */
        Cy_GPIO_ClearInterrupt(CYBSP_PIN_0_PORT, CYBSP_PIN_0_PIN);
    }
    /* Check if PIN_1 interrupt is triggered */
    if(GPIO_PORT_intr_status & ( 1 << CYBSP_PIN_1_PIN))
    {
        /* Set PIN_1 flag */
        PIN_1_Flag = 1;
        /* Clear the Interrupt for the PIN_1 */
        Cy_GPIO_ClearInterrupt(CYBSP_PIN_1_PORT, CYBSP_PIN_1_PIN);
    }
}

/*******************************************************************************
* Function Name: LED_blink
********************************************************************************
*
* Summary:
*  This function blinks the User LED.
*
*******************************************************************************/
void LED_blink(void)
{
    for(uint8_t i=0; i<2; i++)
    {
        /* Toggle the user LED state */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        /* Delay between LED toggles */
        Cy_SysLib_Delay(125);
    }
}
/* [] END OF FILE */
