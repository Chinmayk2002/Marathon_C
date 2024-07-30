/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

volatile uint32_t button1_pressed =0;
volatile uint32_t button2_pressed =0;
volatile uint32_t counter1 = 0;
volatile uint32_t counter2 = 0;
#define LED_DELAY (10000000)

int main(void)
{
    SYSCFG_DL_init();
    while (1) {

        if (DL_GPIO_readPins(GPIO_BTN1_PORT,GPIO_BTN1_PIN)) {
            button1_pressed = 1;
            counter1++;
        }else {
            button1_pressed = 0;
        }
        if (DL_GPIO_readPins(GPIO_BTN2_PORT,GPIO_BTN2_PIN)) {
            button2_pressed = 1;
            counter2++;
        }else {
            button2_pressed = 0;
        }  

        if (button1_pressed==1) {
            DL_GPIO_setPins(GPIO_LED1_PORT, GPIO_LED1_PIN);
            delay_cycles(LED_DELAY);
            DL_GPIO_clearPins(GPIO_LED1_PORT,GPIO_LED1_PIN);
            DL_GPIO_clearPins(GPIO_LED2_PORT,GPIO_LED2_PIN);
            delay_cycles(LED_DELAY); 
            DL_GPIO_setPins(GPIO_LED2_PORT, GPIO_LED2_PIN);
        }
        if (button2_pressed==1) {
            DL_GPIO_clearPins(GPIO_LED2_PORT,GPIO_LED2_PIN);
            delay_cycles(LED_DELAY); 
            DL_GPIO_setPins(GPIO_LED2_PORT, GPIO_LED2_PIN);
            DL_GPIO_setPins(GPIO_LED1_PORT, GPIO_LED1_PIN);
            delay_cycles(LED_DELAY);
            DL_GPIO_clearPins(GPIO_LED1_PORT,GPIO_LED1_PIN);           
        }
        
    }
}
