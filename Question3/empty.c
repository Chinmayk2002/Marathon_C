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

#include "ti_msp_dl_config.h"
#define LED_ON 800

volatile uint16_t uint_16_current_btn_state = 0;
volatile uint16_t uint_16_stable_btn_state = 0;
volatile uint16_t uint_16_previous_btn_state = 0;

uint32_t delay_counter =0;
uint32_t on_count =0;



int button_1_pressed(void);

int button_1_pressed(){
    if(uint_16_previous_btn_state==uint_16_current_btn_state){
        if(uint_16_current_btn_state !=uint_16_stable_btn_state){
            if(uint_16_current_btn_state){
                uint_16_stable_btn_state =1;
                return 1;
            }
            else{
                uint_16_stable_btn_state =0;
            }
        }
    }
    return 0;
}

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_0_INST);

    while (1) {
        if(button_1_pressed()){
            on_count =0;
            DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN);
            on_count = delay_counter + LED_ON;
        }
        if(on_count==delay_counter){
            DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN);
            on_count =0;
        }
    }
}

void TIMER_0_INST_IRQHandler(void){
    delay_counter++;
    uint_16_previous_btn_state = uint_16_current_btn_state;
    DL_GPIO_readPins(GPIO_BTN_PORT,GPIO_BTN_PIN) ? (uint_16_current_btn_state =1) : (uint_16_current_btn_state =0);

}