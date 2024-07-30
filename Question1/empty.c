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
#include <machine/_stdint.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

#define SYSCTL_SYSOSCCFG_FREQ_SYSOSCBASE         ((uint32_t)0x00000000U)
#define DL_SYSCTL_SYSOSC_FREQ_BASE  (SYSCTL_SYSOSCCFG_FREQ_SYSOSCBASE)
#define DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE)
#define SYSCTL_BORTHRESHOLD_LEVEL_BORMIN         ((uint32_t)0x00000000U)
#define DL_SYSCTL_BOR_THRESHOLD_LEVEL_0  (SYSCTL_BORTHRESHOLD_LEVEL_BORMIN)
#define DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0)
#define POWER_STARTUP_DELAY                                                (16)
#define GPIOB_BASE (0x400A2000U)

#define GPIO_PORT (GPIOB)
#define DL_GPIO_PIN_22                                              (0x00400000)
#define GPIO_LED1_PIN                                           (DL_GPIO_PIN_22)
#define GPIO_LED1_IOMUX                                          (IOMUX_PINCM50)
#define DL_GPIO_PIN_26                                              (0x04000000)
#define GPIO_LED2_PIN                                           (DL_GPIO_PIN_26)
#define GPIO_LED2_IOMUX                                          (IOMUX_PINCM57)
#define DL_SYSCTL_MCLK_DIVIDER_DISABLE  (0x0)
#define SYSCTL_MCLKCFG_UDIV_NODIVIDE             ((uint32_t)0x00000000U)
#define DL_SYSCTL_ULPCLK_DIV_1  (SYSCTL_MCLKCFG_UDIV_NODIVIDE)

#define DELAY (10000000)

int main(void)
{
    SYSCFG_DL_init();
    uint32_t counter =0;
    while (1) {
        if(counter%2==1){
            DL_GPIO_clearPins(GPIO_PORT, GPIO_LED2_PIN);
            DL_GPIO_togglePins(GPIO_PORT, GPIO_LED1_PIN); 
        }else{
            DL_GPIO_clearPins(GPIO_PORT, GPIO_LED1_PIN);
            DL_GPIO_togglePins(GPIO_PORT, GPIO_LED2_PIN);         
        }
        counter++;
        delay_cycles(DELAY);
    }
}


 void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
}

 void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);
}

 void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initDigitalOutput(GPIO_LED1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_LED2_IOMUX);

    DL_GPIO_clearPins(GPIO_PORT, GPIO_LED1_PIN |
        GPIO_LED2_PIN);
    DL_GPIO_enableOutput(GPIO_PORT, GPIO_LED1_PIN |
        GPIO_LED2_PIN);

}


 void SYSCFG_DL_SYSCTL_init(void)
{

    //Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}

