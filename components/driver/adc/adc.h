/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/*******************************************************************************
    @file     adc.h
    @brief    Contains all functions support for adc driver
    @version  0.0
    @date     18. Oct. 2017
    @author   qing.han



*******************************************************************************/
#ifndef __ADC__H__
#define __ADC__H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "bus_dev.h"
#include "gpio.h"

#define    MAX_ADC_SAMPLE_SIZE     32
#define    ADC_CH_BASE             (0x40050400UL)

#define    ENABLE_ADC_INT       AP_ADCC->intr_mask |= 0x000001ff
#define    MASK_ADC_INT         AP_ADCC->intr_mask &= 0xfffffe00

#define    CLEAR_ADC_INT(n)        AP_ADCC->intr_clear |= BIT(n)
#define    CLEAR_ADC_INT_ALL      {AP_ADCC->intr_clear = 0x3ffff;}

#define    IS_CLAER_ADC_INT_VOICE (AP_ADCC->intr_clear & BIT(8))
#define    IS_CLAER_ADC_INT(n)    (AP_ADCC->intr_clear & BIT(n))




#define    ENABLE_ADC             (AP_PCRM->ANA_CTL |= BIT(3))
#define    DISABLE_ADC            (AP_PCRM->ANA_CTL &= ~BIT(3))

#define    ADC_CLOCK_ENABLE       (AP_PCRM->CLKHF_CTL1 |= BIT(13))
#define    ADC_CLOCK_DISABLE       (AP_PCRM->CLKHF_CTL1 &= ~BIT(13))

#define POLLING_MODE 0
#define INTERRUPT_MODE 1
#define CCOMPARE_MODE 2


#define adcMeasureTask_Compare_EVT                    0x0080

#define adcMeasureTask_Poilling_EVT                   0x0080


#define adcMeasureTask_EVT                            0x0080

#define ADC_USE_TIMEOUT 0
#define ADC_OP_TIMEOUT  100
#if(ADC_USE_TIMEOUT == 1)
#define ADC_INIT_TOUT(to) int to = hal_systick()
#define ADC_CHECK_TOUT(to, timeout, loginfo) {if(hal_ms_intv(to) > timeout){LOG(loginfo);return PPlus_ERR_TIMEOUT;}}
#else
#define ADC_INIT_TOUT(to)
#define ADC_CHECK_TOUT(to, timeout, loginfo)
#endif

/**************************************************************************************
    @fn          hal_get_adc_int_source

    @brief       This function process for get adc interrupt source,such as adc channel NO

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      adc interrupt source bit loaction(uint8_t)
 **************************************************************************************/
/*
    ADC note:
    There are ten pins which can config as analogy,there are some differences between them.
    hardware analogy index:
    gpio<11>/aio<0>
    gpio<23>/aio<1>/micphone bias reference voltage
    gpio<24>/aio<2>
    gpio<14>/aio<3>
    gpio<15>/aio<4>/micphone bias
    gpio<16>/aio<5>/32K XTAL input
    gpio<17>/aio<6>/32K XTAL output
    gpio<18>/aio<7>/pga in+
    gpio<25>/aio<8>
    gpio<20>/aio<9>/pga in-

    There are six pins which can work in adc single mode.Such as:
    ADC_CH0 = 2,ADC_CH1N_P11 = 2,
    ADC_CH1 = 3,ADC_CH1P_P23 = 3,
    ADC_CH2 = 4,ADC_CH2N_P24 = 4,
    ADC_CH3 = 5,ADC_CH2P_P14 = 5,
    ADC_CH4 = 6,ADC_CH3N_P15 = 6,
    ADC_CH9 = 7,ADC_CH3P_P20 = 7,

    There are four pair pins which can work in adc diff mode.Such as:
    ADC_CH0DIFF = 1,p18(p) and P25(n)
    ADC_CH1DIFF = 3,P23(p) and P11(n)
    ADC_CH2DIFF = 5,P14(p) and P24(n)
    ADC_CH3DIFF = 7,P20(p) and P15(n)

    There are two pins which uses with 32.768K crystal oscillator.
    gpio<16>/aio<5>/32K XTAL input
    gpio<17>/aio<6>/32K XTAL output

    There are four pins which uses as pga,voice and so on.
    gpio<23>/aio<1>/micphone bias reference voltage,this pin is selected
    gpio<15>/aio<4>/micphone bias
    gpio<18>/aio<7>/pga in+
    gpio<20>/aio<9>/pga in-
*/
typedef enum
{
    ADC_CH0DIFF = 1,/*p18(positive),p25(negative),only works in diff*/
    ADC_CH0 = 2,ADC_CH1N_P11 = 2,
    ADC_CH1 = 3,ADC_CH1P_P23 = 3,ADC_CH1DIFF = 3,/*P23 and P11*/
    ADC_CH2 = 4,ADC_CH2N_P24 = 4,
    ADC_CH3 = 5,ADC_CH2P_P14 = 5,ADC_CH2DIFF = 5,/*P14 and P24*/
    ADC_CH4 = 6,ADC_CH3N_P15 = 6,
    ADC_CH9 = 7,ADC_CH3P_P20 = 7,ADC_CH3DIFF = 7,/*P20 and P15*/
    ADC_CH_VOICE = 8,
    ADC_CH_NUM =9,
} adc_CH_t;


#define ADC_BIT(ch) (1<<ch)

enum
{
    HAL_ADC_EVT_DATA = 1,
    HAL_ADC_EVT_FAIL = 0xff
};

typedef enum
{
    HAL_ADC_CLOCK_80K = 0,
    HAL_ADC_CLOCK_160K = 1,
    HAL_ADC_CLOCK_320K = 2,
} adc_CLOCK_SEL_t;

typedef struct _adc_Cfg_t
{
    uint8_t channel;
    bool  is_continue_mode;
    uint8_t  is_differential_mode;
    uint8_t  is_high_resolution;
} adc_Cfg_t;


typedef struct _adc_Evt_t
{
    int       type;
    adc_CH_t  ch;
    uint16_t* data;
    uint8_t   size; //word size
} adc_Evt_t;

typedef void (*adc_Hdl_t)(adc_Evt_t* pev);

typedef struct _adc_Contex_t
{
    bool        enable;
    uint8_t     all_channel;
    uint8_t     chs_en_shadow;
    bool        continue_mode;
    adc_Hdl_t   evt_handler[ADC_CH_NUM];
} adc_Ctx_t;

extern gpio_pin_e s_pinmap[ADC_CH_NUM];
/**************************************************************************************
    @fn          hal_adc_init

    @brief       This function process for adc initial

    input parameters

    @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
                ADC_SEMODE_e semode: single-end mode and diff mode select; 1:SINGLE_END(single-end mode) 0:DIFF(Diff mode)
                IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V

    output parameters

    @param       None.

    @return      None.
 **************************************************************************************/
void hal_adc_init(void);

int hal_adc_config_channel(adc_Cfg_t cfg, adc_Hdl_t evt_handler);

int hal_adc_clock_config(adc_CLOCK_SEL_t clk);

int hal_adc_start(uint8_t adc_mode);

int hal_adc_stop(void);

void hal_adc_value_read(adc_CH_t ch);

void __attribute__((weak)) hal_ADC_IRQHandler(void);

double hal_adc_value_cal(adc_CH_t ch,uint16_t* buf, uint32_t size, uint8_t high_resol, uint8_t diff_mode);

int hal_adc_deinit(void);

static void hal_adc_load_calibration_value(void);

int hal_adc_compare_start(void);


#define ADC_COMPARE_FILTER_MAX_TIME  10


static void adc_compare_cb(uint16_t ch,uint32_t status);
static void __attribute__((used)) hal_ADC_compare_IRQHandler(void);
int hal_poilling_adc_stop(void);
extern int hal_adc_compare_enable(adc_CH_t ch,uint32_t flag,uint32_t th_high,uint32_t th_low);
extern  int hal_adc_comppare_reset(adc_CH_t ch);
extern bool adc_get_high_threshold_flag(void);
void clear_adcc_cfg(void);

#ifdef __cplusplus
}
#endif

#endif
