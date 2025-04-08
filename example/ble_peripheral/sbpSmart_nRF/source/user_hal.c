/*********************************************************************
 * INCLUDE
 */
#include "user_hal.h"
#include "log.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "simpleBLEPeripheral.h"
#include "adc.h"
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16_t Battery_Voltage = 0;
/*********************************************************************
 * LOCAL VARIABLES
 */
static adc_Cfg_t adc_cfg =
{
    .channel = ADC_BIT(ADC_CH2P_P14),
    .is_continue_mode = FALSE,
    .is_differential_mode = 0x00,
    .is_high_resolution = 0x00,

};

#define MAX_SAMPLE_POINT    64
static uint16_t adc_debug[6][MAX_SAMPLE_POINT];
static uint8_t channel_done_flag = 0;
/***************************************************************************
* EVENTS
*/

/*********************************************************************
 * FUNCTIONS
 */
/*********************************************************************
adc iqr callbak
*********************************************************************/
__ATTR_SECTION_SRAM__ static void adc_evt(adc_Evt_t* pev)
{
    double value = 0;
    int i = 0;
    bool is_high_resolution = FALSE;
    bool is_differential_mode = FALSE;
    uint8_t ch = 0;

    if((pev->type != HAL_ADC_EVT_DATA) || (pev->ch < 2))
        return;

    osal_memcpy(adc_debug[pev->ch-2],pev->data,2*(pev->size));
    channel_done_flag |= BIT(pev->ch);

    if(channel_done_flag == adc_cfg.channel)
    {
        for(i=2; i<8; i++)
        {
            if(channel_done_flag & BIT(i))
            {
                is_high_resolution = (adc_cfg.is_high_resolution & BIT(i))?TRUE:FALSE;
                is_differential_mode = (adc_cfg.is_differential_mode & BIT(i))?TRUE:FALSE;
                value = hal_adc_value_cal((adc_CH_t)i,adc_debug[i-2], pev->size, is_high_resolution,is_differential_mode);

                switch(i)
                {
                case ADC_CH1N_P11:
                    ch=11;
                    break;

                case ADC_CH1P_P23:
                    ch=23;
                    break;

                case ADC_CH2N_P24:
                    ch=24;
                    break;

                case ADC_CH2P_P14:
                    ch=14;
										Battery_Voltage = value*1000;
                    break;

                case ADC_CH3N_P15:
                    ch=15;
                    break;

                case ADC_CH3P_P20:
                    ch=20;
                    break;

                default:
                    break;
                }

                if(ch!=0)
                {
                    LOG("P%d %d mv ",ch,(int)(value*1000));
                }
                else
                {
                    LOG("invalid channel\n");
                }
            }
        }
        channel_done_flag = 0;
    }
}


/*********************************************************************
battery adc init
*********************************************************************/
static void APP_Batery_Adc_Init( void )
{
    int ret;
    bool batt_mode = TRUE;
    uint8_t batt_ch = ADC_CH2P_P14;
    GPIO_Pin_e pin;
    LOG("adcMeasureTask INTERRUPT\n");

    if(FALSE == batt_mode)
    {
        ret = hal_adc_config_channel(adc_cfg, adc_evt);
    }
    else
    {
        if(((((1 << batt_ch) & adc_cfg.channel) == 0)) || (adc_cfg.is_differential_mode != 0x00))
        {
            return;
        }

        pin = s_pinmap[batt_ch];
        hal_gpio_cfg_analog_io(pin,Bit_DISABLE);
        hal_gpio_write(pin, 1);
        ret = hal_adc_config_channel(adc_cfg, adc_evt);
        hal_gpio_cfg_analog_io(pin,Bit_DISABLE);
    }

    if(ret)
    {
        LOG("ret = %d\n",ret);
        return;
    }

    hal_adc_start(INTERRUPT_MODE);
}

/*********************************************************************
gpio wake up callbak
*********************************************************************/
void gpio_int_wakeup_cb(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
		if(pin == P24)
		{
				LOG("P24\n");
		}
		else if(pin == P25)
		{
				LOG("P25\n");
		}
}

/*********************************************************************
GPIO init
*********************************************************************/
static void APP_GPIO_Init(void)
{
		uint8_t i = 0;
		gpio_pin_e button_list[2] = {P24,P25};
		
		for(i = 0; i < 2; i++)
		{
				hal_gpio_pin_init(button_list[i], GPIO_INPUT);
				hal_gpio_pull_set(button_list[i],GPIO_PULL_UP_S);

				hal_gpio_wakeup_set(button_list[i],POL_FALLING);

				hal_gpioin_register(button_list[i],gpio_int_wakeup_cb,gpio_int_wakeup_cb);
		}
}

/*********************************************************************
wake up callback
*********************************************************************/
static void user_wakeup_process(void)
{
		//LOG("wake up\n");
}

/*********************************************************************
enter sleep callback
*********************************************************************/
static void user_sleep_process(void)
{

}

/*********************************************************************
GPIO init
*********************************************************************/
void User_Hal_Init(void)
{
		APP_GPIO_Init();
	
		APP_Batery_Adc_Init();
		
		hal_pwrmgr_register(MOD_USR1,user_sleep_process,user_wakeup_process);
		hal_pwrmgr_lock(MOD_USR1);
}

