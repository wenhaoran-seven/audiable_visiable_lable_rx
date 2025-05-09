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


#include <string.h>

#include "clock.h"
#include "pwrmgr.h"
#include "error.h"
#include "log.h"
#include "jump_function.h"

#include "dma.h"
#include "flash.h"
#include "uart.h"



dma_ctx_t s_dma_ctx =
{
    .init_flg = FALSE,
};

static DMA_CONN_e get_src_conn(uint32_t addr)
{
    if(addr == (uint32_t)&(AP_SPI0->DataReg))
        return DMA_CONN_SPI0_Rx;

    if(addr == (uint32_t)&(AP_SPI1->DataReg))
        return DMA_CONN_SPI1_Rx;

    if(addr == (uint32_t)&(AP_I2C0->IC_DATA_CMD))
        return DMA_CONN_I2C0_Rx;

    if(addr == (uint32_t)&(AP_I2C1->IC_DATA_CMD))
        return DMA_CONN_I2C1_Rx;

    if(addr == (uint32_t)&(AP_UART0->RBR))
        return DMA_CONN_UART0_Rx;

    if(addr == (uint32_t)&(AP_UART1->RBR))
        return DMA_CONN_UART1_Rx;

    return DMA_CONN_MEM;
}

static DMA_CONN_e get_dst_conn(uint32_t addr)
{
    if(addr == (uint32_t)&(AP_SPI0->DataReg))
        return DMA_CONN_SPI0_Tx;

    if(addr == (uint32_t)&(AP_SPI1->DataReg))
        return DMA_CONN_SPI1_Tx;

    if(addr == (uint32_t)&(AP_I2C0->IC_DATA_CMD))
        return DMA_CONN_I2C0_Tx;

    if(addr == (uint32_t)&(AP_I2C1->IC_DATA_CMD))
        return DMA_CONN_I2C1_Tx;

    if(addr == (uint32_t)&(AP_UART0->THR))
        return DMA_CONN_UART0_Tx;

    if(addr == (uint32_t)&(AP_UART1->THR))
        return DMA_CONN_UART1_Tx;

    return DMA_CONN_MEM;
}

static void dma_wakeup_handler(void)
{
    hal_clk_gate_enable(MOD_DMA);
    NVIC_SetPriority((IRQn_Type)DMAC_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)DMAC_IRQn);
    JUMP_FUNCTION(DMAC_IRQ_HANDLER)      =   (uint32_t)&hal_DMA_IRQHandler;
    AP_DMA_MISC->DmaCfgReg = DMA_DMAC_E;
}


int hal_dma_init_channel(HAL_DMA_t cfg)
{
    DMA_CH_Ctx_t* pctx;
    DMA_CH_t ch;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    ch = cfg.dma_channel;

    if(ch >= DMA_CH_NUM)
        return PPlus_ERR_INVALID_PARAM;

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(pctx ->init_ch)
        return PPlus_ERR_INVALID_STATE;

    pctx->evt_handler = cfg.evt_handler;
    pctx->init_ch = true;
    return PPlus_SUCCESS;
}


int hal_dma_config_channel(DMA_CH_t ch, DMA_CH_CFG_t* cfg)
{
    DMA_CH_Ctx_t* pctx;
    DMA_CONN_e src_conn,dst_conn;
    uint32_t cctrl = 0;
    uint32_t transf_type = DMA_TRANSFERTYPE_M2M;
    uint32_t transf_per = 0;
    uint32_t spif_protect = AP_SPIF->wr_protection;
    uint32_t cache_bypass = AP_PCR->CACHE_BYPASS;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    if(ch >= DMA_CH_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(!pctx->init_ch)
        return PPlus_ERR_INVALID_STATE;

    if ((AP_DMA_MISC->ChEnReg & (DMA_DMACEnbldChns_Ch(ch))) || \
            (pctx->xmit_busy))
    {
        // This channel is enabled, return ERROR, need to release this channel first
        return PPlus_ERR_BUSY;
    }

    // Reset the Interrupt status
    AP_DMA_INT->ClearTfr = DMA_DMACIntTfrClr_Ch(ch);
    // UnMask interrupt
    AP_DMA_INT->MaskTfr = DMA_DMACCxIntMask_E(ch);
    src_conn = get_src_conn(cfg->src_addr);
    dst_conn = get_dst_conn(cfg->dst_addr);

    /* Assign Linker List Item value */
    if(src_conn && dst_conn)
    {
        transf_type = DMA_TRANSFERTYPE_P2P;
        transf_per = DMA_DMACCxConfig_SrcPeripheral(src_conn-1)| \
                     DMA_DMACCxConfig_DestPeripheral(dst_conn-1);
    }
    else if(src_conn)
    {
        transf_type = DMA_TRANSFERTYPE_P2M;
        transf_per = DMA_DMACCxConfig_SrcPeripheral(src_conn-1);
    }
    else if(dst_conn)
    {
        transf_type = DMA_TRANSFERTYPE_M2P;
        transf_per = DMA_DMACCxConfig_DestPeripheral(dst_conn-1);
    }

    if((cfg->dst_addr > 0x11000000) && (cfg->dst_addr <= 0x11080000))
    {
        pctx->xmit_flash = DMA_DST_XIMT_IS_FLASH;

        if(spif_protect)
        {
            AP_SPIF->wr_protection = 0;
        }

        if(cache_bypass == 0)
        {
            AP_PCR->CACHE_BYPASS = 1;
        }
    }    
    else if((cfg->src_addr > 0x11000000) && (cfg->src_addr <= 0x11080000))
    {
        pctx->xmit_flash = DMA_DST_XIMT_IS_FLASH;
        if(cache_bypass == 0)
        {
            AP_PCR->CACHE_BYPASS = 1;
        }
    }
    else
    {
        pctx->xmit_flash = DMA_DST_XIMT_NOT_FLASH;
    }

    AP_DMA_CH_CFG(ch)->SAR = cfg->src_addr;
    AP_DMA_CH_CFG(ch)->DAR = cfg->dst_addr;
    AP_DMA_CH_CFG(ch)->LLP = 0;

    if(DMA_GET_MAX_TRANSPORT_SIZE(ch) < cfg->transf_size)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    AP_DMA_CH_CFG(ch)->CTL_H = DMA_DMACCxControl_TransferSize(cfg->transf_size);
    subWriteReg(&(AP_DMA_CH_CFG(ch)->CFG_H),15,7,transf_per);
    AP_DMA_CH_CFG(ch)->CFG = 0;
    cctrl = DMA_DMACCxConfig_TransferType(transf_type)| \
            DMA_DMACCxControl_SMSize(cfg->src_msize)| \
            DMA_DMACCxControl_DMSize(cfg->dst_msize)| \
            DMA_DMACCxControl_SWidth(cfg->src_tr_width)| \
            DMA_DMACCxControl_DWidth(cfg->dst_tr_width)| \
            DMA_DMACCxControl_SInc(cfg->sinc)| \
            DMA_DMACCxControl_DInc(cfg->dinc)| \
            DMA_DMAC_INT_E;
    AP_DMA_CH_CFG(ch)->CTL = cctrl;

    if(cfg->enable_int)
    {
        AP_DMA_INT->MaskTfr = DMA_DMACCxConfig_E(ch) | BIT(ch);
        pctx->interrupt = true;
    }
    else
    {
        AP_DMA_INT->ClearTfr = DMA_DMACIntTfrClr_Ch(ch);
        AP_DMA_INT->MaskTfr = DMA_DMACCxIntMask_E(ch);
        pctx->interrupt = false;
    }

    return PPlus_SUCCESS;
}

int hal_dma_start_channel(DMA_CH_t ch)
{
    DMA_CH_Ctx_t* pctx;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    pctx = &s_dma_ctx.dma_ch_ctx[ch];
    AP_DMA_MISC->ChEnReg = DMA_DMACCxConfig_E(ch) | BIT(ch);
    pctx->xmit_busy = TRUE;
    hal_pwrmgr_lock(MOD_DMA);
    return PPlus_SUCCESS;
}

int hal_dma_stop_channel(DMA_CH_t ch)
{
    uint32_t spif_protect = AP_SPIF->wr_protection;
    uint32_t cache_bypass = AP_PCR->CACHE_BYPASS;
    DMA_CH_Ctx_t* pctx;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    if(ch >= DMA_CH_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(pctx->xmit_flash == DMA_DST_XIMT_IS_FLASH)
    {
        if(spif_protect)
        {
            AP_SPIF->wr_protection = 2;
        }

        if(cache_bypass == 0)
        {
            AP_PCR->CACHE_BYPASS = 0;
            AP_CACHE->CTRL0 = 0x01;
        }
    }

    // Reset the Interrupt status
    AP_DMA_INT->ClearTfr = DMA_DMACIntTfrClr_Ch(ch);
    // UnMask interrupt
//    AP_DMA_INT->MaskTfr = DMA_DMACCxIntMask_E(ch);
    AP_DMA_MISC->ChEnReg = DMA_DMACCxConfig_E(ch);
    pctx->xmit_busy = FALSE;
    hal_pwrmgr_unlock(MOD_DMA);
    return PPlus_SUCCESS;
}

int hal_dma_status_control(DMA_CH_t ch)
{
    DMA_CH_Ctx_t* pctx;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    if(ch >= DMA_CH_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(pctx->interrupt == false)
        hal_dma_wait_channel_complete(ch);

    return PPlus_SUCCESS;
}

int hal_dma_wait_channel_complete(DMA_CH_t ch)
{
    uint32_t Temp = 0;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    while(1)
    {
        Temp ++;

        if(AP_DMA_INT->RawTfr & BIT(ch))
        {
            break;
        }
    }

    hal_dma_stop_channel(ch);
    // LOG("wait count is %d\n",Temp);
    return PPlus_SUCCESS;
}

int hal_rsp_dma_wait_channel_complete(DMA_CH_t ch,uint32 cnt)
{
    uint32_t Temp = 0;
    uint8 ret=PPlus_SUCCESS;
    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    while(1)
    {
        Temp ++;

        if(Temp>cnt)
        {
           ret= PPlus_ERR_FATAL;
           break;
        }

        if(AP_DMA_INT->RawTfr & BIT(ch))
        {
            break;
        }
    }

    hal_dma_stop_channel(ch);
    // LOG("wait count is %d\n",Temp);
    return ret;
}

int hal_dma_init(void)
{
    uint8_t ret;
    hal_clk_gate_enable(MOD_DMA);
    hal_clk_reset(MOD_DMA);
    NVIC_SetPriority((IRQn_Type)DMAC_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)DMAC_IRQn);
    JUMP_FUNCTION(DMAC_IRQ_HANDLER)      =   (uint32_t)&hal_DMA_IRQHandler;
    ret = hal_pwrmgr_register(MOD_DMA,NULL, dma_wakeup_handler);

    if(ret == PPlus_SUCCESS)
    {
        s_dma_ctx.init_flg = TRUE;
        memset(&(s_dma_ctx.dma_ch_ctx[0]), 0, sizeof(DMA_CH_Ctx_t)*DMA_CH_NUM);
        //dmac controller enable
        AP_DMA_MISC->DmaCfgReg = DMA_DMAC_E;
    }

    return ret;
}

int hal_dma_deinit(void)
{
    //dmac controller disable
    AP_DMA_MISC->DmaCfgReg = DMA_DMAC_D;
    s_dma_ctx.init_flg = FALSE;
    memset(&(s_dma_ctx.dma_ch_ctx[0]), 0, sizeof(DMA_CH_Ctx_t)*DMA_CH_NUM);
    hal_pwrmgr_unregister(MOD_DMA);
    hal_clk_gate_disable(MOD_DMA);
    return PPlus_SUCCESS;
}

void __attribute__((used)) hal_DMA_IRQHandler(void)
{
    DMA_CH_t ch;

    for(ch = DMA_CH_0; ch < DMA_CH_NUM; ch++)
    {
        if(AP_DMA_INT->StatusTfr & BIT(ch))
        {
            hal_dma_stop_channel(ch);

            if(s_dma_ctx.dma_ch_ctx[ch].evt_handler != NULL)
            {
                s_dma_ctx.dma_ch_ctx[ch].evt_handler(ch);
            }
        }
    }
}




