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
    @file     flash.h
    @brief    Contains all functions support for flash driver
    @version  0.0
    @date     27. Nov. 2017
    @author   qing.han



*******************************************************************************/
#ifndef _FLASH_H_
#define _FLASH_H_

#include "rom_sym_def.h"
#include "clock.h"
#include "types.h"
#include "gpio.h"
#include "version.h"

#define CHIP_MADDR_LEN          6
#define CHIP_ID_FLASH_ADDRESS           0x11000800
#define CHIP_MADDR_FLASH_ADDRESS        (CHIP_ID_FLASH_ADDRESS+CHIP_ID_LENGTH*4)
#define FLASH_PROTECT_AREA_5BIT_7C      0x7c// protected area: 0x7c -> ALL
#define FLASH_PROTECT_AREA_5BIT_5C      0x5c// protected area: 0x7c -> ALL
#define FLASH_PROTECT_AREA_3BIT_1C      0x1c// protected area: 0x1c -> ALL
#define FLASH_PROTECT_AREA_2BIT_0C      0x0c// protected area: 0x0c -> ALL

#define SR_SRP0_BIT (BIT(7))
#define SR_CMP_BIT  (BIT(6))
#define SR_QE_BIT   (BIT(1))
#define SET_QE_BIT  (BIT(9))

#ifndef FLASH_PROTECT_FEATURE
    #define FLASH_PROTECT_FEATURE   0
#endif

#define SPIF_TIMEOUT       (0x7ffffff)//1000000

#define SFLG_WIP    1
#define SFLG_WEL    2
#define SFLG_WELWIP 3

//define flash ucds
#define FLASH_BASE_ADDR         (0x11000000)
#define FLASH_UCDS_ADDR_BASE    0x11005000

#define CHIP_ID_LENGTH          64
#define CHIP_ID_PID_LEN         16
#define CHIP_ID_LID_LEN         10
#define CHIP_ID_MID_LEN         16
#define CHIP_ID_TID_LEN         14
#define CHIP_ID_SID_LEN         8

#define CHIP_MADDR_LEN          6

//xip flash read instrcution
#define XFRD_FCMD_READ          0x0000003
#define XFRD_FCMD_READ_DUAL     0x801003B
#define XFRD_FCMD_READ_QUAD     0x802006B


#define FCMD_RESET              0x99  //reset
#define FCMD_ENRST              0x66  //enable reset
#define FCMD_WREN               0x06  //write enable
#define FCMD_WRDIS              0x04  //write disable
#define FCMD_VSRWREN            0x50  //Volatile SR Write Enable


#define FCMD_CERASE             0x60  //(or 0xC7)chip erase
#define FCMD_SERASE             0x20  //sector erase
#define FCMD_BERASE32           0x52  //block erease 32k
#define FCMD_BERASE64           0xD8

#define FCMD_DPWRDN             0xB9  //deep power down
#define FCMD_RLSDPD             0xAB  //release from powerdown(and read device id)
#define FCMD_WRST               0x01  //write status
#define FCMD_RDID               0x9F  //read ID
#define FCMD_RDST               0x05  //read status
#define FCMD_RDST_H             0x35  //read status high byte
#define FCMD_PPROG              0x02  //page program
#define FCMD_READ               0x03  //read
#define FCMD_READF              0x0B  //fast read
#define FCMD_READDO             0x3B  //dual output fast read
#define FCMD_READDIO            0xBB  //dual I/O fast read
#define FCMD_READQO             0x6B  //quad output fast read
#define FCMD_READQIO            0xeB  //quad I/O fast read
#define FCMD_READQIOW           0xe7  //quad I/O fast read word

#if(FLASH_PROTECT_FEATURE == 1)
    #define XT25W02D_ID             0x12600B
    #define TH25D20UA_ID            0x1260EB
    #define XT25W04D_ID             0x13600B
    #define TH25D40HB_ID            0x1360CD
    #define UC25HQ40IAG_ID          0x1360B3
    #define P25D40_ID               0x136085
    #define GD25WD40CGIG_GJG_ID     0x1364C8
    #define TH25Q40UA_ID            0x1360EB
    #define GD25WD80CGIG_ID         0x1464C8
    #define P25D22U_ID              0x124485
    #define MD25D40DGIG_ID          0x134051
    #define P25Q16SU_ID             0x154285
    #define BY25Q40GW_ID            0x131068
    #define GT25Q40C_ID             0x1340C4
    #define TH25D40UB_ID            0x1260CD
    #define ZB25WD40B_ID            0x13325E
    #define GT25D20E_ID             0x1260C4
#endif
typedef struct
{
    uint32_t      rd_instr;
} xflash_Ctx_t;

typedef enum
{
    CHIP_ID_UNCHECK,
    CHIP_ID_EMPTY,
    CHIP_ID_VALID,
    CHIP_ID_INVALID,
} CHIP_ID_STATUS_e;

typedef struct
{
    CHIP_ID_STATUS_e chipMAddrStatus;
    uint8_t mAddr[CHIP_MADDR_LEN];
} chipMAddr_t;

typedef enum
{
    SLB_OTA                = 0x1,
    SINGLE_OTA             = 0x2,
    MAIN_INIT              = 0x3,
    FS_INIT                = 0x4,
    FS_FORMAT              = 0x5,
    FS_GARBAGE_COLLECT     = 0x6,
    FS_ITEM_DEL            = 0x7,
    FS_ITEM_WRITE          = 0x8,
    PS_STORE               = 0x9,
    USER_DEFINE_1          = 0xa,
    USER_DEFINE_2          = 0xb,
    USER_DEFINE_3          = 0xc,
    NONE                   = 0xff, //Only used in main initialization !!!
} module_ID_t;
typedef struct
{
    bool    bypass_flash_lock;
    module_ID_t module_ID;
} FLASH_PROTECT_INFO;

typedef void (*spif_wakeup_hdl)(void);

extern int _spif_wait_nobusy(uint8_t flg, uint32_t tout_ns);
extern int  spif_write(uint32_t addr, uint8_t* data, uint32_t size);
extern int  spif_write_dma(uint32_t addr, uint8_t* data, uint32_t size);
extern int  spif_read(uint32_t addr, uint8_t* data, uint32_t size);
extern int  spif_read_dma(uint32_t addr, uint8_t* data, uint32_t size);
extern int spif_erase_sector(unsigned int addr);
extern int spif_erase_block64(unsigned int addr);
extern int spif_erase_all(void);
extern uint8_t spif_flash_status_reg_0(void);
extern int spif_write_protect(bool en);
extern void spif_cmd(uint8_t op, uint8_t addrlen, uint8_t rdlen, uint8_t wrlen, uint8_t mbit, uint8_t dummy);
extern void spif_rddata(uint8_t* data, uint8_t len);
extern int spif_config(sysclk_t ref_clk, uint8_t div,  uint32_t rd_instr,  uint8_t mode_bit, uint8_t QE);
extern int  spif_read_id(uint32_t* pid);
extern void spif_wrdata(uint8_t* data, uint8_t len);
int hal_spif_cache_init(xflash_Ctx_t cfg);
void hal_cache_tag_flush(void);
#if(FLASH_PROTECT_FEATURE == 1)
    int hal_flash_enable_lock(module_ID_t id);
    int hal_flash_disable_lock(module_ID_t id);
    int hal_flash_write_status_register(uint16_t reg_data, bool high_en);
    int hal_flash_lock(void);
    int hal_flash_unlock(void);
    uint8_t hal_flash_get_lock_state(void);
    uint8_t hal_flash_get_high_lock_state(void);
    int hal_remove_fct_mode(uint32_t sramaddr);
#endif
int hal_flash_write(uint32_t addr, uint8_t* data, uint32_t size);
int hal_flash_write_by_dma(uint32_t addr, uint8_t* data, uint32_t size);
int hal_flash_read(uint32_t addr, uint8_t* data, uint32_t size);
int hal_flash_erase_sector(unsigned int addr);
int hal_flash_erase_block64(unsigned int addr);
int flash_write_word(unsigned int offset, uint32_t  value);

CHIP_ID_STATUS_e chip_id_one_bit_hot_convter(uint8_t* b,uint32_t w);

void LL_PLUS_LoadMACFromFlash(uint32_t addr);

CHIP_ID_STATUS_e LL_PLUS_LoadMACFromChipMAddr(void);


void check_chip_mAddr(void);
void LOG_CHIP_MADDR(void);


#endif








