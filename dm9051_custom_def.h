#ifndef __DM9051_CUSTOM_DEF_H__
#define __DM9051_CUSTOM_DEF_H__

#include "dm9051.h"

//------------------------------------------------------------------------------
#define SPI_SYNC_TRANSFER_BUF_LEN               (4 + DM9051_PKT_MAX) //1024

#define EN_DEBUG

#ifdef EN_DEBUG
#define dbg_log(format, args...)                                                                                        \
                do{                                                                                                                                         \
                                printk(KERN_ERR DRV_NAME": %s() _%d_: " format                       \
                                                                , __FUNCTION__                                                                          \
                                                                , __LINE__                                                                                      \
                                                                , ## args);                                                                                     \
                }while(0)
#else
#define dbg_log(format, args...)     ((void)0)
#endif //EN_DEBUG

//------------------------------------------------------------------------------
struct rx_ctl_mach {
        u16 rxbyte_counter;
        u16 rxbyte_counter0;
};

struct tx_state_mach {
        u16 prob_cntStopped;
};

typedef struct board_info {
        struct rx_ctl_mach bC;
        struct tx_state_mach bt;
        struct spi_device *spidev;
        struct sk_buff_head txq;
        struct spi_message spi_msg1;
        struct spi_transfer spi_xfer1;

        struct mutex addr_lock;         /* dm9051's lock;*/ /* phy and eeprom access lock */
        struct net_device *ndev;        /* dm9051's netdev */

        u8 irq_type;
        u8 imr_all;
        u8 rcr_all;
        int link;
        int ip_summed;

        spinlock_t statelock; /* state lock;*/

        struct mii_if_info mii;
        u32 msg_enable ____cacheline_aligned;

        int in_power_off;

        u8 irq_now;
        //struct delayed_work irq_work;
        struct work_struct irq_work;
        struct work_struct tx_work;
        struct work_struct rxctrl_work;

        //u8 TxDatBuf[SPI_SYNC_TRANSFER_BUF_LEN];

        /* DMA buffer */
        u8 *spi_tx_buf;
        dma_addr_t spi_tx_dma;

        u8 *spi_rx_buf;
        dma_addr_t spi_rx_dma;
} board_info_t;

struct spi_rxhdr { //new
        struct dm9051_rxhdr {
                u8      RxPktReady;
                u8      RxStatus;
                __le16  RxLen;
        } rxhdr;
} __packed;

#endif /* __DM9051_CUSTOM_DEF_H__ */
