/*
 * Copyright 2014 Davicom Semiconductor,Inc.  && Tianyx(zzztyx55@sina.com)
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version 2
 *      of the License, or (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      http://www.davicom.com.tw
 *      2014/03/11  Joseph CHANG  v1.0  Create
 *      2014/05/05  Joseph CHANG  v1.01  Start from Basic functions
 *      2015/06/08  Joseph CHANG  v1.02  Formal naming (Poll version)
 *  Ver: Step1.3 dma: mt_ DMA.. (20151203)
 *  Ver: Step1.3p1 DMA3_PNs design for platforms' config (20151223)
 * Ver: [3p6s]
 * Ver: 3p6ss
 * Ver: 3p11sss (kmalloc re-placement)
 * Ver: V1.1
 * Ver: V1.2  Default as static 'dm9051.o'
 * Ver: V1.5b customization (2017/10/17)
 * Ver: V1.69.3 Support interrupt mode and mdio mutex_lock
 * Ver: V2.0  2018/06/19 tianyx (zzztyx55@sina.com)
 * Ver: V2.1  2019/08/28 Jerry (chongji_wang@davicom.com.tw)
 * Ver: V2.2  2019/09/17 Jerry (chongji_wang@davicom.com.tw)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_net.h>
#include <linux/mod_devicetable.h>
#include <linux/irq.h>

//#include <mach/gpio_const.h>
//#include <mt-plat/mt_gpio.h>
//#include "../../../spi/mediatek/mt6755/mt_spi.h" // relative path

#include "dm9051.h"
#include "dm9051_custom_def.h"

//------------------------------------------------------------------------------
#define DRV_NAME                                "dm9051"
#define DRV_VERSION                             "2.2"

#define NUM_QUEUE_TAIL                          0xFFFE   //(2) //(5)//(65534= 0xFFFE)MAX_QUEUE_TAIL
#define DM9051_PHY                              0x40    /* PHY address 0x01 */
#define DM_EEPROM_MAGIC                         (0x444D394B)
#define MAC_ADDR_LEN                            (6)
#define INVALID_DMA_ADDRESS                     0xffffffff

#define DM9051_MSG_DEFAULT                      (NETIF_MSG_PROBE |  \
                                                 NETIF_MSG_IFUP |   \
                                                 NETIF_MSG_IFDOWN | \
                                                 NETIF_MSG_LINK)

//------------------------------------------------------------------------------
#if 0
static struct pinctrl *dm9051_pinctrl1;

static struct pinctrl_state *dm9051_rst_high;
static struct pinctrl_state *dm9051_rst_low;
static struct pinctrl_state *dm9051_pwr_on;
static struct pinctrl_state *dm9051_pwr_off;
static struct pinctrl_state *spi_switch_on;
static struct pinctrl_state *spi_switch_off;
#endif //0

static char *macaddr = ":";
static int enable_dma = 0; /* Enable SPI DMA. Default: 0 (OFF) */
static int enable_eeprom = 0; /* Enable EEPROM. Default: 0 (OFF) */

/* use ethtool to change the level for any given device */
static struct {
        u32 msg_enable;
} debug = { -1 };

//------------------------------------------------------------------------------
static int dm9051_phy_read(struct net_device *dev, int phy_reg_unused, int reg);
static void dm9051_phy_write(struct net_device *dev, int phyaddr_unused, int reg, int value);

static int dm9051_open(struct net_device *dev);
static int dm9051_stop(struct net_device *dev);

static void dm9051_read_intcr(board_info_t *db);
static void dm9051_set_intcr(board_info_t *db);

static void dm9051_send_out_packet(board_info_t *db);

//------------------------------------------------------------------------------
static void bcopen_rx_info_clear(struct rx_ctl_mach *pbc)
{
        pbc->rxbyte_counter = 0;
        pbc->rxbyte_counter0 = 0;
}

#if 0
static int mt_dm9051_pinctrl_init(struct spi_device *pdev)
{
        int retval = 0;

        dbg_log("mt_dm9051_pinctrl_init \n");
        dm9051_pinctrl1 = devm_pinctrl_get(&pdev->dev);

        if (IS_ERR(dm9051_pinctrl1)) {
                dbg_log("Cannot find pinctrl! \n");
                retval = PTR_ERR(dm9051_pinctrl1);
                return 0;
        }
        dm9051_rst_low = pinctrl_lookup_state(dm9051_pinctrl1, "dm9051_rst0");
        if (IS_ERR(dm9051_rst_low)) {
                dbg_log("Cannot find dm9051_rst_low pinctrl! \n");
                return 0;
        }
        dm9051_rst_high = pinctrl_lookup_state(dm9051_pinctrl1, "dm9051_rst1");
        if (IS_ERR(dm9051_rst_high)) {
                dbg_log("Cannot find dm9051_rst_high pinctrl! \n");
                return 0;
        }

        spi_switch_off = pinctrl_lookup_state(dm9051_pinctrl1, "spi_switch_en0");
        if (IS_ERR(spi_switch_off)) {
                dbg_log("Cannot find en0 pinctrl! \n");
                return 0;
        }

        spi_switch_on = pinctrl_lookup_state(dm9051_pinctrl1, "spi_switch_en1");
        if (IS_ERR(spi_switch_on)) {
                dbg_log("Cannot find en1 pinctrl! \n");
                return 0;
        }

        dm9051_pwr_off = pinctrl_lookup_state(dm9051_pinctrl1, "dm9051_poweroff");
        if (IS_ERR(dm9051_pwr_off)) {
                dbg_log("Cannot find en_low pinctrl! \n");
                return 0;
        }
        dm9051_pwr_on = pinctrl_lookup_state(dm9051_pinctrl1, "dm9051_poweron");
        if (IS_ERR(dm9051_pwr_on)) {
                dbg_log("Cannot find en_high pinctrl! \n");
                return 0;
        }

        return retval;
}

static void dm9051_hw_pwr_on(void)
{
        pinctrl_select_state(dm9051_pinctrl1, spi_switch_on);
        pinctrl_select_state(dm9051_pinctrl1, dm9051_pwr_on);

        mdelay(2);
        pinctrl_select_state(dm9051_pinctrl1, dm9051_rst_low);
        mdelay(100);
        pinctrl_select_state(dm9051_pinctrl1, dm9051_rst_high);
        mdelay(150);
}
#endif //0

static int dm9051_spi_xfer_buf(struct spi_device *spi, unsigned len)
{
        board_info_t *db = spi_get_drvdata(spi);
        struct spi_transfer xfer = {
                .tx_buf = db->spi_tx_buf,
                .rx_buf = db->spi_tx_buf,
                .len = (len + 1),
                .cs_change = 0,
        };
        struct spi_message msg;
        int ret;

        spi_message_init(&msg);

        if (enable_dma) {
                xfer.tx_dma = db->spi_tx_dma;
                xfer.rx_dma = db->spi_rx_dma;
                msg.is_dma_mapped = 1;
        }

        spi_message_add_tail(&xfer, &msg);
        ret = spi_sync(spi, &msg);
        if (ret && netif_msg_drv(db)) {
                dbg_log("spi transfer failed: ret = %d\n", ret);
        }

        return ret;
}

static u8 dm9051_spi_read_reg(board_info_t *db, unsigned reg)
{
        db->spi_tx_buf[0] = (DM_SPI_RD | reg);

        dm9051_spi_xfer_buf(db->spidev, 1);

        return (db->spi_tx_buf[1] & 0xFF);
}

static void dm9051_spi_write_reg(board_info_t *db, unsigned reg, unsigned val)
{
        db->spi_tx_buf[0] = (DM_SPI_WR | reg);
        db->spi_tx_buf[1] = val;

        dm9051_spi_xfer_buf(db->spidev, 1);
}

/*
 *  set mac
 */
static void dm9051_write_mac_addr(board_info_t *db, const unsigned char *mac_src)
{
        int i = 0;

        for (i = 0; i < ETH_ALEN; i++) {
                dm9051_spi_write_reg(db, DM9051_PAR + i, db->ndev->dev_addr[i]);
        }

        dbg_log("Final get MAC address with (%s) : "
                "%02x:%02x:%02x:%02x:%02x:%02x\n",
                mac_src,
                dm9051_spi_read_reg(db, DM9051_PAR + 0),
                dm9051_spi_read_reg(db, DM9051_PAR + 1),
                dm9051_spi_read_reg(db, DM9051_PAR + 2),
                dm9051_spi_read_reg(db, DM9051_PAR + 3),
                dm9051_spi_read_reg(db, DM9051_PAR + 4),
                dm9051_spi_read_reg(db, DM9051_PAR + 5)
               );
}

/*
 *  Set DM9051 multicast address
 */
static void dm9051_hash_table_unlocked(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);
        struct netdev_hw_addr *ha;
        int i, oft;
        u32 hash_val;
        u16 hash_table[4];
        u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;

        for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++) {
                dm9051_spi_write_reg(db, oft, dev->dev_addr[i]);
        }

        /* Clear Hash Table */
        for (i = 0; i < 4; i++) {
                hash_table[i] = 0x0;
        }

        /* broadcast address */
        hash_table[3] = 0x8000;

        if (dev->flags & IFF_PROMISC) {
                rcr |= RCR_PRMSC;
        }

        if (dev->flags & IFF_ALLMULTI) {
                rcr |= RCR_ALL;
        }

        /* the multicast address in Hash Table : 64 bits */
        netdev_for_each_mc_addr(ha, dev) {
                hash_val = ether_crc_le(6, ha->addr) & 0x3f;
                hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
        }

        /* Write the hash table */
        for (i = 0, oft = DM9051_MAR; i < 4; i++) {
                dm9051_spi_write_reg(db, oft++, hash_table[i]);
                dm9051_spi_write_reg(db, oft++, hash_table[i] >> 8);
        }

        dm9051_spi_write_reg(db, DM9051_RCR, rcr);

        db->rcr_all = rcr;
}

static void dm9051_hash_table(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);

        mutex_lock(&db->addr_lock);

        dm9051_hash_table_unlocked(dev);

        mutex_unlock(&db->addr_lock);
}

//------------------------------------------------------------------------------
/*
    read eeprom
 */
static void dm9051_read_eeprom(board_info_t *db, int offset, u8 *to)
{
        mutex_lock(&db->addr_lock);

        dm9051_spi_write_reg(db, DM9051_EPAR, offset);
        dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_ERPRR);

        while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

        dm9051_spi_write_reg(db, DM9051_EPCR, 0x0);

        to[0] = dm9051_spi_read_reg(db, DM9051_EPDRL);
        to[1] = dm9051_spi_read_reg(db, DM9051_EPDRH);

        mutex_unlock(&db->addr_lock);
}

/*
 * Write a word data to SROM
 */
static void dm9051_write_eeprom(board_info_t *db, int offset, u8 *data)
{
        mutex_lock(&db->addr_lock);

        dm9051_spi_write_reg(db, DM9051_EPAR, offset);
        dm9051_spi_write_reg(db, DM9051_EPDRH, data[1]);
        dm9051_spi_write_reg(db, DM9051_EPDRL, data[0]);
        dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);

        while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

        dm9051_spi_write_reg(db, DM9051_EPCR, 0);

        mutex_unlock(&db->addr_lock);
}

//------------------------------------------------------------------------------
/*
    read phy
 */
static int dm9051_phy_read_lock(struct net_device *dev, int phy_reg_unused, int reg)
{
        int val;
        board_info_t *db = netdev_priv(dev);

        mutex_lock(&db->addr_lock);

        val = dm9051_phy_read(dev, 0, reg);

        mutex_unlock(&db->addr_lock);

        return val;
}

static void dm9051_phy_write_lock(struct net_device *dev, int phyaddr_unused, int reg, int value)
{
        board_info_t *db = netdev_priv(dev);

        mutex_lock(&db->addr_lock);

        dm9051_phy_write(dev, 0, reg, value);

        mutex_unlock(&db->addr_lock);
}

static int dm9051_phy_read(struct net_device *dev, int phy_reg_unused, int reg)
{
        board_info_t *db = netdev_priv(dev);
        int ret;

        /* Fill the phyxcer register into REG_0C */
        dm9051_spi_write_reg(db, DM9051_EPAR, DM9051_PHY | reg);
        dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);  /* Issue phyxcer read command */

        while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE);

        dm9051_spi_write_reg(db, DM9051_EPCR, 0x0);     /* Clear phyxcer read command */
        /* The read data keeps on REG_0D & REG_0E */
        ret = (dm9051_spi_read_reg(db, DM9051_EPDRH) << 8) | dm9051_spi_read_reg(db, DM9051_EPDRL);

        return ret;
}

static void dm9051_phy_write(struct net_device *dev, int phyaddr_unused, int reg, int value)
{
        board_info_t *db = netdev_priv(dev);

        //dbg_log("dm9051_spi_write_regPHY[%02d %04x]\n", reg, value);
        /* Fill the phyxcer register into REG_0C */
        dm9051_spi_write_reg(db, DM9051_EPAR, DM9051_PHY | reg);

        /* Fill the written data into REG_0D & REG_0E */
        dm9051_spi_write_reg(db, DM9051_EPDRL, value);
        dm9051_spi_write_reg(db, DM9051_EPDRH, value >> 8);
        dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);  /* Issue phyxcer write command */

        while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

        dm9051_spi_write_reg(db, DM9051_EPCR, 0x0);     /* Clear phyxcer write command */
}

//------------------------------------------------------------------------------
static inline board_info_t *to_dm9051_board(struct net_device *dev)
{
        return netdev_priv(dev);
}

static void dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
        strcpy(info->driver, CARDNAME_9051);
        strcpy(info->version, DRV_VERSION);
        strlcpy(info->bus_info, dev_name(dev->dev.parent), sizeof(info->bus_info));
}

static void dm9051_set_msglevel(struct net_device *dev, u32 value)
{
        board_info_t *dm = to_dm9051_board(dev);
        dm->msg_enable = value;
}

static u32 dm9051_get_msglevel(struct net_device *dev)
{
        board_info_t *dm = to_dm9051_board(dev);

        return dm->msg_enable;
}

static int dm9051_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
        board_info_t *dm = to_dm9051_board(dev);

        mii_ethtool_gset(&dm->mii, cmd);

        return 0;
}

static int dm9051_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
        board_info_t *dm = to_dm9051_board(dev);

        return mii_ethtool_sset(&dm->mii, cmd);
}

static int dm9051_nway_reset(struct net_device *dev)
{
        board_info_t *dm = to_dm9051_board(dev);

        return mii_nway_restart(&dm->mii);
}

static u32 dm9051_get_link(struct net_device *dev)
{
        board_info_t *db = to_dm9051_board(dev);
        /*
           u8 nsr;

           mutex_lock(&db->addr_lock);
           nsr= dm9051_spi_read_reg(db, DM9051_NSR);
           mutex_unlock(&db->addr_lock);
           db->link= !!(nsr & 0x40); //& NSR_LINKST
           if (netif_carrier_ok(dev) != db->link) {
           if (db->link)
           netif_carrier_on(dev);
           else
           netif_carrier_off(dev);
           dbg_log("Link Status is: %d\n", db->link);
           }
         */
        return (u32)db->link;
}

static int dm9051_get_eeprom_len(struct net_device *dev)
{
        return 128;
}

static int dm9051_get_eeprom(struct net_device *dev, struct ethtool_eeprom *ee, u8 *data)
{
        board_info_t *dm = to_dm9051_board(dev);
        int offset = ee->offset;
        int len = ee->len;
        int i;

        /* EEPROM access is aligned to two bytes */
        if ((len & 1) != 0 || (offset & 1) != 0) {
                return -EINVAL;
        }

        ee->magic = DM_EEPROM_MAGIC;

        for (i = 0; i < len; i += 2) {
                dm9051_read_eeprom(dm, (offset + i) / 2, data + i);
        }

        return 0;
}

static int dm9051_set_eeprom(struct net_device *dev, struct ethtool_eeprom *ee, u8 *data)
{
        board_info_t *dm = to_dm9051_board(dev);
        int offset = ee->offset;
        int len = ee->len;
        int i;

        /* EEPROM access is aligned to two bytes */
        if (((len & 1) != 0) || ((offset & 1) != 0)) {
                return -EINVAL;
        }

        if (ee->magic != DM_EEPROM_MAGIC) {
                return -EINVAL;
        }

        for (i = 0; i < len; i += 2) {
                dm9051_write_eeprom(dm, (offset + i) / 2, data + i);
        }

        return 0;
}

static const struct ethtool_ops dm9051_ethtool_ops = {
        .get_drvinfo    = dm9051_get_drvinfo,
        .get_settings   = dm9051_get_settings,
        .set_settings   = dm9051_set_settings,
        .get_msglevel   = dm9051_get_msglevel,
        .set_msglevel   = dm9051_set_msglevel,
        .nway_reset     = dm9051_nway_reset,
        .get_link       = dm9051_get_link,
        /*TBD
        .get_wol        = dm9051_get_wol,
        .set_wol        = dm9051_set_wol,
        */
        .get_eeprom_len = dm9051_get_eeprom_len,
        .get_eeprom     = dm9051_get_eeprom,
        .set_eeprom     = dm9051_set_eeprom,
};

static void dm9051_soft_reset(board_info_t *db)
{
        mdelay(2); // delay 2 ms any need before NCR_RST (20170510)
        dm9051_spi_write_reg(db, DM9051_NCR, NCR_RST);
        mdelay(1);
        dm9051_spi_write_reg(db, 0x5e, 0x80);  // 5eH reg is not exist, why set?  datasheet is not all info???
        mdelay(1);

        //set dm9051 interrupt pin type
        dm9051_set_intcr(db);
}

static void dm9051_chip_reset(board_info_t *db)
{
        //dbg_log("++\n");
        dm9051_soft_reset(db);

        dm9051_spi_write_reg(db, DM9051_FCR, FCR_FLOW_ENABLE);  /* Flow Control */
        dm9051_spi_write_reg(db, DM9051_PPCR, PPCR_SETTING);    /* Fully Pause Packet Count */

        if (db->irq_now == 0) {
                db->imr_all = IMR_ALL;
                dm9051_spi_write_reg(db, DM9051_IMR, db->imr_all);
        }
        dm9051_spi_write_reg(db, DM9051_RCR, db->rcr_all);

        bcopen_rx_info_clear(&db->bC);
}

static void dm9051_fifo_ERRO(board_info_t *db)
{
        if ((db->bC.rxbyte_counter == MAX_READYBIT_ZERO_COUNT) ||
                        (db->bC.rxbyte_counter0 == (NUMRXBYTECOUNTER - 1))) {
                dbg_log("dm9051_chip_reset\n");
                dm9051_chip_reset(db);
        }
}

/*
 * Initialize dm9051 board
 */
static void dm9051_init_dm9051(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);
        int phy4;

        dm9051_spi_write_reg(db, DM9051_GPCR, GPCR_GEP_CNTL); /* Let GPIO0 output */

        /* DBG_20140407 */
        phy4 = dm9051_phy_read(dev, 0, MII_ADVERTISE);
        dm9051_phy_write(dev, 0, MII_ADVERTISE, phy4 | ADVERTISE_PAUSE_CAP); /* dm95 flow-control RX! */
        dm9051_phy_read(dev, 0, MII_ADVERTISE);

        /* Program operating register */
        dm9051_spi_write_reg(db, DM9051_TCR, 0);        /* TX Polling clear */
        dm9051_spi_write_reg(db, DM9051_BPTR, 0x3f);    /* Less 3Kb, 200us */
        dm9051_spi_write_reg(db, DM9051_SMCR, 0);       /* Special Mode */

        /* clear TX status */
        dm9051_spi_write_reg(db, DM9051_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);
        dm9051_spi_write_reg(db, DM9051_ISR, ISR_CLR_STATUS); /* Clear interrupt status */

        //db->tx_mem_size = dm9051_spi_read_reg(db, DM9051_TMSR);
        //db->tx_mem_size *= 1024;
        //dbg_log("tx memory size = %d\r\n", db->tx_mem_size);

        /* Init Driver variable */
        db->imr_all = IMR_ALL;
        db->rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;

        dbg_log("dm9051_chip_reset\n");
        dm9051_chip_reset(db);
}

static void dm9051_set_multicast_list_schedule(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);

        schedule_work(&db->rxctrl_work);
}

static void dm9051_hash_table_work(struct work_struct *work)
{
        board_info_t *db = container_of(work, board_info_t, rxctrl_work);
        struct net_device *dev = db->ndev;

        //dbg_log("dm95 [ndo_set_rx_mode ].s\n");
        dm9051_hash_table(dev);
}

static bool dm9051_chk_data_volid(board_info_t *db, u8 *rdptr)
{
        struct net_device *dev = db->ndev;

        if ((rdptr[0] != dev->dev_addr[0]) ||
                        (rdptr[1] != dev->dev_addr[1]) ||
                        (rdptr[2] != dev->dev_addr[2])
           ) {
                if (rdptr[0] & 1) { //'skb->data[0]'
                        return true;
                } else {
                        dbg_log("check not match, ####### %x %x %x\n", rdptr[0], rdptr[1], rdptr[2]);
                        dbg_log("net mac: %x %x %x\n", dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2]);
                        dm9051_chip_reset(db);

                        return false;
                }
        }

        return true;
}

static int dm9051_write_tx_buf(board_info_t *db, u8 *buff, unsigned len)
{
        unsigned remain_len = len;
        unsigned pkg_len, offset = 0;

        if (len > DM9051_PKT_MAX) {
                dbg_log("warning: send buffer overflow!!!\n");
                return -1;
        }

        do {
                // 1 byte for cmd
                if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
                        pkg_len = remain_len;
                        remain_len = 0;
                } else {
                        pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                        remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                }

                db->spi_tx_buf[0] = (DM_SPI_WR | DM_SPI_MWCMD);

                memcpy(&db->spi_tx_buf[1], buff + offset, pkg_len);
                offset += pkg_len;

                dm9051_spi_xfer_buf(db->spidev, pkg_len);
        } while (remain_len > 0);

        return 0;
}

static void dm9051_read_rx_buf(board_info_t *db, u8 *buff, unsigned len, bool need_read)
{
        unsigned one_pkg_len;
        unsigned remain_len = len, offset = 0;

        if (need_read) { // need data but no buff, return
                if (!buff) {
                        dbg_log("rx sk_buff fail\r\n");
                        return;
                }
        }

        if (len <= 0) {
                dbg_log("rx length fail\r\n");
                return ;
        }

        do {
                // 1 byte for cmd
                if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
                        one_pkg_len = remain_len;
                        remain_len = 0;
                } else {
                        one_pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                        remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                }

                db->spi_tx_buf[0] = (DM_SPI_RD | DM_SPI_MRCMD);

                dm9051_spi_xfer_buf(db->spidev, one_pkg_len);
                if (need_read) {
                        memcpy(buff + offset, &db->spi_tx_buf[1], one_pkg_len);
                        offset += one_pkg_len;
                }
        } while (remain_len > 0);
}

/* routines for packing to use read/write blk */
static void dm9051_rd_rxhdr(board_info_t *db, u8 *buff, unsigned len)
{
        //dbg_log("len=%d\n", len);
        //len is 4
#if 1
        int i;
        for (i = 0; i < len; i++) {
                *buff++ = dm9051_spi_read_reg(db, DM_SPI_MRCMD);
        }
#else
        buff[0] = dm9051_spi_read_reg(db, DM_SPI_MRCMD);
        buff[1] = dm9051_spi_read_reg(db, DM_SPI_MRCMD);
        buff[2] = dm9051_spi_read_reg(db, DM_SPI_MRCMD);
        buff[3] = dm9051_spi_read_reg(db, DM_SPI_MRCMD);
#endif //0
}

/*
 *  Received a packet and pass to upper layer
 */
static void dm9051_rx(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);
        struct dm9051_rxhdr rxhdr = {0};
        struct sk_buff *skb;
        u8 rxbyte = 0;
        u8 *rdptr = NULL;
        u8 isr_reg = 0;
        //bool GoodPacket;
        u16 RxLen = 0;
        u16 calc_MRR = 0;

        /* Get most updated data */
        rxbyte = dm9051_spi_read_reg(db, DM_SPI_MRCMDX); /* Dummy read */
        rxbyte = dm9051_spi_read_reg(db, DM_SPI_MRCMDX); /* Dummy read */

        do {
                if (rxbyte != DM9051_PKT_RDY) {
                        dbg_log("rxbyte error = %x\r\n", rxbyte);
                        if ( rxbyte == 0x00 ) {
                                db->bC.rxbyte_counter0++;
                        } else {
                                db->bC.rxbyte_counter++;
                        }

                        dm9051_fifo_ERRO(db);
                        return;
                } /* Status check: this byte must be 0xff, 0 or 1 */

                bcopen_rx_info_clear(&db->bC);

                //GoodPacket = true;

#ifdef FifoPointCheck
                calc_MRR = (dm9051_spi_read_reg(db, DM9051_MRRH) << 8) +
                           dm9051_spi_read_reg(db, DM9051_MRRL);  //Save RX SRAM pointer
#endif //FifoPointCheck

                dm9051_spi_read_reg(db, DM_SPI_MRCMDX); /* Dummy read */
                dm9051_rd_rxhdr(db, (u8 *)&rxhdr, sizeof(rxhdr));

                RxLen = le16_to_cpu(rxhdr.RxLen);

                /*
                 * [LARGE THAN 1536 or less than 64]!"
                 */
                if (((RxLen & 0xFFFF) > DM9051_PKT_MAX) || ((RxLen & 0xFFFF) < 0x40)) {
                        dbg_log("RX length = %d\r\n", (RxLen & 0xFFFF));
                        dm9051_chip_reset(db);
                        return;
                }

                if (rxhdr.RxStatus & (RSR_FOE | RSR_CE | RSR_AE | RSR_RWTO | RSR_LCS | RSR_RF)) {
                        dbg_log("RX statue error = %x\r\n", rxhdr.RxStatus);

                        if (rxhdr.RxStatus & RSR_FOE) {
                                dev->stats.rx_fifo_errors++;
                        }

                        if (rxhdr.RxStatus & RSR_CE) {
                                dev->stats.rx_crc_errors++;
                        }

                        if (rxhdr.RxStatus & RSR_RF) {
                                dev->stats.rx_length_errors++;
                        }

                        dm9051_chip_reset(db);
                        return;
                }

                /* alloc a packet size space */
                if ((skb = dev_alloc_skb((RxLen & 0xFFFF) + 4)) == NULL)  {
                        dbg_log("alloc packet space fail\r\n");
                        dm9051_read_rx_buf(db, NULL, (RxLen & 0xFFFF), false);  // clear dm9051 buffer
                        return;
                }

                /*
                 *  We note that "#define NET_IP_ALIGN  2"
                 *
                 *      Move data from DM9051
                 *  (Linux skb->len IS LESS than 4, because it = (RxLen & 0xFFFF) - 4)
                 *
                 * Increase the headroom of an empty &skb_buff by
                 * reducing the tail room. Only allowed for an empty buffer.
                 */
                skb_reserve(skb, 2);

                /* A pointer to the first byte is returned */
                rdptr = (u8 *) skb_put(skb, (RxLen & 0xFFFF) - 4);

                /* Read received packet from RX SRAM */
                dm9051_read_rx_buf(db, rdptr, (RxLen & 0xFFFF), true);
                if (!dm9051_chk_data_volid(db, rdptr)) {
                        dbg_log("check rx data fail\r\n");
                        return;
                }

#ifdef FifoPointCheck
                calc_MRR += (RxLen + 4);
                if (calc_MRR > 0x3fff) {
                        calc_MRR -= 0x3400;
                }

                if (calc_MRR != ((dm9051_spi_read_reg(db, DM9051_MRRH) << 8) +
                                 dm9051_spi_read_reg(db, DM9051_MRRL))
                   ) {
                        dbg_log("RX memory pointer error!!\r\n");
                        dbg_log("predict RX read pointer = 0x%X, current pointer = 0x%X\r\n",
                                calc_MRR,
                                ((dm9051_spi_read_reg(db, DM9051_MRRH) << 8) +
                                 dm9051_spi_read_reg(db, DM9051_MRRL)));

                        dm9051_spi_write_reg(db, DM9051_MRRH, (calc_MRR >> 8) & 0xff);
                        dm9051_spi_write_reg(db, DM9051_MRRL, calc_MRR & 0xff);
                }
#endif //FifoPointCheck

                //dbg_log("pass 1 packet to upper\n");

                /* Pass to upper layer */
                skb->protocol = eth_type_trans(skb, dev);
                if (dev->features & NETIF_F_RXCSUM) {
                        if ((((rxbyte & 0x1c) << 3) & rxbyte) == 0) {
                                skb->ip_summed = CHECKSUM_UNNECESSARY;
                        } else {
                                skb_checksum_none_assert(skb);
                        }
                }

                if (in_interrupt()) {
                        netif_rx(skb);
                } else {
                        netif_rx_ni(skb);
                }

                dev->stats.rx_bytes += (RxLen & 0xFFFF);
                dev->stats.rx_packets++;

                /* Get most updated data */
                rxbyte = dm9051_spi_read_reg(db, DM_SPI_MRCMDX); /* Dummy read */
                rxbyte = dm9051_spi_read_reg(db, DM_SPI_MRCMDX); /* Dummy read */ //rxbyte = 0x01; //. readb(db->io_data);

                if (rxbyte == DM9051_PKT_RDY) {
                        // read interrupt status reg
                        isr_reg = dm9051_spi_read_reg(db, DM9051_ISR);
                        dm9051_spi_write_reg(db, DM9051_ISR, isr_reg);  // Clear ISR status ---> can be clear ???
                }
        } while (rxbyte == DM9051_PKT_RDY); // CONSTRAIN-TO: (rxbyte != XX)
}

static void dm9051_send_out_packet(board_info_t *db)
{
        struct net_device *dev = db->ndev;
        struct sk_buff *tx_skb;
        u16 calc_MWR = 0;
        u8 err_count = 0;

        while (!skb_queue_empty(&db->txq)) { // JJ-20140225, When '!empty'
                tx_skb = skb_dequeue(&db->txq);

                // wait for sending complete
                while (dm9051_spi_read_reg(db, DM9051_TCR) & TCR_TXREQ);

#ifdef FifoPointCheck
                calc_MWR = 0;
                calc_MWR = (dm9051_spi_read_reg(db, DM9051_MWRH) << 8) +
                           dm9051_spi_read_reg(db, DM9051_MWRL);
                calc_MWR += tx_skb->len;
                if (calc_MWR > 0x0bff) {
                        calc_MWR -= 0x0c00;
                }
#endif //FifoPointCheck

                if (tx_skb != NULL) {
                        dm9051_spi_write_reg(db, DM9051_TXPLL, tx_skb->len);
                        dm9051_spi_write_reg(db, DM9051_TXPLH, tx_skb->len >> 8);

                        dm9051_write_tx_buf(db, tx_skb->data, tx_skb->len);

                        // start send cmd
                        dm9051_spi_write_reg(db, DM9051_TCR, TCR_TXREQ);

                        dev->stats.tx_bytes += tx_skb->len;
                        dev->stats.tx_packets++;

                        /* done TX */
                        dev_kfree_skb(tx_skb);

#ifdef FifoPointCheck
                        if (calc_MWR != ((dm9051_spi_read_reg(db, DM9051_MWRH) << 8) +
                                         dm9051_spi_read_reg(db, DM9051_MWRL))
                           ) {
                                err_count++;
                                if (err_count >= 5) {
                                        dm9051_chip_reset(db);
                                } else {
                                        dbg_log("tx memory pointer error, calc_MWR = 0x%X,"
                                                " tx_skb->len = 0x%x\n",
                                                calc_MWR, tx_skb->len);
                                        dbg_log("MWRH = 0x%X, MWRL = 0x%X\n",
                                                dm9051_spi_read_reg(db, DM9051_MWRH),
                                                dm9051_spi_read_reg(db, DM9051_MWRL));
                                        dm9051_spi_write_reg(db, DM9051_MWRH, (calc_MWR >> 8) & 0xff);
                                        dm9051_spi_write_reg(db, DM9051_MWRL, calc_MWR & 0xff);
                                }
                        }
#endif //FifoPointCheck
                } //if
        } //while
}

/*
    sNot used interrupt-related function:
    This function make it less efficient for performance
    Since the ISR not RX direct, but use a schedule-work
    So that polling is better in using its poll schedule-work
 */
static irqreturn_t dm951_irq(int irq, void *pw)
{
        board_info_t *db = pw;

        disable_irq_nosync(irq);
        //schedule_delayed_work(&db->irq_work, 0);
        schedule_work(&db->irq_work);

        return IRQ_HANDLED;
}

static void dm9051_tx(board_info_t *db)
{
        struct net_device *dev = db->ndev;

        //dbg_log("++\n");

        mutex_lock(&db->addr_lock);

        dm9051_send_out_packet(db);

        netif_wake_queue(dev);

        mutex_unlock(&db->addr_lock);
}

static void dm9051_irq_work_handler(struct work_struct *work)
{
        //struct delayed_work *dw = to_delayed_work(work);
        //board_info_t *db = container_of(dw, board_info_t, irq_work);
        board_info_t *db = container_of(work, board_info_t, irq_work);
        struct net_device *dev = db->ndev;
        u8 isr_reg = 0;
        u8 ncr = 0;
        u8 nsr = 0;
        u8 imr = 0;
        char *speed = NULL;
        char *duplex = NULL;

        db->irq_now = 1; //trigger interrupt flag for dm9051_chip_reset() check.

        mutex_lock(&db->addr_lock);

        dm9051_spi_write_reg(db, DM9051_IMR, IMR_PAR); // Disable all interrupts

        // read interrupt status reg
        isr_reg = dm9051_spi_read_reg(db, DM9051_ISR);
        dm9051_spi_write_reg(db, DM9051_ISR, isr_reg);  // Clear ISR status ---> can be clear ???
        //dbg_log("isr_reg=0x%x\n", isr_reg);

        /*********** link status check*************/
        if (isr_reg & ISR_LNKCHGS) {
                nsr = dm9051_spi_read_reg(db, DM9051_NSR);
                db->link = !!(nsr & 0x40); //& NSR_LINKST
                if (db->link) {
                        netif_carrier_on(dev);
                } else {
                        netif_carrier_off(dev);
                }
                dbg_log("Link Status is: %d\n", db->link);

                if (db->link == 1) {
                        ncr = dm9051_spi_read_reg(db, DM9051_NCR);
                        speed = (nsr & 0x80) ? "10M" : "100M";
                        duplex = (ncr & 0x08) ? "Full" : "Half";
                        dbg_log("Link up %s %s\r\n", speed, duplex);
                }
        }

        // Receive Overflow Counter Overflow and Receive Overflow
        if (isr_reg & (ISR_ROOS | ISR_ROS)) {
                dbg_log("RX overflow reset\r\n");
                dm9051_chip_reset(db);
        }

        if (isr_reg & ISR_PRS) {
                dm9051_rx(dev);
        }

        db->irq_now = 0;

        //Re-enable interrupt mask
        db->imr_all = IMR_ALL;
        dm9051_spi_write_reg(db, DM9051_IMR, db->imr_all);
        if ((imr = dm9051_spi_read_reg(db, DM9051_IMR)) != IMR_ALL) {
                dbg_log("imr = 0x%x\r\n", imr);
                dm9051_spi_write_reg(db, DM9051_IMR, IMR_ALL);
        }

        mutex_unlock(&db->addr_lock);

        /*************  send packets  ***********/
        if (db->bt.prob_cntStopped) {
                //dbg_log("cntStopped = %d\r\n", db->bt.prob_cntStopped);
                dm9051_tx(db); // tx.ing in_RX
                db->bt.prob_cntStopped = 0;
        }

        enable_irq(db->ndev->irq);
}

static void dm9051_tx_work(struct work_struct *work)
{
        board_info_t *db = container_of(work, board_info_t, tx_work);

        if (db->bt.prob_cntStopped) { // This is more exactly right!!
                dm9051_tx(db);
                db->bt.prob_cntStopped = 0;
        }
}

static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);
        netdev_tx_t ret = NETDEV_TX_OK;

        spin_lock(&db->statelock);

        db->bt.prob_cntStopped++;
        if (db->bt.prob_cntStopped == NUM_QUEUE_TAIL) {
                netif_stop_queue(dev);
                ret = NETDEV_TX_BUSY;
        } else {
                skb_queue_tail(&db->txq, skb);
        }

        spin_unlock(&db->statelock);

        schedule_work(&db->tx_work);

        return ret;
}

static void dm9051_read_intcr(board_info_t *db)
{
        unsigned  inctr;
        unsigned char *int_pol = "active high";

        inctr = dm9051_spi_read_reg(db, DM9051_INTCR);
        if (inctr & 0x01) {
                int_pol = "active low";
        }

        dbg_log("DM9051 interrupt PIN %s\r\n", int_pol);
}

static void dm9051_set_intcr(board_info_t *db)
{

        dbg_log("cpu interrupts irq type = %x\r\n", db->irq_type);
        //set dm9051 interrupt gpio type
        if (db->irq_type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_EDGE_RISING)) {
                dm9051_spi_write_reg(db, DM9051_INTCR, 0x00); //high active(default)
        } else {
                dm9051_spi_write_reg(db, DM9051_INTCR, 0x01); //low active
        }

        //get dm9051 interrupt gpio type
        dm9051_read_intcr(db);
}

static u8 dm9051_is_irq_type(struct device_node *irq_node)
{
        u8 val;
        int rc;
        u32 value;

        val = IRQ_TYPE_NONE;

        if (irq_node) {
                rc = of_property_read_u32_index(irq_node, "interrupts", 1, &value);
                if (!rc) {
                        dbg_log("%s interrupts irq-type = %d\n", irq_node->full_name, value);
                        val = (int) value;
                }
        } else {
                dbg_log("interrupts irq-type <= %d (default)>\n", val);
        }

        if (val) {
                if (val & IRQ_TYPE_EDGE_RISING) {
                        dbg_log("interrupts IRQ Trigger Type = %d (EDGE RISING)\n", val);
                } else if (val & IRQ_TYPE_EDGE_FALLING) {
                        dbg_log("interrupts IRQ Trigger Type = %d (EDGE FALLING)\n", val);
                } else if (val & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_EDGE_RISING)) {
                        dbg_log("interrupts IRQ Trigger Type = %d (EDGE BOTH)\n", val);
                } else if (val & IRQ_TYPE_LEVEL_HIGH) {
                        dbg_log("interrupts IRQ Trigger Type = %d (LEVEL HIGH)\n", val);
                } else if (val & IRQ_TYPE_LEVEL_LOW) {
                        dbg_log("interrupts IRQ Trigger Type = %d (LEVEL LOW)\n", val);
                }
        } else {
                dbg_log("interrupts IRQ Trigger Type = %d (TRIGGER NONE)\n", val);
        }

        return val;
}

static unsigned int dm9051_get_irq_num(board_info_t *db)
{
        unsigned int irq_no = 0; //int ret;
        struct device_node *irq_node;

        const char irq_node_name[] = "davicom,dm9051";

        irq_node = of_find_compatible_node(NULL, NULL, irq_node_name);
        if (irq_node) {
                irq_no = irq_of_parse_and_map(irq_node, 0); //spi->irq
                dbg_log("get irq number irq = %d\n", irq_no);
                //if (irq_no)
                //dbg_log("get irq number ok!!\n");

                db->irq_type = dm9051_is_irq_type(irq_node);
        } else {
                dbg_log("dm9051_id_enable_pin --null irq node!!\n");
        }

        return irq_no;
}

static int dm9051_is_dma_param(struct board_info *db)
{
        int ret = 0;

        db->spidev->dev.coherent_dma_mask = ~0;

        db->spi_tx_buf = dma_alloc_coherent(&db->spidev->dev,
                                            PAGE_SIZE,
                                            &db->spi_tx_dma,
                                            GFP_KERNEL | GFP_DMA); //GFP_KERNEL, GFP_DMA

        if (db->spi_tx_buf) {
                //db->spi_rx_buf = (db->spi_tx_buf + (PAGE_SIZE / 2));
                db->spi_rx_dma = (dma_addr_t)(db->spi_tx_dma +
                                              (PAGE_SIZE / 2));
                ret = 1;
        } else {
                /* Fall back to non-DMA */
                //enable_dma = 0;
                ret = 0;
        }

        return ret;
}

/* Check the macaddr module parameter for a MAC address */
static int dm9051_is_cmdline_macaddr(u8 *dev_mac)
{
        int i = 0, j = 0, got_num = 0, num = 0;
        u8 mtbl[MAC_ADDR_LEN] = {0};

        if (macaddr[0] == ':') {
                return 0;
        }

        i = 0;
        j = 0;
        num = 0;
        got_num = 0;

        while (j < MAC_ADDR_LEN) {
                if (macaddr[i] && macaddr[i] != ':') {
                        got_num++;
                        if ('0' <= macaddr[i] && macaddr[i] <= '9') {
                                num = num * 16 + macaddr[i] - '0';
                        } else if ('A' <= macaddr[i] && macaddr[i] <= 'F') {
                                num = num * 16 + 10 + macaddr[i] - 'A';
                        } else if ('a' <= macaddr[i] && macaddr[i] <= 'f') {
                                num = num * 16 + 10 + macaddr[i] - 'a';
                        } else {
                                break;
                        }
                        i++;
                } else if (got_num == 2) {
                        mtbl[j++] = (u8) num;
                        num = 0;
                        got_num = 0;
                        i++;
                } else {
                        break;
                }
        }

        if (j == MAC_ADDR_LEN) {
                dbg_log("Overriding MAC address with: "
                        "%02x:%02x:%02x:%02x:%02x:%02x\n",
                        mtbl[0], mtbl[1], mtbl[2],
                        mtbl[3], mtbl[4], mtbl[5]);
                for (i = 0; i < MAC_ADDR_LEN; i++) {
                        dev_mac[i] = mtbl[i];
                }

                return 1;
        } else {
                return 0;
        }
}

static void dm9051_is_eeprom_macaddr(board_info_t *db)
{
        int i = 0;
        u8 eeprom_buf[2] = {0};

        for (i = 0; i < 64; i++) {
                dm9051_read_eeprom(db, i, eeprom_buf);
                if (!(i % 8)) {
                        dbg_log("\n");
                }

                if (!(i % 4)) {
                        dbg_log(" ");
                }
                dbg_log(" %02x %02x", eeprom_buf[0], eeprom_buf[1]);
        }
        dbg_log("\n");

        eeprom_buf[0] = 0x08;
        eeprom_buf[1] = 0x00;
        dm9051_write_eeprom(db, (12 + 0) / 2, eeprom_buf);

        eeprom_buf[0] = 0x80;
        eeprom_buf[1] = 0x41; //  0x0180 | (1<<14), DM9051 E1 (old) set WORD7.D14=1 to 'HP Auto-MDIX enable'
        dm9051_write_eeprom(db, (14 + 0) / 2, eeprom_buf);
        dbg_log("[dm9051.write_eeprom():  WORD[%d]= %02x %02x\n",
                (14 + 0) / 2, eeprom_buf[0], eeprom_buf[1]);

        dbg_log("[dm9051.dump_eeprom():");
        for (i = 0; i < 16; i++) {
                dm9051_read_eeprom(db, i, eeprom_buf);
                if (!(i % 8)) {
                        dbg_log("\n ");
                }

                if (!(i % 4)) {
                        dbg_log(" ");
                }
                dbg_log(" %02x %02x", eeprom_buf[0], eeprom_buf[1]);
        }
        dbg_log("\n");

        /* The node address from the attached EEPROM(already loaded into the chip), first. */
        for (i = 0; i < 6; i++) {
                db->ndev->dev_addr[i] = dm9051_spi_read_reg(db, DM9051_PAR + i);
        }
}

/**
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device.
 */
static int dm9051_open(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);

        if (netif_msg_ifup(db)) {
                dev_dbg(&db->spidev->dev, "enabling %s\n", dev->name);
        }

        //dbg_log("++");
        disable_irq_nosync(db->ndev->irq);

        mutex_lock(&db->addr_lock); //Note: must

        /* Note: Reg 1F is not set by reset */
        dm9051_spi_write_reg(db, DM9051_GPR, 0);        /* REG_1F bit0 activate phyxcer */
        mdelay(1); /* delay needs by DM9051 */

        /* Initialize DM9051 board */
        dm9051_soft_reset(db);
        dm9051_init_dm9051(dev);

        mutex_unlock(&db->addr_lock);

        skb_queue_head_init(&db->txq);

        netif_start_queue(dev);

        enable_irq(db->ndev->irq);

        return 0;
}

/**
 * dm951_net_stop - close network device
 * @dev: The device being closed.
 *
 * Called to close down a network device which has been active. Cancell any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state whilst it is not being used.
 */
static int dm9051_stop(struct net_device *dev)
{
        board_info_t *db = netdev_priv(dev);

        /* dm9051_shutdown(dev) */
        mutex_lock(&db->addr_lock);

        dm9051_phy_write(dev, 0, MII_BMCR, BMCR_RESET); /* PHY RESET */
        dm9051_spi_write_reg(db, DM9051_GPR, 0x01);     /* Power-Down PHY */
        dm9051_spi_write_reg(db, DM9051_IMR, IMR_PAR);  /* Disable all interrupt */
        dm9051_spi_write_reg(db, DM9051_RCR, RCR_RX_DISABLE);   /* Disable RX */

        mutex_unlock(&db->addr_lock);

        netif_stop_queue(dev);
        //JJ-Count-on
        netif_carrier_off(dev);

        //cancel_delayed_work_sync(&db->irq_work); //flush_work(&db->irq_work);

        return 0;
}

static const struct net_device_ops dm9051_netdev_ops = {
        .ndo_open            = dm9051_open,
        .ndo_stop            = dm9051_stop,
        .ndo_start_xmit      = dm9051_start_xmit,
        //.ndo_tx_timeout      = dm9051_timeout,
        .ndo_set_rx_mode     = dm9051_set_multicast_list_schedule,
        //.ndo_do_ioctl        = dm9051_ioctl,
        .ndo_change_mtu      = eth_change_mtu,
        //.ndo_set_features    = dm9051_set_features,
        .ndo_validate_addr   = eth_validate_addr,
        .ndo_set_mac_address = eth_mac_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
        //.ndo_poll_controller  =
#endif //CONFIG_NET_POLL_CONTROLLER
};

#if 0
static void SPI_para_config(struct spi_device *spidev)
{
        struct mt_chip_conf *spi_par = (struct mt_chip_conf *)spidev->controller_data;
        if (!spi_par) {
                dbg_log("[dm95_spi] spi config fail");
                return;
        }

        spi_par->setuptime = 15;
        spi_par->holdtime = 15;
        spi_par->high_time = 10;       //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
        spi_par->low_time = 10;
        spi_par->cs_idletime = 20;
        spi_par->rx_mlsb = 1;
        spi_par->tx_mlsb = 1;
        spi_par->tx_endian = 0;
        spi_par->rx_endian = 0;
        spi_par->cpol = 0;
        spi_par->cpha = 0;
        spi_par->com_mod = DMA_TRANSFER;
        //spi_par->com_mod = FIFO_TRANSFER;
        spi_par->pause = 0;
        spi_par->finish_intr = 1;
        spi_par->deassert = 0;

        spidev->max_speed_hz = (45 * 1000 * 1000);
        spidev->mode = SPI_MODE_0;
        spidev->bits_per_word = 8;

        if (spi_setup(spidev)) {
                dbg_log("dm9051_spi spi_setup fail\n");
        }
}
#endif //0

static unsigned dm9051_check_id(struct board_info *db)
{
        unsigned chipid;

        chipid = dm9051_spi_read_reg(db, DM9051_PIDL);
        chipid |= (unsigned)dm9051_spi_read_reg(db, DM9051_PIDH) << 8;

        return chipid;
}

static int dm9051_probe(struct spi_device *spi)
{
        const unsigned char *mac_addr = NULL;
        const unsigned char *mac_src = NULL;
        unsigned chipid = 0;
        int ret = 0;
        struct board_info *db;
        struct net_device *ndev;

        if (netif_msg_drv(&debug))
                dev_info(&spi->dev, DRV_NAME " Ethernet driver %s loaded\n",
                         DRV_VERSION);

        ndev = alloc_etherdev(sizeof(struct board_info));
        if (!ndev) {
                dev_err(&spi->dev, "failed to alloc ethernet device\n");
                return -ENOMEM;
        }
        /* setup board info structure */
        db = netdev_priv(ndev);

        db->ndev = ndev;
        db->spidev = spi;
        db->link = 0;
        db->irq_now = 0;
        //db->tx_space = 0;
        db->msg_enable = netif_msg_init(debug.msg_enable,
                                        DM9051_MSG_DEFAULT);

        //mt_dm9051_pinctrl_init(spi);
        //dm9051_hw_pwr_on();

        //SPI_para_config(spi);
        spi->bits_per_word = 8;
        if (spi_setup(spi)) {
                dbg_log("spi_setup fail\n");
                ret = -ENOMEM;
                goto err_id;
        }

        mutex_init(&db->addr_lock);
        spin_lock_init(&db->statelock); // used in 'dm9051' 'start' 'xmit'

        //get spi buffer size
        dbg_log("spi buffer size = %d\r\n", SPI_SYNC_TRANSFER_BUF_LEN);

        //get spi speed
        dbg_log("spi_max_frequency: %d KHz (%d MHz)\n",
                spi->max_speed_hz / 1000,
                spi->max_speed_hz / 1000 / 1000);

        if (enable_dma == 1) {
                enable_dma = dm9051_is_dma_param(db);
        }

        /* Allocate non-DMA buffers */
        if (enable_dma != 1) {
                db->spi_tx_buf = devm_kzalloc(&spi->dev, SPI_SYNC_TRANSFER_BUF_LEN + 4,
                                              GFP_KERNEL);
                if (!db->spi_tx_buf) {
                        ret = -ENOMEM;
                        goto err_id;
                }

                /*db->spi_rx_buf = devm_kzalloc(&spi->dev, SPI_SYNC_TRANSFER_BUF_LEN,
                                              GFP_KERNEL);
                if (!db->spi_rx_buf) {
                        ret = -ENOMEM;
                        goto err_id;
                }*/
        } else {
                dbg_log("enable spi dma\r\n");
        }

        INIT_WORK(&db->rxctrl_work, dm9051_hash_table_work);
        INIT_WORK(&db->tx_work, dm9051_tx_work);
        //INIT_DELAYED_WORK(&db->irq_work, dm9051_irq_work_handler);
        INIT_WORK(&db->irq_work, dm9051_irq_work_handler);

        /* initialise pre-made spi transfer messages */
        spi_message_init(&db->spi_msg1);
        spi_message_add_tail(&db->spi_xfer1, &db->spi_msg1);
        skb_queue_head_init(&db->txq);

        /* setup mii state */
        db->mii.dev          = ndev;
        db->mii.phy_id_mask  = 1;   //db->mii.phy_id_mask  = 0x1f;
        db->mii.reg_num_mask = 0xf; //db->mii.reg_num_mask = 0x1f;
        db->mii.phy_id       = 1;
        db->mii.mdio_read    = dm9051_phy_read_lock;
        db->mii.mdio_write   = dm9051_phy_write_lock;

        SET_NETDEV_DEV(ndev, &spi->dev);
        dev_set_drvdata(&spi->dev, db);

        dm9051_soft_reset(db);

        /* Get chip ID */
        chipid = dm9051_check_id(db);
        dbg_log("read chip id : 0x%x\n", chipid);
        if (chipid != (DM9051_ID >> 16) && chipid != (DM9051_ID >> 16)) {
                dbg_log("check chip id fail, get id = 0x%04x\n", chipid);
                ret = -ENODEV;
                goto err_id;
        }

        //Set MAC address
        if (enable_eeprom == 1) {
                mac_src = "EEPROM";
                dm9051_is_eeprom_macaddr(db);
        } else {
                /* Check module parameters */
                if (dm9051_is_cmdline_macaddr(ndev->dev_addr) == 1) {
                        mac_src = "cmdline";
                }

                if (!is_valid_ether_addr(ndev->dev_addr)) {
                        mac_addr = of_get_mac_address(spi->dev.of_node);
                        if (mac_addr) {
                                memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
                        }
                        mac_src = "device tree";
                }
        }

        /* The node address by the laboratory fixed (if previous not be valid) */
        if (!is_valid_ether_addr(ndev->dev_addr)) {
                //eth_hw_addr_random(ndev); // get random mac addr
                //mac_src = "random";
                mac_src = "fixed EEPROM"; //"Free-Style";
                ndev->dev_addr[0] = 0x00;
                ndev->dev_addr[1] = 0x60;
                ndev->dev_addr[2] = 0x6e;
                ndev->dev_addr[3] = 0x90;
                ndev->dev_addr[4] = 0x51;
                ndev->dev_addr[5] = 0xee;
        }

        //write MAC address to DM9051 register
        dm9051_write_mac_addr(db, mac_src);

        //get irq number
        spi->irq = dm9051_get_irq_num(db);
        if (spi->irq <= 0) {
                dbg_log("dm9051 failed to get irq_no\n");
                goto err_irq;
        }

        ret = request_irq(spi->irq, dm951_irq, IRQ_TYPE_NONE, ndev->name, db);
        if (ret < 0) {
                dbg_log("dm9051 failed to get irq\n");
                goto err_irq;
        }

        ndev->if_port     = IF_PORT_100BASET;
        ndev->irq         = spi->irq;
        ndev->netdev_ops  = &dm9051_netdev_ops;
        ndev->ethtool_ops = &dm9051_ethtool_ops;

        ret = register_netdev(ndev);
        if (ret) {
                dbg_log("failed to register network device\n");
                goto err_netdev;
        }

        dbg_log("probe success!!!\n");

        return 0;

err_netdev:
        free_irq(spi->irq, db);

err_id:
err_irq:
        free_netdev(ndev);

        return ret;
}

static int dm9051_remove(struct spi_device *spi)
{
        board_info_t *db = dev_get_drvdata(&spi->dev);

        unregister_netdev(db->ndev);

        if (enable_dma == 1) {
                dma_free_coherent(&spi->dev, PAGE_SIZE, db->spi_tx_buf, db->spi_tx_dma);
        }

        free_irq(spi->irq, db);
        free_netdev(db->ndev);

        return 0;
}

#if 0
static int dm9051_suspend(struct spi_device *spi, pm_message_t mesg)
{
        board_info_t *db = dev_get_drvdata(&spi->dev);

        netif_stop_queue(db->ndev);

        dm9051_spi_write_reg(db, DM9051_IMR, IMR_PAR); // disable interrupt

        return 0;
}

static int dm9051_resume(struct spi_device *spi)
{
        board_info_t *db = dev_get_drvdata(&spi->dev);

        dm9051_spi_write_reg(db, DM9051_IMR, db->imr_all); // Re-enable interrupt mask

        netif_wake_queue(db->ndev);

        return 0;
}
#endif //0

//static SIMPLE_DEV_PM_OPS(dm9051_pm_ops, dm9051_suspend, dm9051_resume);

static struct of_device_id dm9051_dt_ids[] = {
        {
                .compatible = "davicom,dm9051",
        },
        { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, dm9051_dt_ids);

static struct spi_driver dm9051_driver = {
        .driver = {
                .name  = DRVNAME_9051,
                .owner = THIS_MODULE,
                .of_match_table = dm9051_dt_ids,
                .bus = &spi_bus_type,
                //.pm = &dm9051_pm_ops,
        },
        .probe = dm9051_probe,
        .remove = dm9051_remove,
        //.suspend = dm9051_suspend,
        //.resume  = dm9051_resume,
};

static int __init dm9051_init(void)
{
        return spi_register_driver(&dm9051_driver);
}

static void __exit dm9051_exit(void)
{
        spi_unregister_driver(&dm9051_driver);
}

module_init(dm9051_init);
module_exit(dm9051_exit);
//module_spi_driver(dm9051_driver);

MODULE_DESCRIPTION("Davicom DM9051 network driver");
MODULE_AUTHOR("Jerry <chongji_wang@davicom.com.tw>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:dm9051");

module_param(macaddr, charp, 0444); //Jerry add
MODULE_PARM_DESC(macaddr, "MAC address source from cmdline.txt,\r\n"
                 "\t\tcmdline.txt write dm9051.macaddr=XX:XX:XX:XX:XX:XX"); //Jerry add

module_param(enable_eeprom, int, 0444);
MODULE_PARM_DESC(enable_eeprom, "Enable read MAC address from EEPROM,\r\n"
                 "\t\tDefault : 0 (OFF), 1 (ON),\r\n"
                 "\t\tcmdline.txt write dm9051.enable_eeprom=1 or 0.\r\n"
                 "\t\tnote : enable_eeprom=1, priority use EEPROM MAC address,\r\n"
                 "\t\tunless the EEPROM reads failed");

//module_param(enable_dma, int, 0444);
//MODULE_PARM_DESC(enable_dma, "Enable SPI DMA. Default: 0 (OFF), 1 (ON)");

//module_param_named(debug, debug.msg_enable, int, 0);
//MODULE_PARM_DESC(debug, "Debug verbosity level (0 = none, ..., ffff = all)");
