/* drivers/net/dm9051.h
 *
 * Copyright 2014 Davicom Semiconductor,Inc.
 *      http://www.davicom.com.tw
 *      2014/03/11  Joseph CHANG  v1.0  Create
 *
 * DM9051 register definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _DM9051_H_
#define _DM9051_H_

#define DRVNAME_9051                            "dm9051"
#define CARDNAME_9051                           "dm9051"
//#define VERSION_9051                            "raspi_v2.2"

#define DM9000_ID                               0x90000A46
#define DM9051_ID                               0x90510A46

#define FifoPointCheck

#define NUMRXBYTECOUNTER                        25
#define MAX_READYBIT_ZERO_COUNT                 5

/*
#define NUM_COUNTR_RXBYTE0      600 //100 //2 * 50
*/
#define NUM_RX_FIFO_FULL                        50
#define NUM_PERIOD_ENOUGH_RX                    25 //3 * 8
#define NUM_PERIOD_ENOUGH_TX                    25 //3 * 8
#define NUM_PERIOD_TRANS                        600 //2 * 60, 120
#define NUM_PERIOD_RXS                          600

#define DM9051_NCR                              0x00
#define DM9051_NSR                              0x01
#define DM9051_TCR                              0x02
#define DM9051_RCR                              0x05
#define DM9051_FCR                              0x0A
#define DM9051_PAR                              0x10
#define DM9051_GPCR                             0x1E

#define DM9051_VIDL                             0x28
#define DM9051_VIDH                             0x29
#define DM9051_PIDL                             0x2A
#define DM9051_PIDH                             0x2B
#define DM9051_INTCR                            0x39
#define DM9051_PPCR                             0x3D

#define DM9051_MPCR                             0x55
#define DM9051_TMSR                             0x5A

#define DM9051_MRRL                             0x74 //0xF4
#define DM9051_MRRH                             0x75 //0xF5
#define DM9051_MWRL                             0x7A //0xFA
#define DM9051_MWRH                             0x7B //0xFB
#define DM9051_TXPLL                            0x7C //0xFC
#define DM9051_TXPLH                            0x7D //0xFD
#define DM9051_ISR                              0x7E //0xFE
#define DM9051_IMR                              0x7F //0xFF
#define DM9051_IMR_OFF                          0x80

#define DM_SPI_MRCMDX                           (0x70)
#define DM_SPI_MRCMD                            (0x72)
#define DM_SPI_MWCMD                            (0x78)

#define DM_SPI_RD                               (0x00)
#define DM_SPI_WR                               (0x80)

//0x01
#define NSR_SPEED                               (1 << 7)
#define NSR_LINKST                              (1 << 6)
#define NSR_WAKEST                              (1 << 5)
#define NSR_TX2END                              (1 << 3)
#define NSR_TX1END                              (1 << 2)
//0x05
#define RCR_DIS_LONG                            (1 << 5)
#define RCR_DIS_CRC                             (1 << 4)
#define RCR_ALL                                 (1 << 3)
#define RCR_PRMSC                               (1 << 1)
#define RCR_RXEN                                (1 << 0)
#define RCR_RX_DISABLE                          (RCR_DIS_LONG | RCR_DIS_CRC) // #define RCR_RX_DISABLE 0x30
//0x0A
#define FCR_FLOW_ENABLE                         0x29
//0x1E
#define GPCR_GEP_CNTL                           (1 << 0)
//0x39
#define INTCR_POL                               (1 << 0)
//0x3D
//#define PPCR_SETTING                            0x00 (Trouble in the way)
//#define PPCR_SETTING                            0x01 (default)
//#define PPCR_SETTING                            0x02 (TO BE TRY ONCE LATER)
//#define PPCR_SETTING                            0x08 (Using now, To work to)
#define PPCR_SETTING                            0x08
//0x55
#define MPCR_RSTTX                              (1 << 1)
#define MPCR_RSTRX                              (1 << 0)
//0xFE
#define ISR_LNKCHGS                             (1 << 5)
#define ISR_ROOS                                (1 << 3)
#define ISR_ROS                                 (1 << 2)
#define ISR_PTS                                 (1 << 1)
#define ISR_PRS                                 (1 << 0)
#define ISR_CLR_STATUS                          (ISR_LNKCHGS | ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)
//0xFF
#define IMR_PAR                                 (1 << 7)
#define IMR_LNKCHGI                             (1 << 5)
#define IMR_ROOI                                (1 << 3)
#define IMR_ROI                                 (1 << 2)
#define IMR_PTM                                 (1 << 1)
#define IMR_PRM                                 (1 << 0)
#define IMR_ALL                                 (IMR_PAR | IMR_ROOI | IMR_ROI | /*IMR_PTM |*/ IMR_PRM | IMR_LNKCHGI)

//Const
#define DM9051_PKT_RDY                          0x01    /* Packet ready to receive */
#define DM9051_PKT_MAX                          1536    /* Received packet max size */

/*
 * dm9000 Ethernet
 */
//#define DM9051_NSR                              0x01
#define DM9051_TCR                              0x02
#define DM9051_RSR                              0x06
#define DM9051_BPTR                             0x08
#define DM9051_EPCR                             0x0B
#define DM9051_EPAR                             0x0C
#define DM9051_EPDRL                            0x0D
#define DM9051_EPDRH                            0x0E
#define DM9051_MAR                              0x16
#define DM9051_GPR                              0x1F
#define DM9051_SMCR                             0x2F
#define DM9051_PBCR                             0x38 /* pbcr version */

//0x00
#define NCR_WAKEEN                              (1 << 6)
#define NCR_FDX                                 (1 << 3)
#define NCR_RST                                 (1 << 0)
//0x02
#define TCR_TXREQ                               (1 << 0)
//0x06
#define RSR_RF                                  (1 << 7)
#define RSR_MF                                  (1 << 6)
#define RSR_LCS                                 (1 << 5)
#define RSR_RWTO                                (1 << 4)
#define RSR_PLE                                 (1 << 3)
#define RSR_AE                                  (1 << 2)
#define RSR_CE                                  (1 << 1)
#define RSR_FOE                                 (1 << 0)
//0x0B
#define EPCR_WEP                                (1 << 4) //=0x10
#define EPCR_EPOS                               (1 << 3)
#define EPCR_ERPRR                              (1 << 2)
#define EPCR_ERPRW                              (1 << 1)
#define EPCR_ERRE                               (1 << 0)

#endif /* _DM9051_H_ */
