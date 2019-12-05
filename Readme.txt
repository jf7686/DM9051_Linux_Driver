  DAVICOM Semiconductor Inc.                            05/05/2014

        A Davicom DM9051 SPI Fast Ethernet driver for Linux.
        Copyright (C) 2009  Stone Shyr

        This program is free software; you can redistribute it and/or
        modify it under the terms of the GNU General Public License
        as published by the Free Software Foundation; either version 2
        of the License, or (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

  A. Compiler command:

        Run make command
        Run make install command.  If DM9051 module is already existed (check this by lsmod),
        Please run "modprobe -r dm9051" first

  B. The following steps teach you how to activate NIC:

      B-1: A simple and temporary method

        1. Used the upper compiler command to compile dm9051.c

        2. Insert DM9051 module into kernel
           "insmod dm9051.ko"           ;;Auto Detection Mode (Suggest)

           NOTE: You can type "man insmod" to see more description.

        3. Config a DM9051 network interface
           "ifconfig eth0 172.22.3.18"
                          ^^^^^^^^^^^ Your IP address

           NOTE: 1. You can type "man ifconfig" to see more description.
                 2. If eth0 has been used, you should use eth1 instead.

        4. Activate the IP routing table. For some distributions, it is not
           necessary. You can type "route" to check.

           "route add default netmask 255.255.255.0 eth0"

           NOTE: 1. You can type "man route" to see more description.
                 2. If eth0 has been used, you should use eth1 instead.

        5. Well done. Your DM9051 adapter actived now.

        Note. This is a temporary method. After you reboot the system, you
              will lost the setting.


      B-2: For Redhat, You can use the following to Activate NIC

        1.  login your system used the superuser.
        2.  copy dm9051.ko into /lib/modules/2.6.x/kernel/drivers/net/
        3.  add the new line with "alias eth0 dm9051" in "/etc/module.conf".
        4.  execute "netconfig -d eth0".
        5.  Fill your IP address, netmask and gateway
        6.  press <ok> to confirm and exit this setting
        7   reboot

        Note. If eth0 has been used, you should use eth1 instead.

  C. Setup driver parameter:
        1. MAC address : source from cmdline.txt
                        cmdline.txt write dm9051.macaddr=XX:XX:XX:XX:XX:XX
                        or
                        $sudo modprobe dm9051 macaddr=XX:XX:XX:XX:XX:XX

        2. Read EEPROM MAC address : source from EEPROM
                        cmdline.txt write dm9051.enable_eeprom=1 or 0.
                        or
                        $sudo modprobe dm9051 enable_eeprom=1 or 0.
                        note : enable_eeprom=1, priority use EEPROM MAC address.

  DAVICOM Web-Site: www.davicom.com.tw

//------------------------------------------------------------------------------
raspberry linux dm9051 driver 1.1

1. MAC address source from cmdlin.txt:
$sudo vim /boot/cmdline.txt

add :
dm9051.macaddr=XX:XX:XX:XX:XX:XX

ex: dm9051.macaddr=00:60:6E:90:51:02

2. open read DM9051 EEPROM get MAC address:
$sudo modprobe dm9051 enable_eeprom=1

3. on raspberry SPI clock table:
 cdiv    speed
     2    125.0 MHz
     4     62.5 MHz
     8     31.2 MHz
    16     15.6 MHz
    32      7.8 MHz
    64      3.9 MHz
   128     1953 kHz
   256      976 kHz
   512      488 kHz
  1024      244 kHz
  2048      122 kHz
  4096       61 kHz
  8192     30.5 kHz
 16384     15.2 kHz
 32768     7629 Hz

(1) use on raspberry pi 3 model B can't over than 15.6MHz, dts file set 15600000
(2) use on raspberry pi zero can't over than 7.8Mhz

4. on raspberry test throughput :
  use SPI0, on SPI0 transfer more than 96 byte auto enable DMA,
SPI clock = 24MHz. iperf test result about:
[ ID] Interval       Transfer     Bandwidth
[  4]  0.0-11.3 sec  21.8 MBytes  16.1 Mbits/sec

on SPI1, SPI clock 24Mhz, SPI1 & SPI2 not support DMA, iperf test result about:
[ ID] Interval       Transfer     Bandwidth
[  4]  0.0-10.7 sec  15.6 MBytes  12.2 Mbits/sec

//debug log --------------------------------------------------------------------
2019/09/24
1. on TX/RX function add FIFO pointer check, debug can't ping overnight issue.
