// SPDX-License-Identifier: GPL-2.0-only
/*
################################################################################
#
# r8168 is the Linux device driver released for Realtek Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2021 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

/*
 * This driver is modified from r8169.c in Linux kernel 2.6.18
 */

/* In Linux 5.4 asm_inline was introduced, but it's not supported by clang.
 * Redefine it to just asm to enable successful compilation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/ip.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
#include <linux/ipv6.h>
#include <net/ip6_checksum.h>
#endif
#include <linux/tcp.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>
#include <linux/completion.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
#include <linux/pci-aspm.h>
#endif
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,37)
#include <linux/prefetch.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define dev_printk(A,B,fmt,args...) printk(A fmt,##args)
#else
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
#include <linux/mdio.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>

#include "r8168.h"
#include "r8168_asf.h"
#include "rtl_eeprom.h"
#include "rtltool.h"
#include "r8168_firmware.h"

#ifdef ENABLE_R8168_PROCFS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#define FIRMWARE_8168D_1    "rtl_nic/rtl8168d-1.fw"
#define FIRMWARE_8168D_2    "rtl_nic/rtl8168d-2.fw"
#define FIRMWARE_8168E_1    "rtl_nic/rtl8168e-1.fw"
#define FIRMWARE_8168E_2    "rtl_nic/rtl8168e-2.fw"
#define FIRMWARE_8168E_3    "rtl_nic/rtl8168e-3.fw"
#define FIRMWARE_8168E_4    "rtl_nic/rtl8168e-4.fw"
#define FIRMWARE_8168F_1    "rtl_nic/rtl8168f-1.fw"
#define FIRMWARE_8168F_2    "rtl_nic/rtl8168f-2.fw"
#define FIRMWARE_8411_1     "rtl_nic/rtl8411-1.fw"
#define FIRMWARE_8411_2     "rtl_nic/rtl8411-2.fw"
#define FIRMWARE_8168G_2    "rtl_nic/rtl8168g-2.fw"
#define FIRMWARE_8168G_3    "rtl_nic/rtl8168g-3.fw"
#define FIRMWARE_8168EP_1   "rtl_nic/rtl8168ep-1.fw"
#define FIRMWARE_8168EP_2   "rtl_nic/rtl8168ep-2.fw"
#define FIRMWARE_8168EP_3   "rtl_nic/rtl8168ep-3.fw"
#define FIRMWARE_8168H_1    "rtl_nic/rtl8168h-1.fw"
#define FIRMWARE_8168H_2    "rtl_nic/rtl8168h-2.fw"
#define FIRMWARE_8168FP_3   "rtl_nic/rtl8168fp-3.fw"
#define FIRMWARE_8168FP_4   "rtl_nic/rtl8168fp-4.fw"

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
   The RTL chips use a 64 element hash table based on the Ethernet CRC. */
static const int multicast_filter_limit = 32;

static const struct {
        const char *name;
        const char *fw_name;
} rtl_chip_fw_infos[] = {
        /* PCI-E devices. */
        [CFG_METHOD_1] = {"RTL8168B/8111",          },
        [CFG_METHOD_2] = {"RTL8168B/8111",          },
        [CFG_METHOD_3] = {"RTL8168B/8111",          },
        [CFG_METHOD_4] = {"RTL8168C/8111C",         },
        [CFG_METHOD_5] = {"RTL8168C/8111C",         },
        [CFG_METHOD_6] = {"RTL8168C/8111C",         },
        [CFG_METHOD_7] = {"RTL8168CP/8111CP",       },
        [CFG_METHOD_8] = {"RTL8168CP/8111CP",       },
        [CFG_METHOD_9] = {"RTL8168D/8111D",         FIRMWARE_8168D_1},
        [CFG_METHOD_10] = {"RTL8168D/8111D",        FIRMWARE_8168D_2},
        [CFG_METHOD_11] = {"RTL8168DP/8111DP",      },
        [CFG_METHOD_12] = {"RTL8168DP/8111DP",      },
        [CFG_METHOD_13] = {"RTL8168DP/8111DP",      },
        [CFG_METHOD_14] = {"RTL8168E/8111E",        FIRMWARE_8168E_1},
        [CFG_METHOD_15] = {"RTL8168E/8111E",        FIRMWARE_8168E_2},
        [CFG_METHOD_16] = {"RTL8168E-VL/8111E-VL",  FIRMWARE_8168E_3},
        [CFG_METHOD_17] = {"RTL8168E-VL/8111E-VL",  FIRMWARE_8168E_4},
        [CFG_METHOD_18] = {"RTL8168F/8111F",        FIRMWARE_8168F_1},
        [CFG_METHOD_19] = {"RTL8168F/8111F",        FIRMWARE_8168F_2},
        [CFG_METHOD_20] = {"RTL8411",               FIRMWARE_8411_1},
        [CFG_METHOD_21] = {"RTL8168G/8111G",        FIRMWARE_8168G_2},
        [CFG_METHOD_22] = {"RTL8168G/8111G",        },
        [CFG_METHOD_23] = {"RTL8168EP/8111EP",      FIRMWARE_8168EP_1},
        [CFG_METHOD_24] = {"RTL8168GU/8111GU",      },
        [CFG_METHOD_25] = {"RTL8168GU/8111GU",      FIRMWARE_8168G_3},
        [CFG_METHOD_26] = {"8411B",                 FIRMWARE_8411_2},
        [CFG_METHOD_27] = {"RTL8168EP/8111EP",      FIRMWARE_8168EP_2},
        [CFG_METHOD_28] = {"RTL8168EP/8111EP",      FIRMWARE_8168EP_3},
        [CFG_METHOD_29] = {"RTL8168H/8111H",        FIRMWARE_8168H_1},
        [CFG_METHOD_30] = {"RTL8168H/8111H",        FIRMWARE_8168H_2},
        [CFG_METHOD_31] = {"RTL8168FP/8111FP",      },
        [CFG_METHOD_32] = {"RTL8168FP/8111FP",      FIRMWARE_8168FP_3},
        [CFG_METHOD_33] = {"RTL8168FP/8111FP",      FIRMWARE_8168FP_4},
        [CFG_METHOD_DEFAULT] = {"Unknown",          },
};

#define _R(NAME,MAC,RCR,MASK, JumFrameSz) \
    { .name = NAME, .mcfg = MAC, .RCR_Cfg = RCR, .RxConfigMask = MASK, .jumbo_frame_sz = JumFrameSz }

static const struct {
        const char *name;
        u8 mcfg;
        u32 RCR_Cfg;
        u32 RxConfigMask;   /* Clears the bits supported by this chip */
        u32 jumbo_frame_sz;
} rtl_chip_info[] = {
        _R("RTL8168B/8111B",
        CFG_METHOD_1,
        (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_4k),

        _R("RTL8168B/8111B",
        CFG_METHOD_2,
        (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_4k),

        _R("RTL8168B/8111B",
        CFG_METHOD_3,
        (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_4k),

        _R("RTL8168C/8111C",
        CFG_METHOD_4,
        RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_6k),

        _R("RTL8168C/8111C",
        CFG_METHOD_5,
        RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_6k),

        _R("RTL8168C/8111C",
        CFG_METHOD_6,
        RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_6k),

        _R("RTL8168CP/8111CP",
        CFG_METHOD_7,
        RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_6k),

        _R("RTL8168CP/8111CP",
        CFG_METHOD_8,
        RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_6k),

        _R("RTL8168D/8111D",
        CFG_METHOD_9,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168D/8111D",
        CFG_METHOD_10,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168DP/8111DP",
        CFG_METHOD_11,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168DP/8111DP",
        CFG_METHOD_12,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168DP/8111DP",
        CFG_METHOD_13,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168E/8111E",
        CFG_METHOD_14,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168E/8111E",
        CFG_METHOD_15,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168E-VL/8111E-VL",
        CFG_METHOD_16,
        RxCfg_128_int_en | RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e0080,
        Jumbo_Frame_9k),

        _R("RTL8168E-VL/8111E-VL",
        CFG_METHOD_17,
        RxCfg_128_int_en | RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168F/8111F",
        CFG_METHOD_18,
        RxCfg_128_int_en | RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168F/8111F",
        CFG_METHOD_19,
        RxCfg_128_int_en | RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8411",
        CFG_METHOD_20,
        RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e1880,
        Jumbo_Frame_9k),

        _R("RTL8168G/8111G",
        CFG_METHOD_21,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168G/8111G",
        CFG_METHOD_22,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168EP/8111EP",
        CFG_METHOD_23,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168GU/8111GU",
        CFG_METHOD_24,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168GU/8111GU",
        CFG_METHOD_25,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("8411B",
        CFG_METHOD_26,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168EP/8111EP",
        CFG_METHOD_27,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168EP/8111EP",
        CFG_METHOD_28,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168H/8111H",
        CFG_METHOD_29,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168H/8111H",
        CFG_METHOD_30,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168FP/8111FP",
        CFG_METHOD_31,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168FP/8111FP",
        CFG_METHOD_32,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8168FP/8111FP",
        CFG_METHOD_33,
        RxCfg_128_int_en | RxEarly_off_V2 | Rx_Single_fetch_V2 | (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("Unknown",
        CFG_METHOD_DEFAULT,
        (RX_DMA_BURST << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_1k)
};
#undef _R

#ifndef PCI_VENDOR_ID_DLINK
#define PCI_VENDOR_ID_DLINK 0x1186
#endif

static struct pci_device_id rtl8168_pci_tbl[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8168), },
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8161), },
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x2502), },
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x2600), },
        { PCI_VENDOR_ID_DLINK, 0x4300, 0x1186, 0x4b10,},
        {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8168_pci_tbl);

static int rx_copybreak = 0;
static int use_dac = 1;
static int timer_count = 0x2600;
static int dynamic_aspm_packet_threshold = 10;

static struct {
        u32 msg_enable;
} debug = { -1 };

static unsigned int speed_mode = SPEED_1000;
static unsigned int duplex_mode = DUPLEX_FULL;
static unsigned int autoneg_mode = AUTONEG_ENABLE;
static unsigned int advertising_mode =  ADVERTISED_10baseT_Half |
                                        ADVERTISED_10baseT_Full |
                                        ADVERTISED_100baseT_Half |
                                        ADVERTISED_100baseT_Full |
                                        ADVERTISED_1000baseT_Half |
                                        ADVERTISED_1000baseT_Full;
#ifdef CONFIG_ASPM
static int aspm = 1;
#else
static int aspm = 0;
#endif
#ifdef CONFIG_DYNAMIC_ASPM
static int dynamic_aspm = 1;
#else
static int dynamic_aspm = 0;
#endif
#ifdef ENABLE_S5WOL
static int s5wol = 1;
#else
static int s5wol = 0;
#endif
#ifdef ENABLE_S5_KEEP_CURR_MAC
static int s5_keep_curr_mac = 1;
#else
static int s5_keep_curr_mac = 0;
#endif
#ifdef ENABLE_EEE
static int eee_enable = 1;
#else
static int eee_enable = 0;
#endif
#ifdef CONFIG_SOC_LAN
static ulong hwoptimize = HW_PATCH_SOC_LAN;
#else
static ulong hwoptimize = 0;
#endif
#ifdef ENABLE_S0_MAGIC_PACKET
static int s0_magic_packet = 1;
#else
static int s0_magic_packet = 0;
#endif

MODULE_AUTHOR("Realtek and the Linux r8168 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("RealTek RTL-8168 Gigabit Ethernet driver");

module_param(speed_mode, uint, 0);
MODULE_PARM_DESC(speed_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(duplex_mode, uint, 0);
MODULE_PARM_DESC(duplex_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(autoneg_mode, uint, 0);
MODULE_PARM_DESC(autoneg_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(advertising_mode, uint, 0);
MODULE_PARM_DESC(advertising_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(aspm, int, 0);
MODULE_PARM_DESC(aspm, "Enable ASPM.");

module_param(dynamic_aspm, int, 0);
MODULE_PARM_DESC(aspm, "Enable Software Dynamic ASPM.");

module_param(s5wol, int, 0);
MODULE_PARM_DESC(s5wol, "Enable Shutdown Wake On Lan.");

module_param(s5_keep_curr_mac, int, 0);
MODULE_PARM_DESC(s5_keep_curr_mac, "Enable Shutdown Keep Current MAC Address.");

module_param(rx_copybreak, int, 0);
MODULE_PARM_DESC(rx_copybreak, "Copy breakpoint for copy-only-tiny-frames");

module_param(use_dac, int, 0);
MODULE_PARM_DESC(use_dac, "Enable PCI DAC. Unsafe on 32 bit PCI slot.");

module_param(timer_count, int, 0);
MODULE_PARM_DESC(timer_count, "Timer Interrupt Interval.");

module_param(eee_enable, int, 0);
MODULE_PARM_DESC(eee_enable, "Enable Energy Efficient Ethernet.");

module_param(hwoptimize, ulong, 0);
MODULE_PARM_DESC(hwoptimize, "Enable HW optimization function.");

module_param(s0_magic_packet, int, 0);
MODULE_PARM_DESC(s0_magic_packet, "Enable S0 Magic Packet.");

module_param(dynamic_aspm_packet_threshold, int, 0);
MODULE_PARM_DESC(dynamic_aspm_packet_threshold, "Dynamic ASPM packet threshold.");

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

MODULE_LICENSE("GPL");
#ifdef ENABLE_USE_FIRMWARE_FILE
MODULE_FIRMWARE(FIRMWARE_8168D_1);
MODULE_FIRMWARE(FIRMWARE_8168D_2);
MODULE_FIRMWARE(FIRMWARE_8168E_1);
MODULE_FIRMWARE(FIRMWARE_8168E_2);
MODULE_FIRMWARE(FIRMWARE_8168E_3);
MODULE_FIRMWARE(FIRMWARE_8168E_4);
MODULE_FIRMWARE(FIRMWARE_8168F_1);
MODULE_FIRMWARE(FIRMWARE_8168F_2);
MODULE_FIRMWARE(FIRMWARE_8411_1);
MODULE_FIRMWARE(FIRMWARE_8411_2);
MODULE_FIRMWARE(FIRMWARE_8168G_2);
MODULE_FIRMWARE(FIRMWARE_8168G_3);
MODULE_FIRMWARE(FIRMWARE_8168EP_1);
MODULE_FIRMWARE(FIRMWARE_8168EP_2);
MODULE_FIRMWARE(FIRMWARE_8168EP_3);
MODULE_FIRMWARE(FIRMWARE_8168H_1);
MODULE_FIRMWARE(FIRMWARE_8168H_2);
MODULE_FIRMWARE(FIRMWARE_8168FP_3);
MODULE_FIRMWARE(FIRMWARE_8168FP_4);
#endif

MODULE_VERSION(RTL8168_VERSION);

static void rtl8168_sleep_rx_enable(struct net_device *dev);
static void rtl8168_dsm(struct net_device *dev, int dev_state);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static void rtl8168_esd_timer(unsigned long __opaque);
#else
static void rtl8168_esd_timer(struct timer_list *t);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static void rtl8168_link_timer(unsigned long __opaque);
#else
static void rtl8168_link_timer(struct timer_list *t);
#endif
static void rtl8168_tx_clear(struct rtl8168_private *tp);
static void rtl8168_rx_clear(struct rtl8168_private *tp);

static int rtl8168_open(struct net_device *dev);
static netdev_tx_t rtl8168_start_xmit(struct sk_buff *skb, struct net_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance, struct pt_regs *regs);
#else
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance);
#endif
static void rtl8168_rx_desc_offset0_init(struct rtl8168_private *, int);
static int rtl8168_init_ring(struct net_device *dev);
static void rtl8168_hw_config(struct net_device *dev);
static void rtl8168_hw_start(struct net_device *dev);
static int rtl8168_close(struct net_device *dev);
static void rtl8168_set_rx_mode(struct net_device *dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static void rtl8168_tx_timeout(struct net_device *dev, unsigned int txqueue);
#else
static void rtl8168_tx_timeout(struct net_device *dev);
#endif
static struct net_device_stats *rtl8168_get_stats(struct net_device *dev);
static int rtl8168_rx_interrupt(struct net_device *, struct rtl8168_private *, napi_budget);
static int rtl8168_change_mtu(struct net_device *dev, int new_mtu);
static void rtl8168_down(struct net_device *dev);

static int rtl8168_set_mac_address(struct net_device *dev, void *p);
void rtl8168_rar_set(struct rtl8168_private *tp, uint8_t *addr);
static void rtl8168_desc_addr_fill(struct rtl8168_private *);
static void rtl8168_tx_desc_init(struct rtl8168_private *tp);
static void rtl8168_rx_desc_init(struct rtl8168_private *tp);

static u16 rtl8168_get_hw_phy_mcu_code_ver(struct rtl8168_private *tp);

static void rtl8168_hw_reset(struct net_device *dev);

static void rtl8168_phy_power_up(struct net_device *dev);
static void rtl8168_phy_power_down(struct net_device *dev);
static int rtl8168_set_speed(struct net_device *dev, u8 autoneg, u32 speed, u8 duplex, u32 adv);

static int rtl8168_set_phy_mcu_patch_request(struct rtl8168_private *tp);
static int rtl8168_clear_phy_mcu_patch_request(struct rtl8168_private *tp);

#ifdef CONFIG_R8168_NAPI
static int rtl8168_poll(napi_ptr napi, napi_budget budget);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8168_reset_task(void *_data);
#else
static void rtl8168_reset_task(struct work_struct *work);
#endif

static inline struct device *tp_to_dev(struct rtl8168_private *tp)
{
        return &tp->pci_dev->dev;
}

#if ((LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0) && \
     LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,00)))
void ethtool_convert_legacy_u32_to_link_mode(unsigned long *dst,
                u32 legacy_u32)
{
        bitmap_zero(dst, __ETHTOOL_LINK_MODE_MASK_NBITS);
        dst[0] = legacy_u32;
}

bool ethtool_convert_link_mode_to_legacy_u32(u32 *legacy_u32,
                const unsigned long *src)
{
        bool retval = true;

        /* TODO: following test will soon always be true */
        if (__ETHTOOL_LINK_MODE_MASK_NBITS > 32) {
                __ETHTOOL_DECLARE_LINK_MODE_MASK(ext);

                bitmap_zero(ext, __ETHTOOL_LINK_MODE_MASK_NBITS);
                bitmap_fill(ext, 32);
                bitmap_complement(ext, ext, __ETHTOOL_LINK_MODE_MASK_NBITS);
                if (bitmap_intersects(ext, src,
                                      __ETHTOOL_LINK_MODE_MASK_NBITS)) {
                        /* src mask goes beyond bit 31 */
                        retval = false;
                }
        }
        *legacy_u32 = src[0];
        return retval;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)

#ifndef LPA_1000FULL
#define LPA_1000FULL            0x0800
#endif

#ifndef LPA_1000HALF
#define LPA_1000HALF            0x0400
#endif

static inline u32 mii_adv_to_ethtool_adv_t(u32 adv)
{
        u32 result = 0;

        if (adv & ADVERTISE_10HALF)
                result |= ADVERTISED_10baseT_Half;
        if (adv & ADVERTISE_10FULL)
                result |= ADVERTISED_10baseT_Full;
        if (adv & ADVERTISE_100HALF)
                result |= ADVERTISED_100baseT_Half;
        if (adv & ADVERTISE_100FULL)
                result |= ADVERTISED_100baseT_Full;
        if (adv & ADVERTISE_PAUSE_CAP)
                result |= ADVERTISED_Pause;
        if (adv & ADVERTISE_PAUSE_ASYM)
                result |= ADVERTISED_Asym_Pause;

        return result;
}

static inline u32 mii_lpa_to_ethtool_lpa_t(u32 lpa)
{
        u32 result = 0;

        if (lpa & LPA_LPACK)
                result |= ADVERTISED_Autoneg;

        return result | mii_adv_to_ethtool_adv_t(lpa);
}

static inline u32 mii_stat1000_to_ethtool_lpa_t(u32 lpa)
{
        u32 result = 0;

        if (lpa & LPA_1000HALF)
                result |= ADVERTISED_1000baseT_Half;
        if (lpa & LPA_1000FULL)
                result |= ADVERTISED_1000baseT_Full;

        return result;
}

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
static inline void eth_hw_addr_random(struct net_device *dev)
{
        random_ether_addr(dev->dev_addr);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#undef ethtool_ops
#define ethtool_ops _kc_ethtool_ops

struct _kc_ethtool_ops {
        int  (*get_settings)(struct net_device *, struct ethtool_cmd *);
        int  (*set_settings)(struct net_device *, struct ethtool_cmd *);
        void (*get_drvinfo)(struct net_device *, struct ethtool_drvinfo *);
        int  (*get_regs_len)(struct net_device *);
        void (*get_regs)(struct net_device *, struct ethtool_regs *, void *);
        void (*get_wol)(struct net_device *, struct ethtool_wolinfo *);
        int  (*set_wol)(struct net_device *, struct ethtool_wolinfo *);
        u32  (*get_msglevel)(struct net_device *);
        void (*set_msglevel)(struct net_device *, u32);
        int  (*nway_reset)(struct net_device *);
        u32  (*get_link)(struct net_device *);
        int  (*get_eeprom_len)(struct net_device *);
        int  (*get_eeprom)(struct net_device *, struct ethtool_eeprom *, u8 *);
        int  (*set_eeprom)(struct net_device *, struct ethtool_eeprom *, u8 *);
        int  (*get_coalesce)(struct net_device *, struct ethtool_coalesce *);
        int  (*set_coalesce)(struct net_device *, struct ethtool_coalesce *);
        void (*get_ringparam)(struct net_device *, struct ethtool_ringparam *);
        int  (*set_ringparam)(struct net_device *, struct ethtool_ringparam *);
        void (*get_pauseparam)(struct net_device *,
                               struct ethtool_pauseparam*);
        int  (*set_pauseparam)(struct net_device *,
                               struct ethtool_pauseparam*);
        u32  (*get_rx_csum)(struct net_device *);
        int  (*set_rx_csum)(struct net_device *, u32);
        u32  (*get_tx_csum)(struct net_device *);
        int  (*set_tx_csum)(struct net_device *, u32);
        u32  (*get_sg)(struct net_device *);
        int  (*set_sg)(struct net_device *, u32);
        u32  (*get_tso)(struct net_device *);
        int  (*set_tso)(struct net_device *, u32);
        int  (*self_test_count)(struct net_device *);
        void (*self_test)(struct net_device *, struct ethtool_test *, u64 *);
        void (*get_strings)(struct net_device *, u32 stringset, u8 *);
        int  (*phys_id)(struct net_device *, u32);
        int  (*get_stats_count)(struct net_device *);
        void (*get_ethtool_stats)(struct net_device *, struct ethtool_stats *,
                                  u64 *);
} *ethtool_ops = NULL;

#undef SET_ETHTOOL_OPS
#define SET_ETHTOOL_OPS(netdev, ops) (ethtool_ops = (ops))

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
#ifndef SET_ETHTOOL_OPS
#define SET_ETHTOOL_OPS(netdev,ops) \
         ( (netdev)->ethtool_ops = (ops) )
#endif //SET_ETHTOOL_OPS
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)

//#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)
#ifndef netif_msg_init
#define netif_msg_init _kc_netif_msg_init
/* copied from linux kernel 2.6.20 include/linux/netdevice.h */
static inline u32 netif_msg_init(int debug_value, int default_msg_enable_bits)
{
        /* use default */
        if (debug_value < 0 || debug_value >= (sizeof(u32) * 8))
                return default_msg_enable_bits;
        if (debug_value == 0)   /* no output */
                return 0;
        /* set low N bits */
        return (1 << debug_value) - 1;
}

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
static inline void eth_copy_and_sum (struct sk_buff *dest,
                                     const unsigned char *src,
                                     int len, int base)
{
        memcpy (dest->data, src, len);
}
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
/* copied from linux kernel 2.6.20 /include/linux/time.h */
/* Parameters used to convert the timespec values: */
#define MSEC_PER_SEC    1000L

/* copied from linux kernel 2.6.20 /include/linux/jiffies.h */
/*
 * Change timeval to jiffies, trying to avoid the
 * most obvious overflows..
 *
 * And some not so obvious.
 *
 * Note that we don't want to return MAX_LONG, because
 * for various timeout reasons we often end up having
 * to wait "jiffies+1" in order to guarantee that we wait
 * at _least_ "jiffies" - so "jiffies+1" had better still
 * be positive.
 */
#define MAX_JIFFY_OFFSET ((~0UL >> 1)-1)

/*
 * Convert jiffies to milliseconds and back.
 *
 * Avoid unnecessary multiplications/divisions in the
 * two most common HZ cases:
 */
static inline unsigned int _kc_jiffies_to_msecs(const unsigned long j)
{
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
        return (MSEC_PER_SEC / HZ) * j;
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
        return (j + (HZ / MSEC_PER_SEC) - 1)/(HZ / MSEC_PER_SEC);
#else
        return (j * MSEC_PER_SEC) / HZ;
#endif
}

static inline unsigned long _kc_msecs_to_jiffies(const unsigned int m)
{
        if (m > _kc_jiffies_to_msecs(MAX_JIFFY_OFFSET))
                return MAX_JIFFY_OFFSET;
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
        return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
        return m * (HZ / MSEC_PER_SEC);
#else
        return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
#endif
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)

/* copied from linux kernel 2.6.12.6 /include/linux/pm.h */
typedef int __bitwise pci_power_t;

/* copied from linux kernel 2.6.12.6 /include/linux/pci.h */
typedef u32 __bitwise pm_message_t;

#define PCI_D0  ((pci_power_t __force) 0)
#define PCI_D1  ((pci_power_t __force) 1)
#define PCI_D2  ((pci_power_t __force) 2)
#define PCI_D3hot   ((pci_power_t __force) 3)
#define PCI_D3cold  ((pci_power_t __force) 4)
#define PCI_POWER_ERROR ((pci_power_t __force) -1)

/* copied from linux kernel 2.6.12.6 /drivers/pci/pci.c */
/**
 * pci_choose_state - Choose the power state of a PCI device
 * @dev: PCI device to be suspended
 * @state: target sleep state for the whole system. This is the value
 *  that is passed to suspend() function.
 *
 * Returns PCI power state suitable for given device and given system
 * message.
 */

pci_power_t pci_choose_state(struct pci_dev *dev, pm_message_t state)
{
        if (!pci_find_capability(dev, PCI_CAP_ID_PM))
                return PCI_D0;

        switch (state) {
        case 0:
                return PCI_D0;
        case 3:
                return PCI_D3hot;
        default:
                printk("They asked me for state %d\n", state);
//      BUG();
        }
        return PCI_D0;
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
/**
 * msleep_interruptible - sleep waiting for waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
#define msleep_interruptible _kc_msleep_interruptible
unsigned long _kc_msleep_interruptible(unsigned int msecs)
{
        unsigned long timeout = _kc_msecs_to_jiffies(msecs);

        while (timeout && !signal_pending(current)) {
                set_current_state(TASK_INTERRUPTIBLE);
                timeout = schedule_timeout(timeout);
        }
        return _kc_jiffies_to_msecs(timeout);
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
/* copied from linux kernel 2.6.20 include/linux/sched.h */
#ifndef __sched
#define __sched     __attribute__((__section__(".sched.text")))
#endif

/* copied from linux kernel 2.6.20 kernel/timer.c */
signed long __sched schedule_timeout_uninterruptible(signed long timeout)
{
        __set_current_state(TASK_UNINTERRUPTIBLE);
        return schedule_timeout(timeout);
}

/* copied from linux kernel 2.6.20 include/linux/mii.h */
#undef if_mii
#define if_mii _kc_if_mii
static inline struct mii_ioctl_data *if_mii(struct ifreq *rq)
{
        return (struct mii_ioctl_data *) &rq->ifr_ifru;
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)

struct rtl8168_counters {
        u64 tx_packets;
        u64 rx_packets;
        u64 tx_errors;
        u32 rx_errors;
        u16 rx_missed;
        u16 align_errors;
        u32 tx_one_collision;
        u32 tx_multi_collision;
        u64 rx_unicast;
        u64 rx_broadcast;
        u32 rx_multicast;
        u16 tx_aborted;
        u16 tx_underun;
};

#ifdef ENABLE_R8168_PROCFS
/****************************************************************************
*   -----------------------------PROCFS STUFF-------------------------
*****************************************************************************
*/

static struct proc_dir_entry *rtl8168_proc;
static int proc_init_num = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
static int proc_get_driver_variable(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        seq_puts(m, "\nDump Driver Variable\n");

        spin_lock_irqsave(&tp->lock, flags);
        seq_puts(m, "Variable\tValue\n----------\t-----\n");
        seq_printf(m, "MODULENAME\t%s\n", MODULENAME);
        seq_printf(m, "driver version\t%s\n", RTL8168_VERSION);
        seq_printf(m, "chipset\t%d\n", tp->chipset);
        seq_printf(m, "chipset_name\t%s\n", rtl_chip_info[tp->chipset].name);
        seq_printf(m, "mtu\t%d\n", dev->mtu);
        seq_printf(m, "NUM_RX_DESC\t0x%x\n", NUM_RX_DESC);
        seq_printf(m, "cur_rx\t0x%x\n", tp->cur_rx);
        seq_printf(m, "dirty_rx\t0x%x\n", tp->dirty_rx);
        seq_printf(m, "NUM_TX_DESC\t0x%x\n", NUM_TX_DESC);
        seq_printf(m, "cur_tx\t0x%x\n", tp->cur_tx);
        seq_printf(m, "dirty_tx\t0x%x\n", tp->dirty_tx);
        seq_printf(m, "rx_buf_sz\t0x%x\n", tp->rx_buf_sz);
        seq_printf(m, "esd_flag\t0x%x\n", tp->esd_flag);
        seq_printf(m, "pci_cfg_is_read\t0x%x\n", tp->pci_cfg_is_read);
        seq_printf(m, "rtl8168_rx_config\t0x%x\n", tp->rtl8168_rx_config);
        seq_printf(m, "cp_cmd\t0x%x\n", tp->cp_cmd);
        seq_printf(m, "intr_mask\t0x%x\n", tp->intr_mask);
        seq_printf(m, "timer_intr_mask\t0x%x\n", tp->timer_intr_mask);
        seq_printf(m, "wol_enabled\t0x%x\n", tp->wol_enabled);
        seq_printf(m, "wol_opts\t0x%x\n", tp->wol_opts);
        seq_printf(m, "efuse_ver\t0x%x\n", tp->efuse_ver);
        seq_printf(m, "eeprom_type\t0x%x\n", tp->eeprom_type);
        seq_printf(m, "autoneg\t0x%x\n", tp->autoneg);
        seq_printf(m, "duplex\t0x%x\n", tp->duplex);
        seq_printf(m, "speed\t%d\n", tp->speed);
        seq_printf(m, "advertising\t0x%x\n", tp->advertising);
        seq_printf(m, "eeprom_len\t0x%x\n", tp->eeprom_len);
        seq_printf(m, "cur_page\t0x%x\n", tp->cur_page);
        seq_printf(m, "bios_setting\t0x%x\n", tp->bios_setting);
        seq_printf(m, "features\t0x%x\n", tp->features);
        seq_printf(m, "org_pci_offset_99\t0x%x\n", tp->org_pci_offset_99);
        seq_printf(m, "org_pci_offset_180\t0x%x\n", tp->org_pci_offset_180);
        seq_printf(m, "issue_offset_99_event\t0x%x\n", tp->issue_offset_99_event);
        seq_printf(m, "org_pci_offset_80\t0x%x\n", tp->org_pci_offset_80);
        seq_printf(m, "org_pci_offset_81\t0x%x\n", tp->org_pci_offset_81);
        seq_printf(m, "use_timer_interrrupt\t0x%x\n", tp->use_timer_interrrupt);
        seq_printf(m, "HwIcVerUnknown\t0x%x\n", tp->HwIcVerUnknown);
        seq_printf(m, "NotWrRamCodeToMicroP\t0x%x\n", tp->NotWrRamCodeToMicroP);
        seq_printf(m, "NotWrMcuPatchCode\t0x%x\n", tp->NotWrMcuPatchCode);
        seq_printf(m, "HwHasWrRamCodeToMicroP\t0x%x\n", tp->HwHasWrRamCodeToMicroP);
        seq_printf(m, "sw_ram_code_ver\t0x%x\n", tp->sw_ram_code_ver);
        seq_printf(m, "hw_ram_code_ver\t0x%x\n", tp->hw_ram_code_ver);
        seq_printf(m, "rtk_enable_diag\t0x%x\n", tp->rtk_enable_diag);
        seq_printf(m, "ShortPacketSwChecksum\t0x%x\n", tp->ShortPacketSwChecksum);
        seq_printf(m, "UseSwPaddingShortPkt\t0x%x\n", tp->UseSwPaddingShortPkt);
        seq_printf(m, "RequireAdcBiasPatch\t0x%x\n", tp->RequireAdcBiasPatch);
        seq_printf(m, "AdcBiasPatchIoffset\t0x%x\n", tp->AdcBiasPatchIoffset);
        seq_printf(m, "RequireAdjustUpsTxLinkPulseTiming\t0x%x\n", tp->RequireAdjustUpsTxLinkPulseTiming);
        seq_printf(m, "SwrCnt1msIni\t0x%x\n", tp->SwrCnt1msIni);
        seq_printf(m, "HwSuppNowIsOobVer\t0x%x\n", tp->HwSuppNowIsOobVer);
        seq_printf(m, "HwFiberModeVer\t0x%x\n", tp->HwFiberModeVer);
        seq_printf(m, "HwFiberStat\t0x%x\n", tp->HwFiberStat);
        seq_printf(m, "HwSwitchMdiToFiber\t0x%x\n", tp->HwSwitchMdiToFiber);
        seq_printf(m, "HwSuppSerDesPhyVer\t0x%x\n", tp->HwSuppSerDesPhyVer);
        seq_printf(m, "NicCustLedValue\t0x%x\n", tp->NicCustLedValue);
        seq_printf(m, "RequiredSecLanDonglePatch\t0x%x\n", tp->RequiredSecLanDonglePatch);
        seq_printf(m, "HwSuppDashVer\t0x%x\n", tp->HwSuppDashVer);
        seq_printf(m, "DASH\t0x%x\n", tp->DASH);
        seq_printf(m, "dash_printer_enabled\t0x%x\n", tp->dash_printer_enabled);
        seq_printf(m, "HwSuppKCPOffloadVer\t0x%x\n", tp->HwSuppKCPOffloadVer);
        seq_printf(m, "speed_mode\t0x%x\n", speed_mode);
        seq_printf(m, "duplex_mode\t0x%x\n", duplex_mode);
        seq_printf(m, "autoneg_mode\t0x%x\n", autoneg_mode);
        seq_printf(m, "advertising_mode\t0x%x\n", advertising_mode);
        seq_printf(m, "aspm\t0x%x\n", aspm);
        seq_printf(m, "s5wol\t0x%x\n", s5wol);
        seq_printf(m, "s5_keep_curr_mac\t0x%x\n", s5_keep_curr_mac);
        seq_printf(m, "eee_enable\t0x%x\n", tp->eee_enabled);
        seq_printf(m, "hwoptimize\t0x%lx\n", hwoptimize);
        seq_printf(m, "proc_init_num\t0x%x\n", proc_init_num);
        seq_printf(m, "s0_magic_packet\t0x%x\n", s0_magic_packet);
        seq_printf(m, "HwSuppMagicPktVer\t0x%x\n", tp->HwSuppMagicPktVer);
        seq_printf(m, "HwSuppUpsVer\t0x%x\n", tp->HwSuppUpsVer);
        seq_printf(m, "HwSuppEsdVer\t0x%x\n", tp->HwSuppEsdVer);
        seq_printf(m, "HwSuppCheckPhyDisableModeVer\t0x%x\n", tp->HwSuppCheckPhyDisableModeVer);
        seq_printf(m, "HwPkgDet\t0x%x\n", tp->HwPkgDet);
        seq_printf(m, "random_mac\t0x%x\n", tp->random_mac);
        seq_printf(m, "org_mac_addr\t%pM\n", tp->org_mac_addr);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
        seq_printf(m, "perm_addr\t%pM\n", dev->perm_addr);
#endif
        seq_printf(m, "dev_addr\t%pM\n", dev->dev_addr);
        spin_unlock_irqrestore(&tp->lock, flags);

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_tally_counter(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        struct rtl8168_private *tp = netdev_priv(dev);
        struct rtl8168_counters *counters;
        dma_addr_t paddr;
        u32 cmd;
        u32 WaitCnt;
        unsigned long flags;

        seq_puts(m, "\nDump Tally Counter\n");

        //ASSERT_RTNL();

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters) {
                seq_puts(m, "\nDump Tally Counter Fail\n");
                return 0;
        }

        spin_lock_irqsave(&tp->lock, flags);
        RTL_W32(tp, CounterAddrHigh, (u64)paddr >> 32);
        cmd = (u64)paddr & DMA_BIT_MASK(32);
        RTL_W32(tp, CounterAddrLow, cmd);
        RTL_W32(tp, CounterAddrLow, cmd | CounterDump);

        WaitCnt = 0;
        while (RTL_R32(tp, CounterAddrLow) & CounterDump) {
                udelay(10);

                WaitCnt++;
                if (WaitCnt > 20)
                        break;
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        seq_puts(m, "Statistics\tValue\n----------\t-----\n");
        seq_printf(m, "tx_packets\t%lld\n", le64_to_cpu(counters->tx_packets));
        seq_printf(m, "rx_packets\t%lld\n", le64_to_cpu(counters->rx_packets));
        seq_printf(m, "tx_errors\t%lld\n", le64_to_cpu(counters->tx_errors));
        seq_printf(m, "rx_missed\t%lld\n", le64_to_cpu(counters->rx_missed));
        seq_printf(m, "align_errors\t%lld\n", le64_to_cpu(counters->align_errors));
        seq_printf(m, "tx_one_collision\t%lld\n", le64_to_cpu(counters->tx_one_collision));
        seq_printf(m, "tx_multi_collision\t%lld\n", le64_to_cpu(counters->tx_multi_collision));
        seq_printf(m, "rx_unicast\t%lld\n", le64_to_cpu(counters->rx_unicast));
        seq_printf(m, "rx_broadcast\t%lld\n", le64_to_cpu(counters->rx_broadcast));
        seq_printf(m, "rx_multicast\t%lld\n", le64_to_cpu(counters->rx_multicast));
        seq_printf(m, "tx_aborted\t%lld\n", le64_to_cpu(counters->tx_aborted));
        seq_printf(m, "tx_underun\t%lld\n", le64_to_cpu(counters->tx_underun));

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_registers(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8168_MAC_REGS_SIZE;
        u8 byte_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;
        unsigned long flags;

        seq_puts(m, "\nDump MAC Registers\n");
        seq_puts(m, "Offset\tValue\n------\t-----\n");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%02x:\t", n);

                for (i = 0; i < 16 && n < max; i++, n++) {
                        byte_rd = readb(ioaddr + n);
                        seq_printf(m, "%02x ", byte_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_pcie_phy(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8168_EPHY_REGS_SIZE/2;
        u16 word_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        seq_puts(m, "\nDump PCIE PHY\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%02x:\t", n);

                for (i = 0; i < 8 && n < max; i++, n++) {
                        word_rd = rtl8168_ephy_read(tp, n);
                        seq_printf(m, "%04x ", word_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_eth_phy(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8168_PHY_REGS_SIZE/2;
        u16 word_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        seq_puts(m, "\nDump Ethernet PHY\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        spin_lock_irqsave(&tp->lock, flags);
        seq_puts(m, "\n####################page 0##################\n ");
        rtl8168_mdio_write(tp, 0x1f, 0x0000);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%02x:\t", n);

                for (i = 0; i < 8 && n < max; i++, n++) {
                        word_rd = rtl8168_mdio_read(tp, n);
                        seq_printf(m, "%04x ", word_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_extended_registers(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8168_ERI_REGS_SIZE;
        u32 dword_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
                /* RTL8168B does not support Extend GMAC */
                seq_puts(m, "\nNot Support Dump Extended Registers\n");
                return 0;
        }

        seq_puts(m, "\nDump Extended Registers\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%02x:\t", n);

                for (i = 0; i < 4 && n < max; i++, n+=4) {
                        dword_rd = rtl8168_eri_read(tp, n, 4, ERIAR_ExGMAC);
                        seq_printf(m, "%08x ", dword_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        seq_putc(m, '\n');
        return 0;
}

static int proc_get_pci_registers(struct seq_file *m, void *v)
{
        struct net_device *dev = m->private;
        int i, n, max = R8168_PCI_REGS_SIZE;
        u32 dword_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        seq_puts(m, "\nDump PCI Registers\n");
        seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                seq_printf(m, "\n0x%03x:\t", n);

                for (i = 0; i < 4 && n < max; i++, n+=4) {
                        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
                        seq_printf(m, "%08x ", dword_rd);
                }
        }

        n = 0x110;
        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
        seq_printf(m, "\n0x%03x:\t%08x ", n, dword_rd);
        n = 0x70c;
        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
        seq_printf(m, "\n0x%03x:\t%08x ", n, dword_rd);

        spin_unlock_irqrestore(&tp->lock, flags);

        seq_putc(m, '\n');
        return 0;
}
#else

static int proc_get_driver_variable(char *page, char **start,
                                    off_t offset, int count,
                                    int *eof, void *data)
{
        struct net_device *dev = data;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
        int len = 0;

        len += snprintf(page + len, count - len,
                        "\nDump Driver Driver\n");

        spin_lock_irqsave(&tp->lock, flags);
        len += snprintf(page + len, count - len,
                        "Variable\tValue\n----------\t-----\n");

        len += snprintf(page + len, count - len,
                        "MODULENAME\t%s\n"
                        "driver version\t%s\n"
                        "chipset\t%d\n"
                        "chipset_name\t%s\n"
                        "mtu\t%d\n"
                        "NUM_RX_DESC\t0x%x\n"
                        "cur_rx\t0x%x\n"
                        "dirty_rx\t0x%x\n"
                        "NUM_TX_DESC\t0x%x\n"
                        "cur_tx\t0x%x\n"
                        "dirty_tx\t0x%x\n"
                        "rx_buf_sz\t0x%x\n"
                        "esd_flag\t0x%x\n"
                        "pci_cfg_is_read\t0x%x\n"
                        "rtl8168_rx_config\t0x%x\n"
                        "cp_cmd\t0x%x\n"
                        "intr_mask\t0x%x\n"
                        "timer_intr_mask\t0x%x\n"
                        "wol_enabled\t0x%x\n"
                        "wol_opts\t0x%x\n"
                        "efuse_ver\t0x%x\n"
                        "eeprom_type\t0x%x\n"
                        "autoneg\t0x%x\n"
                        "duplex\t0x%x\n"
                        "speed\t%d\n"
                        "advertising\t0x%x\n"
                        "eeprom_len\t0x%x\n"
                        "cur_page\t0x%x\n"
                        "bios_setting\t0x%x\n"
                        "features\t0x%x\n"
                        "org_pci_offset_99\t0x%x\n"
                        "org_pci_offset_180\t0x%x\n"
                        "issue_offset_99_event\t0x%x\n"
                        "org_pci_offset_80\t0x%x\n"
                        "org_pci_offset_81\t0x%x\n"
                        "use_timer_interrrupt\t0x%x\n"
                        "HwIcVerUnknown\t0x%x\n"
                        "NotWrRamCodeToMicroP\t0x%x\n"
                        "NotWrMcuPatchCode\t0x%x\n"
                        "HwHasWrRamCodeToMicroP\t0x%x\n"
                        "sw_ram_code_ver\t0x%x\n"
                        "hw_ram_code_ver\t0x%x\n"
                        "rtk_enable_diag\t0x%x\n"
                        "ShortPacketSwChecksum\t0x%x\n"
                        "UseSwPaddingShortPkt\t0x%x\n"
                        "RequireAdcBiasPatch\t0x%x\n"
                        "AdcBiasPatchIoffset\t0x%x\n"
                        "RequireAdjustUpsTxLinkPulseTiming\t0x%x\n"
                        "SwrCnt1msIni\t0x%x\n"
                        "HwSuppNowIsOobVer\t0x%x\n"
                        "HwFiberModeVer\t0x%x\n"
                        "HwFiberStat\t0x%x\n"
                        "HwSwitchMdiToFiber\t0x%x\n"
                        "HwSuppSerDesPhyVer\t0x%x\n"
                        "NicCustLedValue\t0x%x\n"
                        "RequiredSecLanDonglePatch\t0x%x\n"
                        "HwSuppDashVer\t0x%x\n"
                        "DASH\t0x%x\n"
                        "dash_printer_enabled\t0x%x\n"
                        "HwSuppKCPOffloadVer\t0x%x\n"
                        "speed_mode\t0x%x\n"
                        "duplex_mode\t0x%x\n"
                        "autoneg_mode\t0x%x\n"
                        "advertising_mode\t0x%x\n"
                        "aspm\t0x%x\n"
                        "s5wol\t0x%x\n"
                        "s5_keep_curr_mac\t0x%x\n"
                        "eee_enable\t0x%x\n"
                        "hwoptimize\t0x%lx\n"
                        "proc_init_num\t0x%x\n"
                        "s0_magic_packet\t0x%x\n"
                        "HwSuppMagicPktVer\t0x%x\n"
                        "HwSuppUpsVer\t0x%x\n"
                        "HwSuppEsdVer\t0x%x\n"
                        "HwSuppCheckPhyDisableModeVer\t0x%x\n"
                        "HwPkgDet\t0x%x\n"
                        "random_mac\t0x%x\n"
                        "org_mac_addr\t%pM\n"
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
                        "perm_addr\t%pM\n"
#endif
                        "dev_addr\t%pM\n",
                        MODULENAME,
                        RTL8168_VERSION,
                        tp->chipset,
                        rtl_chip_info[tp->chipset].name,
                        dev->mtu,
                        NUM_RX_DESC,
                        tp->cur_rx,
                        tp->dirty_rx,
                        NUM_TX_DESC,
                        tp->cur_tx,
                        tp->dirty_tx,
                        tp->rx_buf_sz,
                        tp->esd_flag,
                        tp->pci_cfg_is_read,
                        tp->rtl8168_rx_config,
                        tp->cp_cmd,
                        tp->intr_mask,
                        tp->timer_intr_mask,
                        tp->wol_enabled,
                        tp->wol_opts,
                        tp->efuse_ver,
                        tp->eeprom_type,
                        tp->autoneg,
                        tp->duplex,
                        tp->speed,
                        tp->advertising,
                        tp->eeprom_len,
                        tp->cur_page,
                        tp->bios_setting,
                        tp->features,
                        tp->org_pci_offset_99,
                        tp->org_pci_offset_180,
                        tp->issue_offset_99_event,
                        tp->org_pci_offset_80,
                        tp->org_pci_offset_81,
                        tp->use_timer_interrrupt,
                        tp->HwIcVerUnknown,
                        tp->NotWrRamCodeToMicroP,
                        tp->NotWrMcuPatchCode,
                        tp->HwHasWrRamCodeToMicroP,
                        tp->sw_ram_code_ver,
                        tp->hw_ram_code_ver,
                        tp->rtk_enable_diag,
                        tp->ShortPacketSwChecksum,
                        tp->UseSwPaddingShortPkt,
                        tp->RequireAdcBiasPatch,
                        tp->AdcBiasPatchIoffset,
                        tp->RequireAdjustUpsTxLinkPulseTiming,
                        tp->SwrCnt1msIni,
                        tp->HwSuppNowIsOobVer,
                        tp->HwFiberModeVer,
                        tp->HwFiberStat,
                        tp->HwSwitchMdiToFiber,
                        tp->HwSuppSerDesPhyVer,
                        tp->NicCustLedValue,
                        tp->RequiredSecLanDonglePatch,
                        tp->HwSuppDashVer,
                        tp->DASH,
                        tp->dash_printer_enabled,
                        tp->HwSuppKCPOffloadVer,
                        speed_mode,
                        duplex_mode,
                        autoneg_mode,
                        advertising_mode,
                        aspm,
                        s5wol,
                        s5_keep_curr_mac,
                        tp->eee_enabled,
                        hwoptimize,
                        proc_init_num,
                        s0_magic_packet,
                        tp->HwSuppMagicPktVer,
                        tp->HwSuppUpsVer,
                        tp->HwSuppEsdVer,
                        tp->HwSuppCheckPhyDisableModeVer,
                        tp->HwPkgDet,
                        tp->random_mac,
                        tp->org_mac_addr,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
                        dev->perm_addr,
#endif
                        dev->dev_addr
                       );
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len, "\n");

        *eof = 1;
        return len;
}

static int proc_get_tally_counter(char *page, char **start,
                                  off_t offset, int count,
                                  int *eof, void *data)
{
        struct net_device *dev = data;
        struct rtl8168_private *tp = netdev_priv(dev);
        struct rtl8168_counters *counters;
        dma_addr_t paddr;
        u32 cmd;
        u32 WaitCnt;
        unsigned long flags;
        int len = 0;

        len += snprintf(page + len, count - len,
                        "\nDump Tally Counter\n");

        //ASSERT_RTNL();

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters) {
                len += snprintf(page + len, count - len,
                                "\nDump Tally Counter Fail\n");
                goto out;
        }

        spin_lock_irqsave(&tp->lock, flags);
        RTL_W32(tp, CounterAddrHigh, (u64)paddr >> 32);
        cmd = (u64)paddr & DMA_BIT_MASK(32);
        RTL_W32(tp, CounterAddrLow, cmd);
        RTL_W32(tp, CounterAddrLow, cmd | CounterDump);

        WaitCnt = 0;
        while (RTL_R32(tp, CounterAddrLow) & CounterDump) {
                udelay(10);

                WaitCnt++;
                if (WaitCnt > 20)
                        break;
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len,
                        "Statistics\tValue\n----------\t-----\n");

        len += snprintf(page + len, count - len,
                        "tx_packets\t%lld\n"
                        "rx_packets\t%lld\n"
                        "tx_errors\t%lld\n"
                        "rx_missed\t%lld\n"
                        "align_errors\t%lld\n"
                        "tx_one_collision\t%lld\n"
                        "tx_multi_collision\t%lld\n"
                        "rx_unicast\t%lld\n"
                        "rx_broadcast\t%lld\n"
                        "rx_multicast\t%lld\n"
                        "tx_aborted\t%lld\n"
                        "tx_underun\t%lld\n",
                        le64_to_cpu(counters->tx_packets),
                        le64_to_cpu(counters->rx_packets),
                        le64_to_cpu(counters->tx_errors),
                        le64_to_cpu(counters->rx_missed),
                        le64_to_cpu(counters->align_errors),
                        le64_to_cpu(counters->tx_one_collision),
                        le64_to_cpu(counters->tx_multi_collision),
                        le64_to_cpu(counters->rx_unicast),
                        le64_to_cpu(counters->rx_broadcast),
                        le64_to_cpu(counters->rx_multicast),
                        le64_to_cpu(counters->tx_aborted),
                        le64_to_cpu(counters->tx_underun)
                       );

        len += snprintf(page + len, count - len, "\n");
out:
        *eof = 1;
        return len;
}

static int proc_get_registers(char *page, char **start,
                              off_t offset, int count,
                              int *eof, void *data)
{
        struct net_device *dev = data;
        int i, n, max = R8168_MAC_REGS_SIZE;
        u8 byte_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;
        unsigned long flags;
        int len = 0;

        len += snprintf(page + len, count - len,
                        "\nDump MAC Registers\n"
                        "Offset\tValue\n------\t-----\n");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                len += snprintf(page + len, count - len,
                                "\n0x%02x:\t",
                                n);

                for (i = 0; i < 16 && n < max; i++, n++) {
                        byte_rd = readb(ioaddr + n);
                        len += snprintf(page + len, count - len,
                                        "%02x ",
                                        byte_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len, "\n");

        *eof = 1;
        return len;
}

static int proc_get_pcie_phy(char *page, char **start,
                             off_t offset, int count,
                             int *eof, void *data)
{
        struct net_device *dev = data;
        int i, n, max = R8168_EPHY_REGS_SIZE/2;
        u16 word_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
        int len = 0;

        len += snprintf(page + len, count - len,
                        "\nDump PCIE PHY\n"
                        "Offset\tValue\n------\t-----\n");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                len += snprintf(page + len, count - len,
                                "\n0x%02x:\t",
                                n);

                for (i = 0; i < 8 && n < max; i++, n++) {
                        word_rd = rtl8168_ephy_read(tp, n);
                        len += snprintf(page + len, count - len,
                                        "%04x ",
                                        word_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len, "\n");

        *eof = 1;
        return len;
}

static int proc_get_eth_phy(char *page, char **start,
                            off_t offset, int count,
                            int *eof, void *data)
{
        struct net_device *dev = data;
        int i, n, max = R8168_PHY_REGS_SIZE/2;
        u16 word_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
        int len = 0;

        len += snprintf(page + len, count - len,
                        "\nDump Ethernet PHY\n"
                        "Offset\tValue\n------\t-----\n");

        spin_lock_irqsave(&tp->lock, flags);
        len += snprintf(page + len, count - len,
                        "\n####################page 0##################\n");
        rtl8168_mdio_write(tp, 0x1f, 0x0000);
        for (n = 0; n < max;) {
                len += snprintf(page + len, count - len,
                                "\n0x%02x:\t",
                                n);

                for (i = 0; i < 8 && n < max; i++, n++) {
                        word_rd = rtl8168_mdio_read(tp, n);
                        len += snprintf(page + len, count - len,
                                        "%04x ",
                                        word_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len, "\n");

        *eof = 1;
        return len;
}

static int proc_get_extended_registers(char *page, char **start,
                                       off_t offset, int count,
                                       int *eof, void *data)
{
        struct net_device *dev = data;
        int i, n, max = R8168_ERI_REGS_SIZE;
        u32 dword_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
        int len = 0;

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
                /* RTL8168B does not support Extend GMAC */
                len += snprintf(page + len, count - len,
                                "\nNot Support Dump Extended Registers\n");

                goto out;
        }

        len += snprintf(page + len, count - len,
                        "\nDump Extended Registers\n"
                        "Offset\tValue\n------\t-----\n");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                len += snprintf(page + len, count - len,
                                "\n0x%02x:\t",
                                n);

                for (i = 0; i < 4 && n < max; i++, n+=4) {
                        dword_rd = rtl8168_eri_read(tp, n, 4, ERIAR_ExGMAC);
                        len += snprintf(page + len, count - len,
                                        "%08x ",
                                        dword_rd);
                }
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len, "\n");
out:
        *eof = 1;
        return len;
}

static int proc_get_pci_registers(char *page, char **start,
                                  off_t offset, int count,
                                  int *eof, void *data)
{
        struct net_device *dev = data;
        int i, n, max = R8168_PCI_REGS_SIZE;
        u32 dword_rd;
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
        int len = 0;

        len += snprintf(page + len, count - len,
                        "\nDump PCI Registers\n"
                        "Offset\tValue\n------\t-----\n");

        spin_lock_irqsave(&tp->lock, flags);
        for (n = 0; n < max;) {
                len += snprintf(page + len, count - len,
                                "\n0x%03x:\t",
                                n);

                for (i = 0; i < 4 && n < max; i++, n+=4) {
                        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
                        len += snprintf(page + len, count - len,
                                        "%08x ",
                                        dword_rd);
                }
        }

        n = 0x110;
        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
        len += snprintf(page + len, count - len,
                        "\n0x%03x:\t%08x ",
                        n,
                        dword_rd);
        n = 0x70c;
        pci_read_config_dword(tp->pci_dev, n, &dword_rd);
        len += snprintf(page + len, count - len,
                        "\n0x%03x:\t%08x ",
                        n,
                        dword_rd);
        spin_unlock_irqrestore(&tp->lock, flags);

        len += snprintf(page + len, count - len, "\n");

        *eof = 1;
        return len;
}
#endif
static void rtl8168_proc_module_init(void)
{
        //create /proc/net/r8168
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
        rtl8168_proc = proc_mkdir(MODULENAME, init_net.proc_net);
#else
        rtl8168_proc = proc_mkdir(MODULENAME, proc_net);
#endif
        if (!rtl8168_proc)
                dprintk("cannot create %s proc entry \n", MODULENAME);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
/*
 * seq_file wrappers for procfile show routines.
 */
static int rtl8168_proc_open(struct inode *inode, struct file *file)
{
        struct net_device *dev = proc_get_parent_data(inode);
        int (*show)(struct seq_file *, void *) = PDE_DATA(inode);

        return single_open(file, show, dev);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static const struct proc_ops rtl8168_proc_fops = {
        .proc_open           = rtl8168_proc_open,
        .proc_read           = seq_read,
        .proc_lseek          = seq_lseek,
        .proc_release        = single_release,
};
#else
static const struct file_operations rtl8168_proc_fops = {
        .open           = rtl8168_proc_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};
#endif

#endif

/*
 * Table of proc files we need to create.
 */
struct rtl8168_proc_file {
        char name[12];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        int (*show)(struct seq_file *, void *);
#else
        int (*show)(char *, char **, off_t, int, int *, void *);
#endif
};

static const struct rtl8168_proc_file rtl8168_proc_files[] = {
        { "driver_var", &proc_get_driver_variable },
        { "tally", &proc_get_tally_counter },
        { "registers", &proc_get_registers },
        { "pcie_phy", &proc_get_pcie_phy },
        { "eth_phy", &proc_get_eth_phy },
        { "ext_regs", &proc_get_extended_registers },
        { "pci_regs", &proc_get_pci_registers },
        { "" }
};

static void rtl8168_proc_init(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        const struct rtl8168_proc_file *f;
        struct proc_dir_entry *dir;

        if (rtl8168_proc && !tp->proc_dir) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
                dir = proc_mkdir_data(dev->name, 0, rtl8168_proc, dev);
                if (!dir) {
                        printk("Unable to initialize /proc/net/%s/%s\n",
                               MODULENAME, dev->name);
                        return;
                }

                tp->proc_dir = dir;
                proc_init_num++;

                for (f = rtl8168_proc_files; f->name[0]; f++) {
                        if (!proc_create_data(f->name, S_IFREG | S_IRUGO, dir,
                                              &rtl8168_proc_fops, f->show)) {
                                printk("Unable to initialize "
                                       "/proc/net/%s/%s/%s\n",
                                       MODULENAME, dev->name, f->name);
                                return;
                        }
                }
#else
                dir = proc_mkdir(dev->name, rtl8168_proc);
                if (!dir) {
                        printk("Unable to initialize /proc/net/%s/%s\n",
                               MODULENAME, dev->name);
                        return;
                }

                tp->proc_dir = dir;
                proc_init_num++;

                for (f = rtl8168_proc_files; f->name[0]; f++) {
                        if (!create_proc_read_entry(f->name, S_IFREG | S_IRUGO,
                                                    dir, f->show, dev)) {
                                printk("Unable to initialize "
                                       "/proc/net/%s/%s/%s\n",
                                       MODULENAME, dev->name, f->name);
                                return;
                        }
                }
#endif
        }
}

static void rtl8168_proc_remove(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (tp->proc_dir) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
                remove_proc_subtree(dev->name, rtl8168_proc);
                proc_init_num--;

#else
                const struct rtl8168_proc_file *f;
                struct rtl8168_private *tp = netdev_priv(dev);

                for (f = rtl8168_proc_files; f->name[0]; f++)
                        remove_proc_entry(f->name, tp->proc_dir);

                remove_proc_entry(dev->name, rtl8168_proc);
                proc_init_num--;
#endif
                tp->proc_dir = NULL;
        }
}

#endif //ENABLE_R8168_PROCFS

static inline u16 map_phy_ocp_addr(u16 PageNum, u8 RegNum)
{
        u16 OcpPageNum = 0;
        u8 OcpRegNum = 0;
        u16 OcpPhyAddress = 0;

        if ( PageNum == 0 ) {
                OcpPageNum = OCP_STD_PHY_BASE_PAGE + ( RegNum / 8 );
                OcpRegNum = 0x10 + ( RegNum % 8 );
        } else {
                OcpPageNum = PageNum;
                OcpRegNum = RegNum;
        }

        OcpPageNum <<= 4;

        if ( OcpRegNum < 16 ) {
                OcpPhyAddress = 0;
        } else {
                OcpRegNum -= 16;
                OcpRegNum <<= 1;

                OcpPhyAddress = OcpPageNum + OcpRegNum;
        }


        return OcpPhyAddress;
}

static void mdio_real_direct_write_phy_ocp(struct rtl8168_private *tp,
                u32 RegAddr,
                u32 value)
{
        u32 data32;
        int i;

        if (tp->HwSuppPhyOcpVer == 0) goto out;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(RegAddr % 2);
#endif
        data32 = RegAddr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 |= OCPR_Write | value;

        RTL_W32(tp, PHYOCP, data32);
        for (i = 0; i < 100; i++) {
                udelay(1);

                if (!(RTL_R32(tp, PHYOCP) & OCPR_Flag))
                        break;
        }
out:
        return;
}

static void mdio_direct_write_phy_ocp(struct rtl8168_private *tp,
                                      u16 RegAddr,
                                      u16 value)
{
        if (tp->rtk_enable_diag) return;

        mdio_real_direct_write_phy_ocp(tp, RegAddr, value);
}

static void rtl8168_mdio_write_phy_ocp(struct rtl8168_private *tp,
                                       u16 PageNum,
                                       u32 RegAddr,
                                       u32 value)
{
        u16 ocp_addr;

        if (tp->rtk_enable_diag) return;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        mdio_direct_write_phy_ocp(tp, ocp_addr, value);
}

static void rtl8168_mdio_real_write_phy_ocp(struct rtl8168_private *tp,
                u16 PageNum,
                u32 RegAddr,
                u32 value)
{
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        mdio_real_direct_write_phy_ocp(tp, ocp_addr, value);
}

static void mdio_real_write(struct rtl8168_private *tp,
                            u32 RegAddr,
                            u32 value)
{
        int i;

        if (RegAddr == 0x1F) {
                tp->cur_page = value;
        }

        if (tp->mcfg == CFG_METHOD_11) {
                RTL_W32(tp, OCPDR, OCPDR_Write |
                        (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift |
                        (value & OCPDR_Data_Mask));
                RTL_W32(tp, OCPAR, OCPAR_GPHY_Write);
                RTL_W32(tp, EPHY_RXER_NUM, 0);

                for (i = 0; i < 100; i++) {
                        mdelay(1);
                        if (!(RTL_R32(tp, OCPAR) & OCPAR_Flag))
                                break;
                }
        } else {
                if (tp->HwSuppPhyOcpVer > 0) {
                        if (RegAddr == 0x1F) {
                                return;
                        }
                        rtl8168_mdio_real_write_phy_ocp(tp, tp->cur_page, RegAddr, value);
                } else {
                        if (tp->mcfg == CFG_METHOD_12 || tp->mcfg == CFG_METHOD_13)
                                RTL_W32(tp, 0xD0, RTL_R32(tp, 0xD0) & ~0x00020000);

                        RTL_W32(tp, PHYAR, PHYAR_Write |
                                (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift |
                                (value & PHYAR_Data_Mask));

                        for (i = 0; i < 10; i++) {
                                udelay(100);

                                /* Check if the RTL8168 has completed writing to the specified MII register */
                                if (!(RTL_R32(tp, PHYAR) & PHYAR_Flag)) {
                                        udelay(20);
                                        break;
                                }
                        }

                        if (tp->mcfg == CFG_METHOD_12 || tp->mcfg == CFG_METHOD_13)
                                RTL_W32(tp, 0xD0, RTL_R32(tp, 0xD0) | 0x00020000);
                }
        }
}

void rtl8168_mdio_write(struct rtl8168_private *tp,
                        u16 RegAddr,
                        u16 value)
{
        if (tp->rtk_enable_diag) return;

        mdio_real_write(tp, RegAddr, value);
}

void rtl8168_mdio_prot_write(struct rtl8168_private *tp,
                             u32 RegAddr,
                             u32 value)
{
        mdio_real_write(tp, RegAddr, value);
}

void rtl8168_mdio_prot_direct_write_phy_ocp(struct rtl8168_private *tp,
                u32 RegAddr,
                u32 value)
{
        mdio_real_direct_write_phy_ocp(tp, RegAddr, value);
}

static u32 mdio_real_direct_read_phy_ocp(struct rtl8168_private *tp,
                u32 RegAddr)
{
        u32 data32;
        int i, value = 0;

        if (tp->HwSuppPhyOcpVer == 0) goto out;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(RegAddr % 2);
#endif
        data32 = RegAddr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        RTL_W32(tp, PHYOCP, data32);
        for (i = 0; i < 100; i++) {
                udelay(1);

                if (RTL_R32(tp, PHYOCP) & OCPR_Flag)
                        break;
        }
        value = RTL_R32(tp, PHYOCP) & OCPDR_Data_Mask;

out:
        return value;
}

static u32 mdio_direct_read_phy_ocp(struct rtl8168_private *tp,
                                    u16 RegAddr)
{
        if (tp->rtk_enable_diag) return 0xffffffff;

        return mdio_real_direct_read_phy_ocp(tp, RegAddr);
}

static u32 rtl8168_mdio_read_phy_ocp(struct rtl8168_private *tp,
                                     u16 PageNum,
                                     u32 RegAddr)
{
        u16 ocp_addr;

        if (tp->rtk_enable_diag) return 0xffffffff;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        return mdio_direct_read_phy_ocp(tp, ocp_addr);
}

static u32 rtl8168_mdio_real_read_phy_ocp(struct rtl8168_private *tp,
                u16 PageNum,
                u32 RegAddr)
{
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        return mdio_real_direct_read_phy_ocp(tp, ocp_addr);
}

u32 mdio_real_read(struct rtl8168_private *tp,
                   u32 RegAddr)
{
        int i, value = 0;

        if (tp->mcfg==CFG_METHOD_11) {
                RTL_W32(tp, OCPDR, OCPDR_Read |
                        (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift);
                RTL_W32(tp, OCPAR, OCPAR_GPHY_Write);
                RTL_W32(tp, EPHY_RXER_NUM, 0);

                for (i = 0; i < 100; i++) {
                        mdelay(1);
                        if (!(RTL_R32(tp, OCPAR) & OCPAR_Flag))
                                break;
                }

                mdelay(1);
                RTL_W32(tp, OCPAR, OCPAR_GPHY_Read);
                RTL_W32(tp, EPHY_RXER_NUM, 0);

                for (i = 0; i < 100; i++) {
                        mdelay(1);
                        if (RTL_R32(tp, OCPAR) & OCPAR_Flag)
                                break;
                }

                value = RTL_R32(tp, OCPDR) & OCPDR_Data_Mask;
        } else {
                if (tp->HwSuppPhyOcpVer > 0) {
                        value = rtl8168_mdio_real_read_phy_ocp(tp, tp->cur_page, RegAddr);
                } else {
                        if (tp->mcfg == CFG_METHOD_12 || tp->mcfg == CFG_METHOD_13)
                                RTL_W32(tp, 0xD0, RTL_R32(tp, 0xD0) & ~0x00020000);

                        RTL_W32(tp, PHYAR,
                                PHYAR_Read | (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift);

                        for (i = 0; i < 10; i++) {
                                udelay(100);

                                /* Check if the RTL8168 has completed retrieving data from the specified MII register */
                                if (RTL_R32(tp, PHYAR) & PHYAR_Flag) {
                                        value = RTL_R32(tp, PHYAR) & PHYAR_Data_Mask;
                                        udelay(20);
                                        break;
                                }
                        }

                        if (tp->mcfg == CFG_METHOD_12 || tp->mcfg == CFG_METHOD_13)
                                RTL_W32(tp, 0xD0, RTL_R32(tp, 0xD0) | 0x00020000);
                }
        }

        return value;
}

u32 rtl8168_mdio_read(struct rtl8168_private *tp,
                      u16 RegAddr)
{
        if (tp->rtk_enable_diag) return 0xffffffff;

        return mdio_real_read(tp, RegAddr);
}

u32 rtl8168_mdio_prot_read(struct rtl8168_private *tp,
                           u32 RegAddr)
{
        return mdio_real_read(tp, RegAddr);
}

u32 rtl8168_mdio_prot_direct_read_phy_ocp(struct rtl8168_private *tp,
                u32 RegAddr)
{
        return mdio_real_direct_read_phy_ocp(tp, RegAddr);
}

static void ClearAndSetEthPhyBit(struct rtl8168_private *tp, u8  addr, u16 clearmask, u16 setmask)
{
        u16 PhyRegValue;


        PhyRegValue = rtl8168_mdio_read(tp, addr);
        PhyRegValue &= ~clearmask;
        PhyRegValue |= setmask;
        rtl8168_mdio_write(tp, addr, PhyRegValue);
}

void rtl8168_clear_eth_phy_bit(struct rtl8168_private *tp, u8 addr, u16 mask)
{
        ClearAndSetEthPhyBit(tp,
                             addr,
                             mask,
                             0
                            );
}

void rtl8168_set_eth_phy_bit(struct rtl8168_private *tp,  u8  addr, u16  mask)
{
        ClearAndSetEthPhyBit(tp,
                             addr,
                             0,
                             mask
                            );
}

void rtl8168_mac_ocp_write(struct rtl8168_private *tp, u16 reg_addr, u16 value)
{
        u32 data32;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(reg_addr % 2);
#endif

        data32 = reg_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 += value;
        data32 |= OCPR_Write;

        RTL_W32(tp, MACOCP, data32);
}

u16 rtl8168_mac_ocp_read(struct rtl8168_private *tp, u16 reg_addr)
{
        u32 data32;
        u16 data16 = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(reg_addr % 2);
#endif

        data32 = reg_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        RTL_W32(tp, MACOCP, data32);
        data16 = (u16)RTL_R32(tp, MACOCP);

        return data16;
}

#ifdef ENABLE_USE_FIRMWARE_FILE
static void mac_mcu_write(struct rtl8168_private *tp, u16 reg, u16 value)
{
        if (reg == 0x1f) {
                tp->ocp_base = value << 4;
                return;
        }

        rtl8168_mac_ocp_write(tp, tp->ocp_base + reg, value);
}

static u32 mac_mcu_read(struct rtl8168_private *tp, u16 reg)
{
        return rtl8168_mac_ocp_read(tp, tp->ocp_base + reg);
}
#endif

static void
rtl8168_clear_and_set_mcu_ocp_bit(
        struct rtl8168_private *tp,
        u16   addr,
        u16   clearmask,
        u16   setmask
)
{
        u16 RegValue;

        RegValue = rtl8168_mac_ocp_read(tp, addr);
        RegValue &= ~clearmask;
        RegValue |= setmask;
        rtl8168_mac_ocp_write(tp, addr, RegValue);
}

/*
static void
rtl8168_clear_mcu_ocp_bit(
        struct rtl8168_private *tp,
        u16   addr,
        u16   mask
)
{
        rtl8168_clear_and_set_mcu_ocp_bit(tp,
                                          addr,
                                          mask,
                                          0
                                         );
}
*/

static void
rtl8168_set_mcu_ocp_bit(
        struct rtl8168_private *tp,
        u16   addr,
        u16   mask
)
{
        rtl8168_clear_and_set_mcu_ocp_bit(tp,
                                          addr,
                                          0,
                                          mask
                                         );
}

static u32 real_ocp_read(struct rtl8168_private *tp, u16 addr, u8 len)
{
        int i, val_shift, shift = 0;
        u32 value1 = 0, value2 = 0, mask;

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % 4;
                addr = addr & ~0x3;

                RTL_W32(tp, OCPAR, (0x0F<<12) | (addr&0xFFF));

                for (i = 0; i < 20; i++) {
                        udelay(100);
                        if (RTL_R32(tp, OCPAR) & OCPAR_Flag)
                                break;
                }

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = RTL_R32(tp, OCPDR) & mask;
                value2 |= (value1 >> val_shift * 8) << shift * 8;

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        udelay(20);

        return value2;
}

u32 rtl8168_ocp_read_with_oob_base_address(struct rtl8168_private *tp, u16 addr, u8 len, const u32 base_address)
{
        return rtl8168_eri_read_with_oob_base_address(tp, addr, len, ERIAR_OOB, base_address);
}

u32 rtl8168_ocp_read(struct rtl8168_private *tp, u16 addr, u8 len)
{
        u32 value = 0;

        if (HW_DASH_SUPPORT_TYPE_2(tp))
                value = rtl8168_ocp_read_with_oob_base_address(tp, addr, len, NO_BASE_ADDRESS);
        else if (HW_DASH_SUPPORT_TYPE_3(tp))
                value = rtl8168_ocp_read_with_oob_base_address(tp, addr, len, RTL8168FP_OOBMAC_BASE);
        else
                value = real_ocp_read(tp, addr, len);

        return value;
}

static int real_ocp_write(struct rtl8168_private *tp, u16 addr, u8 len, u32 value)
{
        int i, val_shift, shift = 0;
        u32 value1 = 0, mask;

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % 4;
                addr = addr & ~0x3;

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = rtl8168_ocp_read(tp, addr, 4) & ~mask;
                value1 |= ((value << val_shift * 8) >> shift * 8);

                RTL_W32(tp, OCPDR, value1);
                RTL_W32(tp, OCPAR, OCPAR_Flag | (0x0F<<12) | (addr&0xFFF));

                for (i = 0; i < 10; i++) {
                        udelay(100);

                        /* Check if the RTL8168 has completed ERI write */
                        if (!(RTL_R32(tp, OCPAR) & OCPAR_Flag))
                                break;
                }

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        udelay(20);

        return 0;
}

u32 rtl8168_ocp_write_with_oob_base_address(struct rtl8168_private *tp, u16 addr, u8 len, u32 value, const u32 base_address)
{
        return rtl8168_eri_write_with_oob_base_address(tp, addr, len, value, ERIAR_OOB, base_address);
}

void rtl8168_ocp_write(struct rtl8168_private *tp, u16 addr, u8 len, u32 value)
{
        if (HW_DASH_SUPPORT_TYPE_2(tp))
                rtl8168_ocp_write_with_oob_base_address(tp, addr, len, value, NO_BASE_ADDRESS);
        else if (HW_DASH_SUPPORT_TYPE_3(tp))
                rtl8168_ocp_write_with_oob_base_address(tp, addr, len, value, RTL8168FP_OOBMAC_BASE);
        else
                real_ocp_write(tp, addr, len, value);
}

void rtl8168_oob_mutex_lock(struct rtl8168_private *tp)
{
        u8 reg_16, reg_a0;
        u32 wait_cnt_0, wait_Cnt_1;
        u16 ocp_reg_mutex_ib;
        u16 ocp_reg_mutex_oob;
        u16 ocp_reg_mutex_prio;

        if (!tp->DASH) return;

        switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
                ocp_reg_mutex_oob = 0x16;
                ocp_reg_mutex_ib = 0x17;
                ocp_reg_mutex_prio = 0x9C;
                break;
        case CFG_METHOD_13:
                ocp_reg_mutex_oob = 0x06;
                ocp_reg_mutex_ib = 0x07;
                ocp_reg_mutex_prio = 0x9C;
                break;
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
        default:
                ocp_reg_mutex_oob = 0x110;
                ocp_reg_mutex_ib = 0x114;
                ocp_reg_mutex_prio = 0x11C;
                break;
        }

        rtl8168_ocp_write(tp, ocp_reg_mutex_ib, 1, BIT_0);
        reg_16 = rtl8168_ocp_read(tp, ocp_reg_mutex_oob, 1);
        wait_cnt_0 = 0;
        while(reg_16) {
                reg_a0 = rtl8168_ocp_read(tp, ocp_reg_mutex_prio, 1);
                if (reg_a0) {
                        rtl8168_ocp_write(tp, ocp_reg_mutex_ib, 1, 0x00);
                        reg_a0 = rtl8168_ocp_read(tp, ocp_reg_mutex_prio, 1);
                        wait_Cnt_1 = 0;
                        while(reg_a0) {
                                reg_a0 = rtl8168_ocp_read(tp, ocp_reg_mutex_prio, 1);

                                wait_Cnt_1++;

                                if (wait_Cnt_1 > 2000)
                                        break;
                        };
                        rtl8168_ocp_write(tp, ocp_reg_mutex_ib, 1, BIT_0);

                }
                reg_16 = rtl8168_ocp_read(tp, ocp_reg_mutex_oob, 1);

                wait_cnt_0++;

                if (wait_cnt_0 > 2000)
                        break;
        };
}

void rtl8168_oob_mutex_unlock(struct rtl8168_private *tp)
{
        u16 ocp_reg_mutex_ib;
        u16 ocp_reg_mutex_oob;
        u16 ocp_reg_mutex_prio;

        if (!tp->DASH) return;

        switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
                ocp_reg_mutex_oob = 0x16;
                ocp_reg_mutex_ib = 0x17;
                ocp_reg_mutex_prio = 0x9C;
                break;
        case CFG_METHOD_13:
                ocp_reg_mutex_oob = 0x06;
                ocp_reg_mutex_ib = 0x07;
                ocp_reg_mutex_prio = 0x9C;
                break;
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
        default:
                ocp_reg_mutex_oob = 0x110;
                ocp_reg_mutex_ib = 0x114;
                ocp_reg_mutex_prio = 0x11C;
                break;
        }

        rtl8168_ocp_write(tp, ocp_reg_mutex_prio, 1, BIT_0);
        rtl8168_ocp_write(tp, ocp_reg_mutex_ib, 1, 0x00);
}

void rtl8168_oob_notify(struct rtl8168_private *tp, u8 cmd)
{
        rtl8168_eri_write(tp, 0xE8, 1, cmd, ERIAR_ExGMAC);

        rtl8168_ocp_write(tp, 0x30, 1, 0x01);
}

static int rtl8168_check_dash(struct rtl8168_private *tp)
{
        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                if (rtl8168_ocp_read(tp, 0x128, 1) & BIT_0)
                        return 1;
                else
                        return 0;
        } else {
                u32 reg;

                if (tp->mcfg == CFG_METHOD_13)
                        reg = 0xb8;
                else
                        reg = 0x10;

                if (rtl8168_ocp_read(tp, reg, 2) & 0x00008000)
                        return 1;
                else
                        return 0;
        }
}

void rtl8168_dash2_disable_tx(struct rtl8168_private *tp)
{
        if (!tp->DASH) return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                u16 WaitCnt;
                u8 TmpUchar;

                //Disable oob Tx
                RTL_CMAC_W8(tp, CMAC_IBCR2, RTL_CMAC_R8(tp, CMAC_IBCR2) & ~( BIT_0 ));
                WaitCnt = 0;

                //wait oob tx disable
                do {
                        TmpUchar = RTL_CMAC_R8(tp, CMAC_IBISR0);

                        if ( TmpUchar & ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE ) {
                                break;
                        }

                        udelay( 50 );
                        WaitCnt++;
                } while(WaitCnt < 2000);

                //Clear ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE
                RTL_CMAC_W8(tp, CMAC_IBISR0, RTL_CMAC_R8(tp, CMAC_IBISR0) | ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE);
        }
}

void rtl8168_dash2_enable_tx(struct rtl8168_private *tp)
{
        if (!tp->DASH) return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                RTL_CMAC_W8(tp, CMAC_IBCR2, RTL_CMAC_R8(tp, CMAC_IBCR2) | BIT_0);
        }
}

void rtl8168_dash2_disable_rx(struct rtl8168_private *tp)
{
        if (!tp->DASH) return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                RTL_CMAC_W8(tp, CMAC_IBCR0, RTL_CMAC_R8(tp, CMAC_IBCR0) & ~( BIT_0 ));
        }
}

void rtl8168_dash2_enable_rx(struct rtl8168_private *tp)
{
        if (!tp->DASH) return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                RTL_CMAC_W8(tp, CMAC_IBCR0, RTL_CMAC_R8(tp, CMAC_IBCR0) | BIT_0);
        }
}

static void rtl8168_dash2_disable_txrx(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                rtl8168_dash2_disable_tx( tp );
                rtl8168_dash2_disable_rx( tp );
        }
}

void rtl8168_ephy_write(struct rtl8168_private *tp, int RegAddr, int value)
{
        int i;

        RTL_W32(tp, EPHYAR,
                EPHYAR_Write |
                (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift |
                (value & EPHYAR_Data_Mask));

        for (i = 0; i < 10; i++) {
                udelay(100);

                /* Check if the RTL8168 has completed EPHY write */
                if (!(RTL_R32(tp, EPHYAR) & EPHYAR_Flag))
                        break;
        }

        udelay(20);
}

u16 rtl8168_ephy_read(struct rtl8168_private *tp, int RegAddr)
{
        int i;
        u16 value = 0xffff;

        RTL_W32(tp, EPHYAR,
                EPHYAR_Read | (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift);

        for (i = 0; i < 10; i++) {
                udelay(100);

                /* Check if the RTL8168 has completed EPHY read */
                if (RTL_R32(tp, EPHYAR) & EPHYAR_Flag) {
                        value = (u16) (RTL_R32(tp, EPHYAR) & EPHYAR_Data_Mask);
                        break;
                }
        }

        udelay(20);

        return value;
}

static void ClearAndSetPCIePhyBit(struct rtl8168_private *tp, u8 addr, u16 clearmask, u16 setmask)
{
        u16 EphyValue;

        EphyValue = rtl8168_ephy_read(tp, addr);
        EphyValue &= ~clearmask;
        EphyValue |= setmask;
        rtl8168_ephy_write(tp, addr, EphyValue);
}

static void ClearPCIePhyBit(struct rtl8168_private *tp, u8 addr, u16 mask)
{
        ClearAndSetPCIePhyBit( tp,
                               addr,
                               mask,
                               0
                             );
}

static void SetPCIePhyBit( struct rtl8168_private *tp, u8 addr, u16 mask)
{
        ClearAndSetPCIePhyBit( tp,
                               addr,
                               0,
                               mask
                             );
}

static u32
rtl8168_csi_other_fun_read(struct rtl8168_private *tp,
                           u8 multi_fun_sel_bit,
                           u32 addr)
{
        u32 cmd;
        int i;
        u32 value = 0;

        cmd = CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);

        if (tp->mcfg != CFG_METHOD_20 && tp->mcfg != CFG_METHOD_23 &&
            tp->mcfg != CFG_METHOD_26 && tp->mcfg != CFG_METHOD_27 &&
            tp->mcfg != CFG_METHOD_28 && tp->mcfg != CFG_METHOD_31 &&
            tp->mcfg != CFG_METHOD_32 && tp->mcfg != CFG_METHOD_33) {
                multi_fun_sel_bit = 0;
        }

        if ( multi_fun_sel_bit > 7 ) {
                return 0xffffffff;
        }

        cmd |= multi_fun_sel_bit << 16;

        RTL_W32(tp, CSIAR, cmd);

        for (i = 0; i < 10; i++) {
                udelay(100);

                /* Check if the RTL8168 has completed CSI read */
                if (RTL_R32(tp, CSIAR) & CSIAR_Flag) {
                        value = (u32)RTL_R32(tp, CSIDR);
                        break;
                }
        }

        udelay(20);

        return value;
}

static void
rtl8168_csi_other_fun_write(struct rtl8168_private *tp,
                            u8 multi_fun_sel_bit,
                            u32 addr,
                            u32 value)
{
        u32 cmd;
        int i;

        RTL_W32(tp, CSIDR, value);
        cmd = CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);
        if (tp->mcfg != CFG_METHOD_20 && tp->mcfg != CFG_METHOD_23 &&
            tp->mcfg != CFG_METHOD_26 && tp->mcfg != CFG_METHOD_27 &&
            tp->mcfg != CFG_METHOD_28 && tp->mcfg != CFG_METHOD_31 &&
            tp->mcfg != CFG_METHOD_32 && tp->mcfg != CFG_METHOD_33) {
                multi_fun_sel_bit = 0;
        }

        if ( multi_fun_sel_bit > 7 ) {
                return;
        }

        cmd |= multi_fun_sel_bit << 16;

        RTL_W32(tp, CSIAR, cmd);

        for (i = 0; i < 10; i++) {
                udelay(100);

                /* Check if the RTL8168 has completed CSI write */
                if (!(RTL_R32(tp, CSIAR) & CSIAR_Flag))
                        break;
        }

        udelay(20);
}

static u32
rtl8168_csi_read(struct rtl8168_private *tp,
                 u32 addr)
{
        u8 multi_fun_sel_bit;

        if (tp->mcfg == CFG_METHOD_20)
                multi_fun_sel_bit = 2;
        else if (tp->mcfg == CFG_METHOD_26 || tp->mcfg == CFG_METHOD_31 ||
                 tp->mcfg == CFG_METHOD_32 || tp->mcfg == CFG_METHOD_33)
                multi_fun_sel_bit = 1;
        else
                multi_fun_sel_bit = 0;

        return rtl8168_csi_other_fun_read(tp, multi_fun_sel_bit, addr);
}

static void
rtl8168_csi_write(struct rtl8168_private *tp,
                  u32 addr,
                  u32 value)
{
        u8 multi_fun_sel_bit;

        if (tp->mcfg == CFG_METHOD_20)
                multi_fun_sel_bit = 2;
        else if (tp->mcfg == CFG_METHOD_26 || tp->mcfg == CFG_METHOD_31 ||
                 tp->mcfg == CFG_METHOD_32 || tp->mcfg == CFG_METHOD_33)
                multi_fun_sel_bit = 1;
        else
                multi_fun_sel_bit = 0;

        rtl8168_csi_other_fun_write(tp, multi_fun_sel_bit, addr, value);
}

static u8
rtl8168_csi_fun0_read_byte(struct rtl8168_private *tp,
                           u32 addr)
{
        u8 RetVal = 0;

        if (tp->mcfg == CFG_METHOD_20 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                u32 TmpUlong;
                u16 RegAlignAddr;
                u8 ShiftByte;

                RegAlignAddr = addr & ~(0x3);
                ShiftByte = addr & (0x3);
                TmpUlong = rtl8168_csi_other_fun_read(tp, 0, addr);
                TmpUlong >>= (8*ShiftByte);
                RetVal = (u8)TmpUlong;
        } else {
                struct pci_dev *pdev = tp->pci_dev;

                pci_read_config_byte(pdev, addr, &RetVal);
        }

        udelay(20);

        return RetVal;
}

static void
rtl8168_csi_fun0_write_byte(struct rtl8168_private *tp,
                            u32 addr,
                            u8 value)
{
        if (tp->mcfg == CFG_METHOD_20 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                u32 TmpUlong;
                u16 RegAlignAddr;
                u8 ShiftByte;

                RegAlignAddr = addr & ~(0x3);
                ShiftByte = addr & (0x3);
                TmpUlong = rtl8168_csi_other_fun_read(tp, 0, RegAlignAddr);
                TmpUlong &= ~(0xFF << (8*ShiftByte));
                TmpUlong |= (value << (8*ShiftByte));
                rtl8168_csi_other_fun_write( tp, 0, RegAlignAddr, TmpUlong );
        } else {
                struct pci_dev *pdev = tp->pci_dev;

                pci_write_config_byte(pdev, addr, value);
        }

        udelay(20);
}

static void
rtl8168_clear_and_set_other_fun_pci_bit(struct rtl8168_private *tp,
                                        u8 multi_fun_sel_bit,
                                        u32 addr,
                                        u32 clearmask,
                                        u32 setmask)
{
        u32 TmpUlong;

        TmpUlong = rtl8168_csi_other_fun_read(tp, multi_fun_sel_bit, addr);
        TmpUlong &= ~clearmask;
        TmpUlong |= setmask;
        rtl8168_csi_other_fun_write(tp, multi_fun_sel_bit, addr, TmpUlong);
}

static void
rtl8168_other_fun_dev_pci_setting(struct rtl8168_private *tp,
                                  u32 addr,
                                  u32 clearmask,
                                  u32 setmask,
                                  u8 multi_fun_sel_bit)
{
        u32 TmpUlong;
        u8 i;
        u8 FunBit;

        for (i = 0; i < 8; i++) {
                FunBit = (1 << i);
                if (FunBit & multi_fun_sel_bit) {
                        u8 set_other_fun = TRUE;

                        switch(tp->mcfg) {
                        case CFG_METHOD_23:
                        case CFG_METHOD_27:
                        case CFG_METHOD_28:
                                //0: UMAC, 1: TCR1, 2: TCR2, 3: KCS, 4: EHCI(Control by EHCI Driver)
                                if (i < 5) {
                                        TmpUlong = rtl8168_csi_other_fun_read(tp, i, 0x00);
                                        if (TmpUlong == 0xFFFFFFFF)
                                                set_other_fun = TRUE;
                                        else
                                                set_other_fun = FALSE;
                                }
                                break;
                        case CFG_METHOD_31:
                        case CFG_METHOD_32:
                        case CFG_METHOD_33:
                                //0: BMC, 1: NIC, 2: TCR, 3: VGA/PCIE_TO_USB, 4: EHCI, 5: WIFI, 6: WIFI, 7: KCS
                                if (i == 5 || i == 6) {
                                        if (tp->DASH) {
                                                TmpUlong = rtl8168_ocp_read(tp, 0x184, 4);
                                                if (TmpUlong & BIT_26)
                                                        set_other_fun = FALSE;
                                                else
                                                        set_other_fun = TRUE;
                                        }
                                } else { //function 0/1/2/3/4/7
                                        TmpUlong = rtl8168_csi_other_fun_read(tp, i, 0x00);
                                        if (TmpUlong == 0xFFFFFFFF)
                                                set_other_fun = TRUE;
                                        else
                                                set_other_fun = FALSE;
                                }
                                break;
                        default:
                                return;
                        }

                        if (set_other_fun)
                                rtl8168_clear_and_set_other_fun_pci_bit(tp, i, addr, clearmask, setmask);
                }
        }
}

static void
rtl8168_set_dash_other_fun_dev_state_change(struct rtl8168_private *tp,
                u8 dev_state,
                u8 multi_fun_sel_bit)
{
        u32 clearmask;
        u32 setmask;

        if (dev_state == 0) {
                //
                // goto D0
                //
                clearmask = (BIT_0 | BIT_1);
                setmask = 0;

                rtl8168_other_fun_dev_pci_setting(tp, 0x44, clearmask, setmask, multi_fun_sel_bit);
        } else {
                //
                // goto D3
                //
                clearmask = 0;
                setmask = (BIT_0 | BIT_1);

                rtl8168_other_fun_dev_pci_setting(tp, 0x44, clearmask, setmask, multi_fun_sel_bit);
        }
}

static void
rtl8168_set_dash_other_fun_dev_aspm_clkreq(struct rtl8168_private *tp,
                u8 aspm_val,
                u8 clkreq_en,
                u8 multi_fun_sel_bit)
{
        u32 clearmask;
        u32 setmask;

        aspm_val &= (BIT_0 | BIT_1);
        clearmask = (BIT_0 | BIT_1 | BIT_8);
        setmask = aspm_val;
        if (clkreq_en)
                setmask |= BIT_8;

        rtl8168_other_fun_dev_pci_setting(tp, 0x80, clearmask, setmask, multi_fun_sel_bit);
}

/*
static void
rtl8168_set_dash_other_fun_dev_pci_cmd_register(struct rtl8168_private *tp,
                u8 pci_cmd_reg,
                u8 multi_fun_sel_bit)
{
        u32 clearmask;
        u32 setmask;

        pci_cmd_reg &= (BIT_0 | BIT_1 | BIT_2);

        clearmask = (BIT_0 | BIT_1 | BIT_2);
        setmask = pci_cmd_reg;

        rtl8168_other_fun_dev_pci_setting(tp, 0x04, clearmask, setmask, multi_fun_sel_bit);
}
*/

u32 rtl8168_eri_read_with_oob_base_address(struct rtl8168_private *tp, int addr, int len, int type, const u32 base_address)
{
        int i, val_shift, shift = 0;
        u32 value1 = 0, value2 = 0, mask;
        u32 eri_cmd;
        const u32 transformed_base_address = ((base_address & 0x00FFF000) << 6) | (base_address & 0x000FFF);

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % ERIAR_Addr_Align;
                addr = addr & ~0x3;

                eri_cmd = ERIAR_Read |
                          transformed_base_address |
                          type << ERIAR_Type_shift |
                          ERIAR_ByteEn << ERIAR_ByteEn_shift |
                          (addr & 0x0FFF);
                if (addr & 0xF000) {
                        u32 tmp;

                        tmp = addr & 0xF000;
                        tmp >>= 12;
                        eri_cmd |= (tmp << 20) & 0x00F00000;
                }

                RTL_W32(tp, ERIAR, eri_cmd);

                for (i = 0; i < 10; i++) {
                        udelay(100);

                        /* Check if the RTL8168 has completed ERI read */
                        if (RTL_R32(tp, ERIAR) & ERIAR_Flag)
                                break;
                }

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = RTL_R32(tp, ERIDR) & mask;
                value2 |= (value1 >> val_shift * 8) << shift * 8;

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        udelay(20);

        return value2;
}

u32 rtl8168_eri_read(struct rtl8168_private *tp, int addr, int len, int type)
{
        return rtl8168_eri_read_with_oob_base_address(tp, addr, len, type, 0);
}

int rtl8168_eri_write_with_oob_base_address(struct rtl8168_private *tp, int addr, int len, u32 value, int type, const u32 base_address)
{
        int i, val_shift, shift = 0;
        u32 value1 = 0, mask;
        u32 eri_cmd;
        const u32 transformed_base_address = ((base_address & 0x00FFF000) << 6) | (base_address & 0x000FFF);

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % ERIAR_Addr_Align;
                addr = addr & ~0x3;

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = rtl8168_eri_read_with_oob_base_address(tp, addr, 4, type, base_address) & ~mask;
                value1 |= ((value << val_shift * 8) >> shift * 8);

                RTL_W32(tp, ERIDR, value1);

                eri_cmd = ERIAR_Write |
                          transformed_base_address |
                          type << ERIAR_Type_shift |
                          ERIAR_ByteEn << ERIAR_ByteEn_shift |
                          (addr & 0x0FFF);
                if (addr & 0xF000) {
                        u32 tmp;

                        tmp = addr & 0xF000;
                        tmp >>= 12;
                        eri_cmd |= (tmp << 20) & 0x00F00000;
                }

                RTL_W32(tp, ERIAR, eri_cmd);

                for (i = 0; i < 10; i++) {
                        udelay(100);

                        /* Check if the RTL8168 has completed ERI write */
                        if (!(RTL_R32(tp, ERIAR) & ERIAR_Flag))
                                break;
                }

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        udelay(20);

        return 0;
}

int rtl8168_eri_write(struct rtl8168_private *tp, int addr, int len, u32 value, int type)
{
        return rtl8168_eri_write_with_oob_base_address(tp, addr, len, value, type, NO_BASE_ADDRESS);
}

static void
rtl8168_enable_rxdvgate(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_3);
                mdelay(2);
                break;
        }
}

static void
rtl8168_disable_rxdvgate(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_3);
                mdelay(2);
                break;
        }
}

static u8
rtl8168_is_gpio_low(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u8 gpio_low = FALSE;

        switch (tp->HwSuppCheckPhyDisableModeVer) {
        case 1:
        case 2:
                if (!(rtl8168_mac_ocp_read(tp, 0xDC04) & BIT_9))
                        gpio_low = TRUE;
                break;
        case 3:
                if (!(rtl8168_mac_ocp_read(tp, 0xDC04) & BIT_13))
                        gpio_low = TRUE;
                break;
        }

        if (gpio_low)
                dprintk("gpio is low.\n");

        return gpio_low;
}

static u8
rtl8168_is_phy_disable_mode_enabled(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u8 phy_disable_mode_enabled = FALSE;

        switch (tp->HwSuppCheckPhyDisableModeVer) {
        case 1:
                if (rtl8168_mac_ocp_read(tp, 0xDC20) & BIT_1)
                        phy_disable_mode_enabled = TRUE;
                break;
        case 2:
        case 3:
                if (RTL_R8(tp, 0xF2) & BIT_5)
                        phy_disable_mode_enabled = TRUE;
                break;
        }

        if (phy_disable_mode_enabled)
                dprintk("phy disable mode enabled.\n");

        return phy_disable_mode_enabled;
}

static u8
rtl8168_is_in_phy_disable_mode(struct net_device *dev)
{
        u8 in_phy_disable_mode = FALSE;

        if (rtl8168_is_phy_disable_mode_enabled(dev) && rtl8168_is_gpio_low(dev))
                in_phy_disable_mode = TRUE;

        if (in_phy_disable_mode)
                dprintk("Hardware is in phy disable mode.\n");

        return in_phy_disable_mode;
}

void
rtl8168_wait_txrx_fifo_empty(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int i;

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                for (i = 0; i < 10; i++) {
                        udelay(100);
                        if (RTL_R32(tp, TxConfig) & BIT_11)
                                break;
                }

                for (i = 0; i < 10; i++) {
                        udelay(100);
                        if ((RTL_R8(tp, MCUCmd_reg) & (Txfifo_empty | Rxfifo_empty)) == (Txfifo_empty | Rxfifo_empty))
                                break;

                }
                break;
        }
}

static void rtl8168_driver_start(struct rtl8168_private *tp)
{
        //change other device state to D0.
        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                rtl8168_set_dash_other_fun_dev_aspm_clkreq(tp, 3, 1, 0x1E);
                rtl8168_set_dash_other_fun_dev_state_change(tp, 3, 0x1E);
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_set_dash_other_fun_dev_aspm_clkreq(tp, 3, 1, 0xFC);
                rtl8168_set_dash_other_fun_dev_state_change(tp, 3, 0xFC);
                break;
        }

        if (!tp->DASH)
                return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                int timeout;
                u32 tmp_value;

                rtl8168_ocp_write(tp, 0x180, 1, OOB_CMD_DRIVER_START);
                tmp_value = rtl8168_ocp_read(tp, 0x30, 1);
                tmp_value |= BIT_0;
                rtl8168_ocp_write(tp, 0x30, 1, tmp_value);

                for (timeout = 0; timeout < 10; timeout++) {
                        mdelay(10);
                        if (rtl8168_ocp_read(tp, 0x124, 1) & BIT_0)
                                break;
                }
        } else {
                int timeout;
                u32 reg;

                if (tp->mcfg == CFG_METHOD_13) {
                        RTL_W8(tp, TwiCmdReg, RTL_R8(tp, TwiCmdReg) | ( BIT_7 ));
                }

                rtl8168_oob_notify(tp, OOB_CMD_DRIVER_START);

                if (tp->mcfg == CFG_METHOD_13)
                        reg = 0xB8;
                else
                        reg = 0x10;

                for (timeout = 0; timeout < 10; timeout++) {
                        mdelay(10);
                        if (rtl8168_ocp_read(tp, reg, 2) & BIT_11)
                                break;
                }
        }
}

static void rtl8168_driver_stop(struct rtl8168_private *tp)
{
        if (!tp->DASH)
                goto update_device_state;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                struct net_device *dev = tp->dev;
                int timeout;
                u32 tmp_value;

                rtl8168_dash2_disable_txrx(dev);

                rtl8168_ocp_write(tp, 0x180, 1, OOB_CMD_DRIVER_STOP);
                tmp_value = rtl8168_ocp_read(tp, 0x30, 1);
                tmp_value |= BIT_0;
                rtl8168_ocp_write(tp, 0x30, 1, tmp_value);

                for (timeout = 0; timeout < 10; timeout++) {
                        mdelay(10);
                        if (!(rtl8168_ocp_read(tp, 0x124, 1) & BIT_0))
                                break;
                }
        } else {
                int timeout;
                u32 reg;

                rtl8168_oob_notify(tp, OOB_CMD_DRIVER_STOP);

                if (tp->mcfg == CFG_METHOD_13)
                        reg = 0xB8;
                else
                        reg = 0x10;

                for (timeout = 0; timeout < 10; timeout++) {
                        mdelay(10);
                        if ((rtl8168_ocp_read(tp, reg, 2) & BIT_11) == 0)
                                break;
                }

                if (tp->mcfg == CFG_METHOD_13) {
                        RTL_W8(tp, TwiCmdReg, RTL_R8(tp, TwiCmdReg) & ~( BIT_7 ));
                }
        }

update_device_state:
        //change other device state to D3.
        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                rtl8168_set_dash_other_fun_dev_state_change(tp, 3, 0x0E);
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_set_dash_other_fun_dev_state_change(tp, 3, 0xFD);
                break;
        }
}

#ifdef ENABLE_DASH_SUPPORT

inline void
rtl8168_enable_dash2_interrupt(struct rtl8168_private *tp)
{
        if (!tp->DASH) return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                RTL_CMAC_W8(tp, CMAC_IBIMR0, ( ISRIMR_DASH_TYPE2_ROK | ISRIMR_DASH_TYPE2_TOK | ISRIMR_DASH_TYPE2_TDU | ISRIMR_DASH_TYPE2_RDU | ISRIMR_DASH_TYPE2_RX_DISABLE_IDLE ));
        }
}

static inline void
rtl8168_disable_dash2_interrupt(struct rtl8168_private *tp)
{
        if (!tp->DASH) return;

        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                RTL_CMAC_W8(tp, CMAC_IBIMR0, 0);
        }
}
#endif

static inline void
rtl8168_enable_hw_interrupt(struct rtl8168_private *tp)
{
        RTL_W16(tp, IntrMask, tp->intr_mask);

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH)
                rtl8168_enable_dash2_interrupt(tp);
#endif
}

static inline void
rtl8168_disable_hw_interrupt(struct rtl8168_private *tp)
{
        RTL_W16(tp, IntrMask, 0x0000);

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH)
                rtl8168_disable_dash2_interrupt(tp);
#endif
}


static inline void
rtl8168_switch_to_hw_interrupt(struct rtl8168_private *tp)
{
        RTL_W32(tp, TimeInt0, 0x0000);

        rtl8168_enable_hw_interrupt(tp);
}

static inline void
rtl8168_switch_to_timer_interrupt(struct rtl8168_private *tp)
{
        if (tp->use_timer_interrrupt) {
                RTL_W32(tp, TimeInt0, timer_count);
                RTL_W32(tp, TCTR, timer_count);
                RTL_W16(tp, IntrMask, tp->timer_intr_mask);

#ifdef ENABLE_DASH_SUPPORT
                if (tp->DASH)
                        rtl8168_enable_dash2_interrupt(tp);
#endif
        } else {
                rtl8168_switch_to_hw_interrupt(tp);
        }
}

static void
rtl8168_irq_mask_and_ack(struct rtl8168_private *tp)
{
        rtl8168_disable_hw_interrupt(tp);
#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH) {
                if (tp->dash_printer_enabled) {
                        RTL_W16(tp, IntrStatus, RTL_R16(tp, IntrStatus) &
                                ~(ISRIMR_DASH_INTR_EN | ISRIMR_DASH_INTR_CMAC_RESET));
                } else {
                        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                                RTL_CMAC_W8(tp, CMAC_IBISR0, RTL_CMAC_R8(tp, CMAC_IBISR0));
                        }
                }
        } else {
                RTL_W16(tp, IntrStatus, RTL_R16(tp, IntrStatus));
        }
#else
        RTL_W16(tp, IntrStatus, RTL_R16(tp, IntrStatus));
#endif
}

static void
rtl8168_nic_reset(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int i;

        RTL_W32(tp, RxConfig, (RX_DMA_BURST << RxCfgDMAShift));

        rtl8168_enable_rxdvgate(dev);

        rtl8168_wait_txrx_fifo_empty(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
                mdelay(10);
                break;
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                RTL_W8(tp, ChipCmd, StopReq | CmdRxEnb | CmdTxEnb);
                udelay(100);
                break;
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
                for (i = 0; i < 2000; i++) {
                        if (!(RTL_R8(tp, TxPoll) & NPQ)) break;
                        udelay(100);
                }
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                mdelay(2);
                break;
        default:
                mdelay(10);
                break;
        }

        /* Soft reset the chip. */
        RTL_W8(tp, ChipCmd, CmdReset);

        /* Check that the chip has finished the reset. */
        for (i = 100; i > 0; i--) {
                udelay(100);
                if ((RTL_R8(tp, ChipCmd) & CmdReset) == 0)
                        break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_11:
                rtl8168_oob_mutex_lock(tp);
                rtl8168_ocp_write(tp, 0x10, 2, rtl8168_ocp_read(tp, 0x010, 2)&~0x00004000);
                rtl8168_oob_mutex_unlock(tp);

                rtl8168_oob_notify(tp, OOB_CMD_RESET);

                for (i = 0; i < 10; i++) {
                        mdelay(10);
                        if (rtl8168_ocp_read(tp, 0x010, 2)&0x00004000)
                                break;
                }

                for (i = 0; i < 5; i++) {
                        if ( rtl8168_ocp_read(tp, 0x034, 1) == 0)
                                break;
                }
                break;
        }
}

static void
rtl8168_hw_clear_timer_int(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        RTL_W32(tp, TimeInt0, 0x0000);

        switch (tp->mcfg) {
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
                RTL_W32(tp, TimeInt1, 0x0000);
                break;
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                RTL_W32(tp, TimeInt1, 0x0000);
                RTL_W32(tp, TimeInt2, 0x0000);
                RTL_W32(tp, TimeInt3, 0x0000);
                break;
        }
}

static void
rtl8168_hw_reset(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        /* Disable interrupts */
        rtl8168_irq_mask_and_ack(tp);

        rtl8168_hw_clear_timer_int(dev);

        rtl8168_nic_reset(dev);
}

static void rtl8168_mac_loopback_test(struct rtl8168_private *tp)
{
        struct pci_dev *pdev = tp->pci_dev;
        struct net_device *dev = tp->dev;
        struct sk_buff *skb, *rx_skb;
        dma_addr_t mapping;
        struct TxDesc *txd;
        struct RxDesc *rxd;
        void *tmpAddr;
        u32 len, rx_len, rx_cmd = 0;
        u16 type;
        u8 pattern;
        int i;

        if (tp->DASH)
                return;

        pattern = 0x5A;
        len = 60;
        type = htons(ETH_P_IP);
        txd = tp->TxDescArray;
        rxd = tp->RxDescArray;
        rx_skb = tp->Rx_skbuff[0];
        RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) & ~0x00060000) | 0x00020000);

        do {
                skb = dev_alloc_skb(len + RTK_RX_ALIGN);
                if (unlikely(!skb))
                        dev_printk(KERN_NOTICE, tp_to_dev(tp), "-ENOMEM;\n");
        } while (unlikely(skb == NULL));
        skb_reserve(skb, RTK_RX_ALIGN);

        memcpy(skb_put(skb, dev->addr_len), dev->dev_addr, dev->addr_len);
        memcpy(skb_put(skb, dev->addr_len), dev->dev_addr, dev->addr_len);
        memcpy(skb_put(skb, sizeof(type)), &type, sizeof(type));
        tmpAddr = skb_put(skb, len - 14);

        mapping = dma_map_single(tp_to_dev(tp), skb->data, len, DMA_TO_DEVICE);
        dma_sync_single_for_cpu(tp_to_dev(tp), le64_to_cpu(mapping),
                                len, DMA_TO_DEVICE);
        txd->addr = cpu_to_le64(mapping);
        txd->opts2 = 0;
        while (1) {
                memset(tmpAddr, pattern++, len - 14);
                pci_dma_sync_single_for_device(tp->pci_dev,
                                               le64_to_cpu(mapping),
                                               len, DMA_TO_DEVICE);
                txd->opts1 = cpu_to_le32(DescOwn | FirstFrag | LastFrag | len);

                RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig)  | AcceptMyPhys);

                smp_wmb();
                RTL_W8(tp, TxPoll, NPQ);    /* set polling bit */

                for (i = 0; i < 50; i++) {
                        udelay(200);
                        rx_cmd = le32_to_cpu(rxd->opts1);
                        if ((rx_cmd & DescOwn) == 0)
                                break;
                }

                RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) & ~(AcceptErr | AcceptRunt | AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));

                rx_len = rx_cmd & 0x3FFF;
                rx_len -= 4;
                rxd->opts1 = cpu_to_le32(DescOwn | tp->rx_buf_sz);

                dma_sync_single_for_cpu(tp_to_dev(tp), le64_to_cpu(mapping), len, DMA_TO_DEVICE);

                if (rx_len == len) {
                        dma_sync_single_for_cpu(tp_to_dev(tp), le64_to_cpu(rxd->addr), tp->rx_buf_sz, DMA_FROM_DEVICE);
                        i = memcmp(skb->data, rx_skb->data, rx_len);
                        pci_dma_sync_single_for_device(tp->pci_dev, le64_to_cpu(rxd->addr), tp->rx_buf_sz, DMA_FROM_DEVICE);
                        if (i == 0) {
//              dev_printk(KERN_INFO, tp_to_dev(tp), "loopback test finished\n",rx_len,len);
                                break;
                        }
                }

                rtl8168_hw_reset(dev);
                rtl8168_disable_rxdvgate(dev);
                RTL_W8(tp, ChipCmd, CmdTxEnb | CmdRxEnb);
        }
        tp->dirty_tx++;
        tp->dirty_rx++;
        tp->cur_tx++;
        tp->cur_rx++;
        dma_unmap_single(&pdev->dev, le64_to_cpu(mapping),
                         len, DMA_TO_DEVICE);
        RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) & ~0x00060000);
        dev_kfree_skb_any(skb);
        RTL_W16(tp, IntrStatus, 0xFFBF);
}

static unsigned int
rtl8168_xmii_reset_pending(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned int retval;

        rtl8168_mdio_write(tp, 0x1f, 0x0000);
        retval = rtl8168_mdio_read(tp, MII_BMCR) & BMCR_RESET;

        return retval;
}

static unsigned int
rtl8168_xmii_link_ok(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned int retval;

        retval = (RTL_R8(tp, PHYstatus) & LinkStatus) ? 1 : 0;

        return retval;
}

static int
rtl8168_wait_phy_reset_complete(struct rtl8168_private *tp)
{
        int i, val;

        for (i = 0; i < 2500; i++) {
                val = rtl8168_mdio_read(tp, MII_BMCR) & BMCR_RESET;
                if (!val)
                        return 0;

                mdelay(1);
        }

        return -1;
}

static void
rtl8168_xmii_reset_enable(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (rtl8168_is_in_phy_disable_mode(dev))
                return;

        rtl8168_mdio_write(tp, 0x1f, 0x0000);
        rtl8168_mdio_write(tp, MII_ADVERTISE, rtl8168_mdio_read(tp, MII_ADVERTISE) &
                           ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                             ADVERTISE_100HALF | ADVERTISE_100FULL));
        rtl8168_mdio_write(tp, MII_CTRL1000, rtl8168_mdio_read(tp, MII_CTRL1000) &
                           ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL));
        rtl8168_mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);

        if (rtl8168_wait_phy_reset_complete(tp) == 0) return;

        if (netif_msg_link(tp))
                printk(KERN_ERR "%s: PHY reset failed.\n", dev->name);
}

static void
rtl8168dp_10mbps_gphy_para(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u8 status = RTL_R8(tp, PHYstatus);

        if ((status & LinkStatus) && (status & _10bps)) {
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                rtl8168_mdio_write(tp, 0x10, 0x04EE);
        } else {
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                rtl8168_mdio_write(tp, 0x10, 0x01EE);
        }
}

void rtl8168_init_ring_indexes(struct rtl8168_private *tp)
{
        tp->dirty_tx = 0;
        tp->dirty_rx = 0;
        tp->cur_tx = 0;
        tp->cur_rx = 0;
}

static void
rtl8168_issue_offset_99_event(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                if (tp->mcfg == CFG_METHOD_24 || tp->mcfg == CFG_METHOD_25 ||
                    tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28) {
                        rtl8168_eri_write(tp, 0x3FC, 4, 0x00000000, ERIAR_ExGMAC);
                } else {
                        rtl8168_eri_write(tp, 0x3FC, 4, 0x083C083C, ERIAR_ExGMAC);
                }
                csi_tmp = rtl8168_eri_read(tp, 0x3F8, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0x3F8, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x1EA, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0x1EA, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }
}

static void
rtl8168_enable_cfg9346_write(struct rtl8168_private *tp)
{
        RTL_W8(tp, Cfg9346, RTL_R8(tp, Cfg9346) | Cfg9346_Unlock);
}

static void
rtl8168_disable_cfg9346_write(struct rtl8168_private *tp)
{
        RTL_W8(tp, Cfg9346, RTL_R8(tp, Cfg9346) & ~Cfg9346_Unlock);
}

static void
rtl8168_enable_exit_l1_mask(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
                csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                csi_tmp |= (BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_20:
                csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                csi_tmp |= (BIT_10 | BIT_11);
                rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_21 ... CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                csi_tmp |= (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        }
}

static void
rtl8168_disable_exit_l1_mask(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
                csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_20:
                csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_10 | BIT_11);
                rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_21 ... CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        }
}

static void
rtl8168_hw_aspm_clkreq_enable(struct rtl8168_private *tp, bool enable)
{
        if (!tp->HwSuppAspmClkIntrLock) return;

        if (enable && aspm) {
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) | ASPM_en);
                RTL_W8(tp, Config2, RTL_R8(tp, Config2) | ClkReqEn);
        } else {
                RTL_W8(tp, Config2, RTL_R8(tp, Config2) & ~ClkReqEn);
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) & ~ASPM_en);
        }

        udelay(10);
}

#ifdef ENABLE_DASH_SUPPORT
static void
NICChkTypeEnableDashInterrupt(struct rtl8168_private *tp)
{
        if (tp->DASH) {
                //
                // even disconnected, enable 3 dash interrupt mask bits for in-band/out-band communication
                //
                if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                        rtl8168_enable_dash2_interrupt(tp);
                        RTL_W16(tp, IntrMask, (ISRIMR_DASH_INTR_EN | ISRIMR_DASH_INTR_CMAC_RESET));
                } else {
                        RTL_W16(tp, IntrMask, (ISRIMR_DP_DASH_OK | ISRIMR_DP_HOST_OK | ISRIMR_DP_REQSYS_OK));
                }
        }
}
#endif

static void
rtl8168_check_link_status(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int link_status_on;

#ifdef ENABLE_FIBER_SUPPORT
        rtl8168_check_fiber_link_status(dev);
#endif //ENABLE_FIBER_SUPPORT

        link_status_on = tp->link_ok(dev);

        if (tp->mcfg == CFG_METHOD_11)
                rtl8168dp_10mbps_gphy_para(dev);

        if (netif_carrier_ok(dev) != link_status_on) {
                if (link_status_on) {
                        rtl8168_hw_config(dev);

                        if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19 || tp->mcfg == CFG_METHOD_20) {
                                if (RTL_R8(tp, PHYstatus) & _1000bpsF) {
                                        rtl8168_eri_write(tp, 0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
                                        rtl8168_eri_write(tp, 0x1dc, 4, 0x0000001f, ERIAR_ExGMAC);
                                } else if (RTL_R8(tp, PHYstatus) & _100bps) {
                                        rtl8168_eri_write(tp, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                                        rtl8168_eri_write(tp, 0x1dc, 4, 0x0000001f, ERIAR_ExGMAC);
                                } else {
                                        rtl8168_eri_write(tp, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                                        rtl8168_eri_write(tp, 0x1dc, 4, 0x0000002d, ERIAR_ExGMAC);
                                }
                        } else if ((tp->mcfg == CFG_METHOD_16 || tp->mcfg == CFG_METHOD_17) && netif_running(dev)) {
                                if (tp->mcfg == CFG_METHOD_16 && (RTL_R8(tp, PHYstatus) & _10bps)) {
                                        RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) | AcceptAllPhys);
                                } else if (tp->mcfg == CFG_METHOD_17) {
                                        if (RTL_R8(tp, PHYstatus) & _1000bpsF) {
                                                rtl8168_eri_write(tp, 0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
                                                rtl8168_eri_write(tp, 0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                                        } else if (RTL_R8(tp, PHYstatus) & _100bps) {
                                                rtl8168_eri_write(tp, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                                                rtl8168_eri_write(tp, 0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                                        } else {
                                                rtl8168_eri_write(tp, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                                                rtl8168_eri_write(tp, 0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
                                        }
                                }
                        } else if ((tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15) && tp->eee_enabled == 1) {
                                /*Full -Duplex  mode*/
                                if (RTL_R8(tp, PHYstatus)&FullDup) {
                                        rtl8168_mdio_write(tp, 0x1F, 0x0006);
                                        rtl8168_mdio_write(tp, 0x00, 0x5a30);
                                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                                        if (RTL_R8(tp, PHYstatus) & (_10bps | _100bps))
                                                RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) & ~BIT_19) | BIT_25);

                                } else {
                                        rtl8168_mdio_write(tp, 0x1F, 0x0006);
                                        rtl8168_mdio_write(tp, 0x00, 0x5a00);
                                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                                        if (RTL_R8(tp, PHYstatus) & (_10bps | _100bps))
                                                RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) & ~BIT_19) | (InterFrameGap << TxInterFrameGapShift));
                                }
                        } else if ((tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
                                    tp->mcfg == CFG_METHOD_23 || tp->mcfg == CFG_METHOD_24 ||
                                    tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
                                    tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
                                    tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
                                    tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                                    tp->mcfg == CFG_METHOD_33) && netif_running(dev)) {
                                if (RTL_R8(tp, PHYstatus)&FullDup)
                                        RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) | (BIT_24 | BIT_25)) & ~BIT_19);
                                else
                                        RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) | BIT_25) & ~(BIT_19 | BIT_24));
                        }

                        if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
                            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
                            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                            tp->mcfg == CFG_METHOD_33) {
                                /*half mode*/
                                if (!(RTL_R8(tp, PHYstatus)&FullDup)) {
                                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                                        rtl8168_mdio_write(tp, MII_ADVERTISE, rtl8168_mdio_read(tp, MII_ADVERTISE)&~(ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM));
                                }
                        }

                        if ((tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                             tp->mcfg == CFG_METHOD_33) && (RTL_R8(tp, PHYstatus) & _10bps)) {
                                u32 csi_tmp;

                                csi_tmp = rtl8168_eri_read(tp, 0x1D0, 1, ERIAR_ExGMAC);
                                csi_tmp |= BIT_1;
                                rtl8168_eri_write(tp, 0x1D0, 1, csi_tmp, ERIAR_ExGMAC);
                        }

                        rtl8168_hw_start(dev);

                        netif_carrier_on(dev);

                        netif_wake_queue(dev);

                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        tp->phy_reg_anlpar = rtl8168_mdio_read(tp, MII_LPA);

                        if (netif_msg_ifup(tp))
                                printk(KERN_INFO PFX "%s: link up\n", dev->name);
                } else {
                        if (netif_msg_ifdown(tp))
                                printk(KERN_INFO PFX "%s: link down\n", dev->name);

                        tp->phy_reg_anlpar = 0;

                        netif_stop_queue(dev);

                        netif_carrier_off(dev);

                        rtl8168_hw_reset(dev);

                        rtl8168_tx_clear(tp);

                        rtl8168_rx_clear(tp);

                        rtl8168_init_ring(dev);

                        if (dynamic_aspm) {
                                rtl8168_enable_cfg9346_write(tp);
                                rtl8168_hw_aspm_clkreq_enable(tp, true);
                                rtl8168_disable_cfg9346_write(tp);
                        }

                        rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);

                        switch (tp->mcfg) {
                        case CFG_METHOD_21:
                        case CFG_METHOD_22:
                        case CFG_METHOD_23:
                        case CFG_METHOD_24:
                        case CFG_METHOD_25:
                        case CFG_METHOD_27:
                        case CFG_METHOD_28:
                                if (tp->org_pci_offset_99 & BIT_2)
                                        tp->issue_offset_99_event = TRUE;
                                break;
                        }

#ifdef ENABLE_DASH_SUPPORT
                        if (tp->DASH) {
                                NICChkTypeEnableDashInterrupt(tp);
                        }
#endif
                }
        }

        if (!link_status_on) {
                switch (tp->mcfg) {
                case CFG_METHOD_21:
                case CFG_METHOD_22:
                case CFG_METHOD_23:
                case CFG_METHOD_24:
                case CFG_METHOD_25:
                case CFG_METHOD_27:
                case CFG_METHOD_28:
                        if (tp->issue_offset_99_event) {
                                if (!(RTL_R8(tp, PHYstatus) & PowerSaveStatus)) {
                                        tp->issue_offset_99_event = FALSE;
                                        rtl8168_issue_offset_99_event(tp);
                                }
                        }
                        break;
                }
        } else {
                if (dynamic_aspm) {
                        bool enable_hw_aspm_clkreq = true;
                        if (tp->dynamic_aspm_packet_count > dynamic_aspm_packet_threshold)
                                enable_hw_aspm_clkreq = false;

                        rtl8168_enable_cfg9346_write(tp);
                        rtl8168_hw_aspm_clkreq_enable(tp, enable_hw_aspm_clkreq);
                        rtl8168_disable_cfg9346_write(tp);
                }
                tp->dynamic_aspm_packet_count = 0;
        }
}

static void
rtl8168_link_option(u8 *aut,
                    u32 *spd,
                    u8 *dup,
                    u32 *adv)
{
        if ((*spd != SPEED_1000) && (*spd != SPEED_100) && (*spd != SPEED_10))
                *spd = SPEED_1000;

        if ((*dup != DUPLEX_FULL) && (*dup != DUPLEX_HALF))
                *dup = DUPLEX_FULL;

        if ((*aut != AUTONEG_ENABLE) && (*aut != AUTONEG_DISABLE))
                *aut = AUTONEG_ENABLE;

        *adv &= (ADVERTISED_10baseT_Half |
                 ADVERTISED_10baseT_Full |
                 ADVERTISED_100baseT_Half |
                 ADVERTISED_100baseT_Full |
                 ADVERTISED_1000baseT_Half |
                 ADVERTISED_1000baseT_Full);
        if (*adv == 0)
                *adv = (ADVERTISED_10baseT_Half |
                        ADVERTISED_10baseT_Full |
                        ADVERTISED_100baseT_Half |
                        ADVERTISED_100baseT_Full |
                        ADVERTISED_1000baseT_Half |
                        ADVERTISED_1000baseT_Full);
}

static void
rtl8168_enable_ocp_phy_power_saving(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u16 val;

        if (tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                val = rtl8168_mdio_read_phy_ocp(tp, 0x0C41, 0x13);
                if (val != 0x0050) {
                        rtl8168_set_phy_mcu_patch_request(tp);
                        rtl8168_mdio_write_phy_ocp(tp, 0x0C41, 0x13, 0x0000);
                        rtl8168_mdio_write_phy_ocp(tp, 0x0C41, 0x13, 0x0050);
                        rtl8168_clear_phy_mcu_patch_request(tp);
                }
        }
}

static void
rtl8168_disable_ocp_phy_power_saving(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u16 val;

        if (tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                val = rtl8168_mdio_read_phy_ocp(tp, 0x0C41, 0x13);
                if (val != 0x0500) {
                        rtl8168_set_phy_mcu_patch_request(tp);
                        rtl8168_mdio_write_phy_ocp(tp, 0x0C41, 0x13, 0x0000);
                        rtl8168_mdio_write_phy_ocp(tp, 0x0C41, 0x13, 0x0500);
                        rtl8168_clear_phy_mcu_patch_request(tp);
                }
        }
}

void
rtl8168_wait_ll_share_fifo_ready(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int i;

        for (i = 0; i < 10; i++) {
                udelay(100);
                if (RTL_R16(tp, 0xD2) & BIT_9)
                        break;
        }
}

static void
rtl8168_disable_pci_offset_99(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x3F2, 2, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_0 | BIT_1);
                rtl8168_eri_write(tp, 0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_csi_fun0_write_byte(tp, 0x99, 0x00);
                break;
        }
}

static void
rtl8168_enable_pci_offset_99(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_csi_fun0_write_byte(tp, 0x99, tp->org_pci_offset_99);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x3F2, 2, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_0 | BIT_1);
                if (!(tp->org_pci_offset_99 & (BIT_5 | BIT_6)))
                        csi_tmp |= BIT_1;
                if (!(tp->org_pci_offset_99 & BIT_2))
                        csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
                break;
        }
}

static void
rtl8168_init_pci_offset_99(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_26:
                if (tp->org_pci_offset_99 & BIT_2) {
                        csi_tmp = rtl8168_eri_read(tp, 0x5C2, 1, ERIAR_ExGMAC);
                        csi_tmp &= ~BIT_1;
                        rtl8168_eri_write(tp, 0x5C2, 1, csi_tmp, ERIAR_ExGMAC);
                }
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x3F2, 2, ERIAR_ExGMAC);
                csi_tmp &= ~( BIT_8 | BIT_9  | BIT_10 | BIT_11  | BIT_12  | BIT_13  | BIT_14 | BIT_15 );
                csi_tmp |= ( BIT_9 | BIT_10 | BIT_13  | BIT_14 | BIT_15 );
                rtl8168_eri_write(tp, 0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
                csi_tmp = rtl8168_eri_read(tp, 0x3F5, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_6 | BIT_7;
                rtl8168_eri_write(tp, 0x3F5, 1, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mac_ocp_write(tp, 0xE02C, 0x1880);
                rtl8168_mac_ocp_write(tp, 0xE02E, 0x4880);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_26:
                rtl8168_eri_write(tp, 0x5C0, 1, 0xFA, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_26:
                if (tp->org_pci_offset_99 & BIT_2) {
                        csi_tmp = rtl8168_eri_read(tp, 0x5C8, 1, ERIAR_ExGMAC);
                        csi_tmp |= BIT_0;
                        rtl8168_eri_write(tp, 0x5C8, 1, csi_tmp, ERIAR_ExGMAC);
                }
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                if (tp->org_pci_offset_99 & BIT_2)
                        rtl8168_mac_ocp_write(tp, 0xE0A2,  rtl8168_mac_ocp_read(tp, 0xE0A2) | BIT_0);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_23:
                rtl8168_eri_write(tp, 0x2E8, 2, 0x883C, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2EA, 2, 0x8C12, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2EC, 2, 0x9003, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2E2, 2, 0x883C, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2E6, 2, 0x9003, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_eri_write(tp, 0x2E8, 2, 0x9003, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2EA, 2, 0x9003, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2EC, 2, 0x9003, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2E2, 2, 0x883C, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x2E6, 2, 0x9003, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                csi_tmp = rtl8168_eri_read(tp, 0x3FA, 2, ERIAR_ExGMAC);
                csi_tmp |= BIT_14;
                rtl8168_eri_write(tp, 0x3FA, 2, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                if (tp->org_pci_offset_99 & BIT_2)
                        RTL_W8(tp, 0xB6, RTL_R8(tp, 0xB6) | BIT_0);
                break;
        }

        rtl8168_enable_pci_offset_99(tp);
}

static void
rtl8168_disable_pci_offset_180(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x1E2, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_2;
                rtl8168_eri_write(tp, 0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_26:
                rtl8168_eri_write(tp, 0x1E9, 1, 0x0A, ERIAR_ExGMAC);
                break;
        }
}

static void
rtl8168_enable_pci_offset_180(struct rtl8168_private *tp)
{
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_25:
        case CFG_METHOD_28:
                csi_tmp = rtl8168_eri_read(tp, 0x1E8, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(0x0000FF00);
                csi_tmp |= (0x00006400);
                rtl8168_eri_write(tp, 0x1E8, 4, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x1E4, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(0x0000FF00);
                rtl8168_eri_write(tp, 0x1E4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x1E8, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(0x0000FFF0);
                csi_tmp |= (0x00000640);
                rtl8168_eri_write(tp, 0x1E8, 4, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x1E4, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(0x0000FF00);
                rtl8168_eri_write(tp, 0x1E4, 4, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                csi_tmp = rtl8168_eri_read(tp, 0x1E2, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_2;
                rtl8168_eri_write(tp, 0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_26:
                rtl8168_eri_write(tp, 0x1E9, 1, 0x64, ERIAR_ExGMAC);
                break;
        }

        rtl8168_mac_ocp_write(tp, 0xE094, 0x0000);
}

static void
rtl8168_init_pci_offset_180(struct rtl8168_private *tp)
{
        if (tp->org_pci_offset_180 & (BIT_0|BIT_1))
                rtl8168_enable_pci_offset_180(tp);
        else
                rtl8168_disable_pci_offset_180(tp);
}

static void
rtl8168_set_pci_99_180_exit_driver_para(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_issue_offset_99_event(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_disable_pci_offset_99(tp);
                break;
        }
        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_disable_pci_offset_180(tp);
                break;
        }
}

static void
rtl8168_hw_d3_para(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        RTL_W16(tp, RxMaxSize, RX_BUF_SIZE);

        if (tp->HwSuppAspmClkIntrLock) {
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) & ~BIT_7);
                rtl8168_enable_cfg9346_write(tp);
                rtl8168_hw_aspm_clkreq_enable(tp, false);
                rtl8168_disable_cfg9346_write(tp);
        }

        rtl8168_disable_exit_l1_mask(tp);

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8168_set_realwow_d3_para(dev);
#endif

        if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19 || tp->mcfg == CFG_METHOD_20) {
                rtl8168_eri_write(tp, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x1dc, 4, 0x0000002d, ERIAR_ExGMAC);
        } else if (tp->mcfg == CFG_METHOD_16) {
                rtl8168_eri_write(tp, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
        }

        if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
            tp->mcfg == CFG_METHOD_23 || tp->mcfg == CFG_METHOD_24 ||
            tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                rtl8168_eri_write(tp, 0x2F8, 2, 0x0064, ERIAR_ExGMAC);
        }

        if (tp->bios_setting & BIT_28) {
                if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19 ||
                    tp->mcfg == CFG_METHOD_20) {
                        u32 gphy_val;

                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        rtl8168_mdio_write(tp, 0x04, 0x0061);
                        rtl8168_mdio_write(tp, 0x09, 0x0000);
                        rtl8168_mdio_write(tp, 0x00, 0x9200);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8B80);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val &= ~BIT_7;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        mdelay(1);
                        rtl8168_mdio_write(tp, 0x1F, 0x0007);
                        rtl8168_mdio_write(tp, 0x1E, 0x002C);
                        gphy_val = rtl8168_mdio_read(tp, 0x16);
                        gphy_val &= ~BIT_10;
                        rtl8168_mdio_write(tp, 0x16, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }
        }

        rtl8168_set_pci_99_180_exit_driver_para(dev);

        /*disable ocp phy power saving*/
        if (tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33)
                if (!tp->dash_printer_enabled)
                        rtl8168_disable_ocp_phy_power_saving(dev);

        rtl8168_disable_rxdvgate(dev);
}

static void
rtl8168_enable_magic_packet(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 csi_tmp;

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V1:
                rtl8168_enable_cfg9346_write(tp);
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) | MagicPacket);
                rtl8168_disable_cfg9346_write(tp);
                break;
        case WAKEUP_MAGIC_PACKET_V2:
                csi_tmp = rtl8168_eri_read(tp, 0xDE, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDE, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }
}
static void
rtl8168_disable_magic_packet(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 csi_tmp;

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V1:
                rtl8168_enable_cfg9346_write(tp);
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~MagicPacket);
                rtl8168_disable_cfg9346_write(tp);
                break;
        case WAKEUP_MAGIC_PACKET_V2:
                csi_tmp = rtl8168_eri_read(tp, 0xDE, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDE, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

static void
rtl8168_get_hw_wol(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u8 options;
        u32 csi_tmp;
        unsigned long flags;


        spin_lock_irqsave(&tp->lock, flags);

        tp->wol_opts = 0;
        options = RTL_R8(tp, Config1);
        if (!(options & PMEnable))
                goto out_unlock;

        options = RTL_R8(tp, Config3);
        if (options & LinkUp)
                tp->wol_opts |= WAKE_PHY;

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V2:
                csi_tmp = rtl8168_eri_read(tp, 0xDE, 1, ERIAR_ExGMAC);
                if (csi_tmp & BIT_0)
                        tp->wol_opts |= WAKE_MAGIC;
                break;
        default:
                if (options & MagicPacket)
                        tp->wol_opts |= WAKE_MAGIC;
                break;
        }

        options = RTL_R8(tp, Config5);
        if (options & UWF)
                tp->wol_opts |= WAKE_UCAST;
        if (options & BWF)
                tp->wol_opts |= WAKE_BCAST;
        if (options & MWF)
                tp->wol_opts |= WAKE_MCAST;

out_unlock:
        tp->wol_enabled = (tp->wol_opts || tp->dash_printer_enabled) ? WOL_ENABLED : WOL_DISABLED;

        spin_unlock_irqrestore(&tp->lock, flags);
}

static void
rtl8168_set_hw_wol(struct net_device *dev, u32 wolopts)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int i,tmp;
        static struct {
                u32 opt;
                u16 reg;
                u8  mask;
        } cfg[] = {
                { WAKE_PHY,   Config3, LinkUp },
                { WAKE_UCAST, Config5, UWF },
                { WAKE_BCAST, Config5, BWF },
                { WAKE_MCAST, Config5, MWF },
                { WAKE_ANY,   Config5, LanWake },
                { WAKE_MAGIC, Config3, MagicPacket },
        };

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V2:
                tmp = ARRAY_SIZE(cfg) - 1;

                if (wolopts & WAKE_MAGIC)
                        rtl8168_enable_magic_packet(dev);
                else
                        rtl8168_disable_magic_packet(dev);
                break;
        default:
                tmp = ARRAY_SIZE(cfg);
                break;
        }

        rtl8168_enable_cfg9346_write(tp);

        for (i = 0; i < tmp; i++) {
                u8 options = RTL_R8(tp, cfg[i].reg) & ~cfg[i].mask;
                if (wolopts & cfg[i].opt)
                        options |= cfg[i].mask;
                RTL_W8(tp, cfg[i].reg, options);
        }

        if (tp->dash_printer_enabled)
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) | LanWake);

        rtl8168_disable_cfg9346_write(tp);
}

static void
rtl8168_phy_restart_nway(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (rtl8168_is_in_phy_disable_mode(dev)) return;

        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        rtl8168_mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
}

static void
rtl8168_phy_setup_force_mode(struct net_device *dev, u32 speed, u8 duplex)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u16 bmcr_true_force = 0;

        if (rtl8168_is_in_phy_disable_mode(dev)) return;

        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
                bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
                bmcr_true_force = BMCR_SPEED10 | BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
                bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
                bmcr_true_force = BMCR_SPEED100 | BMCR_FULLDPLX;
        } else {
                netif_err(tp, drv, dev, "Failed to set phy force mode!\n");
                return;
        }

        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        rtl8168_mdio_write(tp, MII_BMCR, bmcr_true_force);
}

static void
rtl8168_powerdown_pll(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(tp))
                return;
#endif //ENABLE_FIBER_SUPPORT

        if (tp->wol_enabled == WOL_ENABLED || tp->DASH || tp->EnableKCPOffload) {
                int auto_nego;
                int giga_ctrl;
                u16 anlpar;

                rtl8168_set_hw_wol(dev, tp->wol_opts);

                if (tp->mcfg == CFG_METHOD_16 || tp->mcfg == CFG_METHOD_17 ||
                    tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
                    tp->mcfg == CFG_METHOD_24 || tp->mcfg == CFG_METHOD_25 ||
                    tp->mcfg == CFG_METHOD_26 || tp->mcfg == CFG_METHOD_23 ||
                    tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
                    tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
                    tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                    tp->mcfg == CFG_METHOD_33) {
                        rtl8168_enable_cfg9346_write(tp);
                        RTL_W8(tp, Config2, RTL_R8(tp, Config2) | PMSTS_En);
                        rtl8168_disable_cfg9346_write(tp);
                }

                if (HW_SUPP_SERDES_PHY(tp))
                        return;

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                auto_nego = rtl8168_mdio_read(tp, MII_ADVERTISE);
                auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                               | ADVERTISE_100HALF | ADVERTISE_100FULL);

                if (netif_running(dev))
                        anlpar = tp->phy_reg_anlpar;
                else
                        anlpar = rtl8168_mdio_read(tp, MII_LPA);

#ifdef CONFIG_DOWN_SPEED_100
                auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
#else
                if (anlpar & (LPA_10HALF | LPA_10FULL))
                        auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL);
                else
                        auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
#endif

                if (tp->DASH)
                        auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

                if (((tp->mcfg == CFG_METHOD_7) || (tp->mcfg == CFG_METHOD_8)) && (RTL_R16(tp, CPlusCmd) & ASF))
                        auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

                giga_ctrl = rtl8168_mdio_read(tp, MII_CTRL1000) & ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
                rtl8168_mdio_write(tp, MII_ADVERTISE, auto_nego);
                rtl8168_mdio_write(tp, MII_CTRL1000, giga_ctrl);
                rtl8168_phy_restart_nway(dev);

                RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);

                return;
        }

        if (tp->DASH)
                return;

        if (((tp->mcfg == CFG_METHOD_7) || (tp->mcfg == CFG_METHOD_8)) && (RTL_R16(tp, CPlusCmd) & ASF))
                return;

        rtl8168_phy_power_down(dev);

        if (!tp->HwIcVerUnknown) {
                switch (tp->mcfg) {
                case CFG_METHOD_9:
                case CFG_METHOD_10:
                //case CFG_METHOD_11:
                case CFG_METHOD_12:
                case CFG_METHOD_13:
                case CFG_METHOD_14:
                case CFG_METHOD_15:
                case CFG_METHOD_17:
                case CFG_METHOD_18:
                case CFG_METHOD_19:
                case CFG_METHOD_21:
                case CFG_METHOD_22:
                case CFG_METHOD_24:
                case CFG_METHOD_25:
                case CFG_METHOD_26:
                case CFG_METHOD_27:
                case CFG_METHOD_28:
                case CFG_METHOD_29:
                case CFG_METHOD_30:
                case CFG_METHOD_31:
                case CFG_METHOD_32:
                case CFG_METHOD_33:
                        RTL_W8(tp, PMCH, RTL_R8(tp, PMCH) & ~BIT_7);
                        break;
                }
        }

        switch (tp->mcfg) {
        case CFG_METHOD_14 ... CFG_METHOD_15:
                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~BIT_6);
                break;
        case CFG_METHOD_16 ... CFG_METHOD_33:
                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~BIT_6);
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_6);
                break;
        }
}

static void rtl8168_powerup_pll(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                RTL_W8(tp, PMCH, RTL_R8(tp, PMCH) | BIT_7 | BIT_6);
                break;
        }

        rtl8168_phy_power_up(dev);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
static void
rtl8168_get_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u8 options;
        unsigned long flags;

        wol->wolopts = 0;

        if (tp->mcfg == CFG_METHOD_DEFAULT) {
                wol->supported = 0;
                return;
        } else {
                wol->supported = WAKE_ANY;
        }

        spin_lock_irqsave(&tp->lock, flags);

        options = RTL_R8(tp, Config1);
        if (!(options & PMEnable))
                goto out_unlock;

        wol->wolopts = tp->wol_opts;

out_unlock:
        spin_unlock_irqrestore(&tp->lock, flags);
}

static int
rtl8168_set_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        if (tp->mcfg == CFG_METHOD_DEFAULT)
                return -EOPNOTSUPP;

        spin_lock_irqsave(&tp->lock, flags);

        tp->wol_opts = wol->wolopts;

        tp->wol_enabled = (tp->wol_opts || tp->dash_printer_enabled) ? WOL_ENABLED : WOL_DISABLED;

        spin_unlock_irqrestore(&tp->lock, flags);

        device_set_wakeup_enable(tp_to_dev(tp), tp->wol_enabled);

        return 0;
}

static void
rtl8168_get_drvinfo(struct net_device *dev,
                    struct ethtool_drvinfo *info)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct rtl8168_fw *rtl_fw = tp->rtl_fw;

        strcpy(info->driver, MODULENAME);
        strcpy(info->version, RTL8168_VERSION);
        strcpy(info->bus_info, pci_name(tp->pci_dev));
        info->regdump_len = R8168_REGS_DUMP_SIZE;
        info->eedump_len = tp->eeprom_len;
        BUILD_BUG_ON(sizeof(info->fw_version) < sizeof(rtl_fw->version));
        if (rtl_fw)
                strlcpy(info->fw_version, rtl_fw->version,
                        sizeof(info->fw_version));
}

static int
rtl8168_get_regs_len(struct net_device *dev)
{
        return R8168_REGS_DUMP_SIZE;
}
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)

static int
rtl8168_set_speed_xmii(struct net_device *dev,
                       u8 autoneg,
                       u32 speed,
                       u8 duplex,
                       u32 adv)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int auto_nego = 0;
        int giga_ctrl = 0;
        int rc = -EINVAL;

        if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                //Disable Giga Lite
                rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                rtl8168_clear_eth_phy_bit(tp, 0x14, BIT_9);
                if (tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                    tp->mcfg == CFG_METHOD_33)
                        rtl8168_clear_eth_phy_bit(tp, 0x14, BIT_7);
                rtl8168_mdio_write(tp, 0x1F, 0x0A40);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        }

        if ((speed != SPEED_1000) &&
            (speed != SPEED_100) &&
            (speed != SPEED_10)) {
                speed = SPEED_1000;
                duplex = DUPLEX_FULL;
        }

        giga_ctrl = rtl8168_mdio_read(tp, MII_CTRL1000);
        giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);

        if (autoneg == AUTONEG_ENABLE) {
                /*n-way force*/
                auto_nego = rtl8168_mdio_read(tp, MII_ADVERTISE);
                auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                               ADVERTISE_100HALF | ADVERTISE_100FULL |
                               ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

                if (adv & ADVERTISED_10baseT_Half)
                        auto_nego |= ADVERTISE_10HALF;
                if (adv & ADVERTISED_10baseT_Full)
                        auto_nego |= ADVERTISE_10FULL;
                if (adv & ADVERTISED_100baseT_Half)
                        auto_nego |= ADVERTISE_100HALF;
                if (adv & ADVERTISED_100baseT_Full)
                        auto_nego |= ADVERTISE_100FULL;
                if (adv & ADVERTISED_1000baseT_Half)
                        giga_ctrl |= ADVERTISE_1000HALF;
                if (adv & ADVERTISED_1000baseT_Full)
                        giga_ctrl |= ADVERTISE_1000FULL;

                //flow control
                if (dev->mtu <= ETH_DATA_LEN)
                        auto_nego |= ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM;

                tp->phy_auto_nego_reg = auto_nego;
                tp->phy_1000_ctrl_reg = giga_ctrl;

                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                rtl8168_mdio_write(tp, MII_ADVERTISE, auto_nego);
                rtl8168_mdio_write(tp, MII_CTRL1000, giga_ctrl);
                rtl8168_phy_restart_nway(dev);
                mdelay(20);
        } else {
                /*true force*/
                if (speed == SPEED_10 || speed == SPEED_100)
                        rtl8168_phy_setup_force_mode(dev, speed, duplex);
                else
                        goto out;
        }

        tp->autoneg = autoneg;
        tp->speed = speed;
        tp->duplex = duplex;
        tp->advertising = adv;

        if (tp->mcfg == CFG_METHOD_11)
                rtl8168dp_10mbps_gphy_para(dev);

        rc = 0;
out:
        return rc;
}

static int
rtl8168_set_speed(struct net_device *dev,
                  u8 autoneg,
                  u32 speed,
                  u8 duplex,
                  u32 adv)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int ret;

        ret = tp->set_speed(dev, autoneg, speed, duplex, adv);

        return ret;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
static int
rtl8168_set_settings(struct net_device *dev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
                     struct ethtool_cmd *cmd
#else
                     const struct ethtool_link_ksettings *cmd
#endif
                    )
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int ret;
        unsigned long flags;
        u8 autoneg;
        u32 speed;
        u8 duplex;
        u32 supported, advertising;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        autoneg = cmd->autoneg;
        speed = cmd->speed;
        duplex = cmd->duplex;
        supported = cmd->supported;
        advertising = cmd->advertising;
#else
        const struct ethtool_link_settings *base = &cmd->base;
        autoneg = base->autoneg;
        speed = base->speed;
        duplex = base->duplex;
        ethtool_convert_link_mode_to_legacy_u32(&supported,
                                                cmd->link_modes.supported);
        ethtool_convert_link_mode_to_legacy_u32(&advertising,
                                                cmd->link_modes.advertising);
#endif
        if (advertising & ~supported)
                return -EINVAL;

        spin_lock_irqsave(&tp->lock, flags);
        ret = rtl8168_set_speed(dev, autoneg, speed, duplex, advertising);
        spin_unlock_irqrestore(&tp->lock, flags);

        return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32
rtl8168_get_tx_csum(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 ret;
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        ret = ((dev->features & NETIF_F_IP_CSUM) != 0);
#else
        ret = ((dev->features & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM)) != 0);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        spin_unlock_irqrestore(&tp->lock, flags);

        return ret;
}

static u32
rtl8168_get_rx_csum(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 ret;
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
        ret = tp->cp_cmd & RxChkSum;
        spin_unlock_irqrestore(&tp->lock, flags);

        return ret;
}

static int
rtl8168_set_tx_csum(struct net_device *dev,
                    u32 data)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        if (tp->mcfg == CFG_METHOD_DEFAULT)
                return -EOPNOTSUPP;

        spin_lock_irqsave(&tp->lock, flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        if (data)
                dev->features |= NETIF_F_IP_CSUM;
        else
                dev->features &= ~NETIF_F_IP_CSUM;
#else
        if (data)
                if ((tp->mcfg == CFG_METHOD_1) || (tp->mcfg == CFG_METHOD_2) || (tp->mcfg == CFG_METHOD_3))
                        dev->features |= NETIF_F_IP_CSUM;
                else
                        dev->features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
        else
                dev->features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)

        spin_unlock_irqrestore(&tp->lock, flags);

        return 0;
}

static int
rtl8168_set_rx_csum(struct net_device *dev,
                    u32 data)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        if (tp->mcfg == CFG_METHOD_DEFAULT)
                return -EOPNOTSUPP;

        spin_lock_irqsave(&tp->lock, flags);

        if (data)
                tp->cp_cmd |= RxChkSum;
        else
                tp->cp_cmd &= ~RxChkSum;

        RTL_W16(tp, CPlusCmd, tp->cp_cmd);

        spin_unlock_irqrestore(&tp->lock, flags);

        return 0;
}
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)

#ifdef CONFIG_R8168_VLAN

static inline u32
rtl8168_tx_vlan_tag(struct rtl8168_private *tp,
                    struct sk_buff *skb)
{
        u32 tag;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        tag = (tp->vlgrp && vlan_tx_tag_present(skb)) ?
              TxVlanTag | swab16(vlan_tx_tag_get(skb)) : 0x00;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        tag = (vlan_tx_tag_present(skb)) ?
              TxVlanTag | swab16(vlan_tx_tag_get(skb)) : 0x00;
#else
        tag = (skb_vlan_tag_present(skb)) ?
              TxVlanTag | swab16(skb_vlan_tag_get(skb)) : 0x00;
#endif

        return tag;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)

static void
rtl8168_vlan_rx_register(struct net_device *dev,
                         struct vlan_group *grp)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
        tp->vlgrp = grp;
        if (tp->vlgrp)
                tp->cp_cmd |= RxVlan;
        else
                tp->cp_cmd &= ~RxVlan;
        RTL_W16(tp, CPlusCmd, tp->cp_cmd);
        RTL_R16(tp, CPlusCmd);
        spin_unlock_irqrestore(&tp->lock, flags);
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
static void
rtl8168_vlan_rx_kill_vid(struct net_device *dev,
                         unsigned short vid)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
        if (tp->vlgrp)
                tp->vlgrp->vlan_devices[vid] = NULL;
#else
        vlan_group_set_device(tp->vlgrp, vid, NULL);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
        spin_unlock_irqrestore(&tp->lock, flags);
}
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)

static int
rtl8168_rx_vlan_skb(struct rtl8168_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
        u32 opts2 = le32_to_cpu(desc->opts2);
        int ret = -1;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        if (tp->vlgrp && (opts2 & RxVlanTag)) {
                rtl8168_rx_hwaccel_skb(skb, tp->vlgrp,
                                       swab16(opts2 & 0xffff));
                ret = 0;
        }
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
        if (opts2 & RxVlanTag)
                __vlan_hwaccel_put_tag(skb, swab16(opts2 & 0xffff));
#else
        if (opts2 & RxVlanTag)
                __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), swab16(opts2 & 0xffff));
#endif

        desc->opts2 = 0;
        return ret;
}

#else /* !CONFIG_R8168_VLAN */

static inline u32
rtl8168_tx_vlan_tag(struct rtl8168_private *tp,
                    struct sk_buff *skb)
{
        return 0;
}

static int
rtl8168_rx_vlan_skb(struct rtl8168_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
        return -1;
}

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)

static netdev_features_t rtl8168_fix_features(struct net_device *dev,
                netdev_features_t features)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
        if (dev->mtu > MSS_MAX)
                features &= ~NETIF_F_ALL_TSO;
        if (dev->mtu > ETH_DATA_LEN) {
                features &= ~NETIF_F_ALL_TSO;
                features &= ~NETIF_F_ALL_CSUM;
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        return features;
}

static int rtl8168_hw_set_features(struct net_device *dev,
                                   netdev_features_t features)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 rx_config;

        rx_config = RTL_R32(tp, RxConfig);
        if (features & NETIF_F_RXALL)
                rx_config |= (AcceptErr | AcceptRunt);
        else
                rx_config &= ~(AcceptErr | AcceptRunt);

        RTL_W32(tp, RxConfig, rx_config);

        if (features & NETIF_F_RXCSUM)
                tp->cp_cmd |= RxChkSum;
        else
                tp->cp_cmd &= ~RxChkSum;

        if (dev->features & NETIF_F_HW_VLAN_RX)
                tp->cp_cmd |= RxVlan;
        else
                tp->cp_cmd &= ~RxVlan;

        RTL_W16(tp, CPlusCmd, tp->cp_cmd);
        RTL_R16(tp, CPlusCmd);

        return 0;
}

static int rtl8168_set_features(struct net_device *dev,
                                netdev_features_t features)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        features &= NETIF_F_RXALL | NETIF_F_RXCSUM | NETIF_F_HW_VLAN_RX;

        spin_lock_irqsave(&tp->lock, flags);
        if (features ^ dev->features)
                rtl8168_hw_set_features(dev, features);
        spin_unlock_irqrestore(&tp->lock, flags);

        return 0;
}

#endif

static void rtl8168_gset_xmii(struct net_device *dev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
                              struct ethtool_cmd *cmd
#else
                              struct ethtool_link_ksettings *cmd
#endif
                             )
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u8 status;
        u8 autoneg, duplex;
        u32 speed = 0;
        u16 bmcr, bmsr, anlpar, ctrl1000 = 0, stat1000 = 0;
        u32 supported, advertising, lp_advertising;
        unsigned long flags;

        supported = SUPPORTED_10baseT_Half |
                    SUPPORTED_10baseT_Full |
                    SUPPORTED_100baseT_Half |
                    SUPPORTED_100baseT_Full |
                    SUPPORTED_1000baseT_Full |
                    SUPPORTED_Autoneg |
                    SUPPORTED_TP |
                    SUPPORTED_Pause |
                    SUPPORTED_Asym_Pause;

        advertising = ADVERTISED_TP;

        spin_lock_irqsave(&tp->lock, flags);
        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        bmcr = rtl8168_mdio_read(tp, MII_BMCR);
        bmsr = rtl8168_mdio_read(tp, MII_BMSR);
        anlpar = rtl8168_mdio_read(tp, MII_LPA);
        ctrl1000 = rtl8168_mdio_read(tp, MII_CTRL1000);
        stat1000 = rtl8168_mdio_read(tp, MII_STAT1000);
        spin_unlock_irqrestore(&tp->lock, flags);

        if (bmcr & BMCR_ANENABLE) {
                advertising |= ADVERTISED_Autoneg;
                autoneg = AUTONEG_ENABLE;

                if (bmsr & BMSR_ANEGCOMPLETE) {
                        lp_advertising = mii_lpa_to_ethtool_lpa_t(anlpar);
                        lp_advertising |=
                                mii_stat1000_to_ethtool_lpa_t(stat1000);
                } else {
                        lp_advertising = 0;
                }

                if (tp->phy_auto_nego_reg & ADVERTISE_10HALF)
                        advertising |= ADVERTISED_10baseT_Half;
                if (tp->phy_auto_nego_reg & ADVERTISE_10FULL)
                        advertising |= ADVERTISED_10baseT_Full;
                if (tp->phy_auto_nego_reg & ADVERTISE_100HALF)
                        advertising |= ADVERTISED_100baseT_Half;
                if (tp->phy_auto_nego_reg & ADVERTISE_100FULL)
                        advertising |= ADVERTISED_100baseT_Full;
                if (tp->phy_1000_ctrl_reg & ADVERTISE_1000FULL)
                        advertising |= ADVERTISED_1000baseT_Full;
        } else {
                autoneg = AUTONEG_DISABLE;
                lp_advertising = 0;
        }

        status = RTL_R8(tp, PHYstatus);

        if (status & LinkStatus) {
                /*link on*/
                if (status & _1000bpsF)
                        speed = SPEED_1000;
                else if (status & _100bps)
                        speed = SPEED_100;
                else if (status & _10bps)
                        speed = SPEED_10;

                if (status & TxFlowCtrl)
                        advertising |= ADVERTISED_Asym_Pause;

                if (status & RxFlowCtrl)
                        advertising |= ADVERTISED_Pause;

                duplex = ((status & _1000bpsF) || (status & FullDup)) ?
                         DUPLEX_FULL : DUPLEX_HALF;
        } else {
                /*link down*/
                speed = SPEED_UNKNOWN;
                duplex = DUPLEX_UNKNOWN;
        }

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        cmd->supported = supported;
        cmd->advertising = advertising;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
        cmd->lp_advertising = lp_advertising;
#endif
        cmd->autoneg = autoneg;
        cmd->speed = speed;
        cmd->duplex = duplex;
        cmd->port = PORT_TP;
#else
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
                                                supported);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
                                                advertising);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.lp_advertising,
                                                lp_advertising);
        cmd->base.autoneg = autoneg;
        cmd->base.speed = speed;
        cmd->base.duplex = duplex;
        cmd->base.port = PORT_TP;
#endif
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
static int
rtl8168_get_settings(struct net_device *dev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
                     struct ethtool_cmd *cmd
#else
                     struct ethtool_link_ksettings *cmd
#endif
                    )
{
        struct rtl8168_private *tp = netdev_priv(dev);

        tp->get_settings(dev, cmd);

        return 0;
}

static void rtl8168_get_regs(struct net_device *dev, struct ethtool_regs *regs,
                             void *p)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;
        unsigned int i;
        u8 *data = p;
        unsigned long flags;

        if (regs->len < R8168_REGS_DUMP_SIZE)
                return /* -EINVAL */;

        memset(p, 0, regs->len);

        spin_lock_irqsave(&tp->lock, flags);
        for (i = 0; i < R8168_MAC_REGS_SIZE; i++)
                *data++ = readb(ioaddr + i);
        data = (u8*)p + 256;

        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        for (i = 0; i < R8168_PHY_REGS_SIZE/2; i++) {
                *(u16*)data = rtl8168_mdio_read(tp, i);
                data += 2;
        }
        data = (u8*)p + 256 * 2;

        for (i = 0; i < R8168_EPHY_REGS_SIZE/2; i++) {
                *(u16*)data = rtl8168_ephy_read(tp, i);
                data += 2;
        }
        data = (u8*)p + 256 * 3;

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
                /* RTL8168B does not support Extend GMAC */
                break;
        default:
                for (i = 0; i < R8168_ERI_REGS_SIZE; i+=4) {
                        *(u32*)data = rtl8168_eri_read(tp, i , 4, ERIAR_ExGMAC);
                        data += 4;
                }
                break;
        }
        spin_unlock_irqrestore(&tp->lock, flags);
}

static u32
rtl8168_get_msglevel(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        return tp->msg_enable;
}

static void
rtl8168_set_msglevel(struct net_device *dev,
                     u32 value)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        tp->msg_enable = value;
}

static const char rtl8168_gstrings[][ETH_GSTRING_LEN] = {
        "tx_packets",
        "rx_packets",
        "tx_errors",
        "rx_errors",
        "rx_missed",
        "align_errors",
        "tx_single_collisions",
        "tx_multi_collisions",
        "unicast",
        "broadcast",
        "multicast",
        "tx_aborted",
        "tx_underrun",
};
#endif //#LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
static int rtl8168_get_stats_count(struct net_device *dev)
{
        return ARRAY_SIZE(rtl8168_gstrings);
}
#endif //#LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
#else
static int rtl8168_get_sset_count(struct net_device *dev, int sset)
{
        switch (sset) {
        case ETH_SS_STATS:
                return ARRAY_SIZE(rtl8168_gstrings);
        default:
                return -EOPNOTSUPP;
        }
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
static void
rtl8168_get_ethtool_stats(struct net_device *dev,
                          struct ethtool_stats *stats,
                          u64 *data)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct rtl8168_counters *counters;
        dma_addr_t paddr;
        u32 cmd;
        u32 WaitCnt;
        unsigned long flags;

        ASSERT_RTNL();

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters)
                return;

        spin_lock_irqsave(&tp->lock, flags);
        RTL_W32(tp, CounterAddrHigh, (u64)paddr >> 32);
        cmd = (u64)paddr & DMA_BIT_MASK(32);
        RTL_W32(tp, CounterAddrLow, cmd);
        RTL_W32(tp, CounterAddrLow, cmd | CounterDump);

        WaitCnt = 0;
        while (RTL_R32(tp, CounterAddrLow) & CounterDump) {
                udelay(10);

                WaitCnt++;
                if (WaitCnt > 20)
                        break;
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        data[0] = le64_to_cpu(counters->tx_packets);
        data[1] = le64_to_cpu(counters->rx_packets);
        data[2] = le64_to_cpu(counters->tx_errors);
        data[3] = le32_to_cpu(counters->rx_errors);
        data[4] = le16_to_cpu(counters->rx_missed);
        data[5] = le16_to_cpu(counters->align_errors);
        data[6] = le32_to_cpu(counters->tx_one_collision);
        data[7] = le32_to_cpu(counters->tx_multi_collision);
        data[8] = le64_to_cpu(counters->rx_unicast);
        data[9] = le64_to_cpu(counters->rx_broadcast);
        data[10] = le32_to_cpu(counters->rx_multicast);
        data[11] = le16_to_cpu(counters->tx_aborted);
        data[12] = le16_to_cpu(counters->tx_underun);
}

static void
rtl8168_get_strings(struct net_device *dev,
                    u32 stringset,
                    u8 *data)
{
        switch (stringset) {
        case ETH_SS_STATS:
                memcpy(data, *rtl8168_gstrings, sizeof(rtl8168_gstrings));
                break;
        }
}
#endif //#LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)

static int rtl_get_eeprom_len(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        return tp->eeprom_len;
}

static int rtl_get_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom, u8 *buf)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int i,j,ret;
        int start_w, end_w;
        int VPD_addr, VPD_data;
        u32 *eeprom_buff;
        u16 tmp;

        if (tp->eeprom_type == EEPROM_TYPE_NONE) {
                dev_printk(KERN_DEBUG, tp_to_dev(tp), "Detect none EEPROM\n");
                return -EOPNOTSUPP;
        } else if (eeprom->len == 0 || (eeprom->offset+eeprom->len) > tp->eeprom_len) {
                dev_printk(KERN_DEBUG, tp_to_dev(tp), "Invalid parameter\n");
                return -EINVAL;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_9:
        case CFG_METHOD_10:
                VPD_addr = 0xCE;
                VPD_data = 0xD0;
                break;

        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
                return -EOPNOTSUPP;
        default:
                VPD_addr = 0xD2;
                VPD_data = 0xD4;
                break;
        }

        start_w = eeprom->offset >> 2;
        end_w = (eeprom->offset + eeprom->len - 1) >> 2;

        eeprom_buff = kmalloc(sizeof(u32)*(end_w - start_w + 1), GFP_KERNEL);
        if (!eeprom_buff)
                return -ENOMEM;

        rtl8168_enable_cfg9346_write(tp);
        ret = -EFAULT;
        for (i=start_w; i<=end_w; i++) {
                pci_write_config_word(tp->pci_dev, VPD_addr, (u16)i*4);
                ret = -EFAULT;
                for (j = 0; j < 10; j++) {
                        udelay(400);
                        pci_read_config_word(tp->pci_dev, VPD_addr, &tmp);
                        if (tmp&0x8000) {
                                ret = 0;
                                break;
                        }
                }

                if (ret)
                        break;

                pci_read_config_dword(tp->pci_dev, VPD_data, &eeprom_buff[i-start_w]);
        }
        rtl8168_disable_cfg9346_write(tp);

        if (!ret)
                memcpy(buf, (u8 *)eeprom_buff + (eeprom->offset & 3), eeprom->len);

        kfree(eeprom_buff);

        return ret;
}

#undef ethtool_op_get_link
#define ethtool_op_get_link _kc_ethtool_op_get_link
static u32 _kc_ethtool_op_get_link(struct net_device *dev)
{
        return netif_carrier_ok(dev) ? 1 : 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
#undef ethtool_op_get_sg
#define ethtool_op_get_sg _kc_ethtool_op_get_sg
static u32 _kc_ethtool_op_get_sg(struct net_device *dev)
{
#ifdef NETIF_F_SG
        return (dev->features & NETIF_F_SG) != 0;
#else
        return 0;
#endif
}

#undef ethtool_op_set_sg
#define ethtool_op_set_sg _kc_ethtool_op_set_sg
static int _kc_ethtool_op_set_sg(struct net_device *dev, u32 data)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (tp->mcfg == CFG_METHOD_DEFAULT)
                return -EOPNOTSUPP;

#ifdef NETIF_F_SG
        if (data)
                dev->features |= NETIF_F_SG;
        else
                dev->features &= ~NETIF_F_SG;
#endif

        return 0;
}
#endif

static int rtl8168_enable_EEE(struct rtl8168_private *tp)
{
        int ret;
        u16 data;
        u32 csi_tmp;

        ret = 0;
        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0020);
                data = rtl8168_mdio_read(tp, 0x15) | 0x0100;
                rtl8168_mdio_write(tp, 0x15, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                data = rtl8168_mdio_read(tp, 0x06) | 0x2000;
                rtl8168_mdio_write(tp, 0x06, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0006);
                rtl8168_mdio_write(tp, 0x00, 0x5A30);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0007);
                rtl8168_mdio_write(tp, 0x0E, 0x003C);
                rtl8168_mdio_write(tp, 0x0D, 0x4007);
                rtl8168_mdio_write(tp, 0x0E, 0x0006);
                rtl8168_mdio_write(tp, 0x0D, 0x0000);
                if ((RTL_R8(tp, Config4)&0x40) && (RTL_R8(tp, 0x6D) & BIT_7)) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8AC8);
                        rtl8168_mdio_write(tp, 0x06, RTL_R16(tp, tp->NicCustLedValue));
                        rtl8168_mdio_write(tp, 0x05, 0x8B82);
                        data = rtl8168_mdio_read(tp, 0x06) | 0x0010;
                        rtl8168_mdio_write(tp, 0x05, 0x8B82);
                        rtl8168_mdio_write(tp, 0x06, data);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }
                break;

        case CFG_METHOD_16:
        case CFG_METHOD_17:
                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC) | 0x0003;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mdio_write(tp,0x1F , 0x0004);
                rtl8168_mdio_write(tp,0x1F , 0x0007);
                rtl8168_mdio_write(tp,0x1E , 0x0020);
                data = rtl8168_mdio_read(tp, 0x15)|0x0100;
                rtl8168_mdio_write(tp,0x15 , data);
                rtl8168_mdio_write(tp,0x1F , 0x0002);
                rtl8168_mdio_write(tp,0x1F , 0x0005);
                rtl8168_mdio_write(tp,0x05 , 0x8B85);
                data = rtl8168_mdio_read(tp, 0x06)|0x2000;
                rtl8168_mdio_write(tp,0x06 , data);
                rtl8168_mdio_write(tp,0x1F , 0x0000);
                rtl8168_mdio_write(tp,0x0D , 0x0007);
                rtl8168_mdio_write(tp,0x0E , 0x003C);
                rtl8168_mdio_write(tp,0x0D , 0x4007);
                rtl8168_mdio_write(tp,0x0E , 0x0006);
                rtl8168_mdio_write(tp,0x0D , 0x0000);
                break;

        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp |= BIT_1 | BIT_0;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x0020);
                data = rtl8168_mdio_read(tp, 0x15);
                data |= BIT_8;
                rtl8168_mdio_write(tp, 0x15, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                data = rtl8168_mdio_read(tp, 0x06);
                data |= BIT_13;
                rtl8168_mdio_write(tp, 0x06, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0007);
                rtl8168_mdio_write(tp, 0x0E, 0x003C);
                rtl8168_mdio_write(tp, 0x0D, 0x4007);
                rtl8168_mdio_write(tp, 0x0E, 0x0006);
                rtl8168_mdio_write(tp, 0x0D, 0x0000);
                break;

        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp |= BIT_1 | BIT_0;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x11);
                rtl8168_mdio_write(tp, 0x11, data | BIT_4);
                rtl8168_mdio_write(tp, 0x1F, 0x0A5D);
                rtl8168_mdio_write(tp, 0x10, tp->eee_adv_t);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;

        default:
//      dev_printk(KERN_DEBUG, tp_to_dev(tp), "Not Support EEE\n");
                ret = -EOPNOTSUPP;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mdio_write(tp, 0x1F, 0x0A4A);
                rtl8168_set_eth_phy_bit(tp, 0x11, BIT_9);
                rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                rtl8168_set_eth_phy_bit(tp, 0x14, BIT_7);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        }

        /*Advanced EEE*/
        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_oob_mutex_lock(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_set_phy_mcu_patch_request(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_25:
                rtl8168_eri_write(tp, 0x1EA, 1, 0xFA, ERIAR_ExGMAC);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x10);
                if (data & BIT_10) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                        data = rtl8168_mdio_read(tp, 0x16);
                        data &= ~(BIT_1);
                        rtl8168_mdio_write(tp, 0x16, data);
                } else {
                        rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                        data = rtl8168_mdio_read(tp, 0x16);
                        data |= BIT_1;
                        rtl8168_mdio_write(tp, 0x16, data);
                }
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_26:
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data |= BIT_0;
                rtl8168_mac_ocp_write(tp, 0xE052, data);
                data = rtl8168_mac_ocp_read(tp, 0xE056);
                data &= 0xFF0F;
                data |= (BIT_4 | BIT_5 | BIT_6);
                rtl8168_mac_ocp_write(tp, 0xE056, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x10);
                if (data & BIT_10) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                        data = rtl8168_mdio_read(tp, 0x16);
                        data &= ~(BIT_1);
                        rtl8168_mdio_write(tp, 0x16, data);
                } else {
                        rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                        data = rtl8168_mdio_read(tp, 0x16);
                        data |= BIT_1;
                        rtl8168_mdio_write(tp, 0x16, data);
                }
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data &= ~BIT_0;
                rtl8168_mac_ocp_write(tp, 0xE052, data);
                data = rtl8168_mac_ocp_read(tp, 0xE056);
                data &= 0xFF0F;
                data |= (BIT_4 | BIT_5 | BIT_6);
                rtl8168_mac_ocp_write(tp, 0xE056, data);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data &= ~(BIT_0);
                rtl8168_mac_ocp_write(tp, 0xE052, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x10) | BIT_15;
                rtl8168_mdio_write(tp, 0x10, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                data = rtl8168_mdio_read(tp, 0x11) | BIT_13 | BIT_14;
                data &= ~(BIT_12);
                rtl8168_mdio_write(tp, 0x11, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                /*
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data |= BIT_0;
                rtl8168_mac_ocp_write(tp, 0xE052, data);
                */

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x10) | BIT_15;
                rtl8168_mdio_write(tp, 0x10, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                data = rtl8168_mdio_read(tp, 0x11) | BIT_13 | BIT_14;
                data &= ~(BIT_12);
                rtl8168_mdio_write(tp, 0x11, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_clear_phy_mcu_patch_request(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_oob_mutex_unlock(tp);
                break;
        }

        return ret;
}

static int rtl8168_disable_EEE(struct rtl8168_private *tp)
{
        int ret;
        u16 data;
        u32 csi_tmp;

        ret = 0;
        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                data = rtl8168_mdio_read(tp, 0x06) & ~0x2000;
                rtl8168_mdio_write(tp, 0x06, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0020);
                data = rtl8168_mdio_read(tp, 0x15) & ~0x0100;
                rtl8168_mdio_write(tp, 0x15, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0006);
                rtl8168_mdio_write(tp, 0x00, 0x5A00);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0007);
                rtl8168_mdio_write(tp, 0x0E, 0x003C);
                rtl8168_mdio_write(tp, 0x0D, 0x4007);
                rtl8168_mdio_write(tp, 0x0E, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                if (RTL_R8(tp, Config4) & 0x40) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8B82);
                        data = rtl8168_mdio_read(tp, 0x06) & ~0x0010;
                        rtl8168_mdio_write(tp, 0x05, 0x8B82);
                        rtl8168_mdio_write(tp, 0x06, data);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }
                break;

        case CFG_METHOD_16:
        case CFG_METHOD_17:
                csi_tmp = rtl8168_eri_read(tp, 0x1B0,4, ERIAR_ExGMAC)& ~0x0003;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                data = rtl8168_mdio_read(tp, 0x06) & ~0x2000;
                rtl8168_mdio_write(tp, 0x06, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0004);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0020);
                data = rtl8168_mdio_read(tp, 0x15) & ~0x0100;
                rtl8168_mdio_write(tp,0x15 , data);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0007);
                rtl8168_mdio_write(tp, 0x0E, 0x003C);
                rtl8168_mdio_write(tp, 0x0D, 0x4007);
                rtl8168_mdio_write(tp, 0x0E, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;

        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_1 | BIT_0);
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                data = rtl8168_mdio_read(tp, 0x06);
                data &= ~BIT_13;
                rtl8168_mdio_write(tp, 0x06, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x0020);
                data = rtl8168_mdio_read(tp, 0x15);
                data &= ~BIT_8;
                rtl8168_mdio_write(tp, 0x15, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0007);
                rtl8168_mdio_write(tp, 0x0E, 0x003C);
                rtl8168_mdio_write(tp, 0x0D, 0x4007);
                rtl8168_mdio_write(tp, 0x0E, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;

        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_1 | BIT_0);
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x11);
                rtl8168_mdio_write(tp, 0x11, data & ~BIT_4);
                rtl8168_mdio_write(tp, 0x1F, 0x0A5D);
                rtl8168_mdio_write(tp, 0x10, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;

        default:
//      dev_printk(KERN_DEBUG, tp_to_dev(tp), "Not Support EEE\n");
                ret = -EOPNOTSUPP;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                rtl8168_clear_eth_phy_bit(tp, 0x14, BIT_7);
                rtl8168_mdio_write(tp, 0x1F, 0x0A4A);
                rtl8168_clear_eth_phy_bit(tp, 0x11, BIT_9);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        }

        /*Advanced EEE*/
        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_oob_mutex_lock(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_set_phy_mcu_patch_request(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_25:
                rtl8168_eri_write(tp, 0x1EA, 1, 0x00, ERIAR_ExGMAC);

                rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                data = rtl8168_mdio_read(tp, 0x16);
                data &= ~(BIT_1);
                rtl8168_mdio_write(tp, 0x16, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_26:
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data &= ~(BIT_0);
                rtl8168_mac_ocp_write(tp, 0xE052, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                data = rtl8168_mdio_read(tp, 0x16);
                data &= ~(BIT_1);
                rtl8168_mdio_write(tp, 0x16, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data &= ~(BIT_0);
                rtl8168_mac_ocp_write(tp, 0xE052, data);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                data = rtl8168_mac_ocp_read(tp, 0xE052);
                data &= ~(BIT_0);
                rtl8168_mac_ocp_write(tp, 0xE052, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                data = rtl8168_mdio_read(tp, 0x10) & ~(BIT_15);
                rtl8168_mdio_write(tp, 0x10, data);

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                data = rtl8168_mdio_read(tp, 0x11) & ~(BIT_12 | BIT_13 | BIT_14);
                rtl8168_mdio_write(tp, 0x11, data);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_clear_phy_mcu_patch_request(tp);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_oob_mutex_unlock(tp);
                break;
        }

        return ret;
}

static int rtl_nway_reset(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
        int ret, bmcr;

        spin_lock_irqsave(&tp->lock, flags);

        if (unlikely(tp->rtk_enable_diag)) {
                spin_unlock_irqrestore(&tp->lock, flags);
                return -EBUSY;
        }

        /* if autoneg is off, it's an error */
        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        bmcr = rtl8168_mdio_read(tp, MII_BMCR);

        if (bmcr & BMCR_ANENABLE) {
                bmcr |= BMCR_ANRESTART;
                rtl8168_mdio_write(tp, MII_BMCR, bmcr);
                ret = 0;
        } else {
                ret = -EINVAL;
        }

        spin_unlock_irqrestore(&tp->lock, flags);

        return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
static int
rtl_ethtool_get_eee(struct net_device *net, struct ethtool_eee *eee)
{
        struct rtl8168_private *tp = netdev_priv(net);
        u32 lp, adv, supported = 0;
        unsigned long flags;
        u16 val;

        switch (tp->mcfg) {
        case CFG_METHOD_21 ... CFG_METHOD_33:
                break;
        default:
                return -EOPNOTSUPP;
        }

        spin_lock_irqsave(&tp->lock, flags);

        if (unlikely(tp->rtk_enable_diag)) {
                spin_unlock_irqrestore(&tp->lock, flags);
                return -EBUSY;
        }

        rtl8168_mdio_write(tp, 0x1F, 0x0A5C);
        val = rtl8168_mdio_read(tp, 0x12);
        supported = mmd_eee_cap_to_ethtool_sup_t(val);

        rtl8168_mdio_write(tp, 0x1F, 0x0A5D);
        val = rtl8168_mdio_read(tp, 0x10);
        adv = mmd_eee_adv_to_ethtool_adv_t(val);

        val = rtl8168_mdio_read(tp, 0x11);
        lp = mmd_eee_adv_to_ethtool_adv_t(val);

        val = rtl8168_eri_read(tp, 0x1B0, 2, ERIAR_ExGMAC);
        val &= BIT_1 | BIT_0;

        rtl8168_mdio_write(tp, 0x1F, 0x0000);

        spin_unlock_irqrestore(&tp->lock, flags);

        eee->eee_enabled = !!val;
        eee->eee_active = !!(supported & adv & lp);
        eee->supported = supported;
        eee->advertised = adv;
        eee->lp_advertised = lp;

        return 0;
}

static int
rtl_ethtool_set_eee(struct net_device *net, struct ethtool_eee *eee)
{
        struct rtl8168_private *tp = netdev_priv(net);
        unsigned long flags;

        switch (tp->mcfg) {
        case CFG_METHOD_21 ... CFG_METHOD_33:
                break;
        default:
                return -EOPNOTSUPP;
        }

        if (HW_SUPP_SERDES_PHY(tp) ||
            !HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp) ||
            tp->DASH)
                return -EOPNOTSUPP;

        spin_lock_irqsave(&tp->lock, flags);

        if (unlikely(tp->rtk_enable_diag)) {
                spin_unlock_irqrestore(&tp->lock, flags);
                return -EBUSY;
        }

        tp->eee_enabled = eee->eee_enabled;
        tp->eee_adv_t = ethtool_adv_to_mmd_eee_adv_t(eee->advertised);

        if (tp->eee_enabled)
                rtl8168_enable_EEE(tp);
        else
                rtl8168_disable_EEE(tp);

        spin_unlock_irqrestore(&tp->lock, flags);

        rtl_nway_reset(net);

        return 0;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0) */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
static const struct ethtool_ops rtl8168_ethtool_ops = {
        .get_drvinfo        = rtl8168_get_drvinfo,
        .get_regs_len       = rtl8168_get_regs_len,
        .get_link       = ethtool_op_get_link,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
        .get_settings       = rtl8168_get_settings,
        .set_settings       = rtl8168_set_settings,
#else
        .get_link_ksettings       = rtl8168_get_settings,
        .set_link_ksettings       = rtl8168_set_settings,
#endif
        .get_msglevel       = rtl8168_get_msglevel,
        .set_msglevel       = rtl8168_set_msglevel,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
        .get_rx_csum        = rtl8168_get_rx_csum,
        .set_rx_csum        = rtl8168_set_rx_csum,
        .get_tx_csum        = rtl8168_get_tx_csum,
        .set_tx_csum        = rtl8168_set_tx_csum,
        .get_sg         = ethtool_op_get_sg,
        .set_sg         = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
        .get_tso        = ethtool_op_get_tso,
        .set_tso        = ethtool_op_set_tso,
#endif
#endif
        .get_regs       = rtl8168_get_regs,
        .get_wol        = rtl8168_get_wol,
        .set_wol        = rtl8168_set_wol,
        .get_strings        = rtl8168_get_strings,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
        .get_stats_count    = rtl8168_get_stats_count,
#else
        .get_sset_count     = rtl8168_get_sset_count,
#endif
        .get_ethtool_stats  = rtl8168_get_ethtool_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#ifdef ETHTOOL_GPERMADDR
        .get_perm_addr      = ethtool_op_get_perm_addr,
#endif
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
        .get_eeprom     = rtl_get_eeprom,
        .get_eeprom_len     = rtl_get_eeprom_len,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
        .get_ts_info        = ethtool_op_get_ts_info,
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
        .get_eee = rtl_ethtool_get_eee,
        .set_eee = rtl_ethtool_set_eee,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0) */
        .nway_reset = rtl_nway_reset,
};
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)

#if 0

static int rtl8168_enable_green_feature(struct rtl8168_private *tp)
{
        u16 gphy_val;
        unsigned long flags;

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                gphy_val = rtl8168_mdio_read(tp, 0x10) | 0x0400;
                rtl8168_mdio_write(tp, 0x10, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x19) | 0x0001;
                rtl8168_mdio_write(tp, 0x19, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                gphy_val = rtl8168_mdio_read(tp, 0x01) & ~0x0100;
                rtl8168_mdio_write(tp, 0x01, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x00, 0x9200);
                mdelay(20);
                break;

        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
                rtl8168_mdio_write(tp, 0x1f, 0x0003);
                gphy_val = rtl8168_mdio_read(tp, 0x10);
                gphy_val |= BIT_10;
                rtl8168_mdio_write(tp, 0x10, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x19);
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x19, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                gphy_val = rtl8168_mdio_read(tp, 0x01);
                gphy_val |= BIT_8;
                rtl8168_mdio_write(tp, 0x01, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                rtl8168_mdio_write(tp, 0x00, 0x9200);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8011);
                rtl8168_set_eth_phy_bit( tp, 0x14, BIT_14 );
                rtl8168_mdio_write(tp, 0x1F, 0x0A40);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x00, 0x9200);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8045);
                rtl8168_mdio_write(tp, 0x14, 0x0000);
                rtl8168_mdio_write(tp, 0x13, 0x804d);
                rtl8168_mdio_write(tp, 0x14, 0x1222);
                rtl8168_mdio_write(tp, 0x13, 0x805d);
                rtl8168_mdio_write(tp, 0x14, 0x0022);
                rtl8168_mdio_write(tp, 0x13, 0x8011);
                rtl8168_set_eth_phy_bit( tp, 0x14, BIT_15 );
                rtl8168_mdio_write(tp, 0x1F, 0x0A40);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x00, 0x9200);
                break;
        default:
                dev_printk(KERN_DEBUG, tp_to_dev(tp), "Not Support Green Feature\n");
                break;
        }

        return 0;
}

static int rtl8168_disable_green_feature(struct rtl8168_private *tp)
{
        u16 gphy_val;
        unsigned long flags;

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                gphy_val = rtl8168_mdio_read(tp, 0x01) | 0x0100;
                rtl8168_mdio_write(tp, 0x01, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                gphy_val = rtl8168_mdio_read(tp, 0x10) & ~0x0400;
                rtl8168_mdio_write(tp, 0x10, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x19) & ~0x0001;
                rtl8168_mdio_write(tp, 0x19, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x06) & ~0x7000;
                gphy_val |= 0x3000;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x0D) & 0x0700;
                gphy_val |= 0x0500;
                rtl8168_mdio_write(tp, 0x0D, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;

        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
                rtl8168_mdio_write(tp, 0x1f, 0x0003);
                gphy_val = rtl8168_mdio_read(tp, 0x19);
                gphy_val &= ~BIT_0;
                rtl8168_mdio_write(tp, 0x19, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x10);
                gphy_val &= ~BIT_10;
                rtl8168_mdio_write(tp, 0x10, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8011);
                rtl8168_clear_eth_phy_bit( tp, 0x14, BIT_14 );
                rtl8168_mdio_write(tp, 0x1F, 0x0A40);
                rtl8168_mdio_write(tp, 0x00, 0x9200);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8045);
                rtl8168_mdio_write(tp, 0x14, 0x2444);
                rtl8168_mdio_write(tp, 0x13, 0x804d);
                rtl8168_mdio_write(tp, 0x14, 0x2444);
                rtl8168_mdio_write(tp, 0x13, 0x805d);
                rtl8168_mdio_write(tp, 0x14, 0x2444);
                rtl8168_mdio_write(tp, 0x13, 0x8011);
                rtl8168_set_eth_phy_bit( tp, 0x14, BIT_15 );
                rtl8168_mdio_write(tp, 0x1F, 0x0A40);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x00, 0x9200);
                break;
        default:
                dev_printk(KERN_DEBUG, tp_to_dev(tp), "Not Support Green Feature\n");
                break;
        }

        return 0;
}

#endif

static void rtl8168_get_mac_version(struct rtl8168_private *tp)
{
        u32 reg,val32;
        u32 ICVerID;

        val32 = RTL_R32(tp, TxConfig);
        reg = val32 & 0x7c800000;
        ICVerID = val32 & 0x00700000;

        switch (reg) {
        case 0x30000000:
                tp->mcfg = CFG_METHOD_1;
                tp->efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        case 0x38000000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_2;
                } else if (ICVerID == 0x00500000) {
                        tp->mcfg = CFG_METHOD_3;
                } else {
                        tp->mcfg = CFG_METHOD_3;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        case 0x3C000000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_4;
                } else if (ICVerID == 0x00200000) {
                        tp->mcfg = CFG_METHOD_5;
                } else if (ICVerID == 0x00400000) {
                        tp->mcfg = CFG_METHOD_6;
                } else {
                        tp->mcfg = CFG_METHOD_6;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        case 0x3C800000:
                if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_7;
                } else if (ICVerID == 0x00300000) {
                        tp->mcfg = CFG_METHOD_8;
                } else {
                        tp->mcfg = CFG_METHOD_8;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        case 0x28000000:
                if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_9;
                } else if (ICVerID == 0x00300000) {
                        tp->mcfg = CFG_METHOD_10;
                } else {
                        tp->mcfg = CFG_METHOD_10;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V1;
                break;
        case 0x28800000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_11;
                } else if (ICVerID == 0x00200000) {
                        tp->mcfg = CFG_METHOD_12;
                        RTL_W32(tp, 0xD0, RTL_R32(tp, 0xD0) | 0x00020000);
                } else if (ICVerID == 0x00300000) {
                        tp->mcfg = CFG_METHOD_13;
                } else {
                        tp->mcfg = CFG_METHOD_13;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V1;
                break;
        case 0x2C000000:
                if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_14;
                } else if (ICVerID == 0x00200000) {
                        tp->mcfg = CFG_METHOD_15;
                } else {
                        tp->mcfg = CFG_METHOD_15;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V2;
                break;
        case 0x2C800000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_16;
                } else if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_17;
                } else {
                        tp->mcfg = CFG_METHOD_17;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x48000000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_18;
                } else if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_19;
                } else {
                        tp->mcfg = CFG_METHOD_19;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x48800000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_20;
                } else {
                        tp->mcfg = CFG_METHOD_20;
                        tp->HwIcVerUnknown = TRUE;
                }

                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x4C000000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_21;
                } else if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_22;
                } else {
                        tp->mcfg = CFG_METHOD_22;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x50000000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_23;
                } else if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_27;
                } else if (ICVerID == 0x00200000) {
                        tp->mcfg = CFG_METHOD_28;
                } else {
                        tp->mcfg = CFG_METHOD_28;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x50800000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_24;
                } else if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_25;
                } else {
                        tp->mcfg = CFG_METHOD_25;
                        tp->HwIcVerUnknown = TRUE;
                }
                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x5C800000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_26;
                } else {
                        tp->mcfg = CFG_METHOD_26;
                        tp->HwIcVerUnknown = TRUE;
                }

                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x54000000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_29;
                } else if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_30;
                } else {
                        tp->mcfg = CFG_METHOD_30;
                        tp->HwIcVerUnknown = TRUE;
                }

                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case 0x54800000:
                if (ICVerID == 0x00100000) {
                        tp->mcfg = CFG_METHOD_31;
                } else if (ICVerID == 0x00200000) {
                        tp->mcfg = CFG_METHOD_32;
                } else if (ICVerID == 0x00300000) {
                        tp->mcfg = CFG_METHOD_33;
                } else {
                        tp->mcfg = CFG_METHOD_33;
                        tp->HwIcVerUnknown = TRUE;
                }

                tp->efuse_ver = EFUSE_SUPPORT_V3;
                break;
        default:
                printk("unknown chip version (%x)\n",reg);
                tp->mcfg = CFG_METHOD_DEFAULT;
                tp->HwIcVerUnknown = TRUE;
                tp->efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        }
}

static void
rtl8168_print_mac_version(struct rtl8168_private *tp)
{
        int i;
        for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
                if (tp->mcfg == rtl_chip_info[i].mcfg) {
                        dprintk("Realtek PCIe GbE Family Controller mcfg = %04d\n",
                                rtl_chip_info[i].mcfg);
                        return;
                }
        }

        dprintk("mac_version == Unknown\n");
}

static u8 rtl8168_calc_efuse_dummy_bit(u16 reg)
{
        int s,a,b;
        u8 dummyBitPos = 0;


        s=reg% 32;
        a=s % 16;
        b=s/16;

        if (s/16) {
                dummyBitPos = (u8)(16-a);
        } else {
                dummyBitPos = (u8)a;
        }

        return dummyBitPos;
}

static u32 rtl8168_decode_efuse_cmd(struct rtl8168_private *tp, u32 DwCmd)
{
        u16 reg = (u16)((DwCmd & 0x00FE0000) >> 17);
        u32 DummyPos = rtl8168_calc_efuse_dummy_bit(reg);
        u32 DeCodeDwCmd = DwCmd;
        u32 Dw17BitData;


        if (tp->efuse_ver < 3) {
                DeCodeDwCmd = (DwCmd>>(DummyPos+1))<<DummyPos;
                if (DummyPos > 0) {
                        DeCodeDwCmd |= ((DwCmd<<(32-DummyPos))>>(32-DummyPos));
                }
        } else {
                reg = (u16)((DwCmd & 0x007F0000) >> 16);
                DummyPos = rtl8168_calc_efuse_dummy_bit(reg);
                Dw17BitData = ((DwCmd & BIT_23) >> 23);
                Dw17BitData <<= 16;
                Dw17BitData |= (DwCmd & 0x0000FFFF);
                DeCodeDwCmd = (Dw17BitData>>(DummyPos+1))<<DummyPos;
                if (DummyPos > 0) {
                        DeCodeDwCmd |= ((Dw17BitData<<(32-DummyPos))>>(32-DummyPos));
                }
        }

        return DeCodeDwCmd;
}

static u8 rtl8168_efuse_read(struct rtl8168_private *tp, u16 reg)
{
        u8 efuse_data = 0;
        u32 temp;
        int cnt;

        if (tp->efuse_ver == EFUSE_NOT_SUPPORT)
                return EFUSE_READ_FAIL;

        if (tp->efuse_ver == EFUSE_SUPPORT_V1) {
                temp = EFUSE_READ | ((reg & EFUSE_Reg_Mask) << EFUSE_Reg_Shift);
                RTL_W32(tp, EFUSEAR, temp);

                cnt = 0;
                do {
                        udelay(100);
                        temp = RTL_R32(tp, EFUSEAR);
                        cnt++;
                } while (!(temp & EFUSE_READ_OK) && (cnt < EFUSE_Check_Cnt));

                if (cnt == EFUSE_Check_Cnt)
                        efuse_data = EFUSE_READ_FAIL;
                else
                        efuse_data = (u8)(RTL_R32(tp, EFUSEAR) & EFUSE_Data_Mask);
        } else  if (tp->efuse_ver == EFUSE_SUPPORT_V2) {
                temp = (reg/2) & 0x03ff;
                temp <<= 17;
                temp |= EFUSE_READ;
                RTL_W32(tp, EFUSEAR, temp);

                cnt = 0;
                do {
                        udelay(100);
                        temp = RTL_R32(tp, EFUSEAR);
                        cnt++;
                } while (!(temp & EFUSE_READ_OK) && (cnt < EFUSE_Check_Cnt));

                if (cnt == EFUSE_Check_Cnt) {
                        efuse_data = EFUSE_READ_FAIL;
                } else {
                        temp = RTL_R32(tp, EFUSEAR);
                        temp = rtl8168_decode_efuse_cmd(tp, temp);

                        if (reg%2) {
                                temp >>= 8;
                                efuse_data = (u8)temp;
                        } else {
                                efuse_data = (u8)temp;
                        }
                }
        } else  if (tp->efuse_ver == EFUSE_SUPPORT_V3) {
                temp = (reg/2) & 0x03ff;
                temp <<= 16;
                temp |= EFUSE_READ_V3;
                RTL_W32(tp, EFUSEAR, temp);

                cnt = 0;
                do {
                        udelay(100);
                        temp = RTL_R32(tp, EFUSEAR);
                        cnt++;
                } while ((temp & BIT_31) && (cnt < EFUSE_Check_Cnt));

                if (cnt == EFUSE_Check_Cnt) {
                        efuse_data = EFUSE_READ_FAIL;
                } else {
                        temp = RTL_R32(tp, EFUSEAR);
                        temp = rtl8168_decode_efuse_cmd(tp, temp);

                        if (reg%2) {
                                temp >>= 8;
                                efuse_data = (u8)temp;
                        } else {
                                efuse_data = (u8)temp;
                        }
                }
        }

        udelay(20);

        return efuse_data;
}

static void
rtl8168_tally_counter_addr_fill(struct rtl8168_private *tp)
{
        if (!tp->tally_paddr)
                return;

        RTL_W32(tp, CounterAddrHigh, (u64)tp->tally_paddr >> 32);
        RTL_W32(tp, CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32)));
}

static void
rtl8168_tally_counter_clear(struct rtl8168_private *tp)
{
        if (tp->mcfg == CFG_METHOD_1 || tp->mcfg == CFG_METHOD_2 ||
            tp->mcfg == CFG_METHOD_3 )
                return;

        if (!tp->tally_paddr)
                return;

        RTL_W32(tp, CounterAddrHigh, (u64)tp->tally_paddr >> 32);
        RTL_W32(tp, CounterAddrLow, ((u64)tp->tally_paddr & (DMA_BIT_MASK(32))) | CounterReset);
}

static
u16
rtl8168_get_phy_state(struct rtl8168_private *tp)
{
        u16 PhyState = 0xFF;

        if (HW_SUPPORT_UPS_MODE(tp) == FALSE) goto exit;

        switch (tp->HwSuppUpsVer) {
        case 1:
                PhyState = rtl8168_mdio_read_phy_ocp(tp, 0x0A42, 0x10);
                PhyState &= 0x7;  //bit[2:0]
                break;
        }

exit:
        return PhyState;
}

static
bool
rtl8168_wait_phy_state_ready(struct rtl8168_private *tp,
                             u16 PhyState,
                             u32 MicroSecondTimeout
                            )
{
        u16 TmpPhyState;
        u32 WaitCount;
        u32 i = 0;
        bool PhyStateReady = TRUE;

        if (HW_SUPPORT_UPS_MODE(tp) == FALSE) goto exit;

        WaitCount = MicroSecondTimeout / 1000;
        if (WaitCount == 0) WaitCount = 100;

        do {
                TmpPhyState = rtl8168_get_phy_state(tp);
                mdelay(1);
                i++;
        } while ((i < WaitCount) && (TmpPhyState != PhyState));

        PhyStateReady = (i == WaitCount && TmpPhyState != PhyState) ? FALSE : TRUE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(i == WaitCount);
#endif

exit:
        return PhyStateReady;
}

static
bool
rtl8168_test_phy_ocp(struct rtl8168_private *tp)
{
        bool RestorePhyOcpReg = FALSE;

        if (tp->TestPhyOcpReg == FALSE) goto exit;

        if (tp->HwSuppEsdVer == 2) {
                u16 PhyRegValue;
                u8 ResetPhyType = 0;

                if (HW_PHY_STATUS_INI == rtl8168_get_phy_state(tp)) {
                        ResetPhyType = 1;
                } else {
                        rtl8168_mdio_write(tp, 0x1F, 0x0C40);
                        PhyRegValue = rtl8168_mdio_read(tp, 0x12);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        if ((PhyRegValue & 0x03) != 0x00) {
                                ResetPhyType = 2;
                        }
                }

                if (ResetPhyType > 0) {
                        u32 WaitCnt;
                        struct net_device *dev = tp->dev;

                        printk(KERN_ERR "%s: test_phy_ocp ResetPhyType = 0x%02x\n.\n", dev->name, ResetPhyType);

                        rtl8168_mdio_write(tp, 0x1F, 0x0C41);
                        rtl8168_set_eth_phy_bit(tp, 0x14, BIT_0);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        mdelay(24); //24ms

                        rtl8168_mdio_write(tp, 0x1F, 0x0C40);
                        PhyRegValue = rtl8168_mdio_read(tp, 0x12);
                        if ((PhyRegValue & 0x03) != 0x00) {
                                WaitCnt = 0;
                                while ((PhyRegValue & 0x03) != 0x00 && WaitCnt < 5) {
                                        rtl8168_mdio_write(tp, 0x1F, 0x0C40);
                                        rtl8168_set_eth_phy_bit(tp, 0x11, (BIT_15 | BIT_14));
                                        rtl8168_clear_eth_phy_bit(tp, 0x11, (BIT_15 | BIT_14));
                                        mdelay(100);
                                        rtl8168_mdio_write(tp, 0x1F, 0x0C40);
                                        PhyRegValue = rtl8168_mdio_read(tp, 0x12);
                                        WaitCnt++;
                                }
                        }

                        rtl8168_mdio_write(tp, 0x1F, 0x0000);

                        rtl8168_mdio_write(tp, 0x1F, 0x0A46);
                        rtl8168_mdio_write(tp, 0x10, tp->BackupPhyFuseDout_15_0);
                        rtl8168_mdio_write(tp, 0x12, tp->BackupPhyFuseDout_47_32);
                        rtl8168_mdio_write(tp, 0x13, tp->BackupPhyFuseDout_63_48);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);

                        rtl8168_wait_phy_state_ready(tp, HW_PHY_STATUS_INI, 5000000);
                        rtl8168_mdio_write(tp, 0x1F, 0x0A46);
                        rtl8168_set_eth_phy_bit(tp, 0x14, BIT_0);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        rtl8168_wait_phy_state_ready(tp, HW_PHY_STATUS_LAN_ON, 500000);

                        tp->HwHasWrRamCodeToMicroP = FALSE;

                        RestorePhyOcpReg = TRUE;
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        }

exit:
        return RestorePhyOcpReg;
}

static int
rtl8168_is_ups_resume(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        return (rtl8168_mac_ocp_read(tp, 0xD408) & BIT_0);
}

static void
rtl8168_clear_ups_resume_bit(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        rtl8168_mac_ocp_write(tp, 0xD408, rtl8168_mac_ocp_read(tp, 0xD408) & ~(BIT_0));
}

static void
rtl8168_wait_phy_ups_resume(struct net_device *dev, u16 PhyState)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u16 TmpPhyState;
        int i = 0;

        do {
                TmpPhyState = rtl8168_mdio_read_phy_ocp(tp, 0x0A42, 0x10);
                TmpPhyState &= 0x7;
                mdelay(1);
                i++;
        } while ((i < 100) && (TmpPhyState != PhyState));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(i == 100);
#endif
}

void
rtl8168_enable_now_is_oob(struct rtl8168_private *tp)
{
        if ( tp->HwSuppNowIsOobVer == 1 ) {
                RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) | Now_is_oob);
        }
}

void
rtl8168_disable_now_is_oob(struct rtl8168_private *tp)
{
        if ( tp->HwSuppNowIsOobVer == 1 ) {
                RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) & ~Now_is_oob);
        }
}

static void
rtl8168_switch_to_sgmii_mode(
        struct rtl8168_private *tp
)
{
        if (FALSE == HW_SUPP_SERDES_PHY(tp)) return;

        switch (tp->HwSuppSerDesPhyVer) {
        case 1:
                rtl8168_mac_ocp_write(tp, 0xEB00, 0x2);
                rtl8168_set_mcu_ocp_bit(tp, 0xEB16, BIT_1);
                break;
        }
}

static void
rtl8168_exit_oob(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u16 data16;

        RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) & ~(AcceptErr | AcceptRunt | AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));

        if (HW_SUPP_SERDES_PHY(tp)) {
                if (tp->HwSuppSerDesPhyVer == 1) {
                        rtl8168_switch_to_sgmii_mode(tp);
                }
        }

        if (HW_DASH_SUPPORT_DASH(tp)) {
                rtl8168_driver_start(tp);
                rtl8168_dash2_disable_txrx(dev);
#ifdef ENABLE_DASH_SUPPORT
                DashHwInit(dev);
#endif
        }

        //Disable realwow  function
        switch (tp->mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19:
                RTL_W32(tp, MACOCP, 0xE5A90000);
                RTL_W32(tp, MACOCP, 0xF2100010);
                break;
        case CFG_METHOD_20:
                RTL_W32(tp, MACOCP, 0xE5A90000);
                RTL_W32(tp, MACOCP, 0xE4640000);
                RTL_W32(tp, MACOCP, 0xF2100010);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
                RTL_W32(tp, MACOCP, 0x605E0000);
                RTL_W32(tp, MACOCP, (0xE05E << 16) | (RTL_R32(tp, MACOCP) & 0xFFFE));
                RTL_W32(tp, MACOCP, 0xE9720000);
                RTL_W32(tp, MACOCP, 0xF2140010);
                break;
        case CFG_METHOD_26:
                RTL_W32(tp, MACOCP, 0xE05E00FF);
                RTL_W32(tp, MACOCP, 0xE9720000);
                rtl8168_mac_ocp_write(tp, 0xE428, 0x0010);
                break;
        }

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8168_realwow_hw_init(dev);
#else
        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
                rtl8168_eri_write(tp, 0x174, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_mac_ocp_write(tp, 0xE428, 0x0010);
                break;
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_eri_write(tp, 0x174, 2, 0x00FF, ERIAR_ExGMAC);
                rtl8168_mac_ocp_write(tp, 0xE428, 0x0010);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30: {
                u32 csi_tmp;
                csi_tmp = rtl8168_eri_read(tp, 0x174, 2, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_8);
                csi_tmp |= (BIT_15);
                rtl8168_eri_write(tp, 0x174, 2, csi_tmp, ERIAR_ExGMAC);
                rtl8168_mac_ocp_write(tp, 0xE428, 0x0010);
        }
        break;
        }
#endif //ENABLE_REALWOW_SUPPORT

        rtl8168_nic_reset(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_20:
                rtl8168_wait_ll_share_fifo_ready(dev);

                data16 = rtl8168_mac_ocp_read(tp, 0xD4DE) | BIT_15;
                rtl8168_mac_ocp_write(tp, 0xD4DE, data16);

                rtl8168_wait_ll_share_fifo_ready(dev);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_disable_now_is_oob(tp);

                data16 = rtl8168_mac_ocp_read(tp, 0xE8DE) & ~BIT_14;
                rtl8168_mac_ocp_write(tp, 0xE8DE, data16);
                rtl8168_wait_ll_share_fifo_ready(dev);

                data16 = rtl8168_mac_ocp_read(tp, 0xE8DE) | BIT_15;
                rtl8168_mac_ocp_write(tp, 0xE8DE, data16);

                rtl8168_wait_ll_share_fifo_ready(dev);
                break;
        }

        //wait ups resume (phy state 2)
        if (HW_SUPPORT_UPS_MODE(tp))
                if (rtl8168_is_ups_resume(dev)) {
                        rtl8168_wait_phy_ups_resume(dev, HW_PHY_STATUS_EXT_INI);
                        rtl8168_clear_ups_resume_bit(dev);
                }

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(tp))
                rtl8168_hw_init_fiber_nic(dev);
#endif //ENABLE_FIBER_SUPPORT

        tp->phy_reg_anlpar = 0;
}

void
rtl8168_hw_disable_mac_mcu_bps(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (tp->HwSuppAspmClkIntrLock) {
                rtl8168_enable_cfg9346_write(tp);
                rtl8168_hw_aspm_clkreq_enable(tp, false);
                rtl8168_disable_cfg9346_write(tp);
        }

        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mac_ocp_write(tp, 0xFC38, 0x0000);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mac_ocp_write(tp, 0xFC28, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC2A, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC2C, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC2E, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC30, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC32, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC34, 0x0000);
                rtl8168_mac_ocp_write(tp, 0xFC36, 0x0000);
                mdelay(3);
                rtl8168_mac_ocp_write(tp, 0xFC26, 0x0000);
                break;
        }
}

#ifdef ENABLE_USE_FIRMWARE_FILE
static void rtl8168_release_firmware(struct rtl8168_private *tp)
{
        if (tp->rtl_fw) {
                rtl8168_fw_release_firmware(tp->rtl_fw);
                kfree(tp->rtl_fw);
                tp->rtl_fw = NULL;
        }
}

void rtl8168_apply_firmware(struct rtl8168_private *tp)
{
        /* TODO: release firmware if rtl_fw_write_firmware signals failure. */
        if (tp->rtl_fw) {
                rtl8168_fw_write_firmware(tp, tp->rtl_fw);
                /* At least one firmware doesn't reset tp->ocp_base. */
                tp->ocp_base = OCP_STD_PHY_BASE;

                /* PHY soft reset may still be in progress */
                //phy_read_poll_timeout(tp->phydev, MII_BMCR, val,
                //		      !(val & BMCR_RESET),
                //		      50000, 600000, true);
                rtl8168_wait_phy_reset_complete(tp);

                tp->hw_ram_code_ver = rtl8168_get_hw_phy_mcu_code_ver(tp);
                tp->sw_ram_code_ver = tp->hw_ram_code_ver;
                tp->HwHasWrRamCodeToMicroP = TRUE;
        }
}
#endif

static void
rtl8168_hw_init(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 csi_tmp;

        if (tp->HwSuppAspmClkIntrLock) {
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) & ~BIT_7);
                rtl8168_enable_cfg9346_write(tp);
                rtl8168_hw_aspm_clkreq_enable(tp, false);
                rtl8168_disable_cfg9346_write(tp);
        }

        //Disable UPS
        if (HW_SUPPORT_UPS_MODE(tp))
                rtl8168_mac_ocp_write(tp, 0xD400, rtl8168_mac_ocp_read( tp, 0xD400) & ~(BIT_0));

        //Disable DMA Aggregation
        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mac_ocp_write(tp, 0xE63E, rtl8168_mac_ocp_read( tp, 0xE63E) & ~(BIT_3 | BIT_2 | BIT_1));
                rtl8168_mac_ocp_write(tp, 0xE63E, rtl8168_mac_ocp_read( tp, 0xE63E) | (BIT_0));
                rtl8168_mac_ocp_write(tp, 0xE63E, rtl8168_mac_ocp_read( tp, 0xE63E) & ~(BIT_0));
                rtl8168_mac_ocp_write(tp, 0xC094, 0x0);
                rtl8168_mac_ocp_write(tp, 0xC09E, 0x0);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_9:
        case CFG_METHOD_10:
                RTL_W8(tp, DBG_reg, RTL_R8(tp, DBG_reg) | BIT_1 | BIT_7);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
                RTL_W8(tp, 0xF2, (RTL_R8(tp, 0xF2) & ~(BIT_2 | BIT_1 | BIT_0)));
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
                if (aspm) {
                        RTL_W8(tp, 0x6E, RTL_R8(tp, 0x6E) | BIT_6);
                        rtl8168_eri_write(tp, 0x1AE, 2, 0x0403, ERIAR_ExGMAC);
                }
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                if (aspm) {
                        if ((rtl8168_mac_ocp_read(tp, 0xDC00) & BIT_3) || (RTL_R8(tp, Config0) & 0x07)) {
                                RTL_W8(tp, 0x6E, RTL_R8(tp, 0x6E) | BIT_6);
                                rtl8168_eri_write(tp, 0x1AE, 2, 0x0403, ERIAR_ExGMAC);
                        }
                }
                break;
        }

        if (tp->mcfg == CFG_METHOD_10 || tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15)
                RTL_W8(tp, 0xF3, RTL_R8(tp, 0xF3) | BIT_2);

        /*disable ocp phy power saving*/
        if (tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33)
                if (!tp->dash_printer_enabled)
                        rtl8168_disable_ocp_phy_power_saving(dev);

        //Set PCIE uncorrectable error status mask pcie 0x108
        csi_tmp = rtl8168_csi_read(tp, 0x108);
        csi_tmp |= BIT_20;
        rtl8168_csi_write(tp, 0x108, csi_tmp);

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
                csi_tmp = rtl8168_eri_read(tp, 0x1AB, 1, ERIAR_ExGMAC);
                csi_tmp |= ( BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 );
                rtl8168_eri_write(tp, 0x1AB, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        if (s0_magic_packet == 1)
                rtl8168_enable_magic_packet(dev);

#ifdef ENABLE_USE_FIRMWARE_FILE
        if (tp->rtl_fw &&
            !(HW_DASH_SUPPORT_TYPE_3(tp) &&
              tp->HwPkgDet == 0x06))
                rtl8168_apply_firmware(tp);
#endif
}

static void
rtl8168_hw_ephy_config(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u16 ephy_data;

        switch (tp->mcfg) {
        case CFG_METHOD_4:
                /*Set EPHY registers    begin*/
                /*Set EPHY register offset 0x02 bit 11 to 0 and bit 12 to 1*/
                ephy_data = rtl8168_ephy_read(tp, 0x02);
                ephy_data &= ~BIT_11;
                ephy_data |= BIT_12;
                rtl8168_ephy_write(tp, 0x02, ephy_data);

                /*Set EPHY register offset 0x03 bit 1 to 1*/
                ephy_data = rtl8168_ephy_read(tp, 0x03);
                ephy_data |= (1 << 1);
                rtl8168_ephy_write(tp, 0x03, ephy_data);

                /*Set EPHY register offset 0x06 bit 7 to 0*/
                ephy_data = rtl8168_ephy_read(tp, 0x06);
                ephy_data &= ~(1 << 7);
                rtl8168_ephy_write(tp, 0x06, ephy_data);
                /*Set EPHY registers    end*/

                break;
        case CFG_METHOD_5:
                /* set EPHY registers */
                SetPCIePhyBit(tp, 0x01, BIT_0);

                ClearAndSetPCIePhyBit(tp,
                                      0x03,
                                      BIT_10,
                                      BIT_5
                                     );

                break;
        case CFG_METHOD_9:
                /* set EPHY registers */
                rtl8168_ephy_write(tp, 0x01, 0x7C7F);
                rtl8168_ephy_write(tp, 0x02, 0x011F);
                if (tp->eeprom_type != EEPROM_TYPE_NONE) {
                        ClearAndSetPCIePhyBit(tp,
                                              0x03,
                                              0xFFB0,
                                              0x05B0
                                             );
                } else {
                        ClearAndSetPCIePhyBit(tp,
                                              0x03,
                                              0xFFF0,
                                              0x05F0
                                             );
                }
                rtl8168_ephy_write(tp, 0x06, 0xB271);
                rtl8168_ephy_write(tp, 0x07, 0xCE00);

                break;
        case CFG_METHOD_10:
                /* set EPHY registers */
                rtl8168_ephy_write(tp, 0x01, 0x6C7F);
                rtl8168_ephy_write(tp, 0x02, 0x011F);
                ClearAndSetPCIePhyBit(tp,
                                      0x03,
                                      0xFFF0,
                                      0x01B0
                                     );
                rtl8168_ephy_write(tp, 0x1A, 0x0546);
                rtl8168_ephy_write(tp, 0x1C, 0x80C4);
                rtl8168_ephy_write(tp, 0x1D, 0x78E5);
                rtl8168_ephy_write(tp, 0x0A, 0x8100);

                break;
        case CFG_METHOD_12:
        case CFG_METHOD_13:
                ephy_data = rtl8168_ephy_read(tp, 0x0B);
                rtl8168_ephy_write(tp, 0x0B, ephy_data|0x48);
                ephy_data = rtl8168_ephy_read(tp, 0x19);
                ephy_data &= ~0x20;
                rtl8168_ephy_write(tp, 0x19, ephy_data|0x50);
                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~0x100;
                rtl8168_ephy_write(tp, 0x0C, ephy_data|0x20);
                ephy_data = rtl8168_ephy_read(tp, 0x10);
                ephy_data &= ~0x04;
                rtl8168_ephy_write(tp, 0x10, ephy_data);

                break;
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                /* set EPHY registers */
                ephy_data = rtl8168_ephy_read(tp, 0x00) & ~0x0200;
                ephy_data |= 0x0100;
                rtl8168_ephy_write(tp, 0x00, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data |= 0x0004;
                rtl8168_ephy_write(tp, 0x00, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x06) & ~0x0002;
                ephy_data |= 0x0001;
                rtl8168_ephy_write(tp, 0x06, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x06);
                ephy_data |= 0x0030;
                rtl8168_ephy_write(tp, 0x06, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x07);
                ephy_data |= 0x2000;
                rtl8168_ephy_write(tp, 0x07, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data |= 0x0020;
                rtl8168_ephy_write(tp, 0x00, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x03) & ~0x5800;
                ephy_data |= 0x2000;
                rtl8168_ephy_write(tp, 0x03, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x03);
                ephy_data |= 0x0001;
                rtl8168_ephy_write(tp, 0x03, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x01) & ~0x0800;
                ephy_data |= 0x1000;
                rtl8168_ephy_write(tp, 0x01, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x07);
                ephy_data |= 0x4000;
                rtl8168_ephy_write(tp, 0x07, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x1E);
                ephy_data |= 0x2000;
                rtl8168_ephy_write(tp, 0x1E, ephy_data);

                rtl8168_ephy_write(tp, 0x19, 0xFE6C);

                ephy_data = rtl8168_ephy_read(tp, 0x0A);
                ephy_data |= 0x0040;
                rtl8168_ephy_write(tp, 0x0A, ephy_data);

                break;
        case CFG_METHOD_16:
        case CFG_METHOD_17:
                if (tp->mcfg == CFG_METHOD_16) {
                        rtl8168_ephy_write(tp, 0x06, 0xF020);
                        rtl8168_ephy_write(tp, 0x07, 0x01FF);
                        rtl8168_ephy_write(tp, 0x00, 0x5027);
                        rtl8168_ephy_write(tp, 0x01, 0x0003);
                        rtl8168_ephy_write(tp, 0x02, 0x2D16);
                        rtl8168_ephy_write(tp, 0x03, 0x6D49);
                        rtl8168_ephy_write(tp, 0x08, 0x0006);
                        rtl8168_ephy_write(tp, 0x0A, 0x00C8);
                }

                ephy_data = rtl8168_ephy_read(tp, 0x09);
                ephy_data |= BIT_7;
                rtl8168_ephy_write(tp, 0x09, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x19);
                ephy_data |= (BIT_2 | BIT_5 | BIT_9);
                rtl8168_ephy_write(tp, 0x19, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data |= BIT_3;
                rtl8168_ephy_write(tp, 0x00, ephy_data);
                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
                ephy_data |= BIT_9;
                rtl8168_ephy_write(tp, 0x0C, ephy_data);

                break;
        case CFG_METHOD_18:
        case CFG_METHOD_19:
                if (tp->mcfg == CFG_METHOD_18) {
                        ephy_data = rtl8168_ephy_read(tp, 0x06);
                        ephy_data |= BIT_5;
                        ephy_data &= ~(BIT_7 | BIT_6);
                        rtl8168_ephy_write(tp, 0x06, ephy_data);

                        ephy_data = rtl8168_ephy_read(tp, 0x08);
                        ephy_data |= BIT_1;
                        ephy_data &= ~BIT_0;
                        rtl8168_ephy_write(tp, 0x08, ephy_data);
                }

                ephy_data = rtl8168_ephy_read(tp, 0x09);
                ephy_data |= BIT_7;
                rtl8168_ephy_write(tp, 0x09, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x19);
                ephy_data |= (BIT_2 | BIT_5 | BIT_9);
                rtl8168_ephy_write(tp, 0x19, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data |= BIT_3;
                rtl8168_ephy_write(tp, 0x00, ephy_data);
                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
                ephy_data |= BIT_9;
                rtl8168_ephy_write(tp, 0x0C, ephy_data);

                break;
        case CFG_METHOD_20:
                ephy_data = rtl8168_ephy_read(tp, 0x06);
                ephy_data |= BIT_5;
                ephy_data &= ~(BIT_7 | BIT_6);
                rtl8168_ephy_write(tp, 0x06, ephy_data);

                rtl8168_ephy_write(tp, 0x0f, 0x5200);

                ephy_data = rtl8168_ephy_read(tp, 0x19);
                ephy_data |= (BIT_2 | BIT_5 | BIT_9);
                rtl8168_ephy_write(tp, 0x19, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data |= BIT_3;
                rtl8168_ephy_write(tp, 0x00, ephy_data);
                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
                ephy_data |= BIT_9;
                rtl8168_ephy_write(tp, 0x0C, ephy_data);

                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:

                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data &= ~(BIT_3);
                rtl8168_ephy_write(tp, 0x00, ephy_data);
                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
                ephy_data |= (BIT_5 | BIT_11);
                rtl8168_ephy_write(tp, 0x0C, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x1E);
                ephy_data |= (BIT_0);
                rtl8168_ephy_write(tp, 0x1E, ephy_data);

                ephy_data = rtl8168_ephy_read(tp, 0x19);
                ephy_data &= ~(BIT_15);
                rtl8168_ephy_write(tp, 0x19, ephy_data);

                break;
        case CFG_METHOD_25:
                ephy_data = rtl8168_ephy_read(tp, 0x00);
                ephy_data &= ~BIT_3;
                rtl8168_ephy_write(tp, 0x00, ephy_data);
                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10| BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
                ephy_data |= (BIT_5 | BIT_11);
                rtl8168_ephy_write(tp, 0x0C, ephy_data);

                rtl8168_ephy_write(tp, 0x19, 0x7C00);
                rtl8168_ephy_write(tp, 0x1E, 0x20EB);
                rtl8168_ephy_write(tp, 0x0D, 0x1666);
                rtl8168_ephy_write(tp, 0x00, 0x10A3);
                rtl8168_ephy_write(tp, 0x06, 0xF050);

                SetPCIePhyBit(tp, 0x04, BIT_4);
                ClearPCIePhyBit(tp, 0x1D, BIT_14);

                break;
        case CFG_METHOD_26:
                ClearPCIePhyBit(tp, 0x00, BIT_3);
                ClearAndSetPCIePhyBit( tp,
                                       0x0C,
                                       (BIT_13 | BIT_12 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_4),
                                       (BIT_5 | BIT_11)
                                     );
                SetPCIePhyBit(tp, 0x1E, BIT_0);
                ClearPCIePhyBit(tp, 0x19, BIT_15);

                ClearPCIePhyBit(tp, 0x19, (BIT_5 | BIT_0));

                SetPCIePhyBit(tp, 0x1E, BIT_13);
                ClearPCIePhyBit(tp, 0x0D, BIT_8);
                SetPCIePhyBit(tp, 0x0D, BIT_9);
                SetPCIePhyBit(tp, 0x00, BIT_7);

                SetPCIePhyBit(tp, 0x06, BIT_4);

                SetPCIePhyBit(tp, 0x04, BIT_4);
                SetPCIePhyBit(tp, 0x1D, BIT_14);

                break;
        case CFG_METHOD_23:
                rtl8168_ephy_write(tp, 0x00, 0x10AB);
                rtl8168_ephy_write(tp, 0x06, 0xf030);
                rtl8168_ephy_write(tp, 0x08, 0x2006);
                rtl8168_ephy_write(tp, 0x0D, 0x1666);

                ephy_data = rtl8168_ephy_read(tp, 0x0C);
                ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
                rtl8168_ephy_write(tp, 0x0C, ephy_data);

                break;
        case CFG_METHOD_27:
                rtl8168_ephy_write(tp, 0x00, 0x10A3);
                rtl8168_ephy_write(tp, 0x19, 0xFC00);
                rtl8168_ephy_write(tp, 0x1E, 0x20EA);

                break;
        case CFG_METHOD_28:
                rtl8168_ephy_write(tp, 0x00, 0x10AB);
                rtl8168_ephy_write(tp, 0x19, 0xFC00);
                rtl8168_ephy_write(tp, 0x1E, 0x20EB);
                rtl8168_ephy_write(tp, 0x0D, 0x1666);
                ClearPCIePhyBit(tp, 0x0B, BIT_0);
                SetPCIePhyBit(tp, 0x1D, BIT_14);
                ClearAndSetPCIePhyBit(tp,
                                      0x0C,
                                      BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5,
                                      BIT_9 | BIT_4
                                     );

                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                ClearPCIePhyBit(tp, 0x1E, BIT_11);

                SetPCIePhyBit(tp, 0x1E, BIT_0);
                SetPCIePhyBit(tp, 0x1D, BIT_11);

                rtl8168_ephy_write(tp, 0x05, 0x2089);
                rtl8168_ephy_write(tp, 0x06, 0x5881);

                rtl8168_ephy_write(tp, 0x04, 0x854A);
                rtl8168_ephy_write(tp, 0x01, 0x068B);

                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                ClearAndSetPCIePhyBit(tp,
                                      0x19,
                                      BIT_6,
                                      (BIT_12| BIT_8)
                                     );
                ClearAndSetPCIePhyBit(tp,
                                      0x59,
                                      BIT_6,
                                      (BIT_12| BIT_8)
                                     );

                ClearPCIePhyBit(tp, 0x0C, BIT_4);
                ClearPCIePhyBit(tp, 0x4C, BIT_4);
                ClearPCIePhyBit(tp, 0x0B, BIT_0);

                break;
        }
}

static int
rtl8168_set_phy_mcu_patch_request(struct rtl8168_private *tp)
{
        u16 PhyRegValue;
        u32 WaitCnt;
        int retval = TRUE;

        switch (tp->mcfg) {
        case CFG_METHOD_21 ... CFG_METHOD_33:
                rtl8168_mdio_write(tp,0x1f, 0x0B82);
                rtl8168_set_eth_phy_bit(tp, 0x10, BIT_4);

                rtl8168_mdio_write(tp,0x1f, 0x0B80);
                WaitCnt = 0;
                do {
                        PhyRegValue = rtl8168_mdio_read(tp, 0x10);
                        PhyRegValue &= 0x0040;
                        udelay(100);
                        WaitCnt++;
                } while(PhyRegValue != 0x0040 && WaitCnt < 1000);

                if (PhyRegValue != 0x0040 && WaitCnt == 1000) {
                        retval = FALSE;
                }

                rtl8168_mdio_write(tp,0x1f, 0x0000);
                break;
        }

        return retval;
}

static int
rtl8168_clear_phy_mcu_patch_request(struct rtl8168_private *tp)
{
        u16 PhyRegValue;
        u32 WaitCnt;
        int retval = TRUE;

        switch (tp->mcfg) {
        case CFG_METHOD_21 ... CFG_METHOD_33:
                rtl8168_mdio_write(tp, 0x1f, 0x0B82);
                rtl8168_clear_eth_phy_bit(tp, 0x10, BIT_4);

                rtl8168_mdio_write(tp,0x1f, 0x0B80);
                WaitCnt = 0;
                do {
                        PhyRegValue = rtl8168_mdio_read(tp, 0x10);
                        PhyRegValue &= 0x0040;
                        udelay(100);
                        WaitCnt++;
                } while(PhyRegValue != 0x0040 && WaitCnt < 1000);

                if (PhyRegValue != 0x0040 && WaitCnt == 1000) {
                        retval = FALSE;
                }

                rtl8168_mdio_write(tp,0x1f, 0x0000);
                break;
        }

        return retval;
}

static u16
rtl8168_get_hw_phy_mcu_code_ver(struct rtl8168_private *tp)
{
        u16 hw_ram_code_ver = ~0;

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B60);
                hw_ram_code_ver = rtl8168_mdio_read(tp, 0x06);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B30);
                hw_ram_code_ver = rtl8168_mdio_read(tp, 0x06);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x801E);
                hw_ram_code_ver = rtl8168_mdio_read(tp, 0x14);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                break;
        default:
                tp->hw_ram_code_ver = ~0;
                break;
        }

        return hw_ram_code_ver;
}

static void
rtl8168_hw_phy_config(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;
        u16 gphy_val;
        unsigned int i;

        tp->phy_reset_enable(dev);

        if (HW_DASH_SUPPORT_TYPE_3(tp) && tp->HwPkgDet == 0x06) return;

        if (tp->mcfg == CFG_METHOD_1) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x0B, 0x94B0);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x12, 0x6096);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x0D, 0xF8A0);
        } else if (tp->mcfg == CFG_METHOD_2) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x0B, 0x94B0);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x12, 0x6096);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_3) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x0B, 0x94B0);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x12, 0x6096);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_4) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x12, 0x2300);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x16, 0x000A);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x12, 0xC096);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x00, 0x88DE);
                rtl8168_mdio_write(tp, 0x01, 0x82B1);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x08, 0x9E30);
                rtl8168_mdio_write(tp, 0x09, 0x01F0);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x0A, 0x5500);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x03, 0x7002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x0C, 0x00C8);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | (1 << 5));
                rtl8168_mdio_write(tp, 0x0D, rtl8168_mdio_read(tp, 0x0D) & ~(1 << 5));
        } else if (tp->mcfg == CFG_METHOD_5) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x12, 0x2300);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x16, 0x0F0A);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x00, 0x88DE);
                rtl8168_mdio_write(tp, 0x01, 0x82B1);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x0C, 0x7EB8);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x06, 0x0761);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x03, 0x802F);
                rtl8168_mdio_write(tp, 0x02, 0x4F02);
                rtl8168_mdio_write(tp, 0x01, 0x0409);
                rtl8168_mdio_write(tp, 0x00, 0xF099);
                rtl8168_mdio_write(tp, 0x04, 0x9800);
                rtl8168_mdio_write(tp, 0x04, 0x9000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x16, rtl8168_mdio_read(tp, 0x16) | (1 << 0));

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | (1 << 5));
                rtl8168_mdio_write(tp, 0x0D, rtl8168_mdio_read(tp, 0x0D) & ~(1 << 5));

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x1D, 0x3D98);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_6) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x12, 0x2300);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x16, 0x0F0A);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x00, 0x88DE);
                rtl8168_mdio_write(tp, 0x01, 0x82B1);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x0C, 0x7EB8);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x06, 0x5461);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x06, 0x5461);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x16, rtl8168_mdio_read(tp, 0x16) | (1 << 0));

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | (1 << 5));
                rtl8168_mdio_write(tp, 0x0D, rtl8168_mdio_read(tp, 0x0D) & ~(1 << 5));

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x1D, 0x3D98);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_7) {
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | (1 << 5));
                rtl8168_mdio_write(tp, 0x0D, rtl8168_mdio_read(tp, 0x0D) & ~(1 << 5));

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x1D, 0x3D98);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x14, 0xCAA3);
                rtl8168_mdio_write(tp, 0x1C, 0x000A);
                rtl8168_mdio_write(tp, 0x18, 0x65D0);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x17, 0xB580);
                rtl8168_mdio_write(tp, 0x18, 0xFF54);
                rtl8168_mdio_write(tp, 0x19, 0x3954);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x0D, 0x310C);
                rtl8168_mdio_write(tp, 0x0E, 0x310C);
                rtl8168_mdio_write(tp, 0x0F, 0x311C);
                rtl8168_mdio_write(tp, 0x06, 0x0761);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x18, 0xFF55);
                rtl8168_mdio_write(tp, 0x19, 0x3955);
                rtl8168_mdio_write(tp, 0x18, 0xFF54);
                rtl8168_mdio_write(tp, 0x19, 0x3954);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_8) {
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | (1 << 5));
                rtl8168_mdio_write(tp, 0x0D, rtl8168_mdio_read(tp, 0x0D) & ~(1 << 5));

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x14, 0xCAA3);
                rtl8168_mdio_write(tp, 0x1C, 0x000A);
                rtl8168_mdio_write(tp, 0x18, 0x65D0);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x17, 0xB580);
                rtl8168_mdio_write(tp, 0x18, 0xFF54);
                rtl8168_mdio_write(tp, 0x19, 0x3954);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x0D, 0x310C);
                rtl8168_mdio_write(tp, 0x0E, 0x310C);
                rtl8168_mdio_write(tp, 0x0F, 0x311C);
                rtl8168_mdio_write(tp, 0x06, 0x0761);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x18, 0xFF55);
                rtl8168_mdio_write(tp, 0x19, 0x3955);
                rtl8168_mdio_write(tp, 0x18, 0xFF54);
                rtl8168_mdio_write(tp, 0x19, 0x3954);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x16, rtl8168_mdio_read(tp, 0x16) | (1 << 0));

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_9) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x06, 0x4064);
                rtl8168_mdio_write(tp, 0x07, 0x2863);
                rtl8168_mdio_write(tp, 0x08, 0x059C);
                rtl8168_mdio_write(tp, 0x09, 0x26B4);
                rtl8168_mdio_write(tp, 0x0A, 0x6A19);
                rtl8168_mdio_write(tp, 0x0B, 0xDCC8);
                rtl8168_mdio_write(tp, 0x10, 0xF06D);
                rtl8168_mdio_write(tp, 0x14, 0x7F68);
                rtl8168_mdio_write(tp, 0x18, 0x7FD9);
                rtl8168_mdio_write(tp, 0x1C, 0xF0FF);
                rtl8168_mdio_write(tp, 0x1D, 0x3D9C);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x12, 0xF49F);
                rtl8168_mdio_write(tp, 0x13, 0x070B);
                rtl8168_mdio_write(tp, 0x1A, 0x05AD);
                rtl8168_mdio_write(tp, 0x14, 0x94C0);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x0B) & 0xFF00;
                gphy_val |= 0x10;
                rtl8168_mdio_write(tp, 0x0B, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x0C) & 0x00FF;
                gphy_val |= 0xA200;
                rtl8168_mdio_write(tp, 0x0C, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x06, 0x5561);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8332);
                rtl8168_mdio_write(tp, 0x06, 0x5561);

                if (rtl8168_efuse_read(tp, 0x01) == 0xb1) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0002);
                        rtl8168_mdio_write(tp, 0x05, 0x669A);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8330);
                        rtl8168_mdio_write(tp, 0x06, 0x669A);

                        rtl8168_mdio_write(tp, 0x1F, 0x0002);
                        gphy_val = rtl8168_mdio_read(tp, 0x0D);
                        if ((gphy_val & 0x00FF) != 0x006C) {
                                gphy_val &= 0xFF00;
                                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0065);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0066);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0067);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0068);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0069);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x006A);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x006B);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x006C);
                        }
                } else {
                        rtl8168_mdio_write(tp, 0x1F, 0x0002);
                        rtl8168_mdio_write(tp, 0x05, 0x6662);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8330);
                        rtl8168_mdio_write(tp, 0x06, 0x6662);
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x0D);
                gphy_val |= BIT_9;
                gphy_val |= BIT_8;
                rtl8168_mdio_write(tp, 0x0D, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x0F);
                gphy_val |= BIT_4;
                rtl8168_mdio_write(tp, 0x0F, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x02);
                gphy_val &= ~BIT_10;
                gphy_val &= ~BIT_9;
                gphy_val |= BIT_8;
                rtl8168_mdio_write(tp, 0x02, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x03);
                gphy_val &= ~BIT_15;
                gphy_val &= ~BIT_14;
                gphy_val &= ~BIT_13;
                rtl8168_mdio_write(tp, 0x03, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x001B);
                if (rtl8168_mdio_read(tp, 0x06) == 0xBF00) {
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0xfff6);
                        rtl8168_mdio_write(tp, 0x06, 0x0080);
                        rtl8168_mdio_write(tp, 0x05, 0x8000);
                        rtl8168_mdio_write(tp, 0x06, 0xf8f9);
                        rtl8168_mdio_write(tp, 0x06, 0xfaef);
                        rtl8168_mdio_write(tp, 0x06, 0x59ee);
                        rtl8168_mdio_write(tp, 0x06, 0xf8ea);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0xf8eb);
                        rtl8168_mdio_write(tp, 0x06, 0x00e0);
                        rtl8168_mdio_write(tp, 0x06, 0xf87c);
                        rtl8168_mdio_write(tp, 0x06, 0xe1f8);
                        rtl8168_mdio_write(tp, 0x06, 0x7d59);
                        rtl8168_mdio_write(tp, 0x06, 0x0fef);
                        rtl8168_mdio_write(tp, 0x06, 0x0139);
                        rtl8168_mdio_write(tp, 0x06, 0x029e);
                        rtl8168_mdio_write(tp, 0x06, 0x06ef);
                        rtl8168_mdio_write(tp, 0x06, 0x1039);
                        rtl8168_mdio_write(tp, 0x06, 0x089f);
                        rtl8168_mdio_write(tp, 0x06, 0x2aee);
                        rtl8168_mdio_write(tp, 0x06, 0xf8ea);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0xf8eb);
                        rtl8168_mdio_write(tp, 0x06, 0x01e0);
                        rtl8168_mdio_write(tp, 0x06, 0xf87c);
                        rtl8168_mdio_write(tp, 0x06, 0xe1f8);
                        rtl8168_mdio_write(tp, 0x06, 0x7d58);
                        rtl8168_mdio_write(tp, 0x06, 0x409e);
                        rtl8168_mdio_write(tp, 0x06, 0x0f39);
                        rtl8168_mdio_write(tp, 0x06, 0x46aa);
                        rtl8168_mdio_write(tp, 0x06, 0x0bbf);
                        rtl8168_mdio_write(tp, 0x06, 0x8290);
                        rtl8168_mdio_write(tp, 0x06, 0xd682);
                        rtl8168_mdio_write(tp, 0x06, 0x9802);
                        rtl8168_mdio_write(tp, 0x06, 0x014f);
                        rtl8168_mdio_write(tp, 0x06, 0xae09);
                        rtl8168_mdio_write(tp, 0x06, 0xbf82);
                        rtl8168_mdio_write(tp, 0x06, 0x98d6);
                        rtl8168_mdio_write(tp, 0x06, 0x82a0);
                        rtl8168_mdio_write(tp, 0x06, 0x0201);
                        rtl8168_mdio_write(tp, 0x06, 0x4fef);
                        rtl8168_mdio_write(tp, 0x06, 0x95fe);
                        rtl8168_mdio_write(tp, 0x06, 0xfdfc);
                        rtl8168_mdio_write(tp, 0x06, 0x05f8);
                        rtl8168_mdio_write(tp, 0x06, 0xf9fa);
                        rtl8168_mdio_write(tp, 0x06, 0xeef8);
                        rtl8168_mdio_write(tp, 0x06, 0xea00);
                        rtl8168_mdio_write(tp, 0x06, 0xeef8);
                        rtl8168_mdio_write(tp, 0x06, 0xeb00);
                        rtl8168_mdio_write(tp, 0x06, 0xe2f8);
                        rtl8168_mdio_write(tp, 0x06, 0x7ce3);
                        rtl8168_mdio_write(tp, 0x06, 0xf87d);
                        rtl8168_mdio_write(tp, 0x06, 0xa511);
                        rtl8168_mdio_write(tp, 0x06, 0x1112);
                        rtl8168_mdio_write(tp, 0x06, 0xd240);
                        rtl8168_mdio_write(tp, 0x06, 0xd644);
                        rtl8168_mdio_write(tp, 0x06, 0x4402);
                        rtl8168_mdio_write(tp, 0x06, 0x8217);
                        rtl8168_mdio_write(tp, 0x06, 0xd2a0);
                        rtl8168_mdio_write(tp, 0x06, 0xd6aa);
                        rtl8168_mdio_write(tp, 0x06, 0xaa02);
                        rtl8168_mdio_write(tp, 0x06, 0x8217);
                        rtl8168_mdio_write(tp, 0x06, 0xae0f);
                        rtl8168_mdio_write(tp, 0x06, 0xa544);
                        rtl8168_mdio_write(tp, 0x06, 0x4402);
                        rtl8168_mdio_write(tp, 0x06, 0xae4d);
                        rtl8168_mdio_write(tp, 0x06, 0xa5aa);
                        rtl8168_mdio_write(tp, 0x06, 0xaa02);
                        rtl8168_mdio_write(tp, 0x06, 0xae47);
                        rtl8168_mdio_write(tp, 0x06, 0xaf82);
                        rtl8168_mdio_write(tp, 0x06, 0x13ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0x0fee);
                        rtl8168_mdio_write(tp, 0x06, 0x834c);
                        rtl8168_mdio_write(tp, 0x06, 0x0fee);
                        rtl8168_mdio_write(tp, 0x06, 0x834f);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0x8351);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834a);
                        rtl8168_mdio_write(tp, 0x06, 0xffee);
                        rtl8168_mdio_write(tp, 0x06, 0x834b);
                        rtl8168_mdio_write(tp, 0x06, 0xffe0);
                        rtl8168_mdio_write(tp, 0x06, 0x8330);
                        rtl8168_mdio_write(tp, 0x06, 0xe183);
                        rtl8168_mdio_write(tp, 0x06, 0x3158);
                        rtl8168_mdio_write(tp, 0x06, 0xfee4);
                        rtl8168_mdio_write(tp, 0x06, 0xf88a);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8be0);
                        rtl8168_mdio_write(tp, 0x06, 0x8332);
                        rtl8168_mdio_write(tp, 0x06, 0xe183);
                        rtl8168_mdio_write(tp, 0x06, 0x3359);
                        rtl8168_mdio_write(tp, 0x06, 0x0fe2);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0x0c24);
                        rtl8168_mdio_write(tp, 0x06, 0x5af0);
                        rtl8168_mdio_write(tp, 0x06, 0x1e12);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8ce5);
                        rtl8168_mdio_write(tp, 0x06, 0xf88d);
                        rtl8168_mdio_write(tp, 0x06, 0xaf82);
                        rtl8168_mdio_write(tp, 0x06, 0x13e0);
                        rtl8168_mdio_write(tp, 0x06, 0x834f);
                        rtl8168_mdio_write(tp, 0x06, 0x10e4);
                        rtl8168_mdio_write(tp, 0x06, 0x834f);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x009f);
                        rtl8168_mdio_write(tp, 0x06, 0x0ae0);
                        rtl8168_mdio_write(tp, 0x06, 0x834f);
                        rtl8168_mdio_write(tp, 0x06, 0xa010);
                        rtl8168_mdio_write(tp, 0x06, 0xa5ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x01e0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7805);
                        rtl8168_mdio_write(tp, 0x06, 0x9e9a);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x049e);
                        rtl8168_mdio_write(tp, 0x06, 0x10e0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7803);
                        rtl8168_mdio_write(tp, 0x06, 0x9e0f);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x019e);
                        rtl8168_mdio_write(tp, 0x06, 0x05ae);
                        rtl8168_mdio_write(tp, 0x06, 0x0caf);
                        rtl8168_mdio_write(tp, 0x06, 0x81f8);
                        rtl8168_mdio_write(tp, 0x06, 0xaf81);
                        rtl8168_mdio_write(tp, 0x06, 0xa3af);
                        rtl8168_mdio_write(tp, 0x06, 0x81dc);
                        rtl8168_mdio_write(tp, 0x06, 0xaf82);
                        rtl8168_mdio_write(tp, 0x06, 0x13ee);
                        rtl8168_mdio_write(tp, 0x06, 0x8348);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0x8349);
                        rtl8168_mdio_write(tp, 0x06, 0x00e0);
                        rtl8168_mdio_write(tp, 0x06, 0x8351);
                        rtl8168_mdio_write(tp, 0x06, 0x10e4);
                        rtl8168_mdio_write(tp, 0x06, 0x8351);
                        rtl8168_mdio_write(tp, 0x06, 0x5801);
                        rtl8168_mdio_write(tp, 0x06, 0x9fea);
                        rtl8168_mdio_write(tp, 0x06, 0xd000);
                        rtl8168_mdio_write(tp, 0x06, 0xd180);
                        rtl8168_mdio_write(tp, 0x06, 0x1f66);
                        rtl8168_mdio_write(tp, 0x06, 0xe2f8);
                        rtl8168_mdio_write(tp, 0x06, 0xeae3);
                        rtl8168_mdio_write(tp, 0x06, 0xf8eb);
                        rtl8168_mdio_write(tp, 0x06, 0x5af8);
                        rtl8168_mdio_write(tp, 0x06, 0x1e20);
                        rtl8168_mdio_write(tp, 0x06, 0xe6f8);
                        rtl8168_mdio_write(tp, 0x06, 0xeae5);
                        rtl8168_mdio_write(tp, 0x06, 0xf8eb);
                        rtl8168_mdio_write(tp, 0x06, 0xd302);
                        rtl8168_mdio_write(tp, 0x06, 0xb3fe);
                        rtl8168_mdio_write(tp, 0x06, 0xe2f8);
                        rtl8168_mdio_write(tp, 0x06, 0x7cef);
                        rtl8168_mdio_write(tp, 0x06, 0x325b);
                        rtl8168_mdio_write(tp, 0x06, 0x80e3);
                        rtl8168_mdio_write(tp, 0x06, 0xf87d);
                        rtl8168_mdio_write(tp, 0x06, 0x9e03);
                        rtl8168_mdio_write(tp, 0x06, 0x7dff);
                        rtl8168_mdio_write(tp, 0x06, 0xff0d);
                        rtl8168_mdio_write(tp, 0x06, 0x581c);
                        rtl8168_mdio_write(tp, 0x06, 0x551a);
                        rtl8168_mdio_write(tp, 0x06, 0x6511);
                        rtl8168_mdio_write(tp, 0x06, 0xa190);
                        rtl8168_mdio_write(tp, 0x06, 0xd3e2);
                        rtl8168_mdio_write(tp, 0x06, 0x8348);
                        rtl8168_mdio_write(tp, 0x06, 0xe383);
                        rtl8168_mdio_write(tp, 0x06, 0x491b);
                        rtl8168_mdio_write(tp, 0x06, 0x56ab);
                        rtl8168_mdio_write(tp, 0x06, 0x08ef);
                        rtl8168_mdio_write(tp, 0x06, 0x56e6);
                        rtl8168_mdio_write(tp, 0x06, 0x8348);
                        rtl8168_mdio_write(tp, 0x06, 0xe783);
                        rtl8168_mdio_write(tp, 0x06, 0x4910);
                        rtl8168_mdio_write(tp, 0x06, 0xd180);
                        rtl8168_mdio_write(tp, 0x06, 0x1f66);
                        rtl8168_mdio_write(tp, 0x06, 0xa004);
                        rtl8168_mdio_write(tp, 0x06, 0xb9e2);
                        rtl8168_mdio_write(tp, 0x06, 0x8348);
                        rtl8168_mdio_write(tp, 0x06, 0xe383);
                        rtl8168_mdio_write(tp, 0x06, 0x49ef);
                        rtl8168_mdio_write(tp, 0x06, 0x65e2);
                        rtl8168_mdio_write(tp, 0x06, 0x834a);
                        rtl8168_mdio_write(tp, 0x06, 0xe383);
                        rtl8168_mdio_write(tp, 0x06, 0x4b1b);
                        rtl8168_mdio_write(tp, 0x06, 0x56aa);
                        rtl8168_mdio_write(tp, 0x06, 0x0eef);
                        rtl8168_mdio_write(tp, 0x06, 0x56e6);
                        rtl8168_mdio_write(tp, 0x06, 0x834a);
                        rtl8168_mdio_write(tp, 0x06, 0xe783);
                        rtl8168_mdio_write(tp, 0x06, 0x4be2);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0xe683);
                        rtl8168_mdio_write(tp, 0x06, 0x4ce0);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0xa000);
                        rtl8168_mdio_write(tp, 0x06, 0x0caf);
                        rtl8168_mdio_write(tp, 0x06, 0x81dc);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4d10);
                        rtl8168_mdio_write(tp, 0x06, 0xe483);
                        rtl8168_mdio_write(tp, 0x06, 0x4dae);
                        rtl8168_mdio_write(tp, 0x06, 0x0480);
                        rtl8168_mdio_write(tp, 0x06, 0xe483);
                        rtl8168_mdio_write(tp, 0x06, 0x4de0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7803);
                        rtl8168_mdio_write(tp, 0x06, 0x9e0b);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x049e);
                        rtl8168_mdio_write(tp, 0x06, 0x04ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x02e0);
                        rtl8168_mdio_write(tp, 0x06, 0x8332);
                        rtl8168_mdio_write(tp, 0x06, 0xe183);
                        rtl8168_mdio_write(tp, 0x06, 0x3359);
                        rtl8168_mdio_write(tp, 0x06, 0x0fe2);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0x0c24);
                        rtl8168_mdio_write(tp, 0x06, 0x5af0);
                        rtl8168_mdio_write(tp, 0x06, 0x1e12);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8ce5);
                        rtl8168_mdio_write(tp, 0x06, 0xf88d);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x30e1);
                        rtl8168_mdio_write(tp, 0x06, 0x8331);
                        rtl8168_mdio_write(tp, 0x06, 0x6801);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8ae5);
                        rtl8168_mdio_write(tp, 0x06, 0xf88b);
                        rtl8168_mdio_write(tp, 0x06, 0xae37);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4e03);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4ce1);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0x1b01);
                        rtl8168_mdio_write(tp, 0x06, 0x9e04);
                        rtl8168_mdio_write(tp, 0x06, 0xaaa1);
                        rtl8168_mdio_write(tp, 0x06, 0xaea8);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4e04);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4f00);
                        rtl8168_mdio_write(tp, 0x06, 0xaeab);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4f78);
                        rtl8168_mdio_write(tp, 0x06, 0x039f);
                        rtl8168_mdio_write(tp, 0x06, 0x14ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x05d2);
                        rtl8168_mdio_write(tp, 0x06, 0x40d6);
                        rtl8168_mdio_write(tp, 0x06, 0x5554);
                        rtl8168_mdio_write(tp, 0x06, 0x0282);
                        rtl8168_mdio_write(tp, 0x06, 0x17d2);
                        rtl8168_mdio_write(tp, 0x06, 0xa0d6);
                        rtl8168_mdio_write(tp, 0x06, 0xba00);
                        rtl8168_mdio_write(tp, 0x06, 0x0282);
                        rtl8168_mdio_write(tp, 0x06, 0x17fe);
                        rtl8168_mdio_write(tp, 0x06, 0xfdfc);
                        rtl8168_mdio_write(tp, 0x06, 0x05f8);
                        rtl8168_mdio_write(tp, 0x06, 0xe0f8);
                        rtl8168_mdio_write(tp, 0x06, 0x60e1);
                        rtl8168_mdio_write(tp, 0x06, 0xf861);
                        rtl8168_mdio_write(tp, 0x06, 0x6802);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x60e5);
                        rtl8168_mdio_write(tp, 0x06, 0xf861);
                        rtl8168_mdio_write(tp, 0x06, 0xe0f8);
                        rtl8168_mdio_write(tp, 0x06, 0x48e1);
                        rtl8168_mdio_write(tp, 0x06, 0xf849);
                        rtl8168_mdio_write(tp, 0x06, 0x580f);
                        rtl8168_mdio_write(tp, 0x06, 0x1e02);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x48e5);
                        rtl8168_mdio_write(tp, 0x06, 0xf849);
                        rtl8168_mdio_write(tp, 0x06, 0xd000);
                        rtl8168_mdio_write(tp, 0x06, 0x0282);
                        rtl8168_mdio_write(tp, 0x06, 0x5bbf);
                        rtl8168_mdio_write(tp, 0x06, 0x8350);
                        rtl8168_mdio_write(tp, 0x06, 0xef46);
                        rtl8168_mdio_write(tp, 0x06, 0xdc19);
                        rtl8168_mdio_write(tp, 0x06, 0xddd0);
                        rtl8168_mdio_write(tp, 0x06, 0x0102);
                        rtl8168_mdio_write(tp, 0x06, 0x825b);
                        rtl8168_mdio_write(tp, 0x06, 0x0282);
                        rtl8168_mdio_write(tp, 0x06, 0x77e0);
                        rtl8168_mdio_write(tp, 0x06, 0xf860);
                        rtl8168_mdio_write(tp, 0x06, 0xe1f8);
                        rtl8168_mdio_write(tp, 0x06, 0x6158);
                        rtl8168_mdio_write(tp, 0x06, 0xfde4);
                        rtl8168_mdio_write(tp, 0x06, 0xf860);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x61fc);
                        rtl8168_mdio_write(tp, 0x06, 0x04f9);
                        rtl8168_mdio_write(tp, 0x06, 0xfafb);
                        rtl8168_mdio_write(tp, 0x06, 0xc6bf);
                        rtl8168_mdio_write(tp, 0x06, 0xf840);
                        rtl8168_mdio_write(tp, 0x06, 0xbe83);
                        rtl8168_mdio_write(tp, 0x06, 0x50a0);
                        rtl8168_mdio_write(tp, 0x06, 0x0101);
                        rtl8168_mdio_write(tp, 0x06, 0x071b);
                        rtl8168_mdio_write(tp, 0x06, 0x89cf);
                        rtl8168_mdio_write(tp, 0x06, 0xd208);
                        rtl8168_mdio_write(tp, 0x06, 0xebdb);
                        rtl8168_mdio_write(tp, 0x06, 0x19b2);
                        rtl8168_mdio_write(tp, 0x06, 0xfbff);
                        rtl8168_mdio_write(tp, 0x06, 0xfefd);
                        rtl8168_mdio_write(tp, 0x06, 0x04f8);
                        rtl8168_mdio_write(tp, 0x06, 0xe0f8);
                        rtl8168_mdio_write(tp, 0x06, 0x48e1);
                        rtl8168_mdio_write(tp, 0x06, 0xf849);
                        rtl8168_mdio_write(tp, 0x06, 0x6808);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x48e5);
                        rtl8168_mdio_write(tp, 0x06, 0xf849);
                        rtl8168_mdio_write(tp, 0x06, 0x58f7);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x48e5);
                        rtl8168_mdio_write(tp, 0x06, 0xf849);
                        rtl8168_mdio_write(tp, 0x06, 0xfc04);
                        rtl8168_mdio_write(tp, 0x06, 0x4d20);
                        rtl8168_mdio_write(tp, 0x06, 0x0002);
                        rtl8168_mdio_write(tp, 0x06, 0x4e22);
                        rtl8168_mdio_write(tp, 0x06, 0x0002);
                        rtl8168_mdio_write(tp, 0x06, 0x4ddf);
                        rtl8168_mdio_write(tp, 0x06, 0xff01);
                        rtl8168_mdio_write(tp, 0x06, 0x4edd);
                        rtl8168_mdio_write(tp, 0x06, 0xff01);
                        rtl8168_mdio_write(tp, 0x06, 0xf8fa);
                        rtl8168_mdio_write(tp, 0x06, 0xfbef);
                        rtl8168_mdio_write(tp, 0x06, 0x79bf);
                        rtl8168_mdio_write(tp, 0x06, 0xf822);
                        rtl8168_mdio_write(tp, 0x06, 0xd819);
                        rtl8168_mdio_write(tp, 0x06, 0xd958);
                        rtl8168_mdio_write(tp, 0x06, 0x849f);
                        rtl8168_mdio_write(tp, 0x06, 0x09bf);
                        rtl8168_mdio_write(tp, 0x06, 0x82be);
                        rtl8168_mdio_write(tp, 0x06, 0xd682);
                        rtl8168_mdio_write(tp, 0x06, 0xc602);
                        rtl8168_mdio_write(tp, 0x06, 0x014f);
                        rtl8168_mdio_write(tp, 0x06, 0xef97);
                        rtl8168_mdio_write(tp, 0x06, 0xfffe);
                        rtl8168_mdio_write(tp, 0x06, 0xfc05);
                        rtl8168_mdio_write(tp, 0x06, 0x17ff);
                        rtl8168_mdio_write(tp, 0x06, 0xfe01);
                        rtl8168_mdio_write(tp, 0x06, 0x1700);
                        rtl8168_mdio_write(tp, 0x06, 0x0102);
                        rtl8168_mdio_write(tp, 0x05, 0x83d8);
                        rtl8168_mdio_write(tp, 0x06, 0x8051);
                        rtl8168_mdio_write(tp, 0x05, 0x83d6);
                        rtl8168_mdio_write(tp, 0x06, 0x82a0);
                        rtl8168_mdio_write(tp, 0x05, 0x83d4);
                        rtl8168_mdio_write(tp, 0x06, 0x8000);
                        rtl8168_mdio_write(tp, 0x02, 0x2010);
                        rtl8168_mdio_write(tp, 0x03, 0xdc00);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                        rtl8168_mdio_write(tp, 0x0b, 0x0600);
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0xfff6);
                        rtl8168_mdio_write(tp, 0x06, 0x00fc);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0xF880);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_10) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x06, 0x4064);
                rtl8168_mdio_write(tp, 0x07, 0x2863);
                rtl8168_mdio_write(tp, 0x08, 0x059C);
                rtl8168_mdio_write(tp, 0x09, 0x26B4);
                rtl8168_mdio_write(tp, 0x0A, 0x6A19);
                rtl8168_mdio_write(tp, 0x0B, 0xDCC8);
                rtl8168_mdio_write(tp, 0x10, 0xF06D);
                rtl8168_mdio_write(tp, 0x14, 0x7F68);
                rtl8168_mdio_write(tp, 0x18, 0x7FD9);
                rtl8168_mdio_write(tp, 0x1C, 0xF0FF);
                rtl8168_mdio_write(tp, 0x1D, 0x3D9C);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x12, 0xF49F);
                rtl8168_mdio_write(tp, 0x13, 0x070B);
                rtl8168_mdio_write(tp, 0x1A, 0x05AD);
                rtl8168_mdio_write(tp, 0x14, 0x94C0);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x06, 0x5561);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8332);
                rtl8168_mdio_write(tp, 0x06, 0x5561);

                if (rtl8168_efuse_read(tp, 0x01) == 0xb1) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0002);
                        rtl8168_mdio_write(tp, 0x05, 0x669A);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8330);
                        rtl8168_mdio_write(tp, 0x06, 0x669A);

                        rtl8168_mdio_write(tp, 0x1F, 0x0002);
                        gphy_val = rtl8168_mdio_read(tp, 0x0D);
                        if ((gphy_val & 0x00FF) != 0x006C) {
                                gphy_val &= 0xFF00;
                                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0065);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0066);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0067);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0068);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x0069);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x006A);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x006B);
                                rtl8168_mdio_write(tp, 0x0D, gphy_val | 0x006C);
                        }
                } else {
                        rtl8168_mdio_write(tp, 0x1F, 0x0002);
                        rtl8168_mdio_write(tp, 0x05, 0x2642);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8330);
                        rtl8168_mdio_write(tp, 0x06, 0x2642);
                }

                if (rtl8168_efuse_read(tp, 0x30) == 0x98) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) & ~BIT_1);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x01, rtl8168_mdio_read(tp, 0x01) | BIT_9);
                } else if (rtl8168_efuse_read(tp, 0x30) == 0x90) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x01, rtl8168_mdio_read(tp, 0x01) & ~BIT_9);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        rtl8168_mdio_write(tp, 0x16, 0x5101);
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x02);
                gphy_val &= ~BIT_10;
                gphy_val &= ~BIT_9;
                gphy_val |= BIT_8;
                rtl8168_mdio_write(tp, 0x02, gphy_val);
                gphy_val = rtl8168_mdio_read(tp, 0x03);
                gphy_val &= ~BIT_15;
                gphy_val &= ~BIT_14;
                gphy_val &= ~BIT_13;
                rtl8168_mdio_write(tp, 0x03, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x0F);
                gphy_val |= BIT_4;
                gphy_val |= BIT_2;
                gphy_val |= BIT_1;
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x0F, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x001B);
                if (rtl8168_mdio_read(tp, 0x06) == 0xB300) {
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0xfff6);
                        rtl8168_mdio_write(tp, 0x06, 0x0080);
                        rtl8168_mdio_write(tp, 0x05, 0x8000);
                        rtl8168_mdio_write(tp, 0x06, 0xf8f9);
                        rtl8168_mdio_write(tp, 0x06, 0xfaee);
                        rtl8168_mdio_write(tp, 0x06, 0xf8ea);
                        rtl8168_mdio_write(tp, 0x06, 0x00ee);
                        rtl8168_mdio_write(tp, 0x06, 0xf8eb);
                        rtl8168_mdio_write(tp, 0x06, 0x00e2);
                        rtl8168_mdio_write(tp, 0x06, 0xf87c);
                        rtl8168_mdio_write(tp, 0x06, 0xe3f8);
                        rtl8168_mdio_write(tp, 0x06, 0x7da5);
                        rtl8168_mdio_write(tp, 0x06, 0x1111);
                        rtl8168_mdio_write(tp, 0x06, 0x12d2);
                        rtl8168_mdio_write(tp, 0x06, 0x40d6);
                        rtl8168_mdio_write(tp, 0x06, 0x4444);
                        rtl8168_mdio_write(tp, 0x06, 0x0281);
                        rtl8168_mdio_write(tp, 0x06, 0xc6d2);
                        rtl8168_mdio_write(tp, 0x06, 0xa0d6);
                        rtl8168_mdio_write(tp, 0x06, 0xaaaa);
                        rtl8168_mdio_write(tp, 0x06, 0x0281);
                        rtl8168_mdio_write(tp, 0x06, 0xc6ae);
                        rtl8168_mdio_write(tp, 0x06, 0x0fa5);
                        rtl8168_mdio_write(tp, 0x06, 0x4444);
                        rtl8168_mdio_write(tp, 0x06, 0x02ae);
                        rtl8168_mdio_write(tp, 0x06, 0x4da5);
                        rtl8168_mdio_write(tp, 0x06, 0xaaaa);
                        rtl8168_mdio_write(tp, 0x06, 0x02ae);
                        rtl8168_mdio_write(tp, 0x06, 0x47af);
                        rtl8168_mdio_write(tp, 0x06, 0x81c2);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4e00);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4d0f);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4c0f);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4f00);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x5100);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4aff);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4bff);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x30e1);
                        rtl8168_mdio_write(tp, 0x06, 0x8331);
                        rtl8168_mdio_write(tp, 0x06, 0x58fe);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8ae5);
                        rtl8168_mdio_write(tp, 0x06, 0xf88b);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x32e1);
                        rtl8168_mdio_write(tp, 0x06, 0x8333);
                        rtl8168_mdio_write(tp, 0x06, 0x590f);
                        rtl8168_mdio_write(tp, 0x06, 0xe283);
                        rtl8168_mdio_write(tp, 0x06, 0x4d0c);
                        rtl8168_mdio_write(tp, 0x06, 0x245a);
                        rtl8168_mdio_write(tp, 0x06, 0xf01e);
                        rtl8168_mdio_write(tp, 0x06, 0x12e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf88c);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8daf);
                        rtl8168_mdio_write(tp, 0x06, 0x81c2);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4f10);
                        rtl8168_mdio_write(tp, 0x06, 0xe483);
                        rtl8168_mdio_write(tp, 0x06, 0x4fe0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7800);
                        rtl8168_mdio_write(tp, 0x06, 0x9f0a);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4fa0);
                        rtl8168_mdio_write(tp, 0x06, 0x10a5);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4e01);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x059e);
                        rtl8168_mdio_write(tp, 0x06, 0x9ae0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7804);
                        rtl8168_mdio_write(tp, 0x06, 0x9e10);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x039e);
                        rtl8168_mdio_write(tp, 0x06, 0x0fe0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7801);
                        rtl8168_mdio_write(tp, 0x06, 0x9e05);
                        rtl8168_mdio_write(tp, 0x06, 0xae0c);
                        rtl8168_mdio_write(tp, 0x06, 0xaf81);
                        rtl8168_mdio_write(tp, 0x06, 0xa7af);
                        rtl8168_mdio_write(tp, 0x06, 0x8152);
                        rtl8168_mdio_write(tp, 0x06, 0xaf81);
                        rtl8168_mdio_write(tp, 0x06, 0x8baf);
                        rtl8168_mdio_write(tp, 0x06, 0x81c2);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4800);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4900);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x5110);
                        rtl8168_mdio_write(tp, 0x06, 0xe483);
                        rtl8168_mdio_write(tp, 0x06, 0x5158);
                        rtl8168_mdio_write(tp, 0x06, 0x019f);
                        rtl8168_mdio_write(tp, 0x06, 0xead0);
                        rtl8168_mdio_write(tp, 0x06, 0x00d1);
                        rtl8168_mdio_write(tp, 0x06, 0x801f);
                        rtl8168_mdio_write(tp, 0x06, 0x66e2);
                        rtl8168_mdio_write(tp, 0x06, 0xf8ea);
                        rtl8168_mdio_write(tp, 0x06, 0xe3f8);
                        rtl8168_mdio_write(tp, 0x06, 0xeb5a);
                        rtl8168_mdio_write(tp, 0x06, 0xf81e);
                        rtl8168_mdio_write(tp, 0x06, 0x20e6);
                        rtl8168_mdio_write(tp, 0x06, 0xf8ea);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0xebd3);
                        rtl8168_mdio_write(tp, 0x06, 0x02b3);
                        rtl8168_mdio_write(tp, 0x06, 0xfee2);
                        rtl8168_mdio_write(tp, 0x06, 0xf87c);
                        rtl8168_mdio_write(tp, 0x06, 0xef32);
                        rtl8168_mdio_write(tp, 0x06, 0x5b80);
                        rtl8168_mdio_write(tp, 0x06, 0xe3f8);
                        rtl8168_mdio_write(tp, 0x06, 0x7d9e);
                        rtl8168_mdio_write(tp, 0x06, 0x037d);
                        rtl8168_mdio_write(tp, 0x06, 0xffff);
                        rtl8168_mdio_write(tp, 0x06, 0x0d58);
                        rtl8168_mdio_write(tp, 0x06, 0x1c55);
                        rtl8168_mdio_write(tp, 0x06, 0x1a65);
                        rtl8168_mdio_write(tp, 0x06, 0x11a1);
                        rtl8168_mdio_write(tp, 0x06, 0x90d3);
                        rtl8168_mdio_write(tp, 0x06, 0xe283);
                        rtl8168_mdio_write(tp, 0x06, 0x48e3);
                        rtl8168_mdio_write(tp, 0x06, 0x8349);
                        rtl8168_mdio_write(tp, 0x06, 0x1b56);
                        rtl8168_mdio_write(tp, 0x06, 0xab08);
                        rtl8168_mdio_write(tp, 0x06, 0xef56);
                        rtl8168_mdio_write(tp, 0x06, 0xe683);
                        rtl8168_mdio_write(tp, 0x06, 0x48e7);
                        rtl8168_mdio_write(tp, 0x06, 0x8349);
                        rtl8168_mdio_write(tp, 0x06, 0x10d1);
                        rtl8168_mdio_write(tp, 0x06, 0x801f);
                        rtl8168_mdio_write(tp, 0x06, 0x66a0);
                        rtl8168_mdio_write(tp, 0x06, 0x04b9);
                        rtl8168_mdio_write(tp, 0x06, 0xe283);
                        rtl8168_mdio_write(tp, 0x06, 0x48e3);
                        rtl8168_mdio_write(tp, 0x06, 0x8349);
                        rtl8168_mdio_write(tp, 0x06, 0xef65);
                        rtl8168_mdio_write(tp, 0x06, 0xe283);
                        rtl8168_mdio_write(tp, 0x06, 0x4ae3);
                        rtl8168_mdio_write(tp, 0x06, 0x834b);
                        rtl8168_mdio_write(tp, 0x06, 0x1b56);
                        rtl8168_mdio_write(tp, 0x06, 0xaa0e);
                        rtl8168_mdio_write(tp, 0x06, 0xef56);
                        rtl8168_mdio_write(tp, 0x06, 0xe683);
                        rtl8168_mdio_write(tp, 0x06, 0x4ae7);
                        rtl8168_mdio_write(tp, 0x06, 0x834b);
                        rtl8168_mdio_write(tp, 0x06, 0xe283);
                        rtl8168_mdio_write(tp, 0x06, 0x4de6);
                        rtl8168_mdio_write(tp, 0x06, 0x834c);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4da0);
                        rtl8168_mdio_write(tp, 0x06, 0x000c);
                        rtl8168_mdio_write(tp, 0x06, 0xaf81);
                        rtl8168_mdio_write(tp, 0x06, 0x8be0);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0x10e4);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0xae04);
                        rtl8168_mdio_write(tp, 0x06, 0x80e4);
                        rtl8168_mdio_write(tp, 0x06, 0x834d);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x4e78);
                        rtl8168_mdio_write(tp, 0x06, 0x039e);
                        rtl8168_mdio_write(tp, 0x06, 0x0be0);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x7804);
                        rtl8168_mdio_write(tp, 0x06, 0x9e04);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4e02);
                        rtl8168_mdio_write(tp, 0x06, 0xe083);
                        rtl8168_mdio_write(tp, 0x06, 0x32e1);
                        rtl8168_mdio_write(tp, 0x06, 0x8333);
                        rtl8168_mdio_write(tp, 0x06, 0x590f);
                        rtl8168_mdio_write(tp, 0x06, 0xe283);
                        rtl8168_mdio_write(tp, 0x06, 0x4d0c);
                        rtl8168_mdio_write(tp, 0x06, 0x245a);
                        rtl8168_mdio_write(tp, 0x06, 0xf01e);
                        rtl8168_mdio_write(tp, 0x06, 0x12e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf88c);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8de0);
                        rtl8168_mdio_write(tp, 0x06, 0x8330);
                        rtl8168_mdio_write(tp, 0x06, 0xe183);
                        rtl8168_mdio_write(tp, 0x06, 0x3168);
                        rtl8168_mdio_write(tp, 0x06, 0x01e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf88a);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x8bae);
                        rtl8168_mdio_write(tp, 0x06, 0x37ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x03e0);
                        rtl8168_mdio_write(tp, 0x06, 0x834c);
                        rtl8168_mdio_write(tp, 0x06, 0xe183);
                        rtl8168_mdio_write(tp, 0x06, 0x4d1b);
                        rtl8168_mdio_write(tp, 0x06, 0x019e);
                        rtl8168_mdio_write(tp, 0x06, 0x04aa);
                        rtl8168_mdio_write(tp, 0x06, 0xa1ae);
                        rtl8168_mdio_write(tp, 0x06, 0xa8ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834e);
                        rtl8168_mdio_write(tp, 0x06, 0x04ee);
                        rtl8168_mdio_write(tp, 0x06, 0x834f);
                        rtl8168_mdio_write(tp, 0x06, 0x00ae);
                        rtl8168_mdio_write(tp, 0x06, 0xabe0);
                        rtl8168_mdio_write(tp, 0x06, 0x834f);
                        rtl8168_mdio_write(tp, 0x06, 0x7803);
                        rtl8168_mdio_write(tp, 0x06, 0x9f14);
                        rtl8168_mdio_write(tp, 0x06, 0xee83);
                        rtl8168_mdio_write(tp, 0x06, 0x4e05);
                        rtl8168_mdio_write(tp, 0x06, 0xd240);
                        rtl8168_mdio_write(tp, 0x06, 0xd655);
                        rtl8168_mdio_write(tp, 0x06, 0x5402);
                        rtl8168_mdio_write(tp, 0x06, 0x81c6);
                        rtl8168_mdio_write(tp, 0x06, 0xd2a0);
                        rtl8168_mdio_write(tp, 0x06, 0xd6ba);
                        rtl8168_mdio_write(tp, 0x06, 0x0002);
                        rtl8168_mdio_write(tp, 0x06, 0x81c6);
                        rtl8168_mdio_write(tp, 0x06, 0xfefd);
                        rtl8168_mdio_write(tp, 0x06, 0xfc05);
                        rtl8168_mdio_write(tp, 0x06, 0xf8e0);
                        rtl8168_mdio_write(tp, 0x06, 0xf860);
                        rtl8168_mdio_write(tp, 0x06, 0xe1f8);
                        rtl8168_mdio_write(tp, 0x06, 0x6168);
                        rtl8168_mdio_write(tp, 0x06, 0x02e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf860);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x61e0);
                        rtl8168_mdio_write(tp, 0x06, 0xf848);
                        rtl8168_mdio_write(tp, 0x06, 0xe1f8);
                        rtl8168_mdio_write(tp, 0x06, 0x4958);
                        rtl8168_mdio_write(tp, 0x06, 0x0f1e);
                        rtl8168_mdio_write(tp, 0x06, 0x02e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf848);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x49d0);
                        rtl8168_mdio_write(tp, 0x06, 0x0002);
                        rtl8168_mdio_write(tp, 0x06, 0x820a);
                        rtl8168_mdio_write(tp, 0x06, 0xbf83);
                        rtl8168_mdio_write(tp, 0x06, 0x50ef);
                        rtl8168_mdio_write(tp, 0x06, 0x46dc);
                        rtl8168_mdio_write(tp, 0x06, 0x19dd);
                        rtl8168_mdio_write(tp, 0x06, 0xd001);
                        rtl8168_mdio_write(tp, 0x06, 0x0282);
                        rtl8168_mdio_write(tp, 0x06, 0x0a02);
                        rtl8168_mdio_write(tp, 0x06, 0x8226);
                        rtl8168_mdio_write(tp, 0x06, 0xe0f8);
                        rtl8168_mdio_write(tp, 0x06, 0x60e1);
                        rtl8168_mdio_write(tp, 0x06, 0xf861);
                        rtl8168_mdio_write(tp, 0x06, 0x58fd);
                        rtl8168_mdio_write(tp, 0x06, 0xe4f8);
                        rtl8168_mdio_write(tp, 0x06, 0x60e5);
                        rtl8168_mdio_write(tp, 0x06, 0xf861);
                        rtl8168_mdio_write(tp, 0x06, 0xfc04);
                        rtl8168_mdio_write(tp, 0x06, 0xf9fa);
                        rtl8168_mdio_write(tp, 0x06, 0xfbc6);
                        rtl8168_mdio_write(tp, 0x06, 0xbff8);
                        rtl8168_mdio_write(tp, 0x06, 0x40be);
                        rtl8168_mdio_write(tp, 0x06, 0x8350);
                        rtl8168_mdio_write(tp, 0x06, 0xa001);
                        rtl8168_mdio_write(tp, 0x06, 0x0107);
                        rtl8168_mdio_write(tp, 0x06, 0x1b89);
                        rtl8168_mdio_write(tp, 0x06, 0xcfd2);
                        rtl8168_mdio_write(tp, 0x06, 0x08eb);
                        rtl8168_mdio_write(tp, 0x06, 0xdb19);
                        rtl8168_mdio_write(tp, 0x06, 0xb2fb);
                        rtl8168_mdio_write(tp, 0x06, 0xfffe);
                        rtl8168_mdio_write(tp, 0x06, 0xfd04);
                        rtl8168_mdio_write(tp, 0x06, 0xf8e0);
                        rtl8168_mdio_write(tp, 0x06, 0xf848);
                        rtl8168_mdio_write(tp, 0x06, 0xe1f8);
                        rtl8168_mdio_write(tp, 0x06, 0x4968);
                        rtl8168_mdio_write(tp, 0x06, 0x08e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf848);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x4958);
                        rtl8168_mdio_write(tp, 0x06, 0xf7e4);
                        rtl8168_mdio_write(tp, 0x06, 0xf848);
                        rtl8168_mdio_write(tp, 0x06, 0xe5f8);
                        rtl8168_mdio_write(tp, 0x06, 0x49fc);
                        rtl8168_mdio_write(tp, 0x06, 0x044d);
                        rtl8168_mdio_write(tp, 0x06, 0x2000);
                        rtl8168_mdio_write(tp, 0x06, 0x024e);
                        rtl8168_mdio_write(tp, 0x06, 0x2200);
                        rtl8168_mdio_write(tp, 0x06, 0x024d);
                        rtl8168_mdio_write(tp, 0x06, 0xdfff);
                        rtl8168_mdio_write(tp, 0x06, 0x014e);
                        rtl8168_mdio_write(tp, 0x06, 0xddff);
                        rtl8168_mdio_write(tp, 0x06, 0x01f8);
                        rtl8168_mdio_write(tp, 0x06, 0xfafb);
                        rtl8168_mdio_write(tp, 0x06, 0xef79);
                        rtl8168_mdio_write(tp, 0x06, 0xbff8);
                        rtl8168_mdio_write(tp, 0x06, 0x22d8);
                        rtl8168_mdio_write(tp, 0x06, 0x19d9);
                        rtl8168_mdio_write(tp, 0x06, 0x5884);
                        rtl8168_mdio_write(tp, 0x06, 0x9f09);
                        rtl8168_mdio_write(tp, 0x06, 0xbf82);
                        rtl8168_mdio_write(tp, 0x06, 0x6dd6);
                        rtl8168_mdio_write(tp, 0x06, 0x8275);
                        rtl8168_mdio_write(tp, 0x06, 0x0201);
                        rtl8168_mdio_write(tp, 0x06, 0x4fef);
                        rtl8168_mdio_write(tp, 0x06, 0x97ff);
                        rtl8168_mdio_write(tp, 0x06, 0xfefc);
                        rtl8168_mdio_write(tp, 0x06, 0x0517);
                        rtl8168_mdio_write(tp, 0x06, 0xfffe);
                        rtl8168_mdio_write(tp, 0x06, 0x0117);
                        rtl8168_mdio_write(tp, 0x06, 0x0001);
                        rtl8168_mdio_write(tp, 0x06, 0x0200);
                        rtl8168_mdio_write(tp, 0x05, 0x83d8);
                        rtl8168_mdio_write(tp, 0x06, 0x8000);
                        rtl8168_mdio_write(tp, 0x05, 0x83d6);
                        rtl8168_mdio_write(tp, 0x06, 0x824f);
                        rtl8168_mdio_write(tp, 0x02, 0x2010);
                        rtl8168_mdio_write(tp, 0x03, 0xdc00);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                        rtl8168_mdio_write(tp, 0x0b, 0x0600);
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0xfff6);
                        rtl8168_mdio_write(tp, 0x06, 0x00fc);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x0D, 0xF880);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_11) {
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x10, 0x0008);
                rtl8168_mdio_write(tp, 0x0D, 0x006C);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x0B, 0xA4D8);
                rtl8168_mdio_write(tp, 0x09, 0x281C);
                rtl8168_mdio_write(tp, 0x07, 0x2883);
                rtl8168_mdio_write(tp, 0x0A, 0x6B35);
                rtl8168_mdio_write(tp, 0x1D, 0x3DA4);
                rtl8168_mdio_write(tp, 0x1C, 0xEFFD);
                rtl8168_mdio_write(tp, 0x14, 0x7F52);
                rtl8168_mdio_write(tp, 0x18, 0x7FC6);
                rtl8168_mdio_write(tp, 0x08, 0x0601);
                rtl8168_mdio_write(tp, 0x06, 0x4063);
                rtl8168_mdio_write(tp, 0x10, 0xF074);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x13, 0x0789);
                rtl8168_mdio_write(tp, 0x12, 0xF4BD);
                rtl8168_mdio_write(tp, 0x1A, 0x04FD);
                rtl8168_mdio_write(tp, 0x14, 0x84B0);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x00, 0x9200);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x01, 0x0340);
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x04, 0x4000);
                rtl8168_mdio_write(tp, 0x03, 0x1D21);
                rtl8168_mdio_write(tp, 0x02, 0x0C32);
                rtl8168_mdio_write(tp, 0x01, 0x0200);
                rtl8168_mdio_write(tp, 0x00, 0x5554);
                rtl8168_mdio_write(tp, 0x04, 0x4800);
                rtl8168_mdio_write(tp, 0x04, 0x4000);
                rtl8168_mdio_write(tp, 0x04, 0xF000);
                rtl8168_mdio_write(tp, 0x03, 0xDF01);
                rtl8168_mdio_write(tp, 0x02, 0xDF20);
                rtl8168_mdio_write(tp, 0x01, 0x101A);
                rtl8168_mdio_write(tp, 0x00, 0xA0FF);
                rtl8168_mdio_write(tp, 0x04, 0xF800);
                rtl8168_mdio_write(tp, 0x04, 0xF000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0023);
                rtl8168_mdio_write(tp, 0x16, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                gphy_val = rtl8168_mdio_read(tp, 0x0D);
                gphy_val |= BIT_5;
                rtl8168_mdio_write(tp, 0x0D, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x0C);
                gphy_val |= BIT_10;
                rtl8168_mdio_write(tp, 0x0C, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_12) {
                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x17, 0x0CC0);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x0D);
                gphy_val |= BIT_5;
                rtl8168_mdio_write(tp, 0x0D, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x0C);
                gphy_val |= BIT_10;
                rtl8168_mdio_write(tp, 0x0C, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x002C);
                rtl8168_mdio_write(tp, 0x15, 0x035D);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x01, 0x0300);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_13) {
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x0D);
                gphy_val |= BIT_5;
                rtl8168_mdio_write(tp, 0x0D, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x0C);
                gphy_val |= BIT_10;
                rtl8168_mdio_write(tp, 0x0C, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15) {
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0023);
                gphy_val = rtl8168_mdio_read(tp, 0x17) | BIT_1;
                if (tp->RequiredSecLanDonglePatch)
                        gphy_val &= ~(BIT_2);
                else
                        gphy_val |= (BIT_2);
                rtl8168_mdio_write(tp, 0x17, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8b80);
                rtl8168_mdio_write(tp, 0x06, 0xc896);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x0B, 0x6C20);
                rtl8168_mdio_write(tp, 0x07, 0x2872);
                rtl8168_mdio_write(tp, 0x1C, 0xEFFF);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x14, 0x6420);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                gphy_val = rtl8168_mdio_read(tp, 0x08) & 0x00FF;
                rtl8168_mdio_write(tp, 0x08, gphy_val | 0x8000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                rtl8168_mdio_write(tp, 0x18, gphy_val | 0x0010);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x14);
                rtl8168_mdio_write(tp, 0x14, gphy_val | 0x8000);

                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x00, 0x080B);
                rtl8168_mdio_write(tp, 0x0B, 0x09D7);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                                rtl8168_mdio_write(tp, 0x15, 0x1006);
                        }
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x19, 0x7F46);
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8AD2);
                rtl8168_mdio_write(tp, 0x06, 0x6810);
                rtl8168_mdio_write(tp, 0x05, 0x8AD4);
                rtl8168_mdio_write(tp, 0x06, 0x8002);
                rtl8168_mdio_write(tp, 0x05, 0x8ADE);
                rtl8168_mdio_write(tp, 0x06, 0x8025);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x002F);
                rtl8168_mdio_write(tp, 0x15, 0x1919);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                rtl8168_mdio_write(tp, 0x18, gphy_val | 0x0040);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B86);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                rtl8168_mdio_write(tp, 0x06, gphy_val | 0x0001);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x00AC);
                rtl8168_mdio_write(tp, 0x18, 0x0006);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_16) {
                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B80);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_2 | BIT_1;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0004);
                rtl8168_mdio_write(tp, 0x1f, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                gphy_val |= BIT_4;
                rtl8168_mdio_write(tp, 0x18, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0002);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x14);
                gphy_val |= BIT_15;
                rtl8168_mdio_write(tp, 0x14, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x15, 0x1006);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B86);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0001);
                rtl8168_mdio_write(tp, 0x0B, 0x6C14);
                rtl8168_mdio_write(tp, 0x14, 0x7F3D);
                rtl8168_mdio_write(tp, 0x1C, 0xFAFE);
                rtl8168_mdio_write(tp, 0x08, 0x07C5);
                rtl8168_mdio_write(tp, 0x10, 0xF090);
                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x14, 0x641A);
                rtl8168_mdio_write(tp, 0x1A, 0x0606);
                rtl8168_mdio_write(tp, 0x12, 0xF480);
                rtl8168_mdio_write(tp, 0x13, 0x0747);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0004);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0078);
                rtl8168_mdio_write(tp, 0x15, 0xA408);
                rtl8168_mdio_write(tp, 0x17, 0x5100);
                rtl8168_mdio_write(tp, 0x19, 0x0008);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x0D, 0x0207);
                rtl8168_mdio_write(tp, 0x02, 0x5FD0);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0004);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x00A1);
                gphy_val = rtl8168_mdio_read(tp, 0x1A);
                gphy_val &= ~BIT_2;
                rtl8168_mdio_write(tp, 0x1A, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0004);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x16);
                gphy_val |= BIT_5;
                rtl8168_mdio_write(tp, 0x16, gphy_val);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0004);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x00AC);
                rtl8168_mdio_write(tp, 0x18, 0x0006);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x09, 0xA20F);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B5B);
                rtl8168_mdio_write(tp, 0x06, 0x9222);
                rtl8168_mdio_write(tp, 0x05, 0x8B6D);
                rtl8168_mdio_write(tp, 0x06, 0x8000);
                rtl8168_mdio_write(tp, 0x05, 0x8B76);
                rtl8168_mdio_write(tp, 0x06, 0x8000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (pdev->subsystem_vendor == 0x1043 &&
                    pdev->subsystem_device == 0x13F7) {

                        static const u16 evl_phy_value[] = {
                                0x8B56, 0x8B5F, 0x8B68, 0x8B71,
                                0x8B7A, 0x8A7B, 0x8A7E, 0x8A81,
                                0x8A84, 0x8A87
                        };

                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        for (i = 0; i < ARRAY_SIZE(evl_phy_value); i++) {
                                rtl8168_mdio_write(tp, 0x05, evl_phy_value[i]);
                                gphy_val = (0xAA << 8) | (rtl8168_mdio_read(tp, 0x06) & 0xFF);
                                rtl8168_mdio_write(tp, 0x06, gphy_val);
                        }
                        rtl8168_mdio_write(tp, 0x1F, 0x0007);
                        rtl8168_mdio_write(tp, 0x1E, 0x0078);
                        rtl8168_mdio_write(tp, 0x17, 0x51AA);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1f, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B54);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8B5D);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8A7C);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A7F);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A82);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A85);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A88);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                gphy_val = rtl8168_mdio_read(tp, 0x06) | BIT_14 | BIT_15;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_17) {
                if (pdev->subsystem_vendor == 0x144d &&
                    pdev->subsystem_device == 0xc0a6) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0001);
                        rtl8168_mdio_write(tp, 0x0e, 0x6b7f);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8B86);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val |= BIT_4;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                } else {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8B80);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val |= BIT_2 | BIT_1;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);

                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8B86);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val &= ~BIT_4;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1f, 0x0004);
                rtl8168_mdio_write(tp, 0x1f, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                gphy_val |= BIT_4;
                rtl8168_mdio_write(tp, 0x18, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0002);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x14);
                gphy_val |= BIT_15;
                rtl8168_mdio_write(tp, 0x14, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B86);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0004);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x00AC);
                rtl8168_mdio_write(tp, 0x18, 0x0006);
                rtl8168_mdio_write(tp, 0x1F, 0x0002);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x09, 0xA20F);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                gphy_val = rtl8168_mdio_read(tp, 0x06) | BIT_14 | BIT_15;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B5B);
                rtl8168_mdio_write(tp, 0x06, 0x9222);
                rtl8168_mdio_write(tp, 0x05, 0x8B6D);
                rtl8168_mdio_write(tp, 0x06, 0x8000);
                rtl8168_mdio_write(tp, 0x05, 0x8B76);
                rtl8168_mdio_write(tp, 0x06, 0x8000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (pdev->subsystem_vendor == 0x1043 &&
                    pdev->subsystem_device == 0x13F7) {

                        static const u16 evl_phy_value[] = {
                                0x8B56, 0x8B5F, 0x8B68, 0x8B71,
                                0x8B7A, 0x8A7B, 0x8A7E, 0x8A81,
                                0x8A84, 0x8A87
                        };

                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        for (i = 0; i < ARRAY_SIZE(evl_phy_value); i++) {
                                rtl8168_mdio_write(tp, 0x05, evl_phy_value[i]);
                                gphy_val = (0xAA << 8) | (rtl8168_mdio_read(tp, 0x06) & 0xFF);
                                rtl8168_mdio_write(tp, 0x06, gphy_val);
                        }
                        rtl8168_mdio_write(tp, 0x1F, 0x0007);
                        rtl8168_mdio_write(tp, 0x1E, 0x0078);
                        rtl8168_mdio_write(tp, 0x17, 0x51AA);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1f, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B54);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8B5D);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8A7C);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A7F);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A82);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A85);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A88);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                                gphy_val = rtl8168_mdio_read(tp, 0x15);
                                gphy_val |= BIT_12;
                                rtl8168_mdio_write(tp, 0x15, gphy_val);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_18) {
                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8b80);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val |= BIT_2 | BIT_1;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1f, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                gphy_val |= BIT_4;
                rtl8168_mdio_write(tp, 0x18, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x14);
                gphy_val |= BIT_15;
                rtl8168_mdio_write(tp, 0x14, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B86);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_14;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x09, 0xA20F);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B55);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x05, 0x8B5E);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x05, 0x8B67);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x05, 0x8B70);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0078);
                rtl8168_mdio_write(tp, 0x17, 0x0000);
                rtl8168_mdio_write(tp, 0x19, 0x00FB);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B79);
                rtl8168_mdio_write(tp, 0x06, 0xAA00);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0003);
                rtl8168_mdio_write(tp, 0x01, 0x328A);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B54);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8B5D);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8A7C);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A7F);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A82);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A85);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A88);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8b85);
                        rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_15);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                }

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                                gphy_val = rtl8168_mdio_read(tp, 0x15);
                                gphy_val |= BIT_12;
                                rtl8168_mdio_write(tp, 0x15, gphy_val);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_19) {
                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8b80);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val |= BIT_2 | BIT_1;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1f, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                gphy_val |= BIT_4;
                rtl8168_mdio_write(tp, 0x18, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x14);
                gphy_val |= BIT_15;
                rtl8168_mdio_write(tp, 0x14, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B86);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B54);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8B5D);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8A7C);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A7F);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A82);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A85);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A88);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8b85);
                        rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_15);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                }

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                                gphy_val = rtl8168_mdio_read(tp, 0x15);
                                gphy_val |= BIT_12;
                                rtl8168_mdio_write(tp, 0x15, gphy_val);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_20) {

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8b80);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val |= BIT_2 | BIT_1;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                rtl8168_mdio_write(tp, 0x1f, 0x0007);
                rtl8168_mdio_write(tp, 0x1e, 0x002D);
                gphy_val = rtl8168_mdio_read(tp, 0x18);
                gphy_val |= BIT_4;
                rtl8168_mdio_write(tp, 0x18, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                gphy_val = rtl8168_mdio_read(tp, 0x14);
                gphy_val |= BIT_15;
                rtl8168_mdio_write(tp, 0x14, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B86);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_0;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B85);
                gphy_val = rtl8168_mdio_read(tp, 0x06);
                gphy_val |= BIT_14;
                rtl8168_mdio_write(tp, 0x06, gphy_val);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0003);
                rtl8168_mdio_write(tp, 0x09, 0xA20F);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B55);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x05, 0x8B5E);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x05, 0x8B67);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x05, 0x8B70);
                rtl8168_mdio_write(tp, 0x06, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, 0x1F, 0x0007);
                rtl8168_mdio_write(tp, 0x1E, 0x0078);
                rtl8168_mdio_write(tp, 0x17, 0x0000);
                rtl8168_mdio_write(tp, 0x19, 0x00FB);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B79);
                rtl8168_mdio_write(tp, 0x06, 0xAA00);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1f, 0x0005);
                rtl8168_mdio_write(tp, 0x05, 0x8B54);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8B5D);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_11);
                rtl8168_mdio_write(tp, 0x05, 0x8A7C);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A7F);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A82);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A85);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x05, 0x8A88);
                rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1f, 0x0000);

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1f, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8b85);
                        rtl8168_mdio_write(tp, 0x06, rtl8168_mdio_read(tp, 0x06) | BIT_15);
                        rtl8168_mdio_write(tp, 0x1f, 0x0000);
                }

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1f, 0x0000);
                                gphy_val = rtl8168_mdio_read(tp, 0x15);
                                gphy_val |= BIT_12;
                                rtl8168_mdio_write(tp, 0x15, gphy_val);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_21) {
                rtl8168_mdio_write(tp, 0x1F, 0x0A46);
                gphy_val = rtl8168_mdio_read(tp, 0x10);
                rtl8168_mdio_write(tp, 0x1F, 0x0BCC);
                if (gphy_val & BIT_8)
                        rtl8168_clear_eth_phy_bit(tp, 0x12, BIT_15);
                else
                        rtl8168_set_eth_phy_bit(tp, 0x12, BIT_15);
                rtl8168_mdio_write(tp, 0x1F, 0x0A46);
                gphy_val = rtl8168_mdio_read(tp, 0x13);
                rtl8168_mdio_write(tp, 0x1F, 0x0C41);
                if (gphy_val & BIT_8)
                        rtl8168_set_eth_phy_bit(tp, 0x15, BIT_1);
                else
                        rtl8168_clear_eth_phy_bit(tp, 0x15, BIT_1);

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_2 | BIT_3);

                rtl8168_mdio_write(tp, 0x1F, 0x0BCC);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_7);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_6);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8084);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~(BIT_14 | BIT_13));
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_12);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_1);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_0);

                rtl8168_mdio_write(tp, 0x1F, 0x0A4B);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_2);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8012);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | BIT_15);

                rtl8168_mdio_write(tp, 0x1F, 0x0C42);
                gphy_val = rtl8168_mdio_read(tp, 0x11);
                gphy_val |= BIT_14;
                gphy_val &= ~BIT_13;
                rtl8168_mdio_write(tp, 0x11, gphy_val);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x809A);
                rtl8168_mdio_write(tp, 0x14, 0x8022);
                rtl8168_mdio_write(tp, 0x13, 0x80A0);
                gphy_val = rtl8168_mdio_read(tp, 0x14) & 0x00FF;
                gphy_val |= 0x1000;
                rtl8168_mdio_write(tp, 0x14, gphy_val);
                rtl8168_mdio_write(tp, 0x13, 0x8088);
                rtl8168_mdio_write(tp, 0x14, 0x9222);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_2);
                        }
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_22) {
                //do nothing
        } else if (tp->mcfg == CFG_METHOD_23) {
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | (BIT_3 | BIT_2));
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0BCC);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_7);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_6);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8084);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~(BIT_14 | BIT_13));
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_12);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_1);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_0);

                rtl8168_mdio_write(tp, 0x1F, 0x0A4B);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_2);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8012);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | BIT_15);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0C42);
                ClearAndSetEthPhyBit(tp,
                                     0x11,
                                     BIT_13,
                                     BIT_14
                                    );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_2);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_24) {
                rtl8168_mdio_write(tp, 0x1F, 0x0BCC);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_7);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_6);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8084);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~(BIT_14 | BIT_13));
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_12);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_1);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_0);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8012);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | BIT_15);

                rtl8168_mdio_write(tp, 0x1F, 0x0C42);
                gphy_val = rtl8168_mdio_read(tp, 0x11);
                gphy_val |= BIT_14;
                gphy_val &= ~BIT_13;
                rtl8168_mdio_write(tp, 0x11, gphy_val);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_2);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26) {
                rtl8168_mdio_write(tp, 0x1F, 0x0BCC);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_7);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_6);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8084);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~(BIT_14 | BIT_13));
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_12);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_1);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_0);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8012);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | BIT_15);

                rtl8168_mdio_write(tp, 0x1F, 0x0BCE);
                rtl8168_mdio_write(tp, 0x12, 0x8860);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x80F3);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x8B00);
                rtl8168_mdio_write(tp, 0x13, 0x80F0);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x3A00);
                rtl8168_mdio_write(tp, 0x13, 0x80EF);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x0500);
                rtl8168_mdio_write(tp, 0x13, 0x80F6);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x6E00);
                rtl8168_mdio_write(tp, 0x13, 0x80EC);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x6800);
                rtl8168_mdio_write(tp, 0x13, 0x80ED);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x7C00);
                rtl8168_mdio_write(tp, 0x13, 0x80F2);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xF400);
                rtl8168_mdio_write(tp, 0x13, 0x80F4);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x8500);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8110);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xA800);
                rtl8168_mdio_write(tp, 0x13, 0x810F);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x1D00);
                rtl8168_mdio_write(tp, 0x13, 0x8111);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xF500);
                rtl8168_mdio_write(tp, 0x13, 0x8113);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x6100);
                rtl8168_mdio_write(tp, 0x13, 0x8115);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x9200);
                rtl8168_mdio_write(tp, 0x13, 0x810E);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x0400);
                rtl8168_mdio_write(tp, 0x13, 0x810C);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x7C00);
                rtl8168_mdio_write(tp, 0x13, 0x810B);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x5A00);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x80D1);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xFF00);
                rtl8168_mdio_write(tp, 0x13, 0x80CD);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x9E00);
                rtl8168_mdio_write(tp, 0x13, 0x80D3);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x0E00);
                rtl8168_mdio_write(tp, 0x13, 0x80D5);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xCA00);
                rtl8168_mdio_write(tp, 0x13, 0x80D7);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x8400);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_2);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28) {
                rtl8168_mdio_write(tp, 0x1F, 0x0BCC);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~BIT_8);
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_7);
                rtl8168_mdio_write(tp, 0x11, rtl8168_mdio_read(tp, 0x11) | BIT_6);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8084);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) & ~(BIT_14 | BIT_13));
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_12);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_1);
                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_0);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8012);
                rtl8168_mdio_write(tp, 0x14, rtl8168_mdio_read(tp, 0x14) | BIT_15);

                rtl8168_mdio_write(tp, 0x1F, 0x0C42);
                rtl8168_mdio_write(tp, 0x11, (rtl8168_mdio_read(tp, 0x11) & ~BIT_13) | BIT_14);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x80F3);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x8B00);
                rtl8168_mdio_write(tp, 0x13, 0x80F0);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x3A00);
                rtl8168_mdio_write(tp, 0x13, 0x80EF);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x0500);
                rtl8168_mdio_write(tp, 0x13, 0x80F6);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x6E00);
                rtl8168_mdio_write(tp, 0x13, 0x80EC);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x6800);
                rtl8168_mdio_write(tp, 0x13, 0x80ED);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x7C00);
                rtl8168_mdio_write(tp, 0x13, 0x80F2);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xF400);
                rtl8168_mdio_write(tp, 0x13, 0x80F4);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x8500);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8110);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xA800);
                rtl8168_mdio_write(tp, 0x13, 0x810F);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x1D00);
                rtl8168_mdio_write(tp, 0x13, 0x8111);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xF500);
                rtl8168_mdio_write(tp, 0x13, 0x8113);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x6100);
                rtl8168_mdio_write(tp, 0x13, 0x8115);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x9200);
                rtl8168_mdio_write(tp, 0x13, 0x810E);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x0400);
                rtl8168_mdio_write(tp, 0x13, 0x810C);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x7C00);
                rtl8168_mdio_write(tp, 0x13, 0x810B);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x5A00);
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x80D1);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xFF00);
                rtl8168_mdio_write(tp, 0x13, 0x80CD);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x9E00);
                rtl8168_mdio_write(tp, 0x13, 0x80D3);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x0E00);
                rtl8168_mdio_write(tp, 0x13, 0x80D5);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0xCA00);
                rtl8168_mdio_write(tp, 0x13, 0x80D7);
                rtl8168_mdio_write(tp, 0x14, (rtl8168_mdio_read(tp, 0x14) & ~0xFF00) | 0x8400);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_mdio_write(tp, 0x10, rtl8168_mdio_read(tp, 0x10) | BIT_2);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_29) {
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x809b);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xF800 ,
                                      0x8000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80A2);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x8000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80A4);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x8500
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x809C);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0xbd00
                                    );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x80AD);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xF800 ,
                                      0x7000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80B4);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x5000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80AC);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x4000
                                    );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x808E);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x1200
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x8090);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0xE500
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x8092);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x9F00
                                    );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        u16 dout_tapbin;

                        dout_tapbin = 0x0000;
                        rtl8168_mdio_write( tp, 0x1F, 0x0A46 );
                        gphy_val = rtl8168_mdio_read( tp, 0x13 );
                        gphy_val &= (BIT_1|BIT_0);
                        gphy_val <<= 2;
                        dout_tapbin |= gphy_val;

                        gphy_val = rtl8168_mdio_read( tp, 0x12 );
                        gphy_val &= (BIT_15|BIT_14);
                        gphy_val >>= 14;
                        dout_tapbin |= gphy_val;

                        dout_tapbin = ~( dout_tapbin^BIT_3 );
                        dout_tapbin <<= 12;
                        dout_tapbin &= 0xF000;

                        rtl8168_mdio_write( tp, 0x1F, 0x0A43 );

                        rtl8168_mdio_write( tp, 0x13, 0x827A );
                        ClearAndSetEthPhyBit( tp,
                                              0x14,
                                              BIT_15|BIT_14|BIT_13|BIT_12,
                                              dout_tapbin
                                            );


                        rtl8168_mdio_write( tp, 0x13, 0x827B );
                        ClearAndSetEthPhyBit( tp,
                                              0x14,
                                              BIT_15|BIT_14|BIT_13|BIT_12,
                                              dout_tapbin
                                            );


                        rtl8168_mdio_write( tp, 0x13, 0x827C );
                        ClearAndSetEthPhyBit( tp,
                                              0x14,
                                              BIT_15|BIT_14|BIT_13|BIT_12,
                                              dout_tapbin
                                            );


                        rtl8168_mdio_write( tp, 0x13, 0x827D );
                        ClearAndSetEthPhyBit( tp,
                                              0x14,
                                              BIT_15|BIT_14|BIT_13|BIT_12,
                                              dout_tapbin
                                            );

                        rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                        rtl8168_mdio_write(tp, 0x13, 0x8011);
                        rtl8168_set_eth_phy_bit(tp, 0x14, BIT_11);
                        rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                        rtl8168_set_eth_phy_bit(tp, 0x16, BIT_1);
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_set_eth_phy_bit( tp, 0x11, BIT_11 );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);


                rtl8168_mdio_write(tp, 0x1F, 0x0BCA);
                ClearAndSetEthPhyBit( tp,
                                      0x17,
                                      (BIT_13 | BIT_12) ,
                                      BIT_14
                                    );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x803F);
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x13, 0x8047);
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x13, 0x804F);
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x13, 0x8057);
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x13, 0x805F);
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x13, 0x8067 );
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x13, 0x806F );
                rtl8168_clear_eth_phy_bit( tp, 0x14, (BIT_13 | BIT_12));
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_set_eth_phy_bit( tp, 0x10, BIT_2 );
                                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_30) {
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x808A);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0,
                                      0x0A );

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                        rtl8168_mdio_write(tp, 0x13, 0x8011);
                        rtl8168_set_eth_phy_bit(tp, 0x14, BIT_11);
                        rtl8168_mdio_write(tp, 0x1F, 0x0A42);
                        rtl8168_set_eth_phy_bit(tp, 0x16, BIT_1);
                }

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_set_eth_phy_bit( tp, 0x11, BIT_11 );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (tp->RequireAdcBiasPatch) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0BCF);
                        rtl8168_mdio_write(tp, 0x16, tp->AdcBiasPatchIoffset);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                {
                        u16 rlen;

                        rtl8168_mdio_write(tp, 0x1F, 0x0BCD);
                        gphy_val = rtl8168_mdio_read( tp, 0x16 );
                        gphy_val &= 0x000F;

                        if ( gphy_val > 3 ) {
                                rlen = gphy_val - 3;
                        } else {
                                rlen = 0;
                        }

                        gphy_val = rlen | (rlen<<4) | (rlen<<8) | (rlen<<12);

                        rtl8168_mdio_write(tp, 0x1F, 0x0BCD);
                        rtl8168_mdio_write(tp, 0x17, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }

                if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                        rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                        rtl8168_mdio_write(tp, 0x13, 0x85FE);
                        ClearAndSetEthPhyBit(
                                tp,
                                0x14,
                                BIT_15|BIT_14|BIT_13|BIT_12|BIT_11|BIT_10|BIT_8,
                                BIT_9);
                        rtl8168_mdio_write(tp, 0x13, 0x85FF);
                        ClearAndSetEthPhyBit(
                                tp,
                                0x14,
                                BIT_15|BIT_14|BIT_13|BIT_12,
                                BIT_11|BIT_10|BIT_9|BIT_8);
                        rtl8168_mdio_write(tp, 0x13, 0x814B);
                        ClearAndSetEthPhyBit(
                                tp,
                                0x14,
                                BIT_15|BIT_14|BIT_13|BIT_11|BIT_10|BIT_9|BIT_8,
                                BIT_12);
                }

                if (aspm) {
                        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_set_eth_phy_bit( tp, 0x10, BIT_2 );
                                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        }
                }
        } else if (tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                   tp->mcfg == CFG_METHOD_33) {
                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x808E);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x4800
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x8090);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0xCC00
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x8092);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0xB000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x8088);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x6000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x808B);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0x3F00 ,
                                      0x0B00
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x808D);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0x1F00 ,
                                      0x0600
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x808C);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0xB000
                                    );

                rtl8168_mdio_write(tp, 0x13, 0x80A0);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x2800
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80A2);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x5000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x809B);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xF800 ,
                                      0xB000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x809A);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x4B00
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x809D);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0x3F00 ,
                                      0x0800
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80A1);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x7000
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x809F);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0x1F00 ,
                                      0x0300
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x809E);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x8800
                                    );

                rtl8168_mdio_write(tp, 0x13, 0x80B2);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x2200
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80AD);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xF800 ,
                                      0x9800
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80AF);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0x3F00 ,
                                      0x0800
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80B3);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x6F00
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80B1);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0x1F00 ,
                                      0x0300
                                    );
                rtl8168_mdio_write(tp, 0x13, 0x80B0);
                ClearAndSetEthPhyBit( tp,
                                      0x14,
                                      0xFF00 ,
                                      0x9300
                                    );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8011);
                rtl8168_set_eth_phy_bit(tp, 0x14, BIT_11);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_set_eth_phy_bit(tp, 0x11, BIT_11);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                rtl8168_mdio_write(tp, 0x13, 0x8016);
                rtl8168_set_eth_phy_bit(tp, 0x14, BIT_10);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if (aspm) {
                        if (!HW_SUPP_SERDES_PHY(tp) &&
                            HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                                rtl8168_mdio_write(tp, 0x1F, 0x0A43);
                                rtl8168_set_eth_phy_bit( tp, 0x10, BIT_2 );
                                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                        }
                }
        }

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(tp))
                rtl8168_hw_fiber_phy_config(dev);
#endif //ENABLE_FIBER_SUPPORT

        //EthPhyPPSW
        if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
            tp->mcfg == CFG_METHOD_24 || tp->mcfg == CFG_METHOD_25 ||
            tp->mcfg == CFG_METHOD_26) {
                //disable EthPhyPPSW
                rtl8168_mdio_write(tp, 0x1F, 0x0BCD);
                rtl8168_mdio_write(tp, 0x14, 0x5065);
                rtl8168_mdio_write(tp, 0x14, 0xD065);
                rtl8168_mdio_write(tp, 0x1F, 0x0BC8);
                rtl8168_mdio_write(tp, 0x11, 0x5655);
                rtl8168_mdio_write(tp, 0x1F, 0x0BCD);
                rtl8168_mdio_write(tp, 0x14, 0x1065);
                rtl8168_mdio_write(tp, 0x14, 0x9065);
                rtl8168_mdio_write(tp, 0x14, 0x1065);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        } else if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
                   tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                   tp->mcfg == CFG_METHOD_33) {
                //enable EthPhyPPSW
                rtl8168_mdio_write(tp, 0x1F, 0x0A44);
                rtl8168_set_eth_phy_bit( tp, 0x11, BIT_7 );
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
        }

        /*ocp phy power saving*/
        if (tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                if (aspm)
                        rtl8168_enable_ocp_phy_power_saving(dev);
        }

        rtl8168_mdio_write(tp, 0x1F, 0x0000);

        if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
                if (tp->eee_enabled)
                        rtl8168_enable_EEE(tp);
                else
                        rtl8168_disable_EEE(tp);
        }
}

static inline void rtl8168_delete_esd_timer(struct net_device *dev, struct timer_list *timer)
{
        del_timer_sync(timer);
}

static inline void rtl8168_request_esd_timer(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct timer_list *timer = &tp->esd_timer;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        setup_timer(timer, rtl8168_esd_timer, (unsigned long)dev);
#else
        timer_setup(timer, rtl8168_esd_timer, 0);
#endif
        mod_timer(timer, jiffies + RTL8168_ESD_TIMEOUT);
}

static inline void rtl8168_delete_link_timer(struct net_device *dev, struct timer_list *timer)
{
        del_timer_sync(timer);
}

static inline void rtl8168_request_link_timer(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct timer_list *timer = &tp->link_timer;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        setup_timer(timer, rtl8168_link_timer, (unsigned long)dev);
#else
        timer_setup(timer, rtl8168_link_timer, 0);
#endif
        mod_timer(timer, jiffies + RTL8168_LINK_TIMEOUT);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void
rtl8168_netpoll(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;

        disable_irq(pdev->irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
        rtl8168_interrupt(pdev->irq, dev, NULL);
#else
        rtl8168_interrupt(pdev->irq, dev);
#endif
        enable_irq(pdev->irq);
}
#endif

static void
rtl8168_get_bios_setting(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->bios_setting = RTL_R32(tp, 0x8c);
                break;
        }
}

static void
rtl8168_set_bios_setting(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                RTL_W32(tp, 0x8C, tp->bios_setting);
                break;
        }
}

static void
rtl8168_init_software_variable(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;

        rtl8168_get_bios_setting(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
                tp->HwSuppDashVer = 1;
                break;
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                tp->HwSuppDashVer = 2;
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppDashVer = 3;
                break;
        default:
                tp->HwSuppDashVer = 0;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwPkgDet = rtl8168_mac_ocp_read(tp, 0xDC00);
                tp->HwPkgDet = (tp->HwPkgDet >> 3) & 0x0F;
                break;
        }

        if (HW_SUPP_SERDES_PHY(tp))
                eee_enable = 0;

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppNowIsOobVer = 1;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppPhyOcpVer = 1;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppUpsVer = 1;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwPcieSNOffset = 0x16C;
                break;
        case CFG_METHOD_DEFAULT:
                tp->HwPcieSNOffset = 0;
                break;
        default:
                tp->HwPcieSNOffset = 0x164;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppAspmClkIntrLock = 1;
                break;
        }

        if (!aspm || !tp->HwSuppAspmClkIntrLock)
                dynamic_aspm = 0;

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8168_get_realwow_hw_version(dev);
#endif //ENABLE_REALWOW_SUPPORT

        if (HW_DASH_SUPPORT_DASH(tp) && rtl8168_check_dash(tp))
                tp->DASH = 1;
        else
                tp->DASH = 0;

        if (tp->DASH) {
                if (HW_DASH_SUPPORT_TYPE_3(tp)) {
                        u64 CmacMemPhysAddress;
                        void __iomem *cmac_ioaddr = NULL;
                        struct pci_dev *pdev_cmac;

                        pdev_cmac = pci_get_slot(pdev->bus, PCI_DEVFN(PCI_SLOT(pdev->devfn), 0));

                        //map CMAC IO space
                        CmacMemPhysAddress = pci_resource_start(pdev_cmac, 2);

                        /* ioremap MMIO region */
                        cmac_ioaddr = ioremap(CmacMemPhysAddress, R8168_REGS_SIZE);

                        if (cmac_ioaddr == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                                if (netif_msg_probe(tp))
                                        dev_err(&pdev->dev, "cannot remap CMAC MMIO, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                                tp->DASH = 0;
                        } else {
                                tp->mapped_cmac_ioaddr = cmac_ioaddr;
                        }
                }

                eee_enable = 0;
        }

#ifdef ENABLE_DASH_SUPPORT
#ifdef ENABLE_DASH_PRINTER_SUPPORT
        if (tp->DASH) {
                if (HW_DASH_SUPPORT_TYPE_3(tp) && tp->HwPkgDet == 0x0F)
                        tp->dash_printer_enabled = 1;
                else if (HW_DASH_SUPPORT_TYPE_2(tp))
                        tp->dash_printer_enabled = 1;
        }
#endif //ENABLE_DASH_PRINTER_SUPPORT
#endif //ENABLE_DASH_SUPPORT

        if (HW_DASH_SUPPORT_TYPE_2(tp))
                tp->cmac_ioaddr = tp->mmio_addr;
        else if (HW_DASH_SUPPORT_TYPE_3(tp))
                tp->cmac_ioaddr = tp->mapped_cmac_ioaddr;

        switch (tp->mcfg) {
        case CFG_METHOD_1:
                tp->intr_mask = RxDescUnavail | RxFIFOOver | TxDescUnavail | TxOK | RxOK | SWInt;
                tp->timer_intr_mask = PCSTimeout | RxFIFOOver;
                break;
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
                tp->intr_mask = RxDescUnavail | TxDescUnavail | TxOK | RxOK | SWInt;
                tp->timer_intr_mask = PCSTimeout;
                break;
        default:
                tp->intr_mask = RxDescUnavail | TxOK | RxOK | SWInt;
                tp->timer_intr_mask = PCSTimeout;
                break;
        }

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH) {
                if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                        tp->timer_intr_mask |= ( ISRIMR_DASH_INTR_EN | ISRIMR_DASH_INTR_CMAC_RESET);
                        tp->intr_mask |= ( ISRIMR_DASH_INTR_EN | ISRIMR_DASH_INTR_CMAC_RESET);
                } else {
                        tp->timer_intr_mask |= ( ISRIMR_DP_DASH_OK | ISRIMR_DP_HOST_OK | ISRIMR_DP_REQSYS_OK );
                        tp->intr_mask |= ( ISRIMR_DP_DASH_OK | ISRIMR_DP_HOST_OK | ISRIMR_DP_REQSYS_OK );
                }
        }
#endif
        if (aspm) {
                switch (tp->mcfg) {
                case CFG_METHOD_21:
                case CFG_METHOD_22:
                case CFG_METHOD_23:
                case CFG_METHOD_24:
                case CFG_METHOD_25:
                case CFG_METHOD_26:
                case CFG_METHOD_27:
                case CFG_METHOD_28:
                case CFG_METHOD_29:
                case CFG_METHOD_30:
                case CFG_METHOD_31:
                case CFG_METHOD_32:
                case CFG_METHOD_33:
                        tp->org_pci_offset_99 = rtl8168_csi_fun0_read_byte(tp, 0x99);
                        tp->org_pci_offset_99 &= ~(BIT_5|BIT_6);
                        break;
                }
                switch (tp->mcfg) {
                case CFG_METHOD_24:
                case CFG_METHOD_25:
                case CFG_METHOD_26:
                case CFG_METHOD_27:
                case CFG_METHOD_28:
                case CFG_METHOD_29:
                case CFG_METHOD_30:
                        tp->org_pci_offset_180 = rtl8168_csi_fun0_read_byte(tp, 0x180);
                        break;
                case CFG_METHOD_31:
                case CFG_METHOD_32:
                case CFG_METHOD_33:
                        tp->org_pci_offset_180 = rtl8168_csi_fun0_read_byte(tp, 0x214);
                        break;
                }
        }

        pci_read_config_byte(pdev, 0x80, &tp->org_pci_offset_80);
        pci_read_config_byte(pdev, 0x81, &tp->org_pci_offset_81);

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
                if ((tp->features & RTL_FEATURE_MSI) && (tp->org_pci_offset_80 & BIT_1))
                        tp->use_timer_interrrupt = FALSE;
                else
                        tp->use_timer_interrrupt = TRUE;
                break;
        default:
                tp->use_timer_interrrupt = TRUE;
                break;
        }

        if (timer_count == 0 || tp->mcfg == CFG_METHOD_DEFAULT)
                tp->use_timer_interrrupt = FALSE;

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
                tp->ShortPacketSwChecksum = TRUE;
                break;
        case CFG_METHOD_16:
        case CFG_METHOD_17:
                tp->ShortPacketSwChecksum = TRUE;
                tp->UseSwPaddingShortPkt = TRUE;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_30: {
                u16 ioffset_p3, ioffset_p2, ioffset_p1, ioffset_p0;
                u16 TmpUshort;

                rtl8168_mac_ocp_write( tp, 0xDD02, 0x807D);
                TmpUshort = rtl8168_mac_ocp_read( tp, 0xDD02 );
                ioffset_p3 = ( (TmpUshort & BIT_7) >>7 );
                ioffset_p3 <<= 3;
                TmpUshort = rtl8168_mac_ocp_read( tp, 0xDD00 );

                ioffset_p3 |= ((TmpUshort & (BIT_15 | BIT_14 | BIT_13))>>13);

                ioffset_p2 = ((TmpUshort & (BIT_12|BIT_11|BIT_10|BIT_9))>>9);
                ioffset_p1 = ((TmpUshort & (BIT_8|BIT_7|BIT_6|BIT_5))>>5);

                ioffset_p0 = ( (TmpUshort & BIT_4) >>4 );
                ioffset_p0 <<= 3;
                ioffset_p0 |= (TmpUshort & (BIT_2| BIT_1 | BIT_0));

                if ((ioffset_p3 == 0x0F) && (ioffset_p2 == 0x0F) && (ioffset_p1 == 0x0F) && (ioffset_p0 == 0x0F)) {
                        tp->RequireAdcBiasPatch = FALSE;
                } else {
                        tp->RequireAdcBiasPatch = TRUE;
                        tp->AdcBiasPatchIoffset = (ioffset_p3<<12)|(ioffset_p2<<8)|(ioffset_p1<<4)|(ioffset_p0);
                }
        }
        break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33: {
                u16 rg_saw_cnt;

                rtl8168_mdio_write(tp, 0x1F, 0x0C42);
                rg_saw_cnt = rtl8168_mdio_read(tp, 0x13);
                rg_saw_cnt &= ~(BIT_15|BIT_14);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                if ( rg_saw_cnt > 0) {
                        tp->SwrCnt1msIni = 16000000/rg_saw_cnt;
                        tp->SwrCnt1msIni &= 0x0FFF;

                        tp->RequireAdjustUpsTxLinkPulseTiming = TRUE;
                }
        }
        break;
        }

#ifdef ENABLE_FIBER_SUPPORT
        rtl8168_check_hw_fiber_mode_support(dev);
#endif //ENABLE_FIBER_SUPPORT

        switch(tp->mcfg) {
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                if (tp->HwPkgDet == 0x06) {
                        u8 tmpUchar = rtl8168_eri_read(tp, 0xE6, 1, ERIAR_ExGMAC);
                        if (tmpUchar == 0x02)
                                tp->HwSuppSerDesPhyVer = 1;
                        else if (tmpUchar == 0x00)
                                tp->HwSuppSerDesPhyVer = 2;
                }
                break;
        }

        if (pdev->subsystem_vendor == 0x144d) {
                if (pdev->subsystem_device == 0xc098 ||
                    pdev->subsystem_device == 0xc0b1 ||
                    pdev->subsystem_device == 0xc0b8)
                        hwoptimize |= HW_PATCH_SAMSUNG_LAN_DONGLE;
        }

        if (hwoptimize & HW_PATCH_SAMSUNG_LAN_DONGLE) {
                switch (tp->mcfg) {
                case CFG_METHOD_14:
                case CFG_METHOD_15:
                case CFG_METHOD_16:
                case CFG_METHOD_17:
                case CFG_METHOD_18:
                case CFG_METHOD_19:
                case CFG_METHOD_20:
                case CFG_METHOD_30:
                        tp->RequiredSecLanDonglePatch = TRUE;
                        break;
                }
        }

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V2;
                break;
        case CFG_METHOD_DEFAULT:
                tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_NOT_SUPPORT;
                break;
        default:
                tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V1;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                tp->HwSuppEsdVer = 2;
                break;
        default:
                tp->HwSuppEsdVer = 1;
                break;
        }

        if (tp->HwSuppEsdVer == 2) {
                rtl8168_mdio_write(tp, 0x1F, 0x0A46);
                tp->BackupPhyFuseDout_15_0 = rtl8168_mdio_read(tp, 0x10);
                tp->BackupPhyFuseDout_47_32 = rtl8168_mdio_read(tp, 0x12);
                tp->BackupPhyFuseDout_63_48 = rtl8168_mdio_read(tp, 0x13);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);

                tp->TestPhyOcpReg = TRUE;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
                tp->HwSuppCheckPhyDisableModeVer = 1;
                break;
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                tp->HwSuppCheckPhyDisableModeVer = 2;
                break;
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->HwSuppCheckPhyDisableModeVer = 3;
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_14;
                break;
        case CFG_METHOD_16:
        case CFG_METHOD_17:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_16;
                break;
        case CFG_METHOD_18:
        case CFG_METHOD_19:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_18;
                break;
        case CFG_METHOD_20:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_20;
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_21;
                break;
        case CFG_METHOD_23:
        case CFG_METHOD_27:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_23;
                break;
        case CFG_METHOD_24:
        case CFG_METHOD_25:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_24;
                break;
        case CFG_METHOD_26:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_26;
                break;
        case CFG_METHOD_28:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_28;
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_29;
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_31;
                break;
        }

        if (tp->HwIcVerUnknown) {
                tp->NotWrRamCodeToMicroP = TRUE;
                tp->NotWrMcuPatchCode = TRUE;
        }

        tp->NicCustLedValue = RTL_R16(tp, CustomLED);

        rtl8168_get_hw_wol(dev);

        rtl8168_link_option((u8*)&autoneg_mode, (u32*)&speed_mode, (u8*)&duplex_mode, (u32*)&advertising_mode);

        tp->autoneg = autoneg_mode;
        tp->speed = speed_mode;
        tp->duplex = duplex_mode;
        tp->advertising = advertising_mode;

        tp->max_jumbo_frame_size = rtl_chip_info[tp->chipset].jumbo_frame_sz;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        /* MTU range: 60 - hw-specific max */
        dev->min_mtu = ETH_MIN_MTU;
        dev->max_mtu = tp->max_jumbo_frame_size;
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
        tp->eee_enabled = eee_enable;
        tp->eee_adv_t = MDIO_EEE_1000T | MDIO_EEE_100TX;

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(tp))
                rtl8168_set_fiber_mode_software_variable(dev);
#endif //ENABLE_FIBER_SUPPORT
}

static void
rtl8168_release_board(struct pci_dev *pdev,
                      struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;

        rtl8168_set_bios_setting(dev);
        rtl8168_rar_set(tp, tp->org_mac_addr);
        tp->wol_enabled = WOL_DISABLED;

        if (!tp->DASH)
                rtl8168_phy_power_down(dev);

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH)
                FreeAllocatedDashShareMemory(dev);
#endif

        if (tp->mapped_cmac_ioaddr != NULL)
                iounmap(tp->mapped_cmac_ioaddr);

        iounmap(ioaddr);
        pci_release_regions(pdev);
        pci_clear_mwi(pdev);
        pci_disable_device(pdev);
        free_netdev(dev);
}

static int
rtl8168_get_mac_address(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int i;
        u8 mac_addr[MAC_ADDR_LEN];

        for (i = 0; i < MAC_ADDR_LEN; i++)
                mac_addr[i] = RTL_R8(tp, MAC0 + i);

        if (tp->mcfg == CFG_METHOD_18 ||
            tp->mcfg == CFG_METHOD_19 ||
            tp->mcfg == CFG_METHOD_20 ||
            tp->mcfg == CFG_METHOD_21 ||
            tp->mcfg == CFG_METHOD_22 ||
            tp->mcfg == CFG_METHOD_23 ||
            tp->mcfg == CFG_METHOD_24 ||
            tp->mcfg == CFG_METHOD_25 ||
            tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 ||
            tp->mcfg == CFG_METHOD_28 ||
            tp->mcfg == CFG_METHOD_29 ||
            tp->mcfg == CFG_METHOD_30 ||
            tp->mcfg == CFG_METHOD_31 ||
            tp->mcfg == CFG_METHOD_32 ||
            tp->mcfg == CFG_METHOD_33) {
                *(u32*)&mac_addr[0] = rtl8168_eri_read(tp, 0xE0, 4, ERIAR_ExGMAC);
                *(u16*)&mac_addr[4] = rtl8168_eri_read(tp, 0xE4, 2, ERIAR_ExGMAC);
        } else {
                if (tp->eeprom_type != EEPROM_TYPE_NONE) {
                        u16 *pUshort = (u16*)mac_addr;
                        /* Get MAC address from EEPROM */
                        if (tp->mcfg == CFG_METHOD_16 ||
                            tp->mcfg == CFG_METHOD_17 ||
                            tp->mcfg == CFG_METHOD_18 ||
                            tp->mcfg == CFG_METHOD_19 ||
                            tp->mcfg == CFG_METHOD_20 ||
                            tp->mcfg == CFG_METHOD_21 ||
                            tp->mcfg == CFG_METHOD_22 ||
                            tp->mcfg == CFG_METHOD_23 ||
                            tp->mcfg == CFG_METHOD_24 ||
                            tp->mcfg == CFG_METHOD_25 ||
                            tp->mcfg == CFG_METHOD_26 ||
                            tp->mcfg == CFG_METHOD_27 ||
                            tp->mcfg == CFG_METHOD_28 ||
                            tp->mcfg == CFG_METHOD_29 ||
                            tp->mcfg == CFG_METHOD_30 ||
                            tp->mcfg == CFG_METHOD_31 ||
                            tp->mcfg == CFG_METHOD_32 ||
                            tp->mcfg == CFG_METHOD_33) {
                                *pUshort++ = rtl8168_eeprom_read_sc(tp, 1);
                                *pUshort++ = rtl8168_eeprom_read_sc(tp, 2);
                                *pUshort = rtl8168_eeprom_read_sc(tp, 3);
                        } else {
                                *pUshort++ = rtl8168_eeprom_read_sc(tp, 7);
                                *pUshort++ = rtl8168_eeprom_read_sc(tp, 8);
                                *pUshort = rtl8168_eeprom_read_sc(tp, 9);
                        }
                }
        }

        if (!is_valid_ether_addr(mac_addr)) {
                netif_err(tp, probe, dev, "Invalid ether addr %pM\n",
                          mac_addr);
                eth_hw_addr_random(dev);
                ether_addr_copy(mac_addr, dev->dev_addr);
                netif_info(tp, probe, dev, "Random ether addr %pM\n",
                           mac_addr);
                tp->random_mac = 1;
        }

        rtl8168_rar_set(tp, mac_addr);

        for (i = 0; i < MAC_ADDR_LEN; i++) {
                dev->dev_addr[i] = RTL_R8(tp, MAC0 + i);
                tp->org_mac_addr[i] = dev->dev_addr[i]; /* keep the original MAC address */
        }
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
        memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);
#endif
//  memcpy(dev->dev_addr, dev->dev_addr, dev->addr_len);

        return 0;
}

/**
 * rtl8168_set_mac_address - Change the Ethernet Address of the NIC
 * @dev: network interface device structure
 * @p:   pointer to an address structure
 *
 * Return 0 on success, negative on failure
 **/
static int
rtl8168_set_mac_address(struct net_device *dev,
                        void *p)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct sockaddr *addr = p;
        unsigned long flags;

        if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;

        spin_lock_irqsave(&tp->lock, flags);

        memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

        rtl8168_rar_set(tp, dev->dev_addr);

        spin_unlock_irqrestore(&tp->lock, flags);

        return 0;
}

/******************************************************************************
 * rtl8168_rar_set - Puts an ethernet address into a receive address register.
 *
 * tp - The private data structure for driver
 * addr - Address to put into receive address register
 *****************************************************************************/
void
rtl8168_rar_set(struct rtl8168_private *tp,
                uint8_t *addr)
{
        uint32_t rar_low = 0;
        uint32_t rar_high = 0;

        rar_low = ((uint32_t) addr[0] |
                   ((uint32_t) addr[1] << 8) |
                   ((uint32_t) addr[2] << 16) |
                   ((uint32_t) addr[3] << 24));

        rar_high = ((uint32_t) addr[4] |
                    ((uint32_t) addr[5] << 8));

        rtl8168_enable_cfg9346_write(tp);
        RTL_W32(tp, MAC0, rar_low);
        RTL_W32(tp, MAC4, rar_high);

        switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_15:
                RTL_W32(tp, SecMAC0, rar_low);
                RTL_W16(tp, SecMAC4, (uint16_t)rar_high);
                break;
        }

        if (tp->mcfg == CFG_METHOD_17) {
                rtl8168_eri_write(tp, 0xf0, 4, rar_low << 16, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xf4, 4, rar_low >> 16 | rar_high << 16, ERIAR_ExGMAC);
        }

        rtl8168_disable_cfg9346_write(tp);
}

#ifdef ETHTOOL_OPS_COMPAT
static int ethtool_get_settings(struct net_device *dev, void *useraddr)
{
        struct ethtool_cmd cmd = { ETHTOOL_GSET };
        int err;

        if (!ethtool_ops->get_settings)
                return -EOPNOTSUPP;

        err = ethtool_ops->get_settings(dev, &cmd);
        if (err < 0)
                return err;

        if (copy_to_user(useraddr, &cmd, sizeof(cmd)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_settings(struct net_device *dev, void *useraddr)
{
        struct ethtool_cmd cmd;

        if (!ethtool_ops->set_settings)
                return -EOPNOTSUPP;

        if (copy_from_user(&cmd, useraddr, sizeof(cmd)))
                return -EFAULT;

        return ethtool_ops->set_settings(dev, &cmd);
}

static int ethtool_get_drvinfo(struct net_device *dev, void *useraddr)
{
        struct ethtool_drvinfo info;
        struct ethtool_ops *ops = ethtool_ops;

        if (!ops->get_drvinfo)
                return -EOPNOTSUPP;

        memset(&info, 0, sizeof(info));
        info.cmd = ETHTOOL_GDRVINFO;
        ops->get_drvinfo(dev, &info);

        if (ops->self_test_count)
                info.testinfo_len = ops->self_test_count(dev);
        if (ops->get_stats_count)
                info.n_stats = ops->get_stats_count(dev);
        if (ops->get_regs_len)
                info.regdump_len = ops->get_regs_len(dev);
        if (ops->get_eeprom_len)
                info.eedump_len = ops->get_eeprom_len(dev);

        if (copy_to_user(useraddr, &info, sizeof(info)))
                return -EFAULT;
        return 0;
}

static int ethtool_get_regs(struct net_device *dev, char *useraddr)
{
        struct ethtool_regs regs;
        struct ethtool_ops *ops = ethtool_ops;
        void *regbuf;
        int reglen, ret;

        if (!ops->get_regs || !ops->get_regs_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&regs, useraddr, sizeof(regs)))
                return -EFAULT;

        reglen = ops->get_regs_len(dev);
        if (regs.len > reglen)
                regs.len = reglen;

        regbuf = kmalloc(reglen, GFP_USER);
        if (!regbuf)
                return -ENOMEM;

        ops->get_regs(dev, &regs, regbuf);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &regs, sizeof(regs)))
                goto out;
        useraddr += offsetof(struct ethtool_regs, data);
        if (copy_to_user(useraddr, regbuf, reglen))
                goto out;
        ret = 0;

out:
        kfree(regbuf);
        return ret;
}

static int ethtool_get_wol(struct net_device *dev, char *useraddr)
{
        struct ethtool_wolinfo wol = { ETHTOOL_GWOL };

        if (!ethtool_ops->get_wol)
                return -EOPNOTSUPP;

        ethtool_ops->get_wol(dev, &wol);

        if (copy_to_user(useraddr, &wol, sizeof(wol)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_wol(struct net_device *dev, char *useraddr)
{
        struct ethtool_wolinfo wol;

        if (!ethtool_ops->set_wol)
                return -EOPNOTSUPP;

        if (copy_from_user(&wol, useraddr, sizeof(wol)))
                return -EFAULT;

        return ethtool_ops->set_wol(dev, &wol);
}

static int ethtool_get_msglevel(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GMSGLVL };

        if (!ethtool_ops->get_msglevel)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_msglevel(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_msglevel(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_msglevel)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        ethtool_ops->set_msglevel(dev, edata.data);
        return 0;
}

static int ethtool_nway_reset(struct net_device *dev)
{
        if (!ethtool_ops->nway_reset)
                return -EOPNOTSUPP;

        return ethtool_ops->nway_reset(dev);
}

static int ethtool_get_link(struct net_device *dev, void *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GLINK };

        if (!ethtool_ops->get_link)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_link(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_get_eeprom(struct net_device *dev, void *useraddr)
{
        struct ethtool_eeprom eeprom;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->get_eeprom || !ops->get_eeprom_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
                return -EFAULT;

        /* Check for wrap and zero */
        if (eeprom.offset + eeprom.len <= eeprom.offset)
                return -EINVAL;

        /* Check for exceeding total eeprom len */
        if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
                return -EINVAL;

        data = kmalloc(eeprom.len, GFP_USER);
        if (!data)
                return -ENOMEM;

        ret = -EFAULT;
        if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
                goto out;

        ret = ops->get_eeprom(dev, &eeprom, data);
        if (ret)
                goto out;

        ret = -EFAULT;
        if (copy_to_user(useraddr, &eeprom, sizeof(eeprom)))
                goto out;
        if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_set_eeprom(struct net_device *dev, void *useraddr)
{
        struct ethtool_eeprom eeprom;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->set_eeprom || !ops->get_eeprom_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
                return -EFAULT;

        /* Check for wrap and zero */
        if (eeprom.offset + eeprom.len <= eeprom.offset)
                return -EINVAL;

        /* Check for exceeding total eeprom len */
        if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
                return -EINVAL;

        data = kmalloc(eeprom.len, GFP_USER);
        if (!data)
                return -ENOMEM;

        ret = -EFAULT;
        if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
                goto out;

        ret = ops->set_eeprom(dev, &eeprom, data);
        if (ret)
                goto out;

        if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
                ret = -EFAULT;

out:
        kfree(data);
        return ret;
}

static int ethtool_get_coalesce(struct net_device *dev, void *useraddr)
{
        struct ethtool_coalesce coalesce = { ETHTOOL_GCOALESCE };

        if (!ethtool_ops->get_coalesce)
                return -EOPNOTSUPP;

        ethtool_ops->get_coalesce(dev, &coalesce);

        if (copy_to_user(useraddr, &coalesce, sizeof(coalesce)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_coalesce(struct net_device *dev, void *useraddr)
{
        struct ethtool_coalesce coalesce;

        if (!ethtool_ops->get_coalesce)
                return -EOPNOTSUPP;

        if (copy_from_user(&coalesce, useraddr, sizeof(coalesce)))
                return -EFAULT;

        return ethtool_ops->set_coalesce(dev, &coalesce);
}

static int ethtool_get_ringparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_ringparam ringparam = { ETHTOOL_GRINGPARAM };

        if (!ethtool_ops->get_ringparam)
                return -EOPNOTSUPP;

        ethtool_ops->get_ringparam(dev, &ringparam);

        if (copy_to_user(useraddr, &ringparam, sizeof(ringparam)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_ringparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_ringparam ringparam;

        if (!ethtool_ops->get_ringparam)
                return -EOPNOTSUPP;

        if (copy_from_user(&ringparam, useraddr, sizeof(ringparam)))
                return -EFAULT;

        return ethtool_ops->set_ringparam(dev, &ringparam);
}

static int ethtool_get_pauseparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_pauseparam pauseparam = { ETHTOOL_GPAUSEPARAM };

        if (!ethtool_ops->get_pauseparam)
                return -EOPNOTSUPP;

        ethtool_ops->get_pauseparam(dev, &pauseparam);

        if (copy_to_user(useraddr, &pauseparam, sizeof(pauseparam)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_pauseparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_pauseparam pauseparam;

        if (!ethtool_ops->get_pauseparam)
                return -EOPNOTSUPP;

        if (copy_from_user(&pauseparam, useraddr, sizeof(pauseparam)))
                return -EFAULT;

        return ethtool_ops->set_pauseparam(dev, &pauseparam);
}

static int ethtool_get_rx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GRXCSUM };

        if (!ethtool_ops->get_rx_csum)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_rx_csum(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_rx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_rx_csum)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        ethtool_ops->set_rx_csum(dev, edata.data);
        return 0;
}

static int ethtool_get_tx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GTXCSUM };

        if (!ethtool_ops->get_tx_csum)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_tx_csum(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_tx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_tx_csum)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_tx_csum(dev, edata.data);
}

static int ethtool_get_sg(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GSG };

        if (!ethtool_ops->get_sg)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_sg(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_sg(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_sg)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_sg(dev, edata.data);
}

static int ethtool_get_tso(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GTSO };

        if (!ethtool_ops->get_tso)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_tso(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_tso(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_tso)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_tso(dev, edata.data);
}

static int ethtool_self_test(struct net_device *dev, char *useraddr)
{
        struct ethtool_test test;
        struct ethtool_ops *ops = ethtool_ops;
        u64 *data;
        int ret;

        if (!ops->self_test || !ops->self_test_count)
                return -EOPNOTSUPP;

        if (copy_from_user(&test, useraddr, sizeof(test)))
                return -EFAULT;

        test.len = ops->self_test_count(dev);
        data = kmalloc(test.len * sizeof(u64), GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->self_test(dev, &test, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &test, sizeof(test)))
                goto out;
        useraddr += sizeof(test);
        if (copy_to_user(useraddr, data, test.len * sizeof(u64)))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_get_strings(struct net_device *dev, void *useraddr)
{
        struct ethtool_gstrings gstrings;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->get_strings)
                return -EOPNOTSUPP;

        if (copy_from_user(&gstrings, useraddr, sizeof(gstrings)))
                return -EFAULT;

        switch (gstrings.string_set) {
        case ETH_SS_TEST:
                if (!ops->self_test_count)
                        return -EOPNOTSUPP;
                gstrings.len = ops->self_test_count(dev);
                break;
        case ETH_SS_STATS:
                if (!ops->get_stats_count)
                        return -EOPNOTSUPP;
                gstrings.len = ops->get_stats_count(dev);
                break;
        default:
                return -EINVAL;
        }

        data = kmalloc(gstrings.len * ETH_GSTRING_LEN, GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->get_strings(dev, gstrings.string_set, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &gstrings, sizeof(gstrings)))
                goto out;
        useraddr += sizeof(gstrings);
        if (copy_to_user(useraddr, data, gstrings.len * ETH_GSTRING_LEN))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_phys_id(struct net_device *dev, void *useraddr)
{
        struct ethtool_value id;

        if (!ethtool_ops->phys_id)
                return -EOPNOTSUPP;

        if (copy_from_user(&id, useraddr, sizeof(id)))
                return -EFAULT;

        return ethtool_ops->phys_id(dev, id.data);
}

static int ethtool_get_stats(struct net_device *dev, void *useraddr)
{
        struct ethtool_stats stats;
        struct ethtool_ops *ops = ethtool_ops;
        u64 *data;
        int ret;

        if (!ops->get_ethtool_stats || !ops->get_stats_count)
                return -EOPNOTSUPP;

        if (copy_from_user(&stats, useraddr, sizeof(stats)))
                return -EFAULT;

        stats.n_stats = ops->get_stats_count(dev);
        data = kmalloc(stats.n_stats * sizeof(u64), GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->get_ethtool_stats(dev, &stats, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &stats, sizeof(stats)))
                goto out;
        useraddr += sizeof(stats);
        if (copy_to_user(useraddr, data, stats.n_stats * sizeof(u64)))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_ioctl(struct ifreq *ifr)
{
        struct net_device *dev = __dev_get_by_name(ifr->ifr_name);
        void *useraddr = (void *) ifr->ifr_data;
        u32 ethcmd;

        /*
         * XXX: This can be pushed down into the ethtool_* handlers that
         * need it.  Keep existing behaviour for the moment.
         */
        if (!capable(CAP_NET_ADMIN))
                return -EPERM;

        if (!dev || !netif_device_present(dev))
                return -ENODEV;

        if (copy_from_user(&ethcmd, useraddr, sizeof (ethcmd)))
                return -EFAULT;

        switch (ethcmd) {
        case ETHTOOL_GSET:
                return ethtool_get_settings(dev, useraddr);
        case ETHTOOL_SSET:
                return ethtool_set_settings(dev, useraddr);
        case ETHTOOL_GDRVINFO:
                return ethtool_get_drvinfo(dev, useraddr);
        case ETHTOOL_GREGS:
                return ethtool_get_regs(dev, useraddr);
        case ETHTOOL_GWOL:
                return ethtool_get_wol(dev, useraddr);
        case ETHTOOL_SWOL:
                return ethtool_set_wol(dev, useraddr);
        case ETHTOOL_GMSGLVL:
                return ethtool_get_msglevel(dev, useraddr);
        case ETHTOOL_SMSGLVL:
                return ethtool_set_msglevel(dev, useraddr);
        case ETHTOOL_NWAY_RST:
                return ethtool_nway_reset(dev);
        case ETHTOOL_GLINK:
                return ethtool_get_link(dev, useraddr);
        case ETHTOOL_GEEPROM:
                return ethtool_get_eeprom(dev, useraddr);
        case ETHTOOL_SEEPROM:
                return ethtool_set_eeprom(dev, useraddr);
        case ETHTOOL_GCOALESCE:
                return ethtool_get_coalesce(dev, useraddr);
        case ETHTOOL_SCOALESCE:
                return ethtool_set_coalesce(dev, useraddr);
        case ETHTOOL_GRINGPARAM:
                return ethtool_get_ringparam(dev, useraddr);
        case ETHTOOL_SRINGPARAM:
                return ethtool_set_ringparam(dev, useraddr);
        case ETHTOOL_GPAUSEPARAM:
                return ethtool_get_pauseparam(dev, useraddr);
        case ETHTOOL_SPAUSEPARAM:
                return ethtool_set_pauseparam(dev, useraddr);
        case ETHTOOL_GRXCSUM:
                return ethtool_get_rx_csum(dev, useraddr);
        case ETHTOOL_SRXCSUM:
                return ethtool_set_rx_csum(dev, useraddr);
        case ETHTOOL_GTXCSUM:
                return ethtool_get_tx_csum(dev, useraddr);
        case ETHTOOL_STXCSUM:
                return ethtool_set_tx_csum(dev, useraddr);
        case ETHTOOL_GSG:
                return ethtool_get_sg(dev, useraddr);
        case ETHTOOL_SSG:
                return ethtool_set_sg(dev, useraddr);
        case ETHTOOL_GTSO:
                return ethtool_get_tso(dev, useraddr);
        case ETHTOOL_STSO:
                return ethtool_set_tso(dev, useraddr);
        case ETHTOOL_TEST:
                return ethtool_self_test(dev, useraddr);
        case ETHTOOL_GSTRINGS:
                return ethtool_get_strings(dev, useraddr);
        case ETHTOOL_PHYS_ID:
                return ethtool_phys_id(dev, useraddr);
        case ETHTOOL_GSTATS:
                return ethtool_get_stats(dev, useraddr);
        default:
                return -EOPNOTSUPP;
        }

        return -EOPNOTSUPP;
}
#endif //ETHTOOL_OPS_COMPAT

static int
rtl8168_do_ioctl(struct net_device *dev,
                 struct ifreq *ifr,
                 int cmd)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct mii_ioctl_data *data = if_mii(ifr);
        int ret;
        unsigned long flags;

        ret = 0;
        switch (cmd) {
        case SIOCGMIIPHY:
                data->phy_id = 32; /* Internal PHY */
                break;

        case SIOCGMIIREG:
                spin_lock_irqsave(&tp->lock, flags);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                data->val_out = rtl8168_mdio_read(tp, data->reg_num);
                spin_unlock_irqrestore(&tp->lock, flags);
                break;

        case SIOCSMIIREG:
                if (!capable(CAP_NET_ADMIN))
                        return -EPERM;
                spin_lock_irqsave(&tp->lock, flags);
                rtl8168_mdio_write(tp, 0x1F, 0x0000);
                rtl8168_mdio_write(tp, data->reg_num, data->val_in);
                spin_unlock_irqrestore(&tp->lock, flags);
                break;

#ifdef ETHTOOL_OPS_COMPAT
        case SIOCETHTOOL:
                ret = ethtool_ioctl(ifr);
                break;
#endif
        case SIOCDEVPRIVATE_RTLASF:
                if (!netif_running(dev)) {
                        ret = -ENODEV;
                        break;
                }

                ret = rtl8168_asf_ioctl(dev, ifr);
                break;

#ifdef ENABLE_DASH_SUPPORT
        case SIOCDEVPRIVATE_RTLDASH:
                if (!netif_running(dev)) {
                        ret = -ENODEV;
                        break;
                }
                if (!capable(CAP_NET_ADMIN)) {
                        ret = -EPERM;
                        break;
                }

                ret = rtl8168_dash_ioctl(dev, ifr);
                break;
#endif

#ifdef ENABLE_REALWOW_SUPPORT
        case SIOCDEVPRIVATE_RTLREALWOW:
                if (!netif_running(dev)) {
                        ret = -ENODEV;
                        break;
                }

                ret = rtl8168_realwow_ioctl(dev, ifr);
                break;
#endif

        case SIOCRTLTOOL:
                ret = rtl8168_tool_ioctl(tp, ifr);
                break;

        default:
                ret = -EOPNOTSUPP;
                break;
        }

        return ret;
}

static void
rtl8168_phy_power_up(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (rtl8168_is_in_phy_disable_mode(dev))
                return;

        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
                rtl8168_mdio_write(tp, 0x0E, 0x0000);
                break;
        }
        rtl8168_mdio_write(tp, MII_BMCR, BMCR_ANENABLE);

        //wait mdc/mdio ready
        switch (tp->mcfg) {
        case CFG_METHOD_23:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
                mdelay(10);
                break;
        }

        //wait ups resume (phy state 3)
        if (HW_SUPPORT_UPS_MODE(tp))
                rtl8168_wait_phy_ups_resume(dev, HW_PHY_STATUS_LAN_ON);
}

static void
rtl8168_phy_power_down(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 csi_tmp;

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
                csi_tmp = rtl8168_eri_read(tp, 0x1AB, 1, ERIAR_ExGMAC);
                csi_tmp &= ~( BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 );
                rtl8168_eri_write(tp, 0x1AB, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        rtl8168_mdio_write(tp, 0x1F, 0x0000);
        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_9:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
                rtl8168_mdio_write(tp, 0x0E, 0x0200);
                rtl8168_mdio_write(tp, MII_BMCR, BMCR_PDOWN);
                break;
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                rtl8168_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);
                break;
        case CFG_METHOD_21:
        case CFG_METHOD_22:
                rtl8168_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);
                break;
        case CFG_METHOD_23:
        case CFG_METHOD_24:
                rtl8168_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);
                break;
        default:
                rtl8168_mdio_write(tp, MII_BMCR, BMCR_PDOWN);
                break;
        }
}

static int __devinit
rtl8168_init_board(struct pci_dev *pdev,
                   struct net_device **dev_out,
                   void __iomem **ioaddr_out)
{
        void __iomem *ioaddr;
        struct net_device *dev;
        struct rtl8168_private *tp;
        int rc = -ENOMEM, i, pm_cap;

        assert(ioaddr_out != NULL);

        /* dev zeroed in alloc_etherdev */
        dev = alloc_etherdev(sizeof (*tp));
        if (dev == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_drv(&debug))
                        dev_err(&pdev->dev, "unable to alloc new ethernet\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                goto err_out;
        }

        SET_MODULE_OWNER(dev);
        SET_NETDEV_DEV(dev, &pdev->dev);
        tp = netdev_priv(dev);
        tp->dev = dev;
        tp->msg_enable = netif_msg_init(debug.msg_enable, R8168_MSG_DEFAULT);

        if (!aspm || tp->mcfg == CFG_METHOD_9) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
                pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
                                       PCIE_LINK_STATE_CLKPM);
#endif
        }

        /* enable device (incl. PCI PM wakeup and hotplug setup) */
        rc = pci_enable_device(pdev);
        if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "enable failure\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                goto err_out_free_dev;
        }

        if (pci_set_mwi(pdev) < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_drv(&debug))
                        dev_info(&pdev->dev, "Mem-Wr-Inval unavailable.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        }

        /* save power state before pci_enable_device overwrites it */
        pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
        if (pm_cap) {
                u16 pwr_command;

                pci_read_config_word(pdev, pm_cap + PCI_PM_CTRL, &pwr_command);
        } else {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp)) {
                        dev_err(&pdev->dev, "PowerManagement capability not found.\n");
                }
#else
                printk("PowerManagement capability not found.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

        }

        /* make sure PCI base addr 1 is MMIO */
        if (!(pci_resource_flags(pdev, 2) & IORESOURCE_MEM)) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "region #1 not an MMIO resource, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                rc = -ENODEV;
                goto err_out_mwi;
        }
        /* check for weird/broken PCI region reporting */
        if (pci_resource_len(pdev, 2) < R8168_REGS_SIZE) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "Invalid PCI region size(s), aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                rc = -ENODEV;
                goto err_out_mwi;
        }

        rc = pci_request_regions(pdev, MODULENAME);
        if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "could not request regions.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                goto err_out_mwi;
        }

        if ((sizeof(dma_addr_t) > 4) &&
            use_dac &&
            !pci_set_dma_mask(pdev, DMA_BIT_MASK(64)) &&
            !pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64))) {
                dev->features |= NETIF_F_HIGHDMA;
        } else {
                rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
                if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                        if (netif_msg_probe(tp))
                                dev_err(&pdev->dev, "DMA configuration failed.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                        goto err_out_free_res;
                }
        }

        /* ioremap MMIO region */
        ioaddr = ioremap(pci_resource_start(pdev, 2), R8168_REGS_SIZE);
        if (ioaddr == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "cannot remap MMIO, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                rc = -EIO;
                goto err_out_free_res;
        }

        tp->mmio_addr = ioaddr;

        /* Identify chip attached to board */
        rtl8168_get_mac_version(tp);

        rtl8168_print_mac_version(tp);

        for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
                if (tp->mcfg == rtl_chip_info[i].mcfg)
                        break;
        }

        if (i < 0) {
                /* Unknown chip: assume array element #0, original RTL-8168 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                if (netif_msg_probe(tp))
                        dev_printk(KERN_DEBUG, &pdev->dev, "unknown chip version, assuming %s\n", rtl_chip_info[0].name);
#else
                printk("Realtek unknown chip version, assuming %s\n", rtl_chip_info[0].name);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
                i++;
        }

        tp->chipset = i;

        *ioaddr_out = ioaddr;
        *dev_out = dev;
out:
        return rc;

err_out_free_res:
        pci_release_regions(pdev);
err_out_mwi:
        pci_clear_mwi(pdev);
        pci_disable_device(pdev);
err_out_free_dev:
        free_netdev(dev);
err_out:
        *ioaddr_out = NULL;
        *dev_out = NULL;
        goto out;
}

static void
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
rtl8168_esd_timer(unsigned long __opaque)
#else
rtl8168_esd_timer(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        struct net_device *dev = (struct net_device *)__opaque;
        struct rtl8168_private *tp = netdev_priv(dev);
        struct timer_list *timer = &tp->esd_timer;
#else
        struct rtl8168_private *tp = from_timer(tp, t, esd_timer);
        struct net_device *dev = tp->dev;
        struct timer_list *timer = t;
#endif
        struct pci_dev *pdev = tp->pci_dev;
        unsigned long timeout = RTL8168_ESD_TIMEOUT;
        unsigned long flags;
        u8 cmd;
        u16 io_base_l;
        u16 mem_base_l;
        u16 mem_base_h;
        u8 ilr;
        u16 resv_0x1c_h;
        u16 resv_0x1c_l;
        u16 resv_0x20_l;
        u16 resv_0x20_h;
        u16 resv_0x24_l;
        u16 resv_0x24_h;
        u16 resv_0x2c_h;
        u16 resv_0x2c_l;
        u32 pci_sn_l;
        u32 pci_sn_h;

        spin_lock_irqsave(&tp->lock, flags);

        tp->esd_flag = 0;

        pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
        if (cmd != tp->pci_cfg_space.cmd) {
                printk(KERN_ERR "%s: cmd = 0x%02x, should be 0x%02x \n.", dev->name, cmd, tp->pci_cfg_space.cmd);
                pci_write_config_byte(pdev, PCI_COMMAND, tp->pci_cfg_space.cmd);
                tp->esd_flag |= BIT_0;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &io_base_l);
        if (io_base_l != tp->pci_cfg_space.io_base_l) {
                printk(KERN_ERR "%s: io_base_l = 0x%04x, should be 0x%04x \n.", dev->name, io_base_l, tp->pci_cfg_space.io_base_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_0, tp->pci_cfg_space.io_base_l);
                tp->esd_flag |= BIT_1;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &mem_base_l);
        if (mem_base_l != tp->pci_cfg_space.mem_base_l) {
                printk(KERN_ERR "%s: mem_base_l = 0x%04x, should be 0x%04x \n.", dev->name, mem_base_l, tp->pci_cfg_space.mem_base_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_2, tp->pci_cfg_space.mem_base_l);
                tp->esd_flag |= BIT_2;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &mem_base_h);
        if (mem_base_h!= tp->pci_cfg_space.mem_base_h) {
                printk(KERN_ERR "%s: mem_base_h = 0x%04x, should be 0x%04x \n.", dev->name, mem_base_h, tp->pci_cfg_space.mem_base_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, tp->pci_cfg_space.mem_base_h);
                tp->esd_flag |= BIT_3;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &resv_0x1c_l);
        if (resv_0x1c_l != tp->pci_cfg_space.resv_0x1c_l) {
                printk(KERN_ERR "%s: resv_0x1c_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x1c_l, tp->pci_cfg_space.resv_0x1c_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_3, tp->pci_cfg_space.resv_0x1c_l);
                tp->esd_flag |= BIT_4;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &resv_0x1c_h);
        if (resv_0x1c_h != tp->pci_cfg_space.resv_0x1c_h) {
                printk(KERN_ERR "%s: resv_0x1c_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x1c_h, tp->pci_cfg_space.resv_0x1c_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, tp->pci_cfg_space.resv_0x1c_h);
                tp->esd_flag |= BIT_5;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &resv_0x20_l);
        if (resv_0x20_l != tp->pci_cfg_space.resv_0x20_l) {
                printk(KERN_ERR "%s: resv_0x20_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x20_l, tp->pci_cfg_space.resv_0x20_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_4, tp->pci_cfg_space.resv_0x20_l);
                tp->esd_flag |= BIT_6;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &resv_0x20_h);
        if (resv_0x20_h != tp->pci_cfg_space.resv_0x20_h) {
                printk(KERN_ERR "%s: resv_0x20_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x20_h, tp->pci_cfg_space.resv_0x20_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, tp->pci_cfg_space.resv_0x20_h);
                tp->esd_flag |= BIT_7;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &resv_0x24_l);
        if (resv_0x24_l != tp->pci_cfg_space.resv_0x24_l) {
                printk(KERN_ERR "%s: resv_0x24_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x24_l, tp->pci_cfg_space.resv_0x24_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_5, tp->pci_cfg_space.resv_0x24_l);
                tp->esd_flag |= BIT_8;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &resv_0x24_h);
        if (resv_0x24_h != tp->pci_cfg_space.resv_0x24_h) {
                printk(KERN_ERR "%s: resv_0x24_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x24_h, tp->pci_cfg_space.resv_0x24_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, tp->pci_cfg_space.resv_0x24_h);
                tp->esd_flag |= BIT_9;
        }

        pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &ilr);
        if (ilr != tp->pci_cfg_space.ilr) {
                printk(KERN_ERR "%s: ilr = 0x%02x, should be 0x%02x \n.", dev->name, ilr, tp->pci_cfg_space.ilr);
                pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, tp->pci_cfg_space.ilr);
                tp->esd_flag |= BIT_10;
        }

        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &resv_0x2c_l);
        if (resv_0x2c_l != tp->pci_cfg_space.resv_0x2c_l) {
                printk(KERN_ERR "%s: resv_0x2c_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x2c_l, tp->pci_cfg_space.resv_0x2c_l);
                pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, tp->pci_cfg_space.resv_0x2c_l);
                tp->esd_flag |= BIT_11;
        }

        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &resv_0x2c_h);
        if (resv_0x2c_h != tp->pci_cfg_space.resv_0x2c_h) {
                printk(KERN_ERR "%s: resv_0x2c_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x2c_h, tp->pci_cfg_space.resv_0x2c_h);
                pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, tp->pci_cfg_space.resv_0x2c_h);
                tp->esd_flag |= BIT_12;
        }

        if (tp->HwPcieSNOffset > 0) {
                pci_sn_l = rtl8168_csi_read(tp, tp->HwPcieSNOffset);
                if (pci_sn_l != tp->pci_cfg_space.pci_sn_l) {
                        printk(KERN_ERR "%s: pci_sn_l = 0x%08x, should be 0x%08x \n.", dev->name, pci_sn_l, tp->pci_cfg_space.pci_sn_l);
                        rtl8168_csi_write(tp, tp->HwPcieSNOffset, tp->pci_cfg_space.pci_sn_l);
                        tp->esd_flag |= BIT_13;
                }

                pci_sn_h = rtl8168_csi_read(tp, tp->HwPcieSNOffset + 4);
                if (pci_sn_h != tp->pci_cfg_space.pci_sn_h) {
                        printk(KERN_ERR "%s: pci_sn_h = 0x%08x, should be 0x%08x \n.", dev->name, pci_sn_h, tp->pci_cfg_space.pci_sn_h);
                        rtl8168_csi_write(tp, tp->HwPcieSNOffset + 4, tp->pci_cfg_space.pci_sn_h);
                        tp->esd_flag |= BIT_14;
                }
        }

        if (tp->TestPhyOcpReg && rtl8168_test_phy_ocp(tp))
                tp->esd_flag |= BIT_15;

        if (tp->esd_flag != 0) {
                printk(KERN_ERR "%s: esd_flag = 0x%04x\n.\n", dev->name, tp->esd_flag);
                netif_stop_queue(dev);
                netif_carrier_off(dev);
                rtl8168_hw_reset(dev);
                rtl8168_tx_clear(tp);
                rtl8168_rx_clear(tp);
                rtl8168_init_ring(dev);
                rtl8168_hw_init(dev);
                rtl8168_powerup_pll(dev);
                rtl8168_hw_ephy_config(dev);
                rtl8168_hw_phy_config(dev);
                rtl8168_hw_config(dev);
                rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);
                tp->esd_flag = 0;
        }
        spin_unlock_irqrestore(&tp->lock, flags);

        mod_timer(timer, jiffies + timeout);
}

static void
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
rtl8168_link_timer(unsigned long __opaque)
#else
rtl8168_link_timer(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        struct net_device *dev = (struct net_device *)__opaque;
        struct rtl8168_private *tp = netdev_priv(dev);
        struct timer_list *timer = &tp->link_timer;
#else
        struct rtl8168_private *tp = from_timer(tp, t, link_timer);
        struct net_device *dev = tp->dev;
        struct timer_list *timer = t;
#endif
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
        rtl8168_check_link_status(dev);
        spin_unlock_irqrestore(&tp->lock, flags);

        mod_timer(timer, jiffies + RTL8168_LINK_TIMEOUT);
}

/* Cfg9346_Unlock assumed. */
static unsigned rtl8168_try_msi(struct pci_dev *pdev, struct rtl8168_private *tp)
{
        unsigned msi = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
                dev_info(&pdev->dev, "Default use INTx.\n");
                break;
        default:
                if (pci_enable_msi(pdev))
                        dev_info(&pdev->dev, "no MSI. Back to INTx.\n");
                else
                        msi |= RTL_FEATURE_MSI;
                break;
        }
#endif

        return msi;
}

static void rtl8168_disable_msi(struct pci_dev *pdev, struct rtl8168_private *tp)
{
        if (tp->features & RTL_FEATURE_MSI) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
                pci_disable_msi(pdev);
#endif
                tp->features &= ~RTL_FEATURE_MSI;
        }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops rtl8168_netdev_ops = {
        .ndo_open       = rtl8168_open,
        .ndo_stop       = rtl8168_close,
        .ndo_get_stats      = rtl8168_get_stats,
        .ndo_start_xmit     = rtl8168_start_xmit,
        .ndo_tx_timeout     = rtl8168_tx_timeout,
        .ndo_change_mtu     = rtl8168_change_mtu,
        .ndo_set_mac_address    = rtl8168_set_mac_address,
        .ndo_do_ioctl       = rtl8168_do_ioctl,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
        .ndo_set_multicast_list = rtl8168_set_rx_mode,
#else
        .ndo_set_rx_mode    = rtl8168_set_rx_mode,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#ifdef CONFIG_R8168_VLAN
        .ndo_vlan_rx_register   = rtl8168_vlan_rx_register,
#endif
#else
        .ndo_fix_features   = rtl8168_fix_features,
        .ndo_set_features   = rtl8168_set_features,
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
        .ndo_poll_controller    = rtl8168_netpoll,
#endif
};
#endif

static int __devinit
rtl8168_init_one(struct pci_dev *pdev,
                 const struct pci_device_id *ent)
{
        struct net_device *dev = NULL;
        struct rtl8168_private *tp;
        void __iomem *ioaddr = NULL;
        static int board_idx = -1;

        int rc;

        assert(pdev != NULL);
        assert(ent != NULL);

        board_idx++;

        if (netif_msg_drv(&debug))
                printk(KERN_INFO "%s Gigabit Ethernet driver %s loaded\n",
                       MODULENAME, RTL8168_VERSION);

        rc = rtl8168_init_board(pdev, &dev, &ioaddr);
        if (rc)
                goto out;

        tp = netdev_priv(dev);
        assert(ioaddr != NULL);

        tp->set_speed = rtl8168_set_speed_xmii;
        tp->get_settings = rtl8168_gset_xmii;
        tp->phy_reset_enable = rtl8168_xmii_reset_enable;
        tp->phy_reset_pending = rtl8168_xmii_reset_pending;
        tp->link_ok = rtl8168_xmii_link_ok;

        tp->features |= rtl8168_try_msi(pdev, tp);

        RTL_NET_DEVICE_OPS(rtl8168_netdev_ops);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,22)
        SET_ETHTOOL_OPS(dev, &rtl8168_ethtool_ops);
#endif

        dev->watchdog_timeo = RTL8168_TX_TIMEOUT;
        dev->irq = pdev->irq;
        dev->base_addr = (unsigned long) ioaddr;

#ifdef CONFIG_R8168_NAPI
        RTL_NAPI_CONFIG(dev, tp, rtl8168_poll, R8168_NAPI_WEIGHT);
#endif

#ifdef CONFIG_R8168_VLAN
        if (tp->mcfg != CFG_METHOD_DEFAULT) {
                dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
                dev->vlan_rx_kill_vid = rtl8168_vlan_rx_kill_vid;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
        }
#endif

        /* There has been a number of reports that using SG/TSO results in
         * tx timeouts. However for a lot of people SG/TSO works fine.
         * Therefore disable both features by default, but allow users to
         * enable them. Use at own risk!
         */
        tp->cp_cmd |= RTL_R16(tp, CPlusCmd);
        if (tp->mcfg != CFG_METHOD_DEFAULT) {
                dev->features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
                tp->cp_cmd |= RxChkSum;
#else
                dev->features |= NETIF_F_RXCSUM;
                dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM |
                                   NETIF_F_RXCSUM | NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
                dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM |
                                     NETIF_F_HIGHDMA;
                if ((tp->mcfg != CFG_METHOD_16) && (tp->mcfg != CFG_METHOD_17)) {
                        //dev->features |= NETIF_F_TSO;
                        dev->hw_features |= NETIF_F_TSO;
                        dev->vlan_features |= NETIF_F_TSO;
                }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
                dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
                dev->hw_features |= NETIF_F_RXALL;
                dev->hw_features |= NETIF_F_RXFCS;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
                if ((tp->mcfg == CFG_METHOD_1) || (tp->mcfg == CFG_METHOD_2) || (tp->mcfg == CFG_METHOD_3)) {
                        dev->hw_features &= ~NETIF_F_IPV6_CSUM;
                        netif_set_gso_max_size(dev, LSO_32K);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
                        dev->gso_max_segs = NIC_MAX_PHYS_BUF_COUNT_LSO_64K;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
                        dev->gso_min_segs = NIC_MIN_PHYS_BUF_COUNT;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
                } else {
                        dev->hw_features |= NETIF_F_IPV6_CSUM;
                        dev->features |=  NETIF_F_IPV6_CSUM;
                        if ((tp->mcfg != CFG_METHOD_16) && (tp->mcfg != CFG_METHOD_17)) {
                                dev->hw_features |= NETIF_F_TSO6;
                                //dev->features |=  NETIF_F_TSO6;
                        }
                        netif_set_gso_max_size(dev, LSO_64K);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
                        dev->gso_max_segs = NIC_MAX_PHYS_BUF_COUNT_LSO2;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
                        dev->gso_min_segs = NIC_MIN_PHYS_BUF_COUNT;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
                }
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        }

        tp->pci_dev = pdev;

        spin_lock_init(&tp->lock);

        rtl8168_init_software_variable(dev);

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH)
                AllocateDashShareMemory(dev);
#endif

        rtl8168_exit_oob(dev);

        rtl8168_hw_init(dev);

        rtl8168_hw_reset(dev);

        /* Get production from EEPROM */
        if (((tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
              tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_29 ||
              tp->mcfg == CFG_METHOD_30) && (rtl8168_mac_ocp_read(tp, 0xDC00) & BIT_3)) ||
            ((tp->mcfg == CFG_METHOD_26) && (rtl8168_mac_ocp_read(tp, 0xDC00) & BIT_4)))
                tp->eeprom_type = EEPROM_TYPE_NONE;
        else
                rtl8168_eeprom_type(tp);

        if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
                rtl8168_set_eeprom_sel_low(tp);

        rtl8168_get_mac_address(dev);

        tp->fw_name = rtl_chip_fw_infos[tp->mcfg].fw_name;

#if defined(ENABLE_DASH_PRINTER_SUPPORT)
        init_completion(&tp->fw_host_ok);
        init_completion(&tp->fw_ack);
        init_completion(&tp->fw_req);
#endif

        tp->tally_vaddr = dma_alloc_coherent(&pdev->dev, sizeof(*tp->tally_vaddr),
                                             &tp->tally_paddr, GFP_KERNEL);
        if (!tp->tally_vaddr) {
                rc = -ENOMEM;
                goto err_out;
        }

        rtl8168_tally_counter_clear(tp);

        pci_set_drvdata(pdev, dev);

        rc = register_netdev(dev);
        if (rc)
                goto err_out;

        printk(KERN_INFO "%s: This product is covered by one or more of the following patents: US6,570,884, US6,115,776, and US6,327,625.\n", MODULENAME);

        rtl8168_disable_rxdvgate(dev);

        device_set_wakeup_enable(&pdev->dev, tp->wol_enabled);

        netif_carrier_off(dev);

        printk("%s", GPL_CLAIM);

out:
        return rc;

err_out:
        if (tp->tally_vaddr != NULL) {
                dma_free_coherent(&pdev->dev, sizeof(*tp->tally_vaddr), tp->tally_vaddr,
                                  tp->tally_paddr);

                tp->tally_vaddr = NULL;
        }
#ifdef  CONFIG_R8168_NAPI
        RTL_NAPI_DEL(tp);
#endif
        rtl8168_disable_msi(pdev, tp);
        rtl8168_release_board(pdev, dev);

        goto out;
}

static void __devexit
rtl8168_remove_one(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8168_private *tp = netdev_priv(dev);

        assert(dev != NULL);
        assert(tp != NULL);

#ifdef  CONFIG_R8168_NAPI
        RTL_NAPI_DEL(tp);
#endif
        if (HW_DASH_SUPPORT_DASH(tp))
                rtl8168_driver_stop(tp);

        unregister_netdev(dev);
        rtl8168_disable_msi(pdev, tp);
#ifdef ENABLE_R8168_PROCFS
        rtl8168_proc_remove(dev);
#endif
        if (tp->tally_vaddr != NULL) {
                dma_free_coherent(&pdev->dev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
                tp->tally_vaddr = NULL;
        }

        rtl8168_release_board(pdev, dev);

#ifdef ENABLE_USE_FIRMWARE_FILE
        rtl8168_release_firmware(tp);
#endif

        pci_set_drvdata(pdev, NULL);
}

static void
rtl8168_set_rxbufsize(struct rtl8168_private *tp,
                      struct net_device *dev)
{
        unsigned int mtu = dev->mtu;

        tp->rx_buf_sz = (mtu > ETH_DATA_LEN) ? mtu + ETH_HLEN + 8 + 1 : RX_BUF_SIZE;
}

#ifdef ENABLE_USE_FIRMWARE_FILE
static void rtl8168_request_firmware(struct rtl8168_private *tp)
{
        struct rtl8168_fw *rtl_fw;

        /* firmware loaded already or no firmware available */
        if (tp->rtl_fw || !tp->fw_name)
                return;

        rtl_fw = kzalloc(sizeof(*rtl_fw), GFP_KERNEL);
        if (!rtl_fw)
                return;

        rtl_fw->phy_write = rtl8168_mdio_write;
        rtl_fw->phy_read = rtl8168_mdio_read;
        rtl_fw->mac_mcu_write = mac_mcu_write;
        rtl_fw->mac_mcu_read = mac_mcu_read;
        rtl_fw->fw_name = tp->fw_name;
        rtl_fw->dev = tp_to_dev(tp);

        if (rtl8168_fw_request_firmware(rtl_fw))
                kfree(rtl_fw);
        else
                tp->rtl_fw = rtl_fw;
}
#endif

static int rtl8168_open(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;
        unsigned long flags;
        int retval;

        retval = -ENOMEM;

#ifdef ENABLE_R8168_PROCFS
        rtl8168_proc_init(dev);
#endif
        rtl8168_set_rxbufsize(tp, dev);
        /*
        * Rx and Tx descriptors needs 256 bytes alignment.
        * pci_alloc_consistent provides more.
        */
        tp->TxDescArray = dma_alloc_coherent(&pdev->dev, R8168_TX_RING_BYTES,
                                             &tp->TxPhyAddr, GFP_KERNEL);
        if (!tp->TxDescArray)
                goto err_free_all_allocated_mem;

        tp->RxDescArray = dma_alloc_coherent(&pdev->dev, R8168_RX_RING_BYTES,
                                             &tp->RxPhyAddr, GFP_KERNEL);
        if (!tp->RxDescArray)
                goto err_free_all_allocated_mem;

        retval = rtl8168_init_ring(dev);
        if (retval < 0)
                goto err_free_all_allocated_mem;

        retval = request_irq(dev->irq, rtl8168_interrupt, (tp->features & RTL_FEATURE_MSI) ? 0 : SA_SHIRQ, dev->name, dev);
        if (retval<0)
                goto err_free_all_allocated_mem;

        if (netif_msg_probe(tp)) {
                printk(KERN_INFO "%s: 0x%lx, "
                       "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
                       "IRQ %d\n",
                       dev->name,
                       dev->base_addr,
                       dev->dev_addr[0], dev->dev_addr[1],
                       dev->dev_addr[2], dev->dev_addr[3],
                       dev->dev_addr[4], dev->dev_addr[5], dev->irq);
        }

#ifdef ENABLE_USE_FIRMWARE_FILE
        rtl8168_request_firmware(tp);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
        INIT_WORK(&tp->task, rtl8168_reset_task, dev);
#else
        INIT_DELAYED_WORK(&tp->task, rtl8168_reset_task);
#endif

        pci_set_master(pdev);

#ifdef  CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif
        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_exit_oob(dev);

        rtl8168_hw_init(dev);

        rtl8168_hw_reset(dev);

        rtl8168_powerup_pll(dev);

        rtl8168_hw_ephy_config(dev);

        rtl8168_hw_phy_config(dev);

        rtl8168_hw_config(dev);

        rtl8168_dsm(dev, DSM_IF_UP);

        rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);

        spin_unlock_irqrestore(&tp->lock, flags);

        if (tp->esd_flag == 0)
                rtl8168_request_esd_timer(dev);

        rtl8168_request_link_timer(dev);

out:

        return retval;

err_free_all_allocated_mem:
        if (tp->RxDescArray != NULL) {
                dma_free_coherent(&pdev->dev, R8168_RX_RING_BYTES, tp->RxDescArray,
                                  tp->RxPhyAddr);
                tp->RxDescArray = NULL;
        }

        if (tp->TxDescArray != NULL) {
                dma_free_coherent(&pdev->dev, R8168_TX_RING_BYTES, tp->TxDescArray,
                                  tp->TxPhyAddr);
                tp->TxDescArray = NULL;
        }

        goto out;
}

static void
rtl8168_dsm(struct net_device *dev, int dev_state)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        switch (dev_state) {
        case DSM_MAC_INIT:
                if ((tp->mcfg == CFG_METHOD_5) || (tp->mcfg == CFG_METHOD_6)) {
                        if (RTL_R8(tp, MACDBG) & 0x80)
                                RTL_W8(tp, GPIO, RTL_R8(tp, GPIO) | GPIO_en);
                        else
                                RTL_W8(tp, GPIO, RTL_R8(tp, GPIO) & ~GPIO_en);
                }

                break;
        case DSM_NIC_GOTO_D3:
        case DSM_IF_DOWN:
                if ((tp->mcfg == CFG_METHOD_5) || (tp->mcfg == CFG_METHOD_6)) {
                        if (RTL_R8(tp, MACDBG) & 0x80)
                                RTL_W8(tp, GPIO, RTL_R8(tp, GPIO) & ~GPIO_en);
                }
                break;

        case DSM_NIC_RESUME_D3:
        case DSM_IF_UP:
                if ((tp->mcfg == CFG_METHOD_5) || (tp->mcfg == CFG_METHOD_6)) {
                        if (RTL_R8(tp, MACDBG) & 0x80)
                                RTL_W8(tp, GPIO, RTL_R8(tp, GPIO) | GPIO_en);
                }

                break;
        }

}

static void
set_offset70F(struct rtl8168_private *tp, u8 setting)
{
        u32 csi_tmp;
        u32 temp = (u32)setting;
        temp = temp << 24;
        /*set PCI configuration space offset 0x70F to setting*/
        /*When the register offset of PCI configuration space larger than 0xff, use CSI to access it.*/

        csi_tmp = rtl8168_csi_read(tp, 0x70c) & 0x00ffffff;
        rtl8168_csi_write(tp, 0x70c, csi_tmp | temp);
}

static void
set_offset79(struct rtl8168_private *tp, u8 setting)
{
        //Set PCI configuration space offset 0x79 to setting

        struct pci_dev *pdev = tp->pci_dev;
        u8 device_control;

        if (hwoptimize & HW_PATCH_SOC_LAN) return;

        pci_read_config_byte(pdev, 0x79, &device_control);
        device_control &= ~0x70;
        device_control |= setting;
        pci_write_config_byte(pdev, 0x79, device_control);
}

static void
set_offset711(struct rtl8168_private *tp, u8 setting)
{
        u32 csi_tmp;
        u32 temp = (u32)setting;
        temp &= 0x0f;
        temp = temp << 12;
        /*set PCI configuration space offset 0x711 to setting*/

        csi_tmp = rtl8168_csi_read(tp, 0x710) & 0xffff0fff;
        rtl8168_csi_write(tp, 0x710, csi_tmp | temp);
}

static void
rtl8168_hw_set_rx_packet_filter(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 mc_filter[2];   /* Multicast hash filter */
        int rx_mode;
        u32 tmp = 0;

        if (dev->flags & IFF_PROMISC) {
                /* Unconditionally log net taps. */
                if (netif_msg_link(tp))
                        printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n",
                               dev->name);

                rx_mode =
                        AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
                        AcceptAllPhys;
                mc_filter[1] = mc_filter[0] = 0xffffffff;
        } else if ((netdev_mc_count(dev) > multicast_filter_limit)
                   || (dev->flags & IFF_ALLMULTI)) {
                /* Too many to filter perfectly -- accept all multicasts. */
                rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
                mc_filter[1] = mc_filter[0] = 0xffffffff;
        } else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
                struct dev_mc_list *mclist;
                unsigned int i;

                rx_mode = AcceptBroadcast | AcceptMyPhys;
                mc_filter[1] = mc_filter[0] = 0;
                for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
                     i++, mclist = mclist->next) {
                        int bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;
                        mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
                        rx_mode |= AcceptMulticast;
                }
#else
                struct netdev_hw_addr *ha;

                rx_mode = AcceptBroadcast | AcceptMyPhys;
                mc_filter[1] = mc_filter[0] = 0;
                netdev_for_each_mc_addr(ha, dev) {
                        int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;
                        mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
                        rx_mode |= AcceptMulticast;
                }
#endif
        }

        if (dev->features & NETIF_F_RXALL)
                rx_mode |= (AcceptErr | AcceptRunt);

        tmp = mc_filter[0];
        mc_filter[0] = swab32(mc_filter[1]);
        mc_filter[1] = swab32(tmp);

        tp->rtl8168_rx_config = rtl_chip_info[tp->chipset].RCR_Cfg;
        tmp = tp->rtl8168_rx_config | rx_mode | (RTL_R32(tp, RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);

        RTL_W32(tp, RxConfig, tmp);
        RTL_W32(tp, MAR0 + 0, mc_filter[0]);
        RTL_W32(tp, MAR0 + 4, mc_filter[1]);
}

static void
rtl8168_set_rx_mode(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_hw_set_rx_packet_filter(dev);

        spin_unlock_irqrestore(&tp->lock, flags);
}

static void
rtl8168_hw_config(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;
        u8 device_control;
        u16 mac_ocp_data;
        u32 csi_tmp;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        if (dev->mtu > ETH_DATA_LEN) {
                dev->features &= ~(NETIF_F_IP_CSUM);
        } else {
                dev->features |= NETIF_F_IP_CSUM;
        }
#endif

        RTL_W32(tp, RxConfig, (RX_DMA_BURST << RxCfgDMAShift));

        rtl8168_hw_reset(dev);

        rtl8168_enable_cfg9346_write(tp);
        if (tp->HwSuppAspmClkIntrLock) {
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) & ~BIT_7);
                rtl8168_hw_aspm_clkreq_enable(tp, false);
        }

        //clear io_rdy_l23
        switch (tp->mcfg) {
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~BIT_1);
                break;
        }

        //keep magic packet only
        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                csi_tmp = rtl8168_eri_read(tp, 0xDE, 1, ERIAR_ExGMAC);
                csi_tmp &= BIT_0;
                rtl8168_eri_write(tp, 0xDE, 1, csi_tmp, ERIAR_ExGMAC);
                break;
        }

        RTL_W8(tp, MTPS, Reserved1_data);

        tp->cp_cmd |= INTT_1;
        if (tp->use_timer_interrrupt)
                tp->cp_cmd |= PktCntrDisable;
        else
                tp->cp_cmd &= ~PktCntrDisable;

        RTL_W16(tp, IntrMitigate, 0x5f51);

        rtl8168_tally_counter_addr_fill(tp);

        rtl8168_desc_addr_fill(tp);

        /* Set DMA burst size and Interframe Gap Time */
        if (tp->mcfg == CFG_METHOD_1)
                RTL_W32(tp, TxConfig, (TX_DMA_BURST_512 << TxDMAShift) |
                        (InterFrameGap << TxInterFrameGapShift));
        else
                RTL_W32(tp, TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
                        (InterFrameGap << TxInterFrameGapShift));

        if (tp->mcfg == CFG_METHOD_4) {
                set_offset70F(tp, 0x27);

                RTL_W8(tp, DBG_reg, (0x0E << 4) | Fix_Nak_1 | Fix_Nak_2);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                //disable clock request.
                pci_write_config_byte(pdev, 0x81, 0x00);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);

                        set_offset79(tp, 0x50);
                }

                //rx checksum offload enable
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
                tp->cp_cmd |= RxChkSum;
#else
                dev->features |= NETIF_F_RXCSUM;
#endif

                tp->cp_cmd &= ~(EnableBist | Macdbgo_oe | Force_halfdup |
                                Force_rxflow_en | Force_txflow_en | Cxpl_dbg_sel |
                                ASF | PktCntrDisable | Macdbgo_sel);
        } else if (tp->mcfg == CFG_METHOD_5) {

                set_offset70F(tp, 0x27);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                //disable clock request.
                pci_write_config_byte(pdev, 0x81, 0x00);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);

                        set_offset79(tp, 0x50);
                }

                //rx checksum offload enable
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
                tp->cp_cmd |= RxChkSum;
#else
                dev->features |= NETIF_F_RXCSUM;
#endif
        } else if (tp->mcfg == CFG_METHOD_6) {
                set_offset70F(tp, 0x27);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                //disable clock request.
                pci_write_config_byte(pdev, 0x81, 0x00);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);

                        set_offset79(tp, 0x50);
                }

                //rx checksum offload enable
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
                tp->cp_cmd |= RxChkSum;
#else
                dev->features |= NETIF_F_RXCSUM;
#endif
        } else if (tp->mcfg == CFG_METHOD_7) {
                set_offset70F(tp, 0x27);

                rtl8168_eri_write(tp, 0x1EC, 1, 0x07, ERIAR_ASF);

                //disable clock request.
                pci_write_config_byte(pdev, 0x81, 0x00);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);


                        set_offset79(tp, 0x50);
                }
        } else if (tp->mcfg == CFG_METHOD_8) {

                set_offset70F(tp, 0x27);

                rtl8168_eri_write(tp, 0x1EC, 1, 0x07, ERIAR_ASF);

                //disable clock request.
                pci_write_config_byte(pdev, 0x81, 0x00);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                RTL_W8(tp, 0xD1, 0x20);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);

                        set_offset79(tp, 0x50);
                }
        } else if (tp->mcfg == CFG_METHOD_9) {
                set_offset70F(tp, 0x27);

                /* disable clock request. */
                pci_write_config_byte(pdev, 0x81, 0x00);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~BIT_4);
                RTL_W8(tp, DBG_reg, RTL_R8(tp, DBG_reg) | BIT_7 | BIT_1);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);

                        set_offset79(tp, 0x50);
                }

                RTL_W8(tp, TDFNR, 0x8);

        } else if (tp->mcfg == CFG_METHOD_10) {
                set_offset70F(tp, 0x27);

                RTL_W8(tp, DBG_reg, RTL_R8(tp, DBG_reg) | BIT_7 | BIT_1);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | Jumbo_En1);

                        set_offset79(tp, 0x20);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~Jumbo_En1);

                        set_offset79(tp, 0x50);
                }

                RTL_W8(tp, TDFNR, 0x8);

                RTL_W8(tp, Config1, RTL_R8(tp, Config1) | 0x10);

                /* disable clock request. */
                pci_write_config_byte(pdev, 0x81, 0x00);
        } else if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_13) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                else
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);

                pci_write_config_byte(pdev, 0x81, 0x00);

                RTL_W8(tp, Config1, RTL_R8(tp, Config1) | 0x10);

        } else if (tp->mcfg == CFG_METHOD_12) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                else
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);

                pci_write_config_byte(pdev, 0x81, 0x01);

                RTL_W8(tp, Config1, RTL_R8(tp, Config1) | 0x10);

        } else if (tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15) {

                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                if (dev->mtu > ETH_DATA_LEN) {
                        RTL_W8(tp, MTPS, 0x24);
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) | Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | 0x01);
                } else {
                        RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Jumbo_En0);
                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~0x01);
                }

                RTL_W8(tp, 0xF3, RTL_R8(tp, 0xF3) | BIT_5);
                RTL_W8(tp, 0xF3, RTL_R8(tp, 0xF3) & ~BIT_5);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_7 | BIT_6);

                RTL_W8(tp, 0xD1, RTL_R8(tp, 0xD1) | BIT_2 | BIT_3);

                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_6 | BIT_5 | BIT_4 | BIT_2 | BIT_1);

                RTL_W8(tp, TDFNR, 0x8);

                /*
                if (aspm)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
                */

                RTL_W8(tp, Config5, RTL_R8(tp, Config5) & ~BIT_3);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                RTL_W8(tp, Config1, RTL_R8(tp, Config1) & ~0x10);
        } else if (tp->mcfg == CFG_METHOD_16 || tp->mcfg == CFG_METHOD_17) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                rtl8168_eri_write(tp, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
                csi_tmp = rtl8168_eri_read(tp, 0x1D0, 4, ERIAR_ExGMAC);
                csi_tmp |= BIT_1;
                rtl8168_eri_write(tp, 0x1D0, 1, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0xDC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

                RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) | BIT_7);
                RTL_W8(tp, 0xD3, RTL_R8(tp, 0xD3) & ~BIT_7);
                RTL_W8(tp, 0x1B, RTL_R8(tp, 0x1B) & ~0x07);

                if (tp->mcfg == CFG_METHOD_16) {
                        RTL_W32(tp, 0xB0, 0xEE480010);
                        RTL_W8(tp, 0x1A, RTL_R8(tp, 0x1A) & ~(BIT_2|BIT_3));
                        rtl8168_eri_write(tp, 0x1DC, 1, 0x64, ERIAR_ExGMAC);
                } else {
                        csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                        csi_tmp |= BIT_4;
                        rtl8168_eri_write(tp, 0x1B0, 1, csi_tmp, ERIAR_ExGMAC);
                        rtl8168_eri_write(tp, 0xCC, 4, 0x00000050, ERIAR_ExGMAC);
                        rtl8168_eri_write(tp, 0xD0, 4, 0x07ff0060, ERIAR_ExGMAC);
                }

                RTL_W8(tp, TDFNR, 0x8);

                RTL_W8(tp, Config2, RTL_R8(tp, Config2) & ~PMSTS_En);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6);
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_6);

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, MTPS, 0x27);

                /* disable clock request. */
                pci_write_config_byte(pdev, 0x81, 0x00);

        } else if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                rtl8168_eri_write(tp, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
                RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) | BIT_7);
                RTL_W8(tp, 0xD3, RTL_R8(tp, 0xD3) & ~BIT_7);
                csi_tmp = rtl8168_eri_read(tp, 0xDC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

                /*
                if (aspm)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
                */

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, MTPS, 0x27);

                RTL_W8(tp, TDFNR, 0x8);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6);
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_6);

                rtl8168_eri_write(tp, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
                RTL_W8(tp, 0x1B,RTL_R8(tp, 0x1B) & ~0x07);

                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_4;
                rtl8168_eri_write(tp, 0x1B0, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp = rtl8168_eri_read(tp, 0x1d0, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_4 | BIT_1;
                rtl8168_eri_write(tp, 0x1d0, 1, csi_tmp, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xCC, 4, 0x00000050, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xd0, 4, 0x00000060, ERIAR_ExGMAC);
        } else if (tp->mcfg == CFG_METHOD_20) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                rtl8168_eri_write(tp, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
                RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) | BIT_7);
                RTL_W8(tp, 0xD3, RTL_R8(tp, 0xD3) & ~BIT_7);
                csi_tmp = rtl8168_eri_read(tp, 0xDC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

                /*
                if (aspm)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
                */

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, MTPS, 0x27);

                RTL_W8(tp, TDFNR, 0x8);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6);
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_6);
                rtl8168_eri_write(tp, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_4;
                rtl8168_eri_write(tp, 0x1B0, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp = rtl8168_eri_read(tp, 0x1d0, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_4 | BIT_1;
                rtl8168_eri_write(tp, 0x1d0, 1, csi_tmp, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xCC, 4, 0x00000050, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xd0, 4, 0x00000060, ERIAR_ExGMAC);
        } else if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
                   tp->mcfg == CFG_METHOD_24 || tp->mcfg == CFG_METHOD_25 ||
                   tp->mcfg == CFG_METHOD_26 || tp->mcfg == CFG_METHOD_29 ||
                   tp->mcfg == CFG_METHOD_30) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);
                if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22)
                        set_offset711(tp, 0x04);

                rtl8168_eri_write(tp, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xCC, 1, 0x38, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xD0, 1, 0x48, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

                RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) | BIT_7);

                csi_tmp = rtl8168_eri_read(tp, 0xDC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

                if (tp->mcfg == CFG_METHOD_26) {
                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD3C0);
                        mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        mac_ocp_data |= 0x0FFF;
                        rtl8168_mac_ocp_write(tp, 0xD3C0, mac_ocp_data);
                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD3C2);
                        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        rtl8168_mac_ocp_write(tp, 0xD3C2, mac_ocp_data);
                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD3C4);
                        mac_ocp_data |= BIT_0;
                        rtl8168_mac_ocp_write(tp, 0xD3C4, mac_ocp_data);
                } else if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30) {

                        if (tp->RequireAdjustUpsTxLinkPulseTiming) {
                                mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD412);
                                mac_ocp_data &= ~(0x0FFF);
                                mac_ocp_data |= tp->SwrCnt1msIni;
                                rtl8168_mac_ocp_write(tp, 0xD412, mac_ocp_data);
                        }

                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xE056);
                        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
                        //mac_ocp_data |= (BIT_6 | BIT_5 | BIT_4);
                        rtl8168_mac_ocp_write(tp, 0xE056, mac_ocp_data);

                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xE052);
                        mac_ocp_data &= ~(BIT_15 | BIT_14 | BIT_13 | BIT_3);
                        mac_ocp_data |= BIT_15;
                        //mac_ocp_data |= BIT_3;
                        rtl8168_mac_ocp_write(tp, 0xE052, mac_ocp_data);

                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD420);
                        mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        mac_ocp_data |= 0x45F;
                        rtl8168_mac_ocp_write(tp, 0xD420, mac_ocp_data);

                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xE0D6);
                        mac_ocp_data &= ~(BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        mac_ocp_data |= 0x17F;
                        rtl8168_mac_ocp_write(tp, 0xE0D6, mac_ocp_data);
                }

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                RTL_W8(tp, 0x1B, RTL_R8(tp, 0x1B) & ~0x07);

                RTL_W8(tp, TDFNR, 0x4);

                RTL_W8(tp, Config2, RTL_R8(tp, Config2) & ~PMSTS_En);

                /*
                if (aspm)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
                */

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, MTPS, 0x27);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6);
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_6);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_7);

                rtl8168_eri_write(tp, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);

                if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30) {
                        rtl8168_mac_ocp_write(tp, 0xE054, 0x0000);

                        rtl8168_eri_write(tp, 0x5F0, 2, 0x4000, ERIAR_ExGMAC);
                } else {
                        rtl8168_eri_write(tp, 0x5F0, 2, 0x4F87, ERIAR_ExGMAC);
                }

                if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30) {
                        csi_tmp = rtl8168_eri_read(tp, 0xDC, 4, ERIAR_ExGMAC);
                        csi_tmp |= (BIT_2 | BIT_3 | BIT_4);
                        rtl8168_eri_write(tp, 0xDC, 4, csi_tmp, ERIAR_ExGMAC);
                }

                if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
                    tp->mcfg == CFG_METHOD_24 || tp->mcfg == CFG_METHOD_25) {
                        rtl8168_mac_ocp_write(tp, 0xC140, 0xFFFF);
                } else if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30) {
                        rtl8168_mac_ocp_write(tp, 0xC140, 0xFFFF);
                        rtl8168_mac_ocp_write(tp, 0xC142, 0xFFFF);
                }

                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_12;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);

                if (tp->mcfg == CFG_METHOD_29 || tp->mcfg == CFG_METHOD_30) {
                        csi_tmp = rtl8168_eri_read(tp, 0x2FC, 1, ERIAR_ExGMAC);
                        csi_tmp &= ~(BIT_2);
                        rtl8168_eri_write(tp, 0x2FC, 1, csi_tmp, ERIAR_ExGMAC);
                } else {
                        csi_tmp = rtl8168_eri_read(tp, 0x2FC, 1, ERIAR_ExGMAC);
                        csi_tmp &= ~(BIT_0 | BIT_1 | BIT_2);
                        csi_tmp |= BIT_0;
                        rtl8168_eri_write(tp, 0x2FC, 1, csi_tmp, ERIAR_ExGMAC);
                }

                csi_tmp = rtl8168_eri_read(tp, 0x1D0, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_1;
                rtl8168_eri_write(tp, 0x1D0, 1, csi_tmp, ERIAR_ExGMAC);
        } else if (tp->mcfg == CFG_METHOD_23 || tp->mcfg == CFG_METHOD_27 ||
                   tp->mcfg == CFG_METHOD_28) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                rtl8168_eri_write(tp, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xCC, 1, 0x2F, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xD0, 1, 0x5F, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

                RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) | BIT_7);

                csi_tmp = rtl8168_eri_read(tp, 0xDC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6);
                RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_6);

                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_7);

                rtl8168_eri_write(tp, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
                RTL_W8(tp, 0x1B, RTL_R8(tp, 0x1B) & ~0x07);

                RTL_W8(tp, TDFNR, 0x4);

                /*
                if (aspm)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
                */

                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_12;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x2FC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_0 | BIT_1 | BIT_2);
                csi_tmp |= (BIT_0 | BIT_1);
                rtl8168_eri_write(tp, 0x2FC, 1, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x1D0, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_1;
                rtl8168_eri_write(tp, 0x1D0, 1, csi_tmp, ERIAR_ExGMAC);

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, MTPS, 0x27);

                if (tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28) {
                        rtl8168_oob_mutex_lock(tp);
                        rtl8168_eri_write(tp, 0x5F0, 2, 0x4F87, ERIAR_ExGMAC);
                        rtl8168_oob_mutex_unlock(tp);
                }

                rtl8168_mac_ocp_write(tp, 0xC140, 0xFFFF);
                rtl8168_mac_ocp_write(tp, 0xC142, 0xFFFF);

                if (tp->mcfg == CFG_METHOD_28) {
                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD3E2);
                        mac_ocp_data &= 0xF000;
                        mac_ocp_data |= 0xAFD;
                        rtl8168_mac_ocp_write(tp, 0xD3E2, mac_ocp_data);

                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD3E4);
                        mac_ocp_data &= 0xFF00;
                        rtl8168_mac_ocp_write(tp, 0xD3E4, mac_ocp_data);

                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xE860);
                        mac_ocp_data |= BIT_7;
                        rtl8168_mac_ocp_write(tp, 0xE860, mac_ocp_data);
                }
        } else if (tp->mcfg == CFG_METHOD_31 || tp->mcfg == CFG_METHOD_32 ||
                   tp->mcfg == CFG_METHOD_33) {
                set_offset70F(tp, 0x27);
                set_offset79(tp, 0x50);

                rtl8168_eri_write(tp, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xCC, 1, 0x2F, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xD0, 1, 0x5F, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

                RTL_W32(tp, TxConfig, RTL_R32(tp, TxConfig) | BIT_7);

                csi_tmp = rtl8168_eri_read(tp, 0xDC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

                if (tp->RequireAdjustUpsTxLinkPulseTiming) {
                        mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD412);
                        mac_ocp_data &= ~(0x0FFF);
                        mac_ocp_data |= tp->SwrCnt1msIni;
                        rtl8168_mac_ocp_write(tp, 0xD412, mac_ocp_data);
                }

                mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xE056);
                mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
                rtl8168_mac_ocp_write(tp, 0xE056, mac_ocp_data);
                if (FALSE == HW_SUPP_SERDES_PHY(tp))
                        rtl8168_mac_ocp_write(tp, 0xEA80, 0x0003);
                else
                        rtl8168_mac_ocp_write(tp, 0xEA80, 0x0000);

                rtl8168_oob_mutex_lock(tp);
                mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xE052);
                mac_ocp_data &= ~(BIT_3 | BIT_0);
                rtl8168_mac_ocp_write(tp, 0xE052, mac_ocp_data);
                rtl8168_oob_mutex_unlock(tp);

                mac_ocp_data = rtl8168_mac_ocp_read(tp, 0xD420);
                mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                mac_ocp_data |= 0x45F;
                rtl8168_mac_ocp_write(tp, 0xD420, mac_ocp_data);

                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                RTL_W8(tp, 0x1B, RTL_R8(tp, 0x1B) & ~0x07);

                RTL_W8(tp, TDFNR, 0x4);

                RTL_W8(tp, Config2, RTL_R8(tp, Config2) & ~PMSTS_En);

                /*
                if (aspm)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
                */

                if (dev->mtu > ETH_DATA_LEN)
                        RTL_W8(tp, MTPS, 0x27);

                if (FALSE == HW_SUPP_SERDES_PHY(tp)) {
                        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6);
                        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_6);
                        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_7);
                } else {
                        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~BIT_6);
                        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_6);
                        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~BIT_7);
                }

                rtl8168_eri_write(tp, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                rtl8168_eri_write(tp, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);

                rtl8168_oob_mutex_lock(tp);
                rtl8168_eri_write(tp, 0x5F0, 2, 0x4000, ERIAR_ExGMAC);
                rtl8168_oob_mutex_unlock(tp);

                if (tp->mcfg == CFG_METHOD_32 || tp->mcfg == CFG_METHOD_33) {
                        csi_tmp = rtl8168_eri_read(tp, 0xD4, 4, ERIAR_ExGMAC);
                        csi_tmp |= BIT_4;
                        rtl8168_eri_write(tp, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
                }

                rtl8168_mac_ocp_write(tp, 0xC140, 0xFFFF);
                rtl8168_mac_ocp_write(tp, 0xC142, 0xFFFF);

                csi_tmp = rtl8168_eri_read(tp, 0x1B0, 4, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_12;
                rtl8168_eri_write(tp, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x2FC, 1, ERIAR_ExGMAC);
                csi_tmp &= ~(BIT_0 | BIT_1);
                csi_tmp |= BIT_0;
                rtl8168_eri_write(tp, 0x2FC, 1, csi_tmp, ERIAR_ExGMAC);

                csi_tmp = rtl8168_eri_read(tp, 0x1D0, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_1;
                rtl8168_eri_write(tp, 0x1D0, 1, csi_tmp, ERIAR_ExGMAC);
        } else if (tp->mcfg == CFG_METHOD_1) {
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                if (dev->mtu > ETH_DATA_LEN) {
                        pci_read_config_byte(pdev, 0x69, &device_control);
                        device_control &= ~0x70;
                        device_control |= 0x28;
                        pci_write_config_byte(pdev, 0x69, device_control);
                } else {
                        pci_read_config_byte(pdev, 0x69, &device_control);
                        device_control &= ~0x70;
                        device_control |= 0x58;
                        pci_write_config_byte(pdev, 0x69, device_control);
                }
        } else if (tp->mcfg == CFG_METHOD_2) {
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                if (dev->mtu > ETH_DATA_LEN) {
                        pci_read_config_byte(pdev, 0x69, &device_control);
                        device_control &= ~0x70;
                        device_control |= 0x28;
                        pci_write_config_byte(pdev, 0x69, device_control);

                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | (1 << 0));
                } else {
                        pci_read_config_byte(pdev, 0x69, &device_control);
                        device_control &= ~0x70;
                        device_control |= 0x58;
                        pci_write_config_byte(pdev, 0x69, device_control);

                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~(1 << 0));
                }
        } else if (tp->mcfg == CFG_METHOD_3) {
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~Beacon_en);

                if (dev->mtu > ETH_DATA_LEN) {
                        pci_read_config_byte(pdev, 0x69, &device_control);
                        device_control &= ~0x70;
                        device_control |= 0x28;
                        pci_write_config_byte(pdev, 0x69, device_control);

                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) | (1 << 0));
                } else {
                        pci_read_config_byte(pdev, 0x69, &device_control);
                        device_control &= ~0x70;
                        device_control |= 0x58;
                        pci_write_config_byte(pdev, 0x69, device_control);

                        RTL_W8(tp, Config4, RTL_R8(tp, Config4) & ~(1 << 0));
                }
        }

        if ((tp->mcfg == CFG_METHOD_1) || (tp->mcfg == CFG_METHOD_2) || (tp->mcfg == CFG_METHOD_3)) {
                /* csum offload command for RTL8168B/8111B */
                tp->tx_tcp_csum_cmd = TxTCPCS;
                tp->tx_udp_csum_cmd = TxUDPCS;
                tp->tx_ip_csum_cmd = TxIPCS;
                tp->tx_ipv6_csum_cmd = 0;
        } else {
                /* csum offload command for RTL8168C/8111C and RTL8168CP/8111CP */
                tp->tx_tcp_csum_cmd = TxTCPCS_C;
                tp->tx_udp_csum_cmd = TxUDPCS_C;
                tp->tx_ip_csum_cmd = TxIPCS_C;
                tp->tx_ipv6_csum_cmd = TxIPV6F_C;
        }


        //other hw parameters
        if (tp->mcfg == CFG_METHOD_21 || tp->mcfg == CFG_METHOD_22 ||
            tp->mcfg == CFG_METHOD_23 || tp->mcfg == CFG_METHOD_24 ||
            tp->mcfg == CFG_METHOD_25 || tp->mcfg == CFG_METHOD_26 ||
            tp->mcfg == CFG_METHOD_27 || tp->mcfg == CFG_METHOD_28)
                rtl8168_eri_write(tp, 0x2F8, 2, 0x1D8F, ERIAR_ExGMAC);

        if (tp->bios_setting & BIT_28) {
                if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19 ||
                    tp->mcfg == CFG_METHOD_20) {
                        u32 gphy_val;

                        rtl8168_mdio_write(tp, 0x1F, 0x0007);
                        rtl8168_mdio_write(tp, 0x1E, 0x002C);
                        gphy_val = rtl8168_mdio_read(tp, 0x16);
                        gphy_val |= BIT_10;
                        rtl8168_mdio_write(tp, 0x16, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0005);
                        rtl8168_mdio_write(tp, 0x05, 0x8B80);
                        gphy_val = rtl8168_mdio_read(tp, 0x06);
                        gphy_val |= BIT_7;
                        rtl8168_mdio_write(tp, 0x06, gphy_val);
                        rtl8168_mdio_write(tp, 0x1F, 0x0000);
                }
        }

        rtl8168_hw_clear_timer_int(dev);

        rtl8168_enable_exit_l1_mask(tp);

        switch (tp->mcfg) {
        case CFG_METHOD_25:
                rtl8168_mac_ocp_write(tp, 0xD3C0, 0x0B00);
                rtl8168_mac_ocp_write(tp, 0xD3C2, 0x0000);
                break;
        case CFG_METHOD_29:
        case CFG_METHOD_30:
                rtl8168_mac_ocp_write(tp, 0xE098, 0x0AA2);
                break;
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                rtl8168_mac_ocp_write(tp, 0xE098, 0xC302);
                break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                if (aspm) {
                        rtl8168_init_pci_offset_99(tp);
                }
                break;
        }
        switch (tp->mcfg) {
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33:
                if (aspm) {
                        rtl8168_init_pci_offset_180(tp);
                }
                break;
        }

        tp->cp_cmd &= ~(EnableBist | Macdbgo_oe | Force_halfdup |
                        Force_rxflow_en | Force_txflow_en | Cxpl_dbg_sel |
                        ASF | Macdbgo_sel);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        RTL_W16(tp, CPlusCmd, tp->cp_cmd);
#else
        rtl8168_hw_set_features(dev, dev->features);
#endif

        switch (tp->mcfg) {
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
        case CFG_METHOD_20:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_26:
        case CFG_METHOD_27:
        case CFG_METHOD_28:
        case CFG_METHOD_29:
        case CFG_METHOD_30:
        case CFG_METHOD_31:
        case CFG_METHOD_32:
        case CFG_METHOD_33: {
                int timeout;
                for (timeout = 0; timeout < 10; timeout++) {
                        if ((rtl8168_eri_read(tp, 0x1AE, 2, ERIAR_ExGMAC) & BIT_13)==0)
                                break;
                        mdelay(1);
                }
        }
        break;
        }

        RTL_W16(tp, RxMaxSize, tp->rx_buf_sz);

        rtl8168_disable_rxdvgate(dev);

        if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12)
                rtl8168_mac_loopback_test(tp);

        if (!tp->pci_cfg_is_read) {
                pci_read_config_byte(pdev, PCI_COMMAND, &tp->pci_cfg_space.cmd);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &tp->pci_cfg_space.io_base_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &tp->pci_cfg_space.io_base_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &tp->pci_cfg_space.mem_base_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &tp->pci_cfg_space.mem_base_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &tp->pci_cfg_space.resv_0x1c_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &tp->pci_cfg_space.resv_0x1c_h);
                pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &tp->pci_cfg_space.ilr);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &tp->pci_cfg_space.resv_0x20_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &tp->pci_cfg_space.resv_0x20_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &tp->pci_cfg_space.resv_0x24_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &tp->pci_cfg_space.resv_0x24_h);
                pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &tp->pci_cfg_space.resv_0x2c_l);
                pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &tp->pci_cfg_space.resv_0x2c_h);
                if (tp->HwPcieSNOffset > 0) {
                        tp->pci_cfg_space.pci_sn_l = rtl8168_csi_read(tp, tp->HwPcieSNOffset);
                        tp->pci_cfg_space.pci_sn_h = rtl8168_csi_read(tp, tp->HwPcieSNOffset + 4);
                }

                tp->pci_cfg_is_read = 1;
        }

        rtl8168_dsm(dev, DSM_MAC_INIT);

        /* Set Rx packet filter */
        rtl8168_hw_set_rx_packet_filter(dev);

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH && !tp->dash_printer_enabled)
                NICChkTypeEnableDashInterrupt(tp);
#endif

        if (tp->HwSuppAspmClkIntrLock)
                rtl8168_hw_aspm_clkreq_enable(tp, true);

        rtl8168_disable_cfg9346_write(tp);

        udelay(10);
}

static void
rtl8168_hw_start(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        RTL_W8(tp, ChipCmd, CmdTxEnb | CmdRxEnb);

        rtl8168_enable_hw_interrupt(tp);
}

static int
rtl8168_change_mtu(struct net_device *dev,
                   int new_mtu)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        int ret = 0;
        unsigned long flags;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
        if (new_mtu < ETH_MIN_MTU)
                return -EINVAL;
        else if (new_mtu > tp->max_jumbo_frame_size)
                new_mtu = tp->max_jumbo_frame_size;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)

        spin_lock_irqsave(&tp->lock, flags);
        dev->mtu = new_mtu;
        spin_unlock_irqrestore(&tp->lock, flags);

        if (!netif_running(dev))
                goto out;

        rtl8168_down(dev);

        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_set_rxbufsize(tp, dev);

        ret = rtl8168_init_ring(dev);

        if (ret < 0) {
                spin_unlock_irqrestore(&tp->lock, flags);
                goto err_out;
        }

#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI

        netif_stop_queue(dev);
        netif_carrier_off(dev);
        rtl8168_hw_config(dev);
        spin_unlock_irqrestore(&tp->lock, flags);

        rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);

        mod_timer(&tp->esd_timer, jiffies + RTL8168_ESD_TIMEOUT);
        mod_timer(&tp->link_timer, jiffies + RTL8168_LINK_TIMEOUT);
out:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
        netdev_update_features(dev);
#endif

err_out:
        return ret;
}

static inline void
rtl8168_make_unusable_by_asic(struct RxDesc *desc)
{
        desc->addr = 0x0badbadbadbadbadull;
        desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

static void
rtl8168_free_rx_skb(struct rtl8168_private *tp,
                    struct sk_buff **sk_buff,
                    struct RxDesc *desc)
{
        struct pci_dev *pdev = tp->pci_dev;

        dma_unmap_single(&pdev->dev, le64_to_cpu(desc->addr), tp->rx_buf_sz,
                         DMA_FROM_DEVICE);
        dev_kfree_skb(*sk_buff);
        *sk_buff = NULL;
        rtl8168_make_unusable_by_asic(desc);
}

static inline void
rtl8168_mark_to_asic(struct RxDesc *desc,
                     u32 rx_buf_sz)
{
        u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

        desc->opts1 = cpu_to_le32(DescOwn | eor | rx_buf_sz);
}

static inline void
rtl8168_map_to_asic(struct RxDesc *desc,
                    dma_addr_t mapping,
                    u32 rx_buf_sz)
{
        desc->addr = cpu_to_le64(mapping);
        wmb();
        rtl8168_mark_to_asic(desc, rx_buf_sz);
}

static int
rtl8168_alloc_rx_skb(struct rtl8168_private *tp,
                     struct sk_buff **sk_buff,
                     struct RxDesc *desc,
                     int rx_buf_sz,
                     u8 in_intr)
{
        struct sk_buff *skb;
        dma_addr_t mapping;
        int ret = 0;

        if (in_intr)
                skb = RTL_ALLOC_SKB_INTR(tp, rx_buf_sz + RTK_RX_ALIGN);
        else
                skb = dev_alloc_skb(rx_buf_sz + RTK_RX_ALIGN);

        if (unlikely(!skb))
                goto err_out;

        skb_reserve(skb, RTK_RX_ALIGN);

        mapping = dma_map_single(tp_to_dev(tp), skb->data, rx_buf_sz,
                                 DMA_FROM_DEVICE);
        if (unlikely(dma_mapping_error(tp_to_dev(tp), mapping))) {
                if (unlikely(net_ratelimit()))
                        netif_err(tp, drv, tp->dev, "Failed to map RX DMA!\n");
                goto err_out;
        }

        *sk_buff = skb;
        rtl8168_map_to_asic(desc, mapping, rx_buf_sz);
out:
        return ret;

err_out:
        if (skb)
                dev_kfree_skb(skb);
        ret = -ENOMEM;
        rtl8168_make_unusable_by_asic(desc);
        goto out;
}

static void
rtl8168_rx_clear(struct rtl8168_private *tp)
{
        int i;

        for (i = 0; i < NUM_RX_DESC; i++) {
                if (tp->Rx_skbuff[i])
                        rtl8168_free_rx_skb(tp, tp->Rx_skbuff + i,
                                            tp->RxDescArray + i);
        }
}

static u32
rtl8168_rx_fill(struct rtl8168_private *tp,
                struct net_device *dev,
                u32 start,
                u32 end,
                u8 in_intr)
{
        u32 cur;

        for (cur = start; end - cur > 0; cur++) {
                int ret, i = cur % NUM_RX_DESC;

                if (tp->Rx_skbuff[i])
                        continue;

                ret = rtl8168_alloc_rx_skb(tp, tp->Rx_skbuff + i,
                                           tp->RxDescArray + i,
                                           tp->rx_buf_sz,
                                           in_intr);
                if (ret < 0)
                        break;
        }
        return cur - start;
}

static inline void
rtl8168_mark_as_last_descriptor(struct RxDesc *desc)
{
        desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8168_desc_addr_fill(struct rtl8168_private *tp)
{
        if (!tp->TxPhyAddr || !tp->RxPhyAddr)
                return;

        RTL_W32(tp, TxDescStartAddrLow, ((u64) tp->TxPhyAddr & DMA_BIT_MASK(32)));
        RTL_W32(tp, TxDescStartAddrHigh, ((u64) tp->TxPhyAddr >> 32));
        RTL_W32(tp, RxDescAddrLow, ((u64) tp->RxPhyAddr & DMA_BIT_MASK(32)));
        RTL_W32(tp, RxDescAddrHigh, ((u64) tp->RxPhyAddr >> 32));
}

static void
rtl8168_tx_desc_init(struct rtl8168_private *tp)
{
        int i = 0;

        memset(tp->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));

        for (i = 0; i < NUM_TX_DESC; i++) {
                if (i == (NUM_TX_DESC - 1))
                        tp->TxDescArray[i].opts1 = cpu_to_le32(RingEnd);
        }
}

static void
rtl8168_rx_desc_offset0_init(struct rtl8168_private *tp, int own)
{
        int i = 0;
        int ownbit = 0;

        if (tp->RxDescArray == NULL) return;

        if (own)
                ownbit = DescOwn;

        for (i = 0; i < NUM_RX_DESC; i++) {
                if (i == (NUM_RX_DESC - 1))
                        tp->RxDescArray[i].opts1 = cpu_to_le32((ownbit | RingEnd) | (unsigned long)tp->rx_buf_sz);
                else
                        tp->RxDescArray[i].opts1 = cpu_to_le32(ownbit | (unsigned long)tp->rx_buf_sz);
        }
}

static void
rtl8168_rx_desc_init(struct rtl8168_private *tp)
{
        memset(tp->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));
}

static int
rtl8168_init_ring(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        rtl8168_init_ring_indexes(tp);

        memset(tp->tx_skb, 0x0, NUM_TX_DESC * sizeof(struct ring_info));
        memset(tp->Rx_skbuff, 0x0, NUM_RX_DESC * sizeof(struct sk_buff *));

        rtl8168_tx_desc_init(tp);
        rtl8168_rx_desc_init(tp);

        if (rtl8168_rx_fill(tp, dev, 0, NUM_RX_DESC, 0) != NUM_RX_DESC)
                goto err_out;

        rtl8168_mark_as_last_descriptor(tp->RxDescArray + NUM_RX_DESC - 1);

        return 0;

err_out:
        rtl8168_rx_clear(tp);
        return -ENOMEM;
}

static void
rtl8168_unmap_tx_skb(struct pci_dev *pdev,
                     struct ring_info *tx_skb,
                     struct TxDesc *desc)
{
        unsigned int len = tx_skb->len;

        dma_unmap_single(&pdev->dev, le64_to_cpu(desc->addr), len, DMA_TO_DEVICE);

        desc->opts1 = cpu_to_le32(RTK_MAGIC_DEBUG_VALUE);
        desc->opts2 = 0x00;
        desc->addr = 0x00;
        tx_skb->len = 0;
}

static void rtl8168_tx_clear_range(struct rtl8168_private *tp, u32 start,
                                   unsigned int n)
{
        unsigned int i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
        struct net_device *dev = tp->dev;
#endif

        for (i = 0; i < n; i++) {
                unsigned int entry = (start + i) % NUM_TX_DESC;
                struct ring_info *tx_skb = tp->tx_skb + entry;
                unsigned int len = tx_skb->len;

                if (len) {
                        struct sk_buff *skb = tx_skb->skb;

                        rtl8168_unmap_tx_skb(tp->pci_dev, tx_skb,
                                             tp->TxDescArray + entry);
                        if (skb) {
                                RTLDEV->stats.tx_dropped++;
                                dev_kfree_skb_any(skb);
                                tx_skb->skb = NULL;
                        }
                }
        }
}

static void
rtl8168_tx_clear(struct rtl8168_private *tp)
{
        rtl8168_tx_clear_range(tp, tp->dirty_tx, NUM_TX_DESC);
        tp->cur_tx = tp->dirty_tx = 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8168_schedule_work(struct net_device *dev, void (*task)(void *))
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        struct rtl8168_private *tp = netdev_priv(dev);

        INIT_WORK(&tp->task, task, dev);
        schedule_delayed_work(&tp->task, 4);
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
}

#define rtl8168_cancel_schedule_work(a)

#else
static void rtl8168_schedule_work(struct net_device *dev, work_func_t task)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        INIT_DELAYED_WORK(&tp->task, task);
        schedule_delayed_work(&tp->task, 4);
}

static void rtl8168_cancel_schedule_work(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct work_struct *work = &tp->task.work;

        if (!work->func) return;

        cancel_delayed_work_sync(&tp->task);
}
#endif

static void
rtl8168_wait_for_quiescence(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        synchronize_irq(dev->irq);

        /* Wait for any pending NAPI task to complete */
#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        RTL_NAPI_DISABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI

        rtl8168_irq_mask_and_ack(tp);

#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI
}

#if 0
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8168_reinit_task(void *_data)
#else
static void rtl8168_reinit_task(struct work_struct *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
        struct net_device *dev = _data;
#else
        struct rtl8168_private *tp =
                container_of(work, struct rtl8168_private, task.work);
        struct net_device *dev = tp->dev;
#endif
        int ret;

        if (netif_running(dev)) {
                rtl8168_wait_for_quiescence(dev);
                rtl8168_close(dev);
        }

        ret = rtl8168_open(dev);
        if (unlikely(ret < 0)) {
                if (unlikely(net_ratelimit())) {
                        struct rtl8168_private *tp = netdev_priv(dev);

                        if (netif_msg_drv(tp)) {
                                printk(PFX KERN_ERR
                                       "%s: reinit failure (status = %d)."
                                       " Rescheduling.\n", dev->name, ret);
                        }
                }
                rtl8168_schedule_work(dev, rtl8168_reinit_task);
        }
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8168_reset_task(void *_data)
{
        struct net_device *dev = _data;
        struct rtl8168_private *tp = netdev_priv(dev);
#else
static void rtl8168_reset_task(struct work_struct *work)
{
        struct rtl8168_private *tp =
                container_of(work, struct rtl8168_private, task.work);
        struct net_device *dev = tp->dev;
#endif
        u32 budget = ~(u32)0;
        unsigned long flags;

        if (!netif_running(dev))
                return;

        rtl8168_wait_for_quiescence(dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
        rtl8168_rx_interrupt(dev, tp, &budget);
#else
        rtl8168_rx_interrupt(dev, tp, budget);
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)

        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_tx_clear(tp);

        if (tp->dirty_rx == tp->cur_rx) {
                rtl8168_rx_clear(tp);
                rtl8168_init_ring(dev);
                rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);
                spin_unlock_irqrestore(&tp->lock, flags);
        } else {
                spin_unlock_irqrestore(&tp->lock, flags);
                if (unlikely(net_ratelimit())) {
                        struct rtl8168_private *tp = netdev_priv(dev);

                        if (netif_msg_intr(tp)) {
                                printk(PFX KERN_EMERG
                                       "%s: Rx buffers shortage\n", dev->name);
                        }
                }
                rtl8168_schedule_work(dev, rtl8168_reset_task);
        }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static void
rtl8168_tx_timeout(struct net_device *dev, unsigned int txqueue)
#else
static void
rtl8168_tx_timeout(struct net_device *dev)
#endif
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        spin_lock_irqsave(&tp->lock, flags);
        netif_stop_queue(dev);
        netif_carrier_off(dev);
        rtl8168_hw_reset(dev);
        spin_unlock_irqrestore(&tp->lock, flags);

        /* Let's wait a bit while any (async) irq lands on */
        rtl8168_schedule_work(dev, rtl8168_reset_task);
}

static u32
rtl8168_get_txd_opts1(u32 opts1, u32 len, unsigned int entry)
{
        u32 status = opts1 | len;

        if (entry == NUM_TX_DESC - 1)
                status |= RingEnd;

        return status;
}

static int
rtl8168_xmit_frags(struct rtl8168_private *tp,
                   struct sk_buff *skb,
                   const u32 *opts)
{
        struct skb_shared_info *info = skb_shinfo(skb);
        unsigned int cur_frag, entry;
        struct TxDesc *txd = NULL;
        const unsigned char nr_frags = info->nr_frags;

        entry = tp->cur_tx;
        for (cur_frag = 0; cur_frag < nr_frags; cur_frag++) {
                skb_frag_t *frag = info->frags + cur_frag;
                dma_addr_t mapping;
                u32 status, len;
                void *addr;

                entry = (entry + 1) % NUM_TX_DESC;

                txd = tp->TxDescArray + entry;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
                len = frag->size;
                addr = ((void *) page_address(frag->page)) + frag->page_offset;
#else
                len = skb_frag_size(frag);
                addr = skb_frag_address(frag);
#endif
                mapping = dma_map_single(tp_to_dev(tp), addr, len, DMA_TO_DEVICE);

                if (unlikely(dma_mapping_error(tp_to_dev(tp), mapping))) {
                        if (unlikely(net_ratelimit()))
                                netif_err(tp, drv, tp->dev,
                                          "Failed to map TX fragments DMA!\n");
                        goto err_out;
                }

                /* anti gcc 2.95.3 bugware (sic) */
                status = rtl8168_get_txd_opts1(opts[0], len, entry);
                if (cur_frag == (nr_frags - 1)) {
                        tp->tx_skb[entry].skb = skb;
                        status |= LastFrag;
                }

                txd->addr = cpu_to_le64(mapping);

                tp->tx_skb[entry].len = len;

                txd->opts2 = cpu_to_le32(opts[1]);
                wmb();
                txd->opts1 = cpu_to_le32(status);
        }

        return cur_frag;

err_out:
        rtl8168_tx_clear_range(tp, tp->cur_tx + 1, cur_frag);
        return -EIO;
}

static inline
__be16 get_protocol(struct sk_buff *skb)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
        return vlan_get_protocol(skb);
#else
        __be16 protocol;

        if (skb->protocol == htons(ETH_P_8021Q))
                protocol = vlan_eth_hdr(skb)->h_vlan_encapsulated_proto;
        else
                protocol = skb->protocol;

        return protocol;
#endif
}

static bool rtl8168_skb_pad(struct sk_buff *skb)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
        if (skb_padto(skb, ETH_ZLEN))
                return false;
        skb_put(skb, ETH_ZLEN - skb->len);
        return true;
#else
        return !eth_skb_pad(skb);
#endif
}

static inline bool
rtl8168_tx_csum(struct sk_buff *skb,
                struct net_device *dev,
                u32 *opts)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        u32 csum_cmd = 0;
        u8 sw_calc_csum = FALSE;

        if (skb->ip_summed == CHECKSUM_PARTIAL) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
                const struct iphdr *ip = skb->nh.iph;

                if (dev->features & NETIF_F_IP_CSUM) {
                        if (ip->protocol == IPPROTO_TCP)
                                csum_cmd = tp->tx_ip_csum_cmd | tp->tx_tcp_csum_cmd;
                        else if (ip->protocol == IPPROTO_UDP)
                                csum_cmd = tp->tx_ip_csum_cmd | tp->tx_udp_csum_cmd;
                        else if (ip->protocol == IPPROTO_IP)
                                csum_cmd = tp->tx_ip_csum_cmd;
                }
#else
                u8 ip_protocol = IPPROTO_RAW;

                switch (get_protocol(skb)) {
                case  __constant_htons(ETH_P_IP):
                        if (dev->features & NETIF_F_IP_CSUM) {
                                ip_protocol = ip_hdr(skb)->protocol;
                                csum_cmd = tp->tx_ip_csum_cmd;
                        }
                        break;
                case  __constant_htons(ETH_P_IPV6):
                        if (dev->features & NETIF_F_IPV6_CSUM) {
                                u32 transport_offset = (u32)skb_transport_offset(skb);
                                if (transport_offset > 0 && transport_offset <= TCPHO_MAX) {
                                        ip_protocol = ipv6_hdr(skb)->nexthdr;
                                        csum_cmd = tp->tx_ipv6_csum_cmd;
                                        csum_cmd |= transport_offset << TCPHO_SHIFT;
                                }
                        }
                        break;
                default:
                        if (unlikely(net_ratelimit()))
                                dprintk("checksum_partial proto=%x!\n", skb->protocol);
                        break;
                }

                if (ip_protocol == IPPROTO_TCP)
                        csum_cmd |= tp->tx_tcp_csum_cmd;
                else if (ip_protocol == IPPROTO_UDP)
                        csum_cmd |= tp->tx_udp_csum_cmd;
#endif
                if (csum_cmd == 0) {
                        sw_calc_csum = TRUE;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
                        WARN_ON(1); /* we need a WARN() */
#endif
                }
        }

        if (csum_cmd != 0) {
                if (tp->ShortPacketSwChecksum && skb->len < ETH_ZLEN) {
                        sw_calc_csum = TRUE;
                        if (!rtl8168_skb_pad(skb))
                                return false;
                } else {
                        if ((tp->mcfg == CFG_METHOD_1) || (tp->mcfg == CFG_METHOD_2) || (tp->mcfg == CFG_METHOD_3))
                                opts[0] |= csum_cmd;
                        else
                                opts[1] |= csum_cmd;
                }
        }

        if (tp->UseSwPaddingShortPkt && skb->len < ETH_ZLEN)
                if (!rtl8168_skb_pad(skb))
                        return false;

        if (sw_calc_csum) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
                skb_checksum_help(&skb, 0);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
                skb_checksum_help(skb, 0);
#else
                skb_checksum_help(skb);
#endif
        }

        return true;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
/* r8169_csum_workaround()
  * The hw limits the value the transport offset. When the offset is out of the
  * range, calculate the checksum by sw.
  */
static void r8168_csum_workaround(struct rtl8168_private *tp,
                                  struct sk_buff *skb)
{
        if (skb_shinfo(skb)->gso_size) {
                netdev_features_t features = tp->dev->features;
                struct sk_buff *segs, *nskb;

                features &= ~(NETIF_F_SG | NETIF_F_IPV6_CSUM | NETIF_F_TSO6);
                segs = skb_gso_segment(skb, features);
                if (IS_ERR(segs) || !segs)
                        goto drop;

                do {
                        nskb = segs;
                        segs = segs->next;
                        nskb->next = NULL;
                        rtl8168_start_xmit(nskb, tp->dev);
                } while (segs);

                dev_consume_skb_any(skb);
        } else if (skb->ip_summed == CHECKSUM_PARTIAL) {
                if (skb_checksum_help(skb) < 0)
                        goto drop;

                rtl8168_start_xmit(skb, tp->dev);
        } else {
                struct net_device_stats *stats;

drop:
                stats = &tp->dev->stats;
                stats->tx_dropped++;
                dev_kfree_skb_any(skb);
        }
}

/* msdn_giant_send_check()
 * According to the document of microsoft, the TCP Pseudo Header excludes the
 * packet length for IPv6 TCP large packets.
 */
static int msdn_giant_send_check(struct sk_buff *skb)
{
        const struct ipv6hdr *ipv6h;
        struct tcphdr *th;
        int ret;

        ret = skb_cow_head(skb, 0);
        if (ret)
                return ret;

        ipv6h = ipv6_hdr(skb);
        th = tcp_hdr(skb);

        th->check = 0;
        th->check = ~tcp_v6_check(0, &ipv6h->saddr, &ipv6h->daddr, 0);

        return ret;
}
#endif

static bool rtl8168_tx_slots_avail(struct rtl8168_private *tp,
                                   unsigned int nr_frags)
{
        unsigned int slots_avail = tp->dirty_tx + NUM_TX_DESC - tp->cur_tx;

        /* A skbuff with nr_frags needs nr_frags+1 entries in the tx queue */
        return slots_avail > nr_frags;
}

static netdev_tx_t
rtl8168_start_xmit(struct sk_buff *skb,
                   struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned int entry;
        struct TxDesc *txd;
        dma_addr_t mapping;
        u32 len;
        u32 opts[2];
        netdev_tx_t ret = NETDEV_TX_OK;
        unsigned long flags, large_send;
        int frags;

        spin_lock_irqsave(&tp->lock, flags);

        if (unlikely(!rtl8168_tx_slots_avail(tp, skb_shinfo(skb)->nr_frags))) {
                if (netif_msg_drv(tp)) {
                        printk(KERN_ERR
                               "%s: BUG! Tx Ring full when queue awake!\n",
                               dev->name);
                }
                goto err_stop;
        }

        entry = tp->cur_tx % NUM_TX_DESC;
        txd = tp->TxDescArray + entry;

        if (unlikely(le32_to_cpu(txd->opts1) & DescOwn)) {
                if (netif_msg_drv(tp)) {
                        printk(KERN_ERR
                               "%s: BUG! Tx Desc is own by hardware!\n",
                               dev->name);
                }
                goto err_stop;
        }

        opts[0] = DescOwn;
        opts[1] = rtl8168_tx_vlan_tag(tp, skb);

        large_send = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (dev->features & (NETIF_F_TSO | NETIF_F_TSO6)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
                u32 mss = skb_shinfo(skb)->tso_size;
#else
                u32 mss = skb_shinfo(skb)->gso_size;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)

                /* TCP Segmentation Offload (or TCP Large Send) */
                if (mss) {
                        if ((tp->mcfg == CFG_METHOD_1) ||
                            (tp->mcfg == CFG_METHOD_2) ||
                            (tp->mcfg == CFG_METHOD_3)) {
                                opts[0] |= LargeSend | (min(mss, MSS_MAX) << 16);
                                large_send = 1;
                        } else {
                                u32 transport_offset = (u32)skb_transport_offset(skb);
                                switch (get_protocol(skb)) {
                                case __constant_htons(ETH_P_IP):
                                        if (transport_offset <= GTTCPHO_MAX) {
                                                opts[0] |= GiantSendv4;
                                                opts[0] |= transport_offset << GTTCPHO_SHIFT;
                                                opts[1] |= min(mss, MSS_MAX) << 18;
                                                large_send = 1;
                                        }
                                        break;
                                case __constant_htons(ETH_P_IPV6):
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
                                        if (msdn_giant_send_check(skb)) {
                                                spin_unlock_irqrestore(&tp->lock, flags);
                                                r8168_csum_workaround(tp, skb);
                                                goto out;
                                        }
#endif
                                        if (transport_offset <= GTTCPHO_MAX) {
                                                opts[0] |= GiantSendv6;
                                                opts[0] |= transport_offset << GTTCPHO_SHIFT;
                                                opts[1] |= min(mss, MSS_MAX) << 18;
                                                large_send = 1;
                                        }
                                        break;
                                default:
                                        if (unlikely(net_ratelimit()))
                                                dprintk("tso proto=%x!\n", skb->protocol);
                                        break;
                                }
                        }

                        if (large_send == 0)
                                goto err_dma_0;
                }
        }
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

        if (large_send == 0) {
                if (unlikely(!rtl8168_tx_csum(skb, dev, opts)))
                        goto err_dma_0;
        }

        frags = rtl8168_xmit_frags(tp, skb, opts);
        if (unlikely(frags < 0))
                goto err_dma_0;
        if (frags) {
                len = skb_headlen(skb);
                opts[0] |= FirstFrag;
        } else {
                len = skb->len;

                tp->tx_skb[entry].skb = skb;

                opts[0] |= FirstFrag | LastFrag;
        }

        opts[0] = rtl8168_get_txd_opts1(opts[0], len, entry);
        mapping = dma_map_single(tp_to_dev(tp), skb->data, len, DMA_TO_DEVICE);
        if (unlikely(dma_mapping_error(tp_to_dev(tp), mapping))) {
                if (unlikely(net_ratelimit()))
                        netif_err(tp, drv, dev, "Failed to map TX DMA!\n");
                goto err_dma_1;
        }
        tp->tx_skb[entry].len = len;
        txd->addr = cpu_to_le64(mapping);
        txd->opts2 = cpu_to_le32(opts[1]);
        wmb();
        txd->opts1 = cpu_to_le32(opts[0]);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
        dev->trans_start = jiffies;
#else
        skb_tx_timestamp(skb);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)

        tp->cur_tx += frags + 1;

        wmb();

        RTL_W8(tp, TxPoll, NPQ);    /* set polling bit */

        if (!rtl8168_tx_slots_avail(tp, MAX_SKB_FRAGS)) {
                netif_stop_queue(dev);
                smp_rmb();
                if (rtl8168_tx_slots_avail(tp, MAX_SKB_FRAGS))
                        netif_wake_queue(dev);
        }

        spin_unlock_irqrestore(&tp->lock, flags);
out:
        return ret;
err_dma_1:
        tp->tx_skb[entry].skb = NULL;
        rtl8168_tx_clear_range(tp, tp->cur_tx + 1, frags);
err_dma_0:
        RTLDEV->stats.tx_dropped++;
        spin_unlock_irqrestore(&tp->lock, flags);
        dev_kfree_skb_any(skb);
        ret = NETDEV_TX_OK;
        goto out;
err_stop:
        netif_stop_queue(dev);
        ret = NETDEV_TX_BUSY;
        RTLDEV->stats.tx_dropped++;

        spin_unlock_irqrestore(&tp->lock, flags);
        goto out;
}

static void
rtl8168_tx_interrupt(struct net_device *dev,
                     struct rtl8168_private *tp)
{
        unsigned int dirty_tx, tx_left;

        assert(dev != NULL);
        assert(tp != NULL);

        dirty_tx = tp->dirty_tx;
        smp_rmb();
        tx_left = tp->cur_tx - dirty_tx;
        tp->dynamic_aspm_packet_count += tx_left;

        while (tx_left > 0) {
                unsigned int entry = dirty_tx % NUM_TX_DESC;
                struct ring_info *tx_skb = tp->tx_skb + entry;
                u32 len = tx_skb->len;
                u32 status;

                rmb();
                status = le32_to_cpu(tp->TxDescArray[entry].opts1);
                if (status & DescOwn)
                        break;

                RTLDEV->stats.tx_bytes += len;
                RTLDEV->stats.tx_packets++;

                rtl8168_unmap_tx_skb(tp->pci_dev,
                                     tx_skb,
                                     tp->TxDescArray + entry);

                if (tx_skb->skb!=NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
                        dev_consume_skb_any(tx_skb->skb);
#else
                        dev_kfree_skb_any(tx_skb->skb);
#endif
                        tx_skb->skb = NULL;
                }
                dirty_tx++;
                tx_left--;
        }

        tp->dynamic_aspm_packet_count -= tx_left;

        if (tp->dirty_tx != dirty_tx) {
                tp->dirty_tx = dirty_tx;
                smp_wmb();
                if (netif_queue_stopped(dev) &&
                    (rtl8168_tx_slots_avail(tp, MAX_SKB_FRAGS))) {
                        netif_wake_queue(dev);
                }
                smp_rmb();
                if (tp->cur_tx != dirty_tx)
                        RTL_W8(tp, TxPoll, NPQ);
        }
}

static inline int
rtl8168_fragmented_frame(u32 status)
{
        return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

static inline void
rtl8168_rx_csum(struct rtl8168_private *tp,
                struct sk_buff *skb,
                struct RxDesc *desc)
{
        u32 opts1 = le32_to_cpu(desc->opts1);
        u32 opts2 = le32_to_cpu(desc->opts2);

        if ((tp->mcfg == CFG_METHOD_1) ||
            (tp->mcfg == CFG_METHOD_2) ||
            (tp->mcfg == CFG_METHOD_3)) {
                u32 status = opts1 & RxProtoMask;

                /* rx csum offload for RTL8168B/8111B */
                if (((status == RxProtoTCP) && !(opts1 & (RxTCPF | RxIPF))) ||
                    ((status == RxProtoUDP) && !(opts1 & (RxUDPF | RxIPF))))
                        skb->ip_summed = CHECKSUM_UNNECESSARY;
                else
                        skb->ip_summed = CHECKSUM_NONE;
        } else {
                /* rx csum offload for RTL8168C/8111C and RTL8168CP/8111CP */
                if (((opts2 & RxV4F) && !(opts1 & RxIPF)) || (opts2 & RxV6F)) {
                        if (((opts1 & RxTCPT) && !(opts1 & RxTCPF)) ||
                            ((opts1 & RxUDPT) && !(opts1 & RxUDPF)))
                                skb->ip_summed = CHECKSUM_UNNECESSARY;
                        else
                                skb->ip_summed = CHECKSUM_NONE;
                } else
                        skb->ip_summed = CHECKSUM_NONE;
        }
}

static inline int
rtl8168_try_rx_copy(struct rtl8168_private *tp,
                    struct sk_buff **sk_buff,
                    int pkt_size,
                    struct RxDesc *desc,
                    int rx_buf_sz)
{
        int ret = -1;

        if (pkt_size < rx_copybreak) {
                struct sk_buff *skb;

                skb = RTL_ALLOC_SKB_INTR(tp, pkt_size + RTK_RX_ALIGN);
                if (skb) {
                        u8 *data;

                        data = sk_buff[0]->data;
                        skb_reserve(skb, RTK_RX_ALIGN);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,37)
                        prefetch(data - RTK_RX_ALIGN);
#endif
                        eth_copy_and_sum(skb, data, pkt_size, 0);
                        *sk_buff = skb;
                        rtl8168_mark_to_asic(desc, rx_buf_sz);
                        ret = 0;
                }
        }
        return ret;
}

static inline void
rtl8168_rx_skb(struct rtl8168_private *tp,
               struct sk_buff *skb)
{
#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
        netif_receive_skb(skb);
#else
        napi_gro_receive(&tp->napi, skb);
#endif
#else
        netif_rx(skb);
#endif
}

static int
rtl8168_rx_interrupt(struct net_device *dev,
                     struct rtl8168_private *tp,
                     napi_budget budget)
{
        unsigned int cur_rx, rx_left;
        unsigned int delta, count = 0;
        unsigned int entry;
        struct RxDesc *desc;
        u32 status;
        u32 rx_quota;

        assert(dev != NULL);
        assert(tp != NULL);

        if (tp->RxDescArray == NULL)
                goto rx_out;

        rx_quota = RTL_RX_QUOTA(budget);
        cur_rx = tp->cur_rx;
        entry = cur_rx % NUM_RX_DESC;
        desc = tp->RxDescArray + entry;
        rx_left = NUM_RX_DESC + tp->dirty_rx - cur_rx;
        rx_left = rtl8168_rx_quota(rx_left, (u32)rx_quota);

        for (; rx_left > 0; rx_left--) {
                rmb();
                status = le32_to_cpu(desc->opts1);
                if (status & DescOwn)
                        break;
                if (unlikely(status & RxRES)) {
                        if (netif_msg_rx_err(tp)) {
                                printk(KERN_INFO
                                       "%s: Rx ERROR. status = %08x\n",
                                       dev->name, status);
                        }

                        RTLDEV->stats.rx_errors++;

                        if (status & (RxRWT | RxRUNT))
                                RTLDEV->stats.rx_length_errors++;
                        if (status & RxCRC)
                                RTLDEV->stats.rx_crc_errors++;
                        if (dev->features & NETIF_F_RXALL)
                                goto process_pkt;

                        rtl8168_mark_to_asic(desc, tp->rx_buf_sz);
                } else {
                        struct sk_buff *skb;
                        int pkt_size;

process_pkt:
                        if (likely(!(dev->features & NETIF_F_RXFCS)))
                                pkt_size = (status & 0x00003fff) - 4;
                        else
                                pkt_size = status & 0x00003fff;

                        /*
                         * The driver does not support incoming fragmented
                         * frames. They are seen as a symptom of over-mtu
                         * sized frames.
                         */
                        if (unlikely(rtl8168_fragmented_frame(status))) {
                                RTLDEV->stats.rx_dropped++;
                                RTLDEV->stats.rx_length_errors++;
                                rtl8168_mark_to_asic(desc, tp->rx_buf_sz);
                                continue;
                        }

                        skb = tp->Rx_skbuff[entry];

                        dma_sync_single_for_cpu(tp_to_dev(tp),
                                                le64_to_cpu(desc->addr), tp->rx_buf_sz,
                                                DMA_FROM_DEVICE);

                        if (rtl8168_try_rx_copy(tp, &skb, pkt_size,
                                                desc, tp->rx_buf_sz)) {
                                tp->Rx_skbuff[entry] = NULL;
                                dma_unmap_single(tp_to_dev(tp), le64_to_cpu(desc->addr),
                                                 tp->rx_buf_sz, DMA_FROM_DEVICE);
                        } else {
                                dma_sync_single_for_device(tp_to_dev(tp), le64_to_cpu(desc->addr),
                                                           tp->rx_buf_sz, DMA_FROM_DEVICE);
                        }

                        if (tp->cp_cmd & RxChkSum)
                                rtl8168_rx_csum(tp, skb, desc);

                        skb->dev = dev;
                        skb_put(skb, pkt_size);
                        skb->protocol = eth_type_trans(skb, dev);

                        if (skb->pkt_type == PACKET_MULTICAST)
                                RTLDEV->stats.multicast++;

                        if (rtl8168_rx_vlan_skb(tp, desc, skb) < 0)
                                rtl8168_rx_skb(tp, skb);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
                        dev->last_rx = jiffies;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
                        RTLDEV->stats.rx_bytes += pkt_size;
                        RTLDEV->stats.rx_packets++;
                }

                cur_rx++;
                entry = cur_rx % NUM_RX_DESC;
                desc = tp->RxDescArray + entry;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,37)
                prefetch(desc);
#endif
        }

        count = cur_rx - tp->cur_rx;
        tp->cur_rx = cur_rx;

        delta = rtl8168_rx_fill(tp, dev, tp->dirty_rx, tp->cur_rx, 1);
        if (!delta && count && netif_msg_intr(tp))
                printk(KERN_INFO "%s: no Rx buffer allocated\n", dev->name);
        tp->dirty_rx += delta;

        tp->dynamic_aspm_packet_count += delta;

        /*
         * FIXME: until there is periodic timer to try and refill the ring,
         * a temporary shortage may definitely kill the Rx process.
         * - disable the asic to try and avoid an overflow and kick it again
         *   after refill ?
         * - how do others driver handle this condition (Uh oh...).
         */
        if ((tp->dirty_rx + NUM_RX_DESC == tp->cur_rx) && netif_msg_intr(tp))
                printk(KERN_EMERG "%s: Rx buffers exhausted\n", dev->name);

rx_out:
        return count;
}

/*
 *The interrupt handler does all of the Rx thread work and cleans up after
 *the Tx thread.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
#else
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance)
#endif
{
        struct net_device *dev = (struct net_device *) dev_instance;
        struct rtl8168_private *tp = netdev_priv(dev);
        int status;
        int handled = 0;

        do {
                status = RTL_R16(tp, IntrStatus);

                if (!(tp->features & RTL_FEATURE_MSI)) {
                        /* hotplug/major error/no more work/shared irq */
                        if ((status == 0xFFFF) || !status)
                                break;

                        if (!(status & (tp->intr_mask | tp->timer_intr_mask)))
                                break;
                }

                handled = 1;

                rtl8168_disable_hw_interrupt(tp);

                switch (tp->mcfg) {
                case CFG_METHOD_9:
                case CFG_METHOD_10:
                case CFG_METHOD_11:
                case CFG_METHOD_12:
                case CFG_METHOD_13:
                case CFG_METHOD_14:
                case CFG_METHOD_15:
                case CFG_METHOD_16:
                case CFG_METHOD_17:
                case CFG_METHOD_18:
                case CFG_METHOD_19:
                case CFG_METHOD_20:
                case CFG_METHOD_21:
                case CFG_METHOD_22:
                case CFG_METHOD_23:
                case CFG_METHOD_24:
                case CFG_METHOD_25:
                case CFG_METHOD_26:
                case CFG_METHOD_27:
                case CFG_METHOD_28:
                case CFG_METHOD_29:
                case CFG_METHOD_30:
                case CFG_METHOD_31:
                case CFG_METHOD_32:
                case CFG_METHOD_33:
                        /* RX_OVERFLOW RE-START mechanism now HW handles it automatically*/
                        RTL_W16(tp, IntrStatus, status&~RxFIFOOver);
                        break;
                default:
                        RTL_W16(tp, IntrStatus, status);
                        break;
                }

                //Work around for rx fifo overflow
                if (unlikely(status & RxFIFOOver)) {
                        if (tp->mcfg == CFG_METHOD_1) {
                                netif_stop_queue(dev);
                                udelay(300);
                                rtl8168_hw_reset(dev);
                                rtl8168_tx_clear(tp);
                                rtl8168_rx_clear(tp);
                                rtl8168_init_ring(dev);
                                rtl8168_hw_config(dev);
                                rtl8168_hw_start(dev);
                                netif_wake_queue(dev);
                        }
                }

#ifdef ENABLE_DASH_SUPPORT
                if ( tp->DASH ) {
                        if (HW_DASH_SUPPORT_TYPE_2(tp) || HW_DASH_SUPPORT_TYPE_3(tp)) {
                                u8 DashIntType2Status;

                                if (status & ISRIMR_DASH_INTR_CMAC_RESET)
                                        tp->CmacResetIntr = TRUE;

                                DashIntType2Status = RTL_CMAC_R8(tp, CMAC_IBISR0);
                                if (DashIntType2Status & ISRIMR_DASH_TYPE2_ROK) {
                                        tp->RcvFwDashOkEvt = TRUE;
                                }
                                if (DashIntType2Status & ISRIMR_DASH_TYPE2_TOK) {
                                        tp->SendFwHostOkEvt = TRUE;
                                }
                                if (DashIntType2Status & ISRIMR_DASH_TYPE2_RX_DISABLE_IDLE) {
                                        tp->DashFwDisableRx = TRUE;
                                }

                                RTL_CMAC_W8(tp, CMAC_IBISR0, DashIntType2Status);
                        } else {
                                if (status & ISRIMR_DP_REQSYS_OK) {
                                        tp->RcvFwReqSysOkEvt = TRUE;
                                }
                                if (status & ISRIMR_DP_DASH_OK) {
                                        tp->RcvFwDashOkEvt = TRUE;
                                }
                                if (status & ISRIMR_DP_HOST_OK) {
                                        tp->SendFwHostOkEvt = TRUE;
                                }
                        }
                }
#endif

#ifdef CONFIG_R8168_NAPI
                if (status & tp->intr_mask || tp->keep_intr_cnt-- > 0) {
                        if (status & tp->intr_mask)
                                tp->keep_intr_cnt = RTK_KEEP_INTERRUPT_COUNT;

                        if (likely(RTL_NETIF_RX_SCHEDULE_PREP(dev, &tp->napi)))
                                __RTL_NETIF_RX_SCHEDULE(dev, &tp->napi);
                        else if (netif_msg_intr(tp))
                                printk(KERN_INFO "%s: interrupt %04x in poll\n",
                                       dev->name, status);
                } else {
                        tp->keep_intr_cnt = RTK_KEEP_INTERRUPT_COUNT;
                        rtl8168_switch_to_hw_interrupt(tp);
                }
#else
                if (status & tp->intr_mask || tp->keep_intr_cnt-- > 0) {
                        u32 budget = ~(u32)0;

                        if (status & tp->intr_mask)
                                tp->keep_intr_cnt = RTK_KEEP_INTERRUPT_COUNT;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
                        rtl8168_rx_interrupt(dev, tp, &budget);
#else
                        rtl8168_rx_interrupt(dev, tp, budget);
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
                        rtl8168_tx_interrupt(dev, tp);

#ifdef ENABLE_DASH_SUPPORT
                        if ( tp->DASH ) {
                                struct net_device *dev = tp->dev;

                                HandleDashInterrupt(dev);
                        }
#endif

                        rtl8168_switch_to_timer_interrupt(tp);
                } else {
                        tp->keep_intr_cnt = RTK_KEEP_INTERRUPT_COUNT;
                        rtl8168_switch_to_hw_interrupt(tp);
                }
#endif

        } while (false);

        return IRQ_RETVAL(handled);
}

#ifdef CONFIG_R8168_NAPI
static int rtl8168_poll(napi_ptr napi, napi_budget budget)
{
        struct rtl8168_private *tp = RTL_GET_PRIV(napi, struct rtl8168_private);
        RTL_GET_NETDEV(tp)
        unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
        unsigned int work_done;
        unsigned long flags;

        work_done = rtl8168_rx_interrupt(dev, tp, budget);

        spin_lock_irqsave(&tp->lock, flags);
        rtl8168_tx_interrupt(dev, tp);
        spin_unlock_irqrestore(&tp->lock, flags);

        RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);

        if (work_done < work_to_do) {
#ifdef ENABLE_DASH_SUPPORT
                if ( tp->DASH ) {
                        struct net_device *dev = tp->dev;

                        spin_lock_irqsave(&tp->lock, flags);
                        HandleDashInterrupt(dev);
                        spin_unlock_irqrestore(&tp->lock, flags);
                }
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
                if (RTL_NETIF_RX_COMPLETE(dev, napi, work_done) == FALSE) return RTL_NAPI_RETURN_VALUE;
#else
                RTL_NETIF_RX_COMPLETE(dev, napi, work_done);
#endif
                /*
                 * 20040426: the barrier is not strictly required but the
                 * behavior of the irq handler could be less predictable
                 * without it. Btw, the lack of flush for the posted pci
                 * write is safe - FR
                 */
                smp_wmb();

                rtl8168_switch_to_timer_interrupt(tp);
        }

        return RTL_NAPI_RETURN_VALUE;
}
#endif//CONFIG_R8168_NAPI

static void rtl8168_sleep_rx_enable(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);

        if (tp->wol_enabled != WOL_ENABLED) return;

        if ((tp->mcfg == CFG_METHOD_1) || (tp->mcfg == CFG_METHOD_2)) {
                RTL_W8(tp, ChipCmd, CmdReset);
                rtl8168_rx_desc_offset0_init(tp, 0);
                RTL_W8(tp, ChipCmd, CmdRxEnb);
        } else if (tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15) {
                rtl8168_ephy_write(tp, 0x19, 0xFF64);
                RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
        }
}

static void rtl8168_down(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;

        rtl8168_delete_esd_timer(dev, &tp->esd_timer);

        rtl8168_delete_link_timer(dev, &tp->link_timer);

#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        RTL_NAPI_DISABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI

        netif_stop_queue(dev);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
        /* Give a racing hard_start_xmit a few cycles to complete. */
        synchronize_rcu();  /* FIXME: should this be synchronize_irq()? */
#endif

        spin_lock_irqsave(&tp->lock, flags);

        netif_carrier_off(dev);

        rtl8168_dsm(dev, DSM_IF_DOWN);

        rtl8168_hw_reset(dev);

        spin_unlock_irqrestore(&tp->lock, flags);

        synchronize_irq(dev->irq);

        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_tx_clear(tp);

        rtl8168_rx_clear(tp);

        spin_unlock_irqrestore(&tp->lock, flags);
}

static int rtl8168_close(struct net_device *dev)
{
        struct rtl8168_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;
        unsigned long flags;

        if (tp->TxDescArray!=NULL && tp->RxDescArray!=NULL) {
                rtl8168_cancel_schedule_work(dev);

                rtl8168_down(dev);

                pci_clear_master(tp->pci_dev);

                spin_lock_irqsave(&tp->lock, flags);

                rtl8168_hw_d3_para(dev);

                rtl8168_powerdown_pll(dev);

                rtl8168_sleep_rx_enable(dev);

                spin_unlock_irqrestore(&tp->lock, flags);

                free_irq(dev->irq, dev);

                dma_free_coherent(&pdev->dev, R8168_RX_RING_BYTES, tp->RxDescArray,
                                  tp->RxPhyAddr);
                dma_free_coherent(&pdev->dev, R8168_TX_RING_BYTES, tp->TxDescArray,
                                  tp->TxPhyAddr);
                tp->TxDescArray = NULL;
                tp->RxDescArray = NULL;
        } else {
                spin_lock_irqsave(&tp->lock, flags);

                rtl8168_hw_d3_para(dev);

                rtl8168_powerdown_pll(dev);

                spin_unlock_irqrestore(&tp->lock, flags);
        }

        return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
static void rtl8168_shutdown(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8168_private *tp = netdev_priv(dev);

        if (HW_DASH_SUPPORT_DASH(tp))
                rtl8168_driver_stop(tp);

        rtl8168_set_bios_setting(dev);
        if (s5_keep_curr_mac == 0 && tp->random_mac == 0)
                rtl8168_rar_set(tp, tp->org_mac_addr);

#ifdef ENABLE_FIBER_SUPPORT
        rtl8168_hw_fiber_nic_d3_para(dev);
#endif  //ENABLE_FIBER_SUPPORT

        if (s5wol == 0)
                tp->wol_enabled = WOL_DISABLED;

        rtl8168_close(dev);
        rtl8168_disable_msi(pdev, tp);

        if (system_state == SYSTEM_POWER_OFF) {
                pci_clear_master(tp->pci_dev);
                rtl8168_sleep_rx_enable(dev);
                pci_wake_from_d3(pdev, tp->wol_enabled);
                pci_set_power_state(pdev, PCI_D3hot);
        }
}
#endif

/**
 *  rtl8168_get_stats - Get rtl8168 read/write statistics
 *  @dev: The Ethernet Device to get statistics for
 *
 *  Get TX/RX statistics for rtl8168
 */
static struct
net_device_stats *rtl8168_get_stats(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
        struct rtl8168_private *tp = netdev_priv(dev);
#endif
        if (netif_running(dev)) {
//      spin_lock_irqsave(&tp->lock, flags);
//      spin_unlock_irqrestore(&tp->lock, flags);
        }

        return &RTLDEV->stats;
}

#ifdef CONFIG_PM

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int
rtl8168_suspend(struct pci_dev *pdev, u32 state)
#else
static int
rtl8168_suspend(struct pci_dev *pdev, pm_message_t state)
#endif
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8168_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        u32 pci_pm_state = pci_choose_state(pdev, state);
#endif
        unsigned long flags;

        if (!netif_running(dev))
                goto out;

        rtl8168_cancel_schedule_work(dev);

        rtl8168_delete_esd_timer(dev, &tp->esd_timer);

        rtl8168_delete_link_timer(dev, &tp->link_timer);

        netif_stop_queue(dev);

        netif_carrier_off(dev);

        netif_device_detach(dev);

        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_dsm(dev, DSM_NIC_GOTO_D3);

        rtl8168_hw_reset(dev);

        pci_clear_master(pdev);

        rtl8168_hw_d3_para(dev);

#ifdef ENABLE_FIBER_SUPPORT
        rtl8168_hw_fiber_nic_d3_para(dev);
#endif  //ENABLE_FIBER_SUPPORT

        rtl8168_powerdown_pll(dev);

        rtl8168_sleep_rx_enable(dev);

        spin_unlock_irqrestore(&tp->lock, flags);

out:
        if (HW_DASH_SUPPORT_DASH(tp)) {
                spin_lock_irqsave(&tp->lock, flags);
                rtl8168_driver_stop(tp);
                spin_unlock_irqrestore(&tp->lock, flags);
        }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        pci_save_state(pdev, &pci_pm_state);
#else
        pci_save_state(pdev);
#endif
        pci_enable_wake(pdev, pci_choose_state(pdev, state), tp->wol_enabled);
//  pci_set_power_state(pdev, pci_choose_state(pdev, state));

        return 0;
}

static int
rtl8168_resume(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8168_private *tp = netdev_priv(dev);
        unsigned long flags;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        u32 pci_pm_state = PCI_D0;
#endif

        pci_set_power_state(pdev, PCI_D0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        pci_restore_state(pdev, &pci_pm_state);
#else
        pci_restore_state(pdev);
#endif
        pci_enable_wake(pdev, PCI_D0, 0);

        spin_lock_irqsave(&tp->lock, flags);

        /* restore last modified mac address */
        rtl8168_rar_set(tp, dev->dev_addr);

        spin_unlock_irqrestore(&tp->lock, flags);

        if (!netif_running(dev)) {
                if (HW_DASH_SUPPORT_DASH(tp)) {
                        spin_lock_irqsave(&tp->lock, flags);
                        rtl8168_driver_start(tp);
                        spin_unlock_irqrestore(&tp->lock, flags);
                }
                goto out;
        }

        pci_set_master(pdev);

        spin_lock_irqsave(&tp->lock, flags);

        rtl8168_exit_oob(dev);

        rtl8168_dsm(dev, DSM_NIC_RESUME_D3);

        rtl8168_hw_init(dev);

        rtl8168_powerup_pll(dev);

        rtl8168_hw_ephy_config(dev);

        rtl8168_hw_phy_config(dev);

        rtl8168_hw_config(dev);

        spin_unlock_irqrestore(&tp->lock, flags);

        rtl8168_schedule_work(dev, rtl8168_reset_task);

        netif_device_attach(dev);

        mod_timer(&tp->esd_timer, jiffies + RTL8168_ESD_TIMEOUT);
        mod_timer(&tp->link_timer, jiffies + RTL8168_LINK_TIMEOUT);
out:
        return 0;
}

#endif /* CONFIG_PM */

static struct pci_driver rtl8168_pci_driver = {
        .name       = MODULENAME,
        .id_table   = rtl8168_pci_tbl,
        .probe      = rtl8168_init_one,
        .remove     = __devexit_p(rtl8168_remove_one),
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
        .shutdown   = rtl8168_shutdown,
#endif
#ifdef CONFIG_PM
        .suspend    = rtl8168_suspend,
        .resume     = rtl8168_resume,
#endif
};

static int __init
rtl8168_init_module(void)
{
#ifdef ENABLE_R8168_PROCFS
        rtl8168_proc_module_init();
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        return pci_register_driver(&rtl8168_pci_driver);
#else
        return pci_module_init(&rtl8168_pci_driver);
#endif
}

static void __exit
rtl8168_cleanup_module(void)
{
        pci_unregister_driver(&rtl8168_pci_driver);
#ifdef ENABLE_R8168_PROCFS
        if (rtl8168_proc) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
                remove_proc_subtree(MODULENAME, init_net.proc_net);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
                remove_proc_entry(MODULENAME, init_net.proc_net);
#else
                remove_proc_entry(MODULENAME, proc_net);
#endif  //LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#endif  //LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
                rtl8168_proc = NULL;
        }
#endif
}

module_init(rtl8168_init_module);
module_exit(rtl8168_cleanup_module);
