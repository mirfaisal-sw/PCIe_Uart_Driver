
enum mac_version {
	/* support for ancient RTL_GIGA_MAC_VER_01 has been removed */
        RTL_GIGA_MAC_VER_02,

	RTL_GIGA_MAC_VER_34 = 27,

	RTL_GIGA_MAC_NONE
};

struct rtl8111_private {

	void __iomem *mmio_addr;        /* memory map physical address */
 	struct pci_dev *pdev;
 	struct net_device *ndev;
	struct phy_device *phydev;

	enum mac_version mac_version;
	
	u32 *msgbuf_addr;
	dma_addr_t msgbuf_dma_addr;
	unsigned int msg_buf_size;
	u16 cp_cmd;
	struct mutex register_mutex;
	u8 dma_using_dac;
};


/* write/read MMIO register */
#define RTL_W8(tp, reg, val8)   writeb((val8), tp->mmio_addr + (reg))
#define RTL_W16(tp, reg, val16) writew((val16), tp->mmio_addr + (reg))
#define RTL_W32(tp, reg, val32) writel((val32), tp->mmio_addr + (reg))
#define RTL_R8(tp, reg)         readb(tp->mmio_addr + (reg))
#define RTL_R16(tp, reg)        readw(tp->mmio_addr + (reg))
#define RTL_R32(tp, reg)        readl(tp->mmio_addr + (reg))

enum rtl_registers {
        MAC0            = 0,    /* Ethernet hardware address. */
        MAC4            = 4,
        MAR0            = 8,    /* Multicast filter. */
        CounterAddrLow          = 0x10,
        CounterAddrHigh         = 0x14,
        TxDescStartAddrLow      = 0x20,
        TxDescStartAddrHigh     = 0x24,
        TxHDescStartAddrLow     = 0x28,
        TxHDescStartAddrHigh    = 0x2c,
        FLASH           = 0x30,
        ERSR            = 0x36,
        ChipCmd         = 0x37,
        TxPoll          = 0x38,
        IntrMask        = 0x3c,
        IntrStatus      = 0x3e,

        TxConfig        = 0x40,
#define TXCFG_AUTO_FIFO                 (1 << 7)        /* 8111e-vl */
#define TXCFG_EMPTY                     (1 << 11)       /* 8111e-vl */

        RxConfig        = 0x44,
#define RX128_INT_EN                    (1 << 15)       /* 8111c and later */
#define RX_MULTI_EN                     (1 << 14)       /* 8111c only */
#define RXCFG_FIFO_SHIFT                13
                                        /* No threshold before first PCI xfer */
#define RX_FIFO_THRESH                  (7 << RXCFG_FIFO_SHIFT)
#define RX_EARLY_OFF                    (1 << 11)
#define RXCFG_DMA_SHIFT                 8
                                        /* Unlimited maximum PCI burst. */
#define RX_DMA_BURST                    (7 << RXCFG_DMA_SHIFT)

        Cfg9346         = 0x50,
        Config0         = 0x51,
        Config1         = 0x52,
        Config2         = 0x53,
#define PME_SIGNAL                      (1 << 5)        /* 8168c and later */

        Config3         = 0x54,
        Config4         = 0x55,
        Config5         = 0x56,
        PHYAR           = 0x60,
        PHYstatus       = 0x6c,
	RxMaxSize       = 0xda,
        CPlusCmd        = 0xe0,
        IntrMitigate    = 0xe2,

#define RTL_COALESCE_TX_USECS   GENMASK(15, 12)
#define RTL_COALESCE_TX_FRAMES  GENMASK(11, 8)
#define RTL_COALESCE_RX_USECS   GENMASK(7, 4)
#define RTL_COALESCE_RX_FRAMES  GENMASK(3, 0)

#define RTL_COALESCE_T_MAX      0x0fU
#define RTL_COALESCE_FRAME_MAX  (RTL_COALESCE_T_MAX * 4)

        RxDescAddrLow   = 0xe4,
        RxDescAddrHigh  = 0xe8,
        EarlyTxThres    = 0xec, /* 8169. Unit of 32 bytes. */

#define NoEarlyTx       0x3f    /* Max value : no early transmit. */

        MaxTxPacketSize = 0xec, /* 8101/8168. Unit of 128 bytes. */

#define TxPacketMax     (8064 >> 7)
#define EarlySize       0x27

        FuncEvent       = 0xf0,
        FuncEventMask   = 0xf4,
        FuncPresetState = 0xf8,
        IBCR0           = 0xf8,
        IBCR2           = 0xf9,
        IBIMR0          = 0xfa,
        IBISR0          = 0xfb,
        FuncForceEvent  = 0xfc,
};

enum rtl_register_content {
        /* InterruptStatusBits */
        SYSErr          = 0x8000,
        PCSTimeout      = 0x4000,
        SWInt           = 0x0100,
        TxDescUnavail   = 0x0080,
        RxFIFOOver      = 0x0040,
        LinkChg         = 0x0020,
        RxOverflow      = 0x0010,
        TxErr           = 0x0008,
        TxOK            = 0x0004,
        RxErr           = 0x0002,
        RxOK            = 0x0001,

        /* RxStatusDesc */
        RxRWT   = (1 << 22), 
        RxRES   = (1 << 21), 
        RxRUNT  = (1 << 20), 
        RxCRC   = (1 << 19), 

        /* ChipCmdBits */
        StopReq         = 0x80,
        CmdReset        = 0x10,
        CmdRxEnb        = 0x08,
        CmdTxEnb        = 0x04,
        RxBufEmpty      = 0x01,

        /* TXPoll register p.5 */
        HPQ             = 0x80,         /* Poll cmd on the high prio queue */
        NPQ             = 0x40,         /* Poll cmd on the low prio queue */
        FSWInt          = 0x01,         /* Forced software interrupt */

        /* Cfg9346Bits */
        Cfg9346_Lock    = 0x00,
        Cfg9346_Unlock  = 0xc0,

        /* rx_mode_bits */
        AcceptErr       = 0x20,
        AcceptRunt      = 0x10,
#define RX_CONFIG_ACCEPT_ERR_MASK       0x30
        AcceptBroadcast = 0x08,
        AcceptMulticast = 0x04,
        AcceptMyPhys    = 0x02,
        AcceptAllPhys   = 0x01,
#define RX_CONFIG_ACCEPT_OK_MASK        0x0f
#define RX_CONFIG_ACCEPT_MASK           0x3f

/* TxConfigBits */
        TxInterFrameGapShift = 24,
        TxDMAShift = 8, /* DMA burst value (0-7) is shift this many bits */

        /* Config1 register p.24 */
        LEDS1           = (1 << 7),
        LEDS0           = (1 << 6),
        Speed_down      = (1 << 4),
        MEMMAP          = (1 << 3),
        IOMAP           = (1 << 2),
        VPD             = (1 << 1),
        PMEnable        = (1 << 0),     /* Power Management Enable */

        /* Config2 register p. 25 */
        ClkReqEn        = (1 << 7),     /* Clock Request Enable */
        MSIEnable       = (1 << 5),     /* 8169 only. Reserved in the 8168. */
        PCI_Clock_66MHz = 0x01,
        PCI_Clock_33MHz = 0x00,

        /* Config3 register p.25 */
        MagicPacket     = (1 << 5),     /* Wake up when receives a Magic Packet */
        LinkUp          = (1 << 4),     /* Wake up when the cable connection is re-established */
        Jumbo_En0       = (1 << 2),     /* 8168 only. Reserved in the 8168b */
        Rdy_to_L23      = (1 << 1),     /* L23 Enable */
        Beacon_en       = (1 << 0),     /* 8168 only. Reserved in the 8168b */

        /* Config4 register */
        Jumbo_En1       = (1 << 1),     /* 8168 only. Reserved in the 8168b */

        /* Config5 register p.27 */
        BWF             = (1 << 6),     /* Accept Broadcast wakeup frame */
        MWF             = (1 << 5),     /* Accept Multicast wakeup frame */
        UWF             = (1 << 4),     /* Accept Unicast wakeup frame */
        Spi_en          = (1 << 3),
        LanWake         = (1 << 1),     /* LanWake enable/disable */
        PMEStatus       = (1 << 0),     /* PME status can be reset by PCI RST# */
        ASPM_en         = (1 << 0),     /* ASPM enable */

	/* CPlusCmd p.31 */
        EnableBist      = (1 << 15),    // 8168 8101
        Mac_dbgo_oe     = (1 << 14),    // 8168 8101
        EnAnaPLL        = (1 << 14),    // 8169
        Normal_mode     = (1 << 13),    // unused
        Force_half_dup  = (1 << 12),    // 8168 8101
        Force_rxflow_en = (1 << 11),    // 8168 8101
        Force_txflow_en = (1 << 10),    // 8168 8101
        Cxpl_dbg_sel    = (1 << 9),     // 8168 8101
        ASF             = (1 << 8),     // 8168 8101
        PktCntrDisable  = (1 << 7),     // 8168 8101
        Mac_dbgo_sel    = 0x001c,       // 8168
        RxVlan          = (1 << 6),
        RxChkSum        = (1 << 5),
        PCIDAC          = (1 << 4),
        PCIMulRW        = (1 << 3),
#define CPCMD_MASK      (RxVlan | RxChkSum)

        /* rtl8169_PHYstatus */
        TBI_Enable      = 0x80,
        TxFlowCtrl      = 0x40,
        RxFlowCtrl      = 0x20,
        _1000bpsF       = 0x10,
        _100bps         = 0x08,
        _10bps          = 0x04,
        LinkStatus      = 0x02,
        FullDup         = 0x01,

        /* ResetCounterCommand */
        CounterReset    = 0x1,

        /* DumpCounterCommand */
        CounterDump     = 0x8,

        /* magic enable v2 */
        MagicPacket_v2  = (1 << 16),    /* Wake up when receives a Magic Packet */
};
