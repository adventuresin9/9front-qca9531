/*
 *  various things to IO with
 */

#define	IO(t,x)		((t*)(KSEG1|((ulong)x)))

#define	UARTBASE	0x18020000

#define	RST_BASE	0x18060000

#define PCI_BASE	0x180C0000  //likely wlan

#define MAC0BASE	0x19000000
#define	MAC1BASE	0x1A000000






/*
 *  interrupt levels
 */

enum {
	IRQsw1		=	0,	//INTR0
	IRQsw2,
	IRQwlan,			//INTR2
	IRQusb,
	IRQeth0,
	IRQeth1,
	IRQmisc,
	IRQtimer,			//INTR7
	IRQtimer0,			//misc_intr0
	IRQerror,
	IRQgpio,
	IRQuart,
	IRQwdog,
	IRQpcnt,
	IRQmisc6,
	IRQmbox,
	IRQtimer1,
	IRQtimer2,
	IRQtimer3,
	IRQmisc11,
	IRQswitch,
	IRQmax,
};


/*
 * Interrupts on secondary controller
 */

#define	MISC_TIMER0		0
#define	MISC_ERROR		1
#define	MISC_GPIO		2
#define	MISC_UART		3
#define	MISC_WDOG		4
#define	MISC_PCNT		5
#define	MISC_MBOX		7
#define	MISC_TIMER1		8
#define	MISC_TIMER2		9
#define	MISC_TIMER3		10
#define	MISC_SWITCH		12


/*
 *	Reset Control, and secondary irq
 */

#define	RST_TIMER0		0x00
#define	RST_TIMER0_RLD	0x04
#define	RST_WDOG_CTL	0x08
#define	RST_WDOG		0x0C
#define	RST_MISC_STAT	0x10
#define RST_MISC_MASK	0x14
#define	RST_GLO_STAT	0x18
#define	RST_RESET		0x1C
#define	RST_REV_ID		0x90
