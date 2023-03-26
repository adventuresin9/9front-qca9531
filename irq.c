/*
 *  Interrupt Handling for the ar9531
 */
#include	"u.h"
#include	"../port/lib.h"
#include	"mem.h"
#include	"dat.h"
#include	"fns.h"
#include	"ureg.h"
#include	"io.h"
#include	"../port/error.h"



/* map the irq number to the interrupt controller */
static const int irq2inc[32] = {
	/* cpu based interrupts */
	[IRQsw1]	=	-1,
	[IRQsw2]	=	-1,
	[IRQwlan]	=	-1,
	[IRQusb]	=	-1,
	[IRQeth0]	=	-1,
	[IRQeth1]	=	-1,
	[IRQmisc]	=	-1,
	[IRQtimer]	=	-1,

	/* irqs on the SoC interrupt controller */
	[IRQtimer0]	=	MISC_TIMER0,
	[IRQerror]	=	MISC_ERROR,
	[IRQgpio]	=	MISC_GPIO,
	[IRQuart]	=	MISC_UART,
	[IRQwdog]	=	MISC_WDOG,
	[IRQpcnt]	=	MISC_PCNT,
	[IRQmbox]	=	MISC_MBOX,
	[IRQtimer1]	=	MISC_TIMER1,
	[IRQtimer2]	=	MISC_TIMER2,
	[IRQtimer3]	=	MISC_TIMER3,
	[IRQswitch]	=	MISC_SWITCH,
};


static const int inc2irq[32] = {
	[MISC_TIMER0]	=	IRQtimer0,
	[MISC_ERROR]	=	IRQerror,
	[MISC_GPIO]		=	IRQgpio,
	[MISC_UART]		=	IRQuart,
	[MISC_WDOG]		=	IRQwdog,
	[MISC_PCNT]		=	IRQpcnt,
	[MISC_MBOX]		=	IRQmbox,
	[MISC_TIMER1]	=	IRQtimer1,
	[MISC_TIMER2]	=	IRQtimer2,
	[MISC_TIMER3]	=	IRQtimer3,
	[MISC_SWITCH]	=	IRQswitch,
};




typedef struct Handler Handler;

struct Handler {
	Handler *next;
	void 	(*f)(Ureg*, void *);
	void	*arg;
	int		irq;
};

static Lock intrlock;
static Handler handlers[IRQmax+1];


void incintr(Ureg*, void*);



static u32int
incread(int offset)
{
	return *IO(u32int, (RST_BASE + offset));
}


static void
incwrite(int offset, u32int val)
{
	*IO(u32int, (RST_BASE + offset)) = val;
}

/*
 * called by main(), clears all the irq's
 * sets SoC interrupt controller to relay
 * IRQs through CPU interrupt 6
 */

void
intrinit(void)
{
	incread(RST_MISC_STAT);
	incwrite(RST_MISC_MASK, 0x0);
	incwrite(RST_MISC_STAT, 0x0);

	intrenable(IRQmisc, incintr, (void *)0, 0, "irqmisc");
}


/* called by drivers to setup irq's */
void
intrenable(int irq, void (*f)(Ureg*, void *), void *arg, int priority, char *name)
{
	Handler *hp;
	u32int r;


	if(irq > IRQmax || irq < 0)
		panic("intrenable: %s gave bad irq number of %d", name, irq);

	/* debugging */
	if(irq == 0 || irq == 1)
		iprint("software irq enabled?");

	hp = &handlers[irq];
	ilock(&intrlock);

	if(hp->f != nil) {
		for(; hp->next != nil; hp = hp->next)
			;
		if((hp->next = xalloc(sizeof *hp)) == nil)
			panic("intrenable: out of memory");
		hp = hp->next;
		hp->next = nil;
	}

	hp->f = f;
	hp->arg = arg;
	hp->irq = irq;

	iunlock(&intrlock);

	if(irq > IRQtimer) {
		r = incread(RST_MISC_MASK);
		r |= (1 << irq2inc[irq]);
		incwrite(RST_MISC_MASK, r);
	} else {
		intron(INTR0 << irq);
	}

}



void
intrdisable(int irq, void (*)(Ureg*, void *), void*, int, char *name)
{
	u32int r;

	if(irq > IRQmax || irq < 0)
		panic("intrdisable: %s gave bad irq number of %d", name, irq);

	if(irq > IRQtimer) {
		r = incread(RST_MISC_MASK);
		r |= (0 << irq2inc[irq]);
		incwrite(RST_MISC_MASK, r);
	} else {
		introff(INTR0 << irq);
	}
}


/* called by trap to handle requests, returns true if a clock interrupt */
int
intr(Ureg* ur)
{	
	ulong cause, mask;
	int clockintr;
	Handler *hh, *hp;

	m->intr++;
	clockintr = 0;
	/*
	 * ignore interrupts that we have disabled, even if their cause bits
	 * are set.
	 */
	cause = ur->cause & ur->status & INTMASK;
	cause &= ~(INTR1|INTR0);		/* ignore sw interrupts */

	if (cause == 0)
		iprint("spurious interrupt\n");

	if(cause & INTR7){
		clock(ur);
		cause &= ~INTR7;
		clockintr = 1;
	}

//	iprint("INTR %luX\n", cause);

	hh = &handlers[2];
	for(mask = INTR2; cause != 0 && mask < INTR7; mask <<= 1){
		if(cause & mask){
			for(hp = hh; hp != nil; hp = hp->next){
				if(hp->f != nil){
					hp->f(ur, hp->arg);
					cause &= ~mask;
				}
			}
		}
		hh++;
	}
	if(cause != 0)
		iprint("unhandled interrupts %lux\n", cause);

	

	/* preemptive scheduling */
	if(up != nil && !clockintr)
		preempted();
	/* if it was a clockintr, sched will be called at end of trap() */
	return clockintr;
}


/* off to handle requests for the SoC interrupt controller */
/*
 * The interrupt controller on the qca9831 SoC is mapped
 * to cpu interrupt 6.  The mask register doesn't seem 
 * to actually mask what comes up on the status register, 
 * so it has to be read and masked here.  Otherwise the 
 * standard counter trips an "unhandled" interrupt (0x701)
 * on a regular basis.  Status then has to me manually cleared.
 */

void
incintr(Ureg *ureg, void *arg)
{
	u32int p;
	u32int mstat;
	u32int mmask;
	u32int pending;
	u32int mask;
	Handler *hh, *hp;


	p = (uintptr)arg;
	mstat = incread(RST_MISC_STAT);
	mmask = incread(RST_MISC_MASK);
	incwrite(RST_MISC_STAT, (mstat & ~mmask));

	pending = mstat & mmask;

	hh = &handlers[8];
	for(mask = 1 ; pending != 0 && mask < 0x80000000; mask <<= 1) {
		if(pending & mask) {
			for(hp = hh; hp != nil; hp = hp->next) {
				if(hp->f != nil) {
					hp->f(ureg, hp->arg);
					pending &= ~mask;
				}
			}
		}
		hh++;
	}

	if(pending != 0){
		iprint("unhandled inc interrupts %uX\n", pending);
		delay(2000);
	}
}


void
intrclear(int irq)
{
	incwrite(RST_MISC_STAT, 1 << irq2inc[irq]);
}


void
intrshutdown(void)
{
	introff(INTMASK);
	incwrite(RST_MISC_MASK, 0x0);
	coherence();

}

/*
 * left over debugging stuff
 */

ulong
incmask(void)
{
	return incread(RST_MISC_MASK);
}

ulong
incstat(void)
{
	return incread(RST_MISC_STAT);
}
