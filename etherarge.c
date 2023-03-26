/*
 *	Ethernet driver for the QCA9531
 *	borrows from teh old rb kernel
 *	also similar to Tulip drivers
 *	
 * 	minimal switch setup included
 */


#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/error.h"
#include "../port/netif.h"
#include "../port/etherif.h"
#include "../port/ethermii.h"


/*
 *	2 Ethernet devices, base addresses at
 *	0x19000000
 *	0x1A000000
 *	the following are offsets to 
 *	particular registers
 */

#define	ETH_MCFG1		0x00
#define	ETH_MCFG2		0x04
#define	ETH_IFG			0x08
#define	ETH_HDPLX		0x0C
#define	ETH_FRMLEN		0x10

#define	ETH_MIICFG		0x20
#define	ETH_MIICMD		0x24
#define	ETH_MIIADDR		0x28
#define	ETH_MIICTL		0x2C
#define	ETH_MIISTS		0x30
#define	ETH_MIIINDIC	0x34

#define	ETH_IFCTL		0x38
#define	ETH_IFSTS		0x3C
#define	ETH_ADDR1		0x40
#define	ETH_ADDR2		0x44

#define	ETH_CFG0		0x48
#define	ETH_CFG1		0x4C
#define	ETH_CFG2		0x50
#define	ETH_CFG3		0x54
#define	ETH_CFG4		0x58
#define	ETH_CFG5		0x5C

#define	ETH_TXCTL		0x180
#define	ETH_TXDESC		0x184
#define	ETH_TXSTS		0x188
#define	ETH_RXCTL		0x18C
#define	ETH_RXDESC		0x190
#define	ETH_RXSTS		0x194

#define	ETH_INTMASK		0x198
#define	ETH_INTSTAT		0x19C


/* massive pile of Tulip setting */
enum {
	Cfg1softrst		= 1 << 31,
	Cfg1simulrst		= 1 << 30,
	Cfg1macrxblkrst		= 1 << 19,
	Cfg1mactxblkrst		= 1 << 18,
	Cfg1rxfuncrst		= 1 << 17,
	Cfg1txfuncrst		= 1 << 16,
	Cfg1loopback		= 1 <<  8,
	Cfg1rxflowctl		= 1 <<  5,
	Cfg1txflowctl		= 1 <<  4,
	Cfg1syncrx		= 1 <<  3,
	Cfg1rxen		= 1 <<  2,
	Cfg1synctx		= 1 <<  1,
	Cfg1txen		= 1 <<  0,

	Cfg2preamblelenmask	= 0xf,
	Cfg2preamblelenshift	= 12,
	Cfg2ifmode1000		= 2 << 8,
	Cfg2ifmode10_100	= 1 << 8,
	Cfg2ifmodeshift		= 8,
	Cfg2ifmodemask		= 3,
	Cfg2hugeframe		= 1 << 5,
	Cfg2lenfield		= 1 << 4,
	Cfg2enpadcrc		= 1 << 2,
	Cfg2encrc		= 1 << 1,
	Cfg2fdx			= 1 << 0,

	Miicfgrst		= 1 << 31,
	Miicfgscanautoinc	= 1 <<  5,
	Miicfgpreamblesup	= 1 <<  4,
	Miicfgclkselmask	= 0x7,
	Miicfgclkdiv4		= 0,
	Miicfgclkdiv6		= 2,
	Miicfgclkdiv8		= 3,
	Miicfgclkdiv10		= 4,
	Miicfgclkdiv14		= 5,
	Miicfgclkdiv20		= 6,
	Miicfgclkdiv28		= 7,

	Miicmdscancycle		= 1 << 1,
	Miicmdread		= 1,
	Miicmdwrite		= 0,

	Miiphyaddrshift		= 8,
	Miiphyaddrmask		= 0x1f,  //ff
	Miiregmask		= 0x1f,

	Miictlmask		= 0xffff,

	Miistsmask		= 0xffff,

	Miiindicinvalid		= 1 << 2,
	Miiindicscanning	= 1 << 1,
	Miiindicbusy		= 1 << 0,

	Ifctlspeed		= 1 << 16,

	Fifocfg0txfabric	= 1 << 4,
	Fifocfg0txsys		= 1 << 3,
	Fifocfg0rxfabric	= 1 << 2,
	Fifocfg0rxsys		= 1 << 1,
	Fifocfg0watermark	= 1 << 0,
	Fifocfg0all		= MASK(5),
	Fifocfg0enshift		= 8,

	/*
	 * these flags applicable both to filter mask and to filter match.
	 * `Ff' is for `fifo filter'.
	 */
	Ffunicast		= 1 << 17,
	Fftruncframe		= 1 << 16,
	Ffvlantag		= 1 << 15,
	Ffunsupopcode		= 1 << 14,
	Ffpauseframe		= 1 << 13,
	Ffctlframe		= 1 << 12,
	Fflongevent		= 1 << 11,
	Ffdribblenibble		= 1 << 10,
	Ffbcast			= 1 <<  9,
	Ffmcast			= 1 <<  8,
	Ffok			= 1 <<  7,
	Ffoorange		= 1 <<  6,
	Fflenmsmtch		= 1 <<  5,
	Ffcrcerr		= 1 <<  4,
	Ffcodeerr		= 1 <<  3,
	Fffalsecarrier		= 1 <<  2,
	Ffrxdvevent		= 1 <<  1,
	Ffdropevent		= 1 <<  0,
	/*
	 * exclude unicast and truncated frames from matching.
	 */
	Ffmatchdflt = Ffvlantag | Ffunsupopcode | Ffpauseframe | Ffctlframe |
		Fflongevent | Ffdribblenibble | Ffbcast | Ffmcast | Ffok |
		Ffoorange | Fflenmsmtch | Ffcrcerr | Ffcodeerr |
		Fffalsecarrier | Ffrxdvevent | Ffdropevent,

	/* `Frm' is for `fifo receive mask'. */
	Frmbytemode		= 1 << 19,
	Frmnoshortframe		= 1 << 18,
	Frmbit17		= 1 << 17,
	Frmbit16		= 1 << 16,
	Frmtruncframe		= 1 << 15,
	Frmlongevent		= 1 << 14,
	Frmvlantag		= 1 << 13,
	Frmunsupopcode		= 1 << 12,
	Frmpauseframe		= 1 << 11,
	Frmctlframe		= 1 << 10,
	Frmdribblenibble	= 1 <<  9,
	Frmbcast		= 1 <<  8,
	Frmmcast		= 1 <<  7,
	Frmok			= 1 <<  6,
	Frmoorange		= 1 <<  5,
	Frmlenmsmtch		= 1 <<  4,
	Frmcodeerr		= 1 <<  3,
	Frmfalsecarrier		= 1 <<  2,
	Frmrxdvevent		= 1 <<  1,
	Frmdropevent		= 1 <<  0,
	/*
	 *  len. mismatch, unsupp. opcode and short frame bits excluded
	 */
	Ffmaskdflt = Frmnoshortframe | Frmbit17 | Frmbit16 | Frmtruncframe |
		Frmlongevent | Frmvlantag | Frmpauseframe | Frmctlframe |
		Frmdribblenibble | Frmbcast | Frmmcast | Frmok | Frmoorange |
		Frmcodeerr | Frmfalsecarrier | Frmrxdvevent | Frmdropevent,

	Dmatxctlen	= 1 << 0,

	/* dma tx status */
	Txpcountmask	= 0xff,
	Txpcountshift	= 16,
	Txbuserr	= 1 << 3,
	Txunderrun	= 1 << 1,
	Txpktsent	= 1 << 0,

	Dmarxctlen	= 1 << 0,

	/* dma rx status */
	Rxpcountmask	= 0xff,
	Rxpcountshift	= 16,
	Rxbuserr	= 1 << 3,
	Rxovflo		= 1 << 2,
	Rxpktrcvd	= 1 << 0,

	/* dmaintr & dmaintrsts bits */
	Dmarxbuserr	= 1 << 7,
	Dmarxovflo	= 1 << 6,
	Dmarxpktrcvd	= 1 << 4,
	Dmatxbuserr	= 1 << 3,
	Dmatxunderrun	= 1 << 1,
	Dmatxpktsent	= 1 << 0,
	/* we don't really need most tx interrupts */
	Dmaall		= Dmarxbuserr | Dmarxovflo | Dmarxpktrcvd | Dmatxbuserr,
};


/* Debugging options */
enum{
	Miidebug	=	0,
	Ethdebug	=	0,
	Attchbug	=	0,
};


enum {
	Ntd	= 64,
	Nrd	= 256,
	Nrb	= 1024,

	Bufalign= 4,
	Rbsz	= ETHERMAXTU + 4,	/* 4 for CRC */
};


enum {
	Descempty	= 1 << 31,
	Descmore	= 1 << 24,
	Descszmask	= MASK(12),
};
#define DMASIZE(len)	((len) & Descszmask)


typedef struct Ctlr Ctlr;
typedef struct Desc Desc;


/* hw descriptors of buffer rings (rx and tx), need to be uncached */
struct Desc {
	u32int	addr;		/* of packet buffer */
	u32int	ctl;
	Desc	*next;
	u32int	_pad;
};


struct Ctlr{
	u32int	base;
	int		attached;
	QLock;
	Lock;				/* for intr */
	Ether	*edev;		/* point back */

	struct {
		Block	*b[Nrd];
		Desc	*d;
		Rendez;
		Lock;
		uint	h;		/* head */
		uint	t;		/* tail */
		uint	nrdfree;		/* rd's awaiting pkts (sort of) */
	}	rx[1];

	struct {
		Block	*b[Ntd];
		Desc	*d;
		Rendez;
		Lock;
		uint	h;
		uint	t;
		int		ntq;
	}	tx[1];

	Mii	*mii;

	QLock	statlock;
	int		irq;
	int		genum;
	int		pktstoread;		/* flag */
	int		pktstosend;		/* flag */
	int		rxstat;
	int		rxintr;
	int		rxdmaerr;
	int		txstat;
	int		txintr;
	int		txdmaerr;
	int		nointr;
	int		badrx;
};

static Ctlr	*ctlrarge[2];


static u32int
rdarge(Ctlr *ctlr, int offset)
{
	return *IO(u32int, (ctlr->base + offset));
}


static void
wrarge(Ctlr *ctlr, int offset, u32int val)
{
	*IO(u32int, (ctlr->base + offset)) = val;
}


static int
rdargemii(Mii *mii, int pa, int ra)
{
	int		timeout;
	int		val = 0;
	int		busy = 0;
	Ctlr	*c;

	c = mii->ctlr;

	if(Miidebug)
		iprint("rdargemii%d, phy_addr; %d phy_reg: %d\n", c->genum, pa, ra);


	/* fill in the addresses */
	wrarge(c, ETH_MIIADDR, ((pa & Miiphyaddrmask) << Miiphyaddrshift) | 
		(ra & Miiregmask));

	/* set the read mode */
	wrarge(c, ETH_MIICMD, Miicmdread);

	delay(1);

	for(timeout = 0; timeout < 2000; timeout++){
		if(!(busy = rdarge(c, ETH_MIIINDIC)))
			break;
		microdelay(100);
	}

	if(busy)	/* read failed? */
		return -1;

	/* fetch read value */
	val = rdarge(c, ETH_MIISTS);
	val &= Miistsmask;

	/* set read mode off */
	wrarge(c, ETH_MIICMD, Miicmdwrite);

	return val;
}


static int
wrargemii(Mii *mii, int pa, int ra, int val)
{
	int		timeout;
	int		busy;
	Ctlr	*c;

	c = mii->ctlr;

	if(Miidebug)
		iprint("wrargemii, phy_addr; %d phy_reg: %d val: 0x%04X\n", pa, ra, val);


	/* fill in the addresses */
	wrarge(c, ETH_MIIADDR, ((pa & Miiphyaddrmask) << Miiphyaddrshift) | 
		(ra & Miiregmask));

	/* send the val */
	wrarge(c, ETH_MIICTL, (val & Miictlmask));

	delay(1);

	for(timeout = 0; timeout < 2000; timeout++){
		if(!(busy = rdarge(c, ETH_MIIINDIC)))
			break;
		microdelay(100);
	}

	if(busy)	/* read failed? */
		return -1;

	return 0;
}


static int
argeinitmii(Ctlr *ctlr)
{
	MiiPhy *phy;
	Ether	*edev = ctlr->edev;
	int		i, buf;
	

	if((ctlr->mii = malloc(sizeof(Mii))) == nil)
		return -1;

	ctlr->mii->ctlr	= ctlr;
	ctlr->mii->mir	= rdargemii;
	ctlr->mii->miw	= wrargemii;

	if(mii(ctlr->mii, ~0) == 0 || (phy = ctlr->mii->curphy) == nil){
		print("#l%d: init mii failure\n", edev->ctlrno);
		free(ctlr->mii);
		ctlr->mii = nil;
		return -1;
	}

	print("#l%d: phy%d id %.8ux oui %x\n", 
		edev->ctlrno, phy->phyno, phy->id, phy->oui);

	miireset(ctlr->mii);

	miiane(ctlr->mii, ~0, ~0, ~0);

	return 0;
}


static void
argegetmac(Ether *edev, Ctlr *ctlr)
{
	u32int	msb, lsb;

	msb = rdarge(ctlr, ETH_ADDR1);
	lsb = rdarge(ctlr, ETH_ADDR2);

//	edev->ea[0] = lsb>>8;
//	edev->ea[1] = lsb>>0;
//	edev->ea[2]	= msb>>24;
//	edev->ea[3] = msb>>16;
//	edev->ea[4] = msb>>8;
//	edev->ea[5] = msb>>0;

/* ugly hack, my atheros board has no mac addresses */
	edev->ea[0] = 0xBA;
	edev->ea[1] = 0xDA;
	edev->ea[2]	= 0xDD;
	edev->ea[3] = 0x00;
	edev->ea[4] = 0x00;
	edev->ea[5] = ctlr->genum;


	if(Attchbug){
		iprint("ether getmac: %04lX %08lX\n", (lsb & 0xFFFF), msb);
		delay(10);
	}
}


static void
rxnewbuf(Ctlr *ctlr, int i)
{
	Block *b;
	Desc *rd;

	if (ctlr->rx->b[i] != nil)
		return;
	b = iallocb(Rbsz);
	if(b == nil)
		panic("#l%d: can't allocate receive buffer",
			ctlr->edev->ctlrno);
	ctlr->rx->b[i] = b;

	dcflush(b->rp, Rbsz);		/* writeback & invalidate */

	rd = &ctlr->rx->d[i];
	rd->addr = PADDR(b->rp);
	rd->ctl = Descempty | DMASIZE(Rbsz);
	ctlr->rx->nrdfree++;
}

static void
rxreclaim(Ctlr *ctlr)
{
	uint rdt;

	rdt = ctlr->rx->t;
	while (rdt != ctlr->rx->h && !(ctlr->rx->d[rdt].ctl & Descempty)){
		rxnewbuf(ctlr, rdt);
		rdt = NEXT(rdt, Nrd);
	}
	ctlr->rx->t = rdt;
}


static int
pktstoread(void* v)
{
	Ctlr *ctlr = v;

	return ctlr->pktstoread || !(ctlr->rx->d[ctlr->rx->h].ctl & Descempty);
}


static void
rproc(void* arg)
{
	uint rdh, sz, buf;
	Block *bp;
	Ctlr *ctlr;
	Desc *rd;
	Ether *edev;

	edev = arg;
	ctlr = edev->ctlr;

	if(Ethdebug)
		iprint("rproc started\n");

	for(;;){
		/* wait for next interrupt */
		ilock(ctlr->rx);
		buf = rdarge(ctlr, ETH_INTMASK);
		buf |= Dmarxpktrcvd;
		wrarge(ctlr, ETH_INTMASK, buf);
		iunlock(ctlr->rx);

		sleep(ctlr->rx, pktstoread, ctlr);
		ctlr->pktstoread = 0;

		rxreclaim(ctlr);
		rdh = ctlr->rx->h;
		for (rd = &ctlr->rx->d[rdh]; !(rd->ctl & Descempty);
		     rd = &ctlr->rx->d[rdh]){
			bp = ctlr->rx->b[rdh];
			assert(bp != nil);
			ctlr->rx->b[rdh] = nil;

			/* omit final 4 bytes (crc), pass pkt upstream */
	//		sz = DMASIZE(rd->ctl) - 4;
			sz = DMASIZE(rd->ctl);	/* pass on crc */
			assert(sz > 0 && sz <= Rbsz);
			bp->wp = bp->rp + sz;
			dcflush(bp->rp, BLEN(bp));	/* move block to ram */
			etheriq(edev, bp);

			if(Ethdebug)
				iprint("rproc: (%ud) len=%ud\n", rdh, sz);

			wrarge(ctlr, ETH_RXSTS, Rxpktrcvd);

			ctlr->rx->nrdfree--;
			rdh = NEXT(rdh, Nrd);
			if(ctlr->rx->nrdfree < Nrd/2) {
				/* rxreclaim reads ctlr->rdh */
				ctlr->rx->h = rdh;
				rxreclaim(edev->ctlr);
			}
		}
		ctlr->rx->h = rdh;
	}
}


static void
txreclaim(Ctlr *ctlr)
{
	uint tdh;
	Block *bp;

	tdh = ctlr->tx->h;
	while (tdh != ctlr->tx->t && (ctlr->tx->d[tdh].ctl & Descempty)){
		wrarge(ctlr, ETH_TXSTS, Txpktsent);

		bp = ctlr->tx->b[tdh];
		ctlr->tx->b[tdh] = nil;
		if (bp)
			freeb(bp);

		ctlr->tx->d[tdh].addr = 0;
		ctlr->tx->ntq--;
		tdh = NEXT(tdh, Ntd);
	}
	ctlr->tx->h = tdh;
}


static int
pktstosend(void* v)
{
	Ether *edev = v;
	Ctlr *ctlr = edev->ctlr;

	return ctlr->pktstosend || ctlr->tx->ntq > 0 || qlen(edev->oq) > 0;
}


static void
tproc(void* arg)
{
	uint tdt, added, buf;
	Block *bp;
	Ctlr *ctlr;
	Desc *td;
	Ether *edev;

	edev = arg;
	ctlr = edev->ctlr;

	if(Ethdebug)
		iprint("tproc started\n");

	for(;;){
		/* wait for next free buffer and output queue block */
		sleep(ctlr->tx, pktstosend, edev);
		ctlr->pktstosend = 0;

		txreclaim(ctlr);

		/* copy as much of my output q as possible into output ring */
		added = 0;
		tdt = ctlr->tx->t;
		while(ctlr->tx->ntq < Ntd - 1){
			td = &ctlr->tx->d[tdt];
			if (!(td->ctl & Descempty))
				break;
			bp = qget(edev->oq);
			if(bp == nil)
				break;

			/* make sure the whole packet is in ram */
			dcflush(bp->rp, BLEN(bp));

			/*
			 * Give ownership of the descriptor to the chip,
			 * increment the software ring descriptor pointer.
			 */
			ctlr->tx->b[tdt] = bp;
			td->addr = PADDR(bp->rp);
			td->ctl = DMASIZE(BLEN(bp));
			coherence();

			added++;
			ctlr->tx->ntq++;
			tdt = NEXT(tdt, Ntd);
		}
		ctlr->tx->t = tdt;
		/*
		 * Underrun turns off TX.  Clear underrun indication.
		 * If there's anything left in the ring, reactivate the tx.
		 */
		if (rdarge(ctlr, ETH_INTSTAT) & Dmatxunderrun)
			wrarge(ctlr, ETH_TXSTS, Txunderrun);
		if(1 || added)
			wrarge(ctlr, ETH_TXCTL, Dmatxctlen);	/* kick xmiter */
		ilock(ctlr->tx);
		if(ctlr->tx->ntq >= Ntd/2){			/* tx ring half-full? */
			buf = rdarge(ctlr, ETH_INTMASK);
			buf |= Dmatxpktsent;
			wrarge(ctlr, ETH_INTMASK, buf);
		}else if (ctlr->tx->ntq > 0){
			buf = rdarge(ctlr, ETH_INTMASK);
			buf |= Dmatxunderrun;
			wrarge(ctlr, ETH_INTMASK, buf);
		}
		iunlock(ctlr->tx);
		txreclaim(ctlr);
	}
}


static void
argeintr(Ureg*, void *arg)
{
	int sts, buf;
	Ctlr *ctlr;
	Ether *ether;

	ether = arg;
	ctlr = ether->ctlr;

	ilock(ctlr);
	sts = rdarge(ctlr, ETH_INTSTAT);
	if (sts & Dmarxpktrcvd) {
		buf = rdarge(ctlr, ETH_INTMASK);
		buf &= ~Dmarxpktrcvd;
		wrarge(ctlr, ETH_INTMASK, buf);
		ctlr->pktstoread = 1;
		wakeup(ctlr->rx);
		ctlr->rxintr++;
		sts &= ~Dmarxpktrcvd;
	}
	if (sts & (Dmatxpktsent | Dmatxunderrun)) {
		buf = rdarge(ctlr, ETH_INTMASK);
		buf &= ~(Dmatxpktsent | Dmatxunderrun);
		wrarge(ctlr, ETH_INTMASK, buf);
		ctlr->pktstosend = 1;
		wakeup(ctlr->tx);
		ctlr->txintr++;
		sts &= ~(Dmatxpktsent | Dmatxunderrun);
	}
	iunlock(ctlr);
	if ((sts & 0xFFFF))
		iprint("argeintr #l%d: sts %#ux\n", ether->ctlrno, sts);
}


static void
linkdescs(Desc *base, int ndesc)
{
	int i;

	for(i = 0; i < ndesc - 1; i++)
		base[i].next = (Desc *)PADDR(&base[i+1]);
	base[ndesc - 1].next = (Desc *)PADDR(&base[0]);
}


static void
argeattach(Ether *edev)
{
	Ctlr	*ctlr;
	Desc	*d;
	u32int	buf;
	int		i;
	char 	name[KNAMELEN];

	ctlr = edev->ctlr;

	if(Attchbug){
		iprint("ether attach called %d\n", ctlr->genum);
		delay(10);
	}

	if(ctlr->attached)
		return;

	qlock(ctlr);

	wrarge(ctlr, ETH_MCFG1, Cfg1syncrx | Cfg1rxen | Cfg1synctx | Cfg1txen);
	buf = rdarge(ctlr, ETH_MCFG2);
	buf |= Cfg2enpadcrc | Cfg2lenfield | Cfg2encrc | Cfg2fdx;
	wrarge(ctlr, ETH_MCFG2, buf);
	wrarge(ctlr, ETH_FRMLEN, Rbsz);

	if(/*edev->ctlrno >*/ 0){
		wrarge(ctlr, ETH_MIICFG, Miicfgrst);
		delay(100);
		wrarge(ctlr, ETH_MIICFG, Miicfgclkdiv28);
		delay(100);
	}
		wrarge(ctlr, ETH_MIICFG, Miicfgrst);
		delay(100);
		wrarge(ctlr, ETH_MIICFG, Miicfgclkdiv28);
		delay(100);

	/* undocumented magic */
	wrarge(ctlr, ETH_CFG0, Fifocfg0all << Fifocfg0enshift);
	wrarge(ctlr, ETH_CFG1, 0x0fff0000);
	wrarge(ctlr, ETH_CFG2, 0x00001fff);

	wrarge(ctlr, ETH_CFG4, Ffmatchdflt);
	wrarge(ctlr, ETH_CFG5, Ffmaskdflt);

	if(argeinitmii(ctlr) < 0)
		iprint("mii failed\n");

	if(Attchbug)
		iprint("done nic setup\n");

	/* Allocate Rx/Tx ring KSEG1, is uncached memmory */
	ctlr->tx->d = (Desc *)KSEG1ADDR(xspanalloc(sizeof(Desc) * Ntd, CACHELINESZ, 0));
	ctlr->rx->d = (Desc *)KSEG1ADDR(xspanalloc(sizeof(Desc) * Nrd, CACHELINESZ, 0));

	if(ctlr->tx->d == nil || ctlr->rx->d == nil)
		error(Enomem);

	/* link descriptors */
	linkdescs(ctlr->rx->d, Nrd);
	linkdescs(ctlr->tx->d, Ntd);

	/* Allocate Rx blocks, initialize Rx ring. */
	for(i = 0; i < Nrd; i++){
		rxnewbuf(ctlr, i);
	}

	ctlr->rx->t = ctlr->rx->h = 0;

	/* Initialize Tx ring */
	for(i = 0; i < Ntd; i++){
		ctlr->tx->b[i] = nil;
		ctlr->tx->d[i].ctl = Descempty;
	}

	ctlr->tx->h = ctlr->tx->t = 0;

	if(Attchbug)
		iprint("done ring alloc\n");

	/* give descriptors to device */
	wrarge(ctlr, ETH_TXDESC, PADDR(ctlr->tx->d));
	wrarge(ctlr, ETH_RXDESC, PADDR(ctlr->rx->d));
	coherence();
	wrarge(ctlr, ETH_RXCTL, Dmarxctlen);

	wrarge(ctlr, ETH_INTMASK, Dmaall);

	edev->link = 1;
	ctlr->attached = 1;
	qunlock(ctlr);
//	poperror();

	snprint(name, KNAMELEN, "#l%drproc", edev->ctlrno);
	kproc(name, rproc, edev);

	snprint(name, KNAMELEN, "#l%dtproc", edev->ctlrno);
	kproc(name, tproc, edev);

	if(Attchbug)
		iprint("ether attach done\n");
}



static void
argereset(Ctlr *ctlr)
{

	/* clear intr mask */
	wrarge(ctlr, ETH_INTMASK, 0x0);

	/* clear RX and TX CTL to stop */
	wrarge(ctlr, ETH_RXCTL, 0x0);
	wrarge(ctlr, ETH_TXCTL, 0x0);

	/*
	 * give tx & rx time to stop, otherwise clearing desc registers
	 * too early will cause random memory corruption.
	 */
	delay(1);

	/* MAC cfg soft reset */
	wrarge(ctlr, ETH_MCFG1, Cfg1softrst);

	/* clear descriptors */
	wrarge(ctlr, ETH_RXDESC, 0x0);
	wrarge(ctlr, ETH_TXDESC, 0x0);

	/*
	 * clear all interrupts
	 * need to ping it till the couter is empty
	 */
	while (rdarge(ctlr, ETH_RXSTS) & Rxpktrcvd)
		wrarge(ctlr, ETH_RXSTS, Rxpktrcvd);
	while (rdarge(ctlr, ETH_TXSTS) & Txpktsent)
		wrarge(ctlr, ETH_TXSTS, Txpktsent);

	/* and errors */
	wrarge(ctlr, ETH_RXSTS, Rxbuserr | Rxovflo);
	wrarge(ctlr, ETH_TXSTS, Txbuserr | Txunderrun);

}


static Ctlr*
argesetup(Ether *edev)
{
	int i;
	Ctlr *ctlr;

	if(ctlrarge[0] != nil && ctlrarge[1] != nil)
		return nil;

	if(ctlrarge[0] == nil)
		i = 0;
	else
		i = 1;

	ctlrarge[i] = malloc(sizeof(Ctlr));
	if(ctlrarge[i] == nil){
		iprint("arge%d malloc failed\n", i);
		return nil;
	}

	edev->ctlr = ctlr = ctlrarge[i];

	switch(i){
	case 0:
		ctlr->base		=	MAC0BASE;
		ctlr->irq		=	IRQeth0;
		ctlr->genum		=	0;
		break;
	case 1:
		ctlr->base		=	MAC1BASE;
		ctlr->irq		=	IRQeth1;
		ctlr->genum		=	1;
		break;
	default:
		panic("argesetup > 1?");
	}

	ctlr->edev = edev;

	argegetmac(edev, ctlr);

	argereset(ctlr);

	return ctlr;
}


static void
argeprom(void*, int)
{
//stub
}

static void
argemulti(void*, uchar*, int)
{
//stub
}

static void
argeshutdown(Ether*)
{
//stub
}


/* kick the transmitter to drain the output ring */
static void
argetransmit(Ether* ether)
{
	Ctlr *ctlr;

	ctlr = ether->ctlr;
	ilock(ctlr);
	ctlr->pktstosend = 1;
	wakeup(ctlr->tx);
	iunlock(ctlr);
}


static int
argepnp(Ether *ether)
{
	Ctlr *ctlr;

	if(Attchbug)
		iprint("ether pnp called\n");



	ctlr = argesetup(ether);

	if(ctlr == nil)
		return -1;

	ether->ctlr = ctlr;
	ether->arg = ether;
	ether->mbps = 1000;
	ether->irq = ctlr->irq;
	ether->port = (uintptr)ctlr->base;

	ether->attach = argeattach;
	ether->transmit = argetransmit;
//	ether->ifstat = argeifstat;
//	ether->ctl = argectl;
	ether->promiscuous = argeprom;
	ether->shutdown = argeshutdown;
	ether->multicast = argemulti;

	intrenable(ether->irq, argeintr, ether, 0, ether->name);

	if(Attchbug)
		iprint("ether pnp done\n");

	return 0;
}


void
etherargelink(void)
{
	addethercard("arge", argepnp);
}

