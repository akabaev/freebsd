/*
 * Copyright (C) 2015 Alexander Kabaev
 * Copyright (C) 2010 Andrew Turner
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 * A driver for the DM9000 MAC
 *
 * TODO:
 *  Get the interrupt working
 *  Port to non-S3C2440 systems
 *  Test with 8 and 32 bit busses
 *  Test on a big endian machine
 *  Implement the rest of dme_detach
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/ethernet.h>
#include <net/bpf.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/dme/if_dmereg.h>
#include <dev/dme/if_dmevar.h>

#include <dev/fdt/fdt_regulator.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/gpio/gpiobusvar.h>

#include "miibus_if.h"

struct dme_softc {
	struct ifnet		*dme_ifp;
	device_t		dme_dev;
	device_t		dme_miibus;
	bus_space_handle_t	dme_handle;
	bus_space_tag_t		dme_tag;
	int			dme_rev;
	int			dme_bits;
	struct resource		*dme_res;
	struct resource		*dme_irq;
	void			*dme_intrhand;
	struct mtx		dme_mtx;
	struct callout		dme_tick_ch;
	struct gpiobus_pin	*gpio_rset;
	uint32_t		dme_ticks;
	uint8_t			dme_macaddr[ETHER_ADDR_LEN];
};

#define DME_CHIP_DM9000		0x00
#define DME_CHIP_DM9000A	0x19
#define DME_CHIP_DM9000B	0x1a
#define	DME_CHIP_DM9000_ID	0x90000A46

#define DME_INT_PHY		1

static int dme_probe(device_t);
static int dme_attach(device_t);
static int dme_detach(device_t);

static void dme_intr(void *arg);
static void dme_init_locked(struct dme_softc *);

static int dme_miibus_writereg(device_t dev, int phy, int reg, int data);
static int dme_miibus_readreg(device_t dev, int phy, int reg);

/* The bit on the address bus attached to the CMD pin */
#define BASE_ADDR	0x000
#define CMD_ADDR	BASE_ADDR
#define	DATA_BIT	1
#define	DATA_ADDR	0x002

static uint8_t
dme_read_reg(struct dme_softc *sc, uint8_t reg)
{

	/* Send the register to read from */
	bus_space_write_1(sc->dme_tag, sc->dme_handle, CMD_ADDR, reg);
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	/* Get the value of the register */
	return bus_space_read_1(sc->dme_tag, sc->dme_handle, DATA_ADDR);
}

static void
dme_write_reg(struct dme_softc *sc, uint8_t reg, uint8_t value)
{

	/* Send the register to write to */
	bus_space_write_1(sc->dme_tag, sc->dme_handle, CMD_ADDR, reg);
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_WRITE);

	/* Write the value to the register */
	bus_space_write_1(sc->dme_tag, sc->dme_handle, DATA_ADDR, value);
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

static void
dme_reset(struct dme_softc *sc)
{
	u_int ncr;

	/* Send a soft reset #1 */
	dme_write_reg(sc, DME_NCR, NCR_RST | NCR_LBK_MAC);
	DELAY(100); /* Wait for the MAC to reset */
	ncr = dme_read_reg(sc, DME_NCR);
	if (ncr & NCR_RST)
		device_printf(sc->dme_dev, "device did not complete first reset\n");

	/* Send a soft reset #2 per Application Notes v1.22 */
	dme_write_reg(sc, DME_NCR, 0);
	dme_write_reg(sc, DME_NCR, NCR_RST | NCR_LBK_MAC);
	DELAY(100); /* Wait for the MAC to reset */
	ncr = dme_read_reg(sc, DME_NCR);
	if (ncr & NCR_RST)
		device_printf(sc->dme_dev, "device did not complete second reset\n");
}

/*
 * Parse string MAC address into usable form
 */
static int
dme_parse_macaddr(const char *str, uint8_t *mac)
{
	int count, i;
	unsigned int amac[ETHER_ADDR_LEN];	/* Aligned version */

	count = sscanf(str, "%x%*c%x%*c%x%*c%x%*c%x%*c%x",
	    &amac[0], &amac[1], &amac[2],
	    &amac[3], &amac[4], &amac[5]);
	if (count < ETHER_ADDR_LEN) {
		memset(mac, 0, ETHER_ADDR_LEN);
		return (1);
	}

	/* Copy aligned to result */
	for (i = 0; i < ETHER_ADDR_LEN; i ++)
		mac[i] = (amac[i] & 0xff);

	return (0);
}

/*
 * Try to determine our own MAC address
 */
static void
dme_get_macaddr(struct dme_softc *sc)
{
	char devid_str[32];
	char *var;
	int i;

	/* Cannot use resource_string_value with static hints mode */
	snprintf(devid_str, 32, "hint.%s.%d.macaddr",
	    device_get_name(sc->dme_dev),
	    device_get_unit(sc->dme_dev));

	/* Try resource hints */
	if ((var = kern_getenv(devid_str)) != NULL) {
		if (!dme_parse_macaddr(var, sc->dme_macaddr)) {
			device_printf(sc->dme_dev, "MAC address: %s (hints)\n", var);
			return;
		}
	}

	/*
	 * Try to read MAC address from the device, in case U-Boot has
	 * pre-programmed one for us.
	 */
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		sc->dme_macaddr[i] = dme_read_reg(sc, DME_PAR(i));

	device_printf(sc->dme_dev, "MAC address %6D (existing)\n",
	    sc->dme_macaddr, ":");
}

static void
dme_config(struct dme_softc *sc)
{
	int i;

	/* Mask all interrupts and reset receive pointer */
	dme_write_reg(sc, DME_IMR, IMR_PAR);

	/* Disable GPIO0 to enable the internal PHY */
	dme_write_reg(sc, DME_GPCR, 1);
	dme_write_reg(sc, DME_GPR, 0);

#if 0
	/*
	 * Supposedly requires special initialization for DSP PHYs
	 * used by DM9000B. Maybe belongs in dedicated PHY driver?
	 */
	if (sc->dme_rev == DME_CHIP_DM9000B) {
		dme_miibus_writereg(sc->dme_dev, DME_INT_PHY, MII_BMCR,
		    BMCR_RESET);
		dme_miibus_writereg(sc->dme_dev, DME_INT_PHY, MII_DME_DSPCR,
		    DSPCR_INIT);
		/* Wait 100ms for it to complete. */
		for (i = 0; i < 100; i++) {
			int reg;

			reg = dme_miibus_readreg(sc->dme_dev, DME_INT_PHY, MII_BMCR);
			if ((reg & BMCR_RESET) == 0)
				break;
			DELAY(1000);
		}
	}
#endif

	/* Select the internal PHY and normal loopback */
	dme_write_reg(sc, DME_NCR, NCR_LBK_NORMAL);
	/* Clear any TX requests */
	dme_write_reg(sc, DME_TCR, 0);
	/* Setup backpressure thresholds to 4k and 600us */
	dme_write_reg(sc, DME_BPTR, BPTR_BPHW(3) | BPTR_JPT(0x0f));
	/* Setup flow control */
	dme_write_reg(sc, DME_FCTR, FCTR_HWOT(0x3) | FCTR_LWOT(0x08));
	/* Enable flow control */
	dme_write_reg(sc, DME_FCR, 0xff);
	/* Clear special modes */
	dme_write_reg(sc, DME_SMCR, 0);
	/* Clear TX status */
	dme_write_reg(sc, DME_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);
	/* Clear interrrupts */
	dme_write_reg(sc, DME_ISR, 0xff);
	/* Set multicast address filter */
	for (i = 0; i < 8; i++)
		dme_write_reg(sc, DME_MAR(i), 0xff);
	/* Set the MAC address */
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		dme_write_reg(sc, DME_PAR(i), sc->dme_macaddr[i]);
	/* Enable the RX buffer */
	dme_write_reg(sc, DME_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);

	/* Enable interrupts we care about */
	dme_write_reg(sc, DME_IMR, IMR_PAR | IMR_PRI/* | IMR_LNKCHGI*/);
}


static void
dme_start_locked(struct ifnet *ifp)
{
	struct dme_softc *sc;
	struct mbuf *m, *mp;
	int len, total_len;

	sc = ifp->if_softc;

	DME_ASSERT_LOCKED(sc);

	if (ifp->if_drv_flags & IFF_DRV_OACTIVE)
		return;

	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;

		/* Wait for the device to be free */
		while (dme_read_reg(sc, DME_TCR) & TCR_TXREQ)
			DELAY(1);

		/* Write the data to the network */
		bus_space_write_1(sc->dme_tag, sc->dme_handle, CMD_ADDR,
		    DME_MWCMD);
		bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
		    BUS_SPACE_BARRIER_WRITE);

		/*
		 * TODO: Fix the case where an mbuf is
		 * not a multiple of the write size.
		 */

		total_len = 0;
		for (mp = m; mp != NULL; mp = mp->m_next) {
			len = mp->m_len;

			/* Ignore empty parts */
			if (len == 0)
				continue;

			total_len += len;

			/*
			 * Note: this only works where mbuf buffers are
			 * a multiple of the IO size.  Otherwise it will
			 * write out padding bytes between each mbuf buffer
			 * and .. well, fail.
			 */
			switch (sc->dme_bits) {
			case 8:
				bus_space_write_multi_1(sc->dme_tag, sc->dme_handle,
				    DATA_ADDR, mtod(mp, uint8_t *), len);
				break;
			case 16:
				bus_space_write_multi_2(sc->dme_tag, sc->dme_handle,
				    DATA_ADDR, mtod(mp, uint16_t *), (len + 1) / 2);
				break;
			case 32:
				bus_space_write_multi_4(sc->dme_tag, sc->dme_handle,
				    DATA_ADDR, mtod(mp, uint32_t *), (len + 3) / 4);
				break;
			}
		}

		/* Send the data length */
		dme_write_reg(sc, DME_TXPLL, total_len & 0xFF);
		dme_write_reg(sc, DME_TXPLH, (total_len >> 8) & 0xFF);

		/* Send the packet */
		dme_write_reg(sc, DME_TCR, TCR_TXREQ);

		BPF_MTAP(ifp, m);

		m_freem(m);
	}
}

static void
dme_start(struct ifnet *ifp)
{
	struct dme_softc *sc;

	sc = ifp->if_softc;
	DME_LOCK(sc);
	dme_start_locked(ifp);
	DME_UNLOCK(sc);
}

static void
dme_stop(struct dme_softc *sc)
{
	struct ifnet *ifp;

	DME_ASSERT_LOCKED(sc);
	/* Disable receiver */
	dme_write_reg(sc, DME_RCR, 0x00);
	/* Mask interrupts */
	dme_write_reg(sc, DME_IMR, 0x00);
	/* Stop poll */
	callout_stop(&sc->dme_tick_ch);

	ifp = sc->dme_ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
}

static int
dme_rxeof(struct dme_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	int len, i;

	DME_ASSERT_LOCKED(sc);

 	ifp = sc->dme_ifp;

	/* Read the first byte to check it correct */
	(void)dme_read_reg(sc, DME_MRCMDX);
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_READ);
	i = bus_space_read_1(sc->dme_tag, sc->dme_handle, DATA_ADDR);
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_READ);
	switch(bus_space_read_1(sc->dme_tag, sc->dme_handle, DATA_ADDR)) {
	case 1:
		/* Correct value */
		break;
	case 0:
		return 1;
	default:
		/* Error */
		return -1;
	}

	i = dme_read_reg(sc, DME_MRRL);
	i |= dme_read_reg(sc, DME_MRRH) << 8;

	len = dme_read_reg(sc, DME_ROCR);

	bus_space_write_1(sc->dme_tag, sc->dme_handle, CMD_ADDR, DME_MRCMD);
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);
	len = 0;
	switch(sc->dme_bits) {
	case 8:
		i = bus_space_read_1(sc->dme_tag, sc->dme_handle, DATA_ADDR);
		i <<= 8;
		i |= bus_space_read_1(sc->dme_tag, sc->dme_handle, DATA_ADDR);

		len = bus_space_read_1(sc->dme_tag, sc->dme_handle, DATA_ADDR);
		len |= bus_space_read_1(sc->dme_tag, sc->dme_handle,
		    DATA_ADDR) << 8;
		break;
	case 16:
		bus_space_read_2(sc->dme_tag, sc->dme_handle, DATA_ADDR);
		len = bus_space_read_2(sc->dme_tag, sc->dme_handle, DATA_ADDR);
		break;
	case 32:
	{
		uint32_t reg;

		reg = bus_space_read_4(sc->dme_tag, sc->dme_handle, DATA_ADDR);
		len = reg & 0xFFFF;
		break;
	}
	}

	MGETHDR(m, M_NOWAIT, MT_DATA);
	if (m == NULL)
		return -1;

	if (len > MHLEN - ETHER_ALIGN) {
		MCLGET(m, M_NOWAIT);
		if (!(m->m_flags & M_EXT)) {
			m_freem(m);
			return -1;
		}
	}

	m->m_pkthdr.rcvif = ifp;
	m->m_len = m->m_pkthdr.len = len;
	m_adj(m, ETHER_ALIGN);

	/* Read the data */
	bus_space_barrier(sc->dme_tag, sc->dme_handle, 0, 4,
	    BUS_SPACE_BARRIER_READ);
	switch (sc->dme_bits) {
	case 8:
		bus_space_read_multi_1(sc->dme_tag, sc->dme_handle, DATA_ADDR,
		    mtod(m, uint8_t *), len);
		break;
	case 16:
		bus_space_read_multi_2(sc->dme_tag, sc->dme_handle, DATA_ADDR,
		    mtod(m, uint16_t *), (len + 1) / 2);
		break;
	case 32:
		bus_space_read_multi_4(sc->dme_tag, sc->dme_handle, DATA_ADDR,
		    mtod(m, uint32_t *), (len + 3) / 4);
		break;
	}

	if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
	DME_UNLOCK(sc);
	(*ifp->if_input)(ifp, m);
	DME_LOCK(sc);

	return 0;
}

static void
dme_tick(void *arg)
{
	struct dme_softc *sc;
	struct mii_data *mii;

	sc = (struct dme_softc *)arg;

	/* Probably too frequent? */
	mii = device_get_softc(sc->dme_miibus);
	mii_tick(mii);

	callout_reset(&sc->dme_tick_ch, hz, dme_tick, sc);
}

static void
dme_intr(void *arg)
{
	struct dme_softc *sc;
	uint32_t intr_status;

	sc = (struct dme_softc *)arg;

	DME_LOCK(sc);

	intr_status = dme_read_reg(sc, DME_ISR);
	dme_write_reg(sc, DME_ISR, intr_status);

	if (intr_status & ISR_PR) {
		/* Read the packets off the device */
		while (dme_rxeof(sc) == 0)
			continue;
	}

	if (intr_status & ISR_LNKCHG) {
		printf("Link change interrupt\n");
	}

	DME_UNLOCK(sc);
}

static void
dme_setmode(struct dme_softc *sc)
{
}

static int
dme_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct dme_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int error = 0;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	switch (command) {
	case SIOCSIFFLAGS:
		/*
		 * Switch interface state between "running" and
		 * "stopped", reflecting the UP flag.
		 */
		DME_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING)==0) {
				dme_init_locked(sc);
			}
		} else {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
				dme_stop(sc);
			}
		}
		dme_setmode(sc);
		DME_UNLOCK(sc);
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		mii = device_get_softc(sc->dme_miibus);
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}
	return (error);
}

static void dme_init_locked(struct dme_softc *sc)
{
	struct ifnet *ifp = sc->dme_ifp;

	DME_ASSERT_LOCKED(sc);

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
		return;

	dme_reset(sc);
	dme_config(sc);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	callout_reset(&sc->dme_tick_ch, hz, dme_tick, sc);
}

static void
dme_init(void *xcs)
{
	struct dme_softc *sc = xcs;

	DME_LOCK(sc);
	dme_init_locked(sc);
	DME_UNLOCK(sc);
}

static int
dme_ifmedia_upd(struct ifnet *ifp)
{
	struct dme_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = device_get_softc(sc->dme_miibus);

	DME_LOCK(sc);
	mii_mediachg(mii);
	DME_UNLOCK(sc);

	return (0);
}

static void
dme_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct dme_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = device_get_softc(sc->dme_miibus);

	DME_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	DME_UNLOCK(sc);
}

static struct ofw_compat_data compat_data[] = {
	{ "davicom,dm9000", true  },
	{ NULL,             false }
};

static int
dme_probe(device_t dev)
{
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);
	device_set_desc(dev, "Davicom DM9000");
	return (0);
}

static int
dme_attach(device_t dev)
{
	struct dme_softc *sc;
	struct ifnet *ifp;
	int error, rid, i;
	uint32_t data;

	sc = device_get_softc(dev);
	sc->dme_dev = dev;

	error = 0;

	mtx_init(&sc->dme_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	callout_init_mtx(&sc->dme_tick_ch, &sc->dme_mtx, 0);

	rid = 0;
	sc->dme_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->dme_res == NULL) {
		device_printf(dev, "unable to map memory\n");
		error = ENXIO;
		goto fail;
	}

	rid = 0;
	sc->dme_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->dme_irq == NULL) {
		device_printf(dev, "unable to map memory\n");
		error = ENXIO;
		goto fail;
	}
	/*
	 * Power the chip up, if necessary
	 */
	error = fdt_regulator_enable_by_name(dev, "vcc-supply");
	if (error != 0) {
		device_printf(dev, "unable to enable power supply\n");
		error = ENXIO;
		goto fail;
	}

	/*
	 * Delay a little.  This seems required on rev-1 boards (green.)
	 */
	DELAY(1000);

	/* Bring controller out of reset */
	error = ofw_gpiobus_parse_gpios(dev, "reset-gpios", &sc->gpio_rset);
	if (error > 1) {
		device_printf(dev, "too many reset gpios\n");
		sc->gpio_rset = NULL;
		error = ENXIO;
		goto fail;
	}

	if (sc->gpio_rset != NULL) {
		error = GPIO_PIN_SET(sc->gpio_rset->dev, sc->gpio_rset->pin, 0);
		if (error != 0) {
			device_printf(dev, "Cannot configure GPIO pin %d on %s\n",
			    sc->gpio_rset->pin, device_get_nameunit(sc->gpio_rset->dev));
			goto fail;
		}

		error = GPIO_PIN_SETFLAGS(sc->gpio_rset->dev, sc->gpio_rset->pin,
		    GPIO_PIN_OUTPUT);
		if (error != 0) {
			device_printf(dev, "Cannot configure GPIO pin %d on %s\n",
			    sc->gpio_rset->pin, device_get_nameunit(sc->gpio_rset->dev));
			goto fail;
		}

		DELAY(2000);

		error = GPIO_PIN_SET(sc->gpio_rset->dev, sc->gpio_rset->pin, 1);
		if (error != 0) {
			device_printf(dev, "Cannot configure GPIO pin %d on %s\n",
			    sc->gpio_rset->pin, device_get_nameunit(sc->gpio_rset->dev));
			goto fail;
		}

		DELAY(4000);
	}

	sc->dme_tag = rman_get_bustag(sc->dme_res);
	sc->dme_handle = rman_get_bushandle(sc->dme_res);

	/* Reset the chip as soon as possible */
	dme_reset(sc);

	/*
	 * loop over and read the device revision until we get what
	 * we expect.  Loop until we get it, or 1 second.
	 */
	for (i = 0; i < 1000; i++) {
		data = dme_read_reg(sc, DME_VIDL);
		data |= dme_read_reg(sc, DME_VIDH) << 8;
		data |= dme_read_reg(sc, DME_PIDL) << 16;
		data |= dme_read_reg(sc, DME_PIDH) << 24;

		if (data == DME_CHIP_DM9000_ID)
			break;
		DELAY(1000);
	}

	/* Figure IO mode */

	/*
	 * This delay is also needed before reading ISR reliably!
	 */
	DELAY(1000);
	switch((dme_read_reg(sc, DME_ISR) >> 6) & 0x03) {
	case 0:
		/* 16 bit */
		sc->dme_bits = 16;
		break;
	case 1:
		/* 32 bit */
		sc->dme_bits = 32;
		break;
	case 2:
		/* 8 bit */
		sc->dme_bits = 8;
		break;
	default:
		/* reserved */
		device_printf(dev, "Unable to determine device mode\n");
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "IO size: %d bits\n", sc->dme_bits);

	/* Read vendor and device id's */
	data = dme_read_reg(sc, DME_VIDH) << 8;
	data |= dme_read_reg(sc, DME_VIDL);
	device_printf(dev, "Vendor ID: 0x%04x\n", data);

	/* Read vendor and device id's */
	data = dme_read_reg(sc, DME_PIDH) << 8;
	data |= dme_read_reg(sc, DME_PIDL);
	device_printf(dev, "Product ID: 0x%04x\n", data);

	/* Chip revision */
	data = dme_read_reg(sc, DME_CHIPR);
	device_printf(dev, "Revision: 0x%04x\n", data);
	if (data != DME_CHIP_DM9000A && data != DME_CHIP_DM9000B)
		data = DME_CHIP_DM9000;
	sc->dme_rev = data;

	/* Try to figure our mac address */
	dme_get_macaddr(sc);

	/* Configure chip after reset */
	dme_config(sc);

	ifp = sc->dme_ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "unable to allocate ifp\n");
		error = ENOSPC;
		goto fail;
	}
	ifp->if_softc = sc;

	/* Setup MII */
	error = mii_attach(dev, &sc->dme_miibus, ifp, dme_ifmedia_upd,
	    dme_ifmedia_sts, BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	/* This should never happen as the DM9000 contains it's own PHY */
	if (error != 0) {
		device_printf(dev, "PHY probe failed\n");
		goto fail;
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = dme_start;
	ifp->if_ioctl = dme_ioctl;
	ifp->if_init = dme_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);

	ether_ifattach(ifp, sc->dme_macaddr);

	error = bus_setup_intr(dev, sc->dme_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, dme_intr, sc, &sc->dme_intrhand);
	if (error) {
		device_printf(dev, "couldn't set up irq\n");
		ether_ifdetach(ifp);
		goto fail;
	}
fail:
	if (error != 0)
		dme_detach(dev);
	return (error);
}

static int
dme_detach(device_t dev)
{
	struct dme_softc *sc;
	struct ifnet *ifp;

	sc = device_get_softc(dev);
	KASSERT(mtx_initialized(&sc->dme_mtx), ("dme mutex not initialized"));

	ifp = sc->dme_ifp;

	if (device_is_attached(dev)) {
		DME_LOCK(sc);
		dme_stop(sc);
		DME_UNLOCK(sc);
		ether_ifdetach(ifp);
		callout_drain(&sc->dme_tick_ch);
	}

	if (sc->dme_miibus)
		device_delete_child(dev, sc->dme_miibus);
	bus_generic_detach(dev);

	if (sc->dme_intrhand)
		bus_teardown_intr(dev, sc->dme_irq, sc->dme_intrhand);
	if (sc->dme_irq)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->dme_irq);
	if (sc->dme_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->dme_res);

	if (ifp != NULL)
		if_free(ifp);

	mtx_destroy(&sc->dme_mtx);

	return (0);
}

/*
 * The MII bus interface
 */
static int
dme_miibus_readreg(device_t dev, int phy, int reg)
{
	struct dme_softc *sc;
	int i, rval;

	/* We have up to 4 PHY's */
	if (phy >= 4)
		return (0);

	sc = device_get_softc(dev);

	/* Send the register to read to the phy and start the read */
	dme_write_reg(sc, DME_EPAR, (phy << 6) | reg);
	dme_write_reg(sc, DME_EPCR, EPCR_EPOS | EPCR_ERPRR);

	/* Wait for the data to be read */
	for (i = 0; i < DME_TIMEOUT; i++) {
		if ((dme_read_reg(sc, DME_EPCR) & EPCR_ERRE) == 0)
			break;
		DELAY(1);
	}

	/* Clear the comand */
	dme_write_reg(sc, DME_EPCR, 0);

	if (i == DME_TIMEOUT)
		return 0;

	rval = (dme_read_reg(sc, DME_EPDRH) << 8) | dme_read_reg(sc, DME_EPDRL);
	return (rval);
}

static int
dme_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct dme_softc *sc;
	int i;

	/* We have up to 4 PHY's */
	if (phy > 3)
		return (0);

	sc = device_get_softc(dev);

	/* Send the register and data to write to the phy */
	dme_write_reg(sc, DME_EPAR, (phy << 6) | reg);
	dme_write_reg(sc, DME_EPDRL, data & 0xFF);
	dme_write_reg(sc, DME_EPDRH, (data >> 8) & 0xFF);
	/* Start the write */
	dme_write_reg(sc, DME_EPCR, EPCR_EPOS | EPCR_ERPRW);

	/* Wait for the data to be written */
	for (i = 0; i < DME_TIMEOUT; i++) {
		if ((dme_read_reg(sc, DME_EPCR) & EPCR_ERRE) == 0)
			break;
		DELAY(1);
	}

	/* Clear the comand */
	dme_write_reg(sc, DME_EPCR, 0);

	return (0);
}

static device_method_t dme_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dme_probe),
	DEVMETHOD(device_attach,	dme_attach),
	DEVMETHOD(device_detach,	dme_detach),

	/* bus interface, for miibus */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,       dme_miibus_readreg),
	DEVMETHOD(miibus_writereg,      dme_miibus_writereg),

	{ 0, 0 }
};

static driver_t dme_driver = {
	"dme",
	dme_methods,
	sizeof(struct dme_softc)
};

static devclass_t dme_devclass;

MODULE_DEPEND(dme, ether, 1, 1, 1);
MODULE_DEPEND(dme, miibus, 1, 1, 1);
DRIVER_MODULE(dme, simplebus, dme_driver, dme_devclass, 0, 0);
DRIVER_MODULE(miibus, dme, miibus_driver, miibus_devclass, 0, 0);

