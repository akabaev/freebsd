/*-
 * Copyright (c) 2015 Alexander Kabaev <kan@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the distribution.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/clk/clk_fixed.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/* Simple placeholder driver for fixed clocks */

struct fixed_clock_softc {
	struct mtx	mtx;
	struct clkdom	*clkdom;
	uint32_t	frequency;
};

static int
fixed_clock_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "fixed-clock")) {
		device_set_desc(dev, "Fixed clock");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
fixed_clock_attach(device_t dev)
{
	struct fixed_clock_softc *sc;
	struct clk_fixed_def cdef;
	phandle_t node;
	pcell_t freq;
	int rv;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	if (OF_getencprop(node, "clock-frequency", &freq, sizeof(freq)) <= 0) {
		device_printf(dev, "missing clock-frequency attribute in FDT\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "fclock", MTX_DEF);
	sc->frequency = freq;

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		goto fail;
	/* Fixed clock has no parents and only one clock module */
	cdef.clkdef.id = 1;
	cdef.clkdef.name = __DECONST(char *, ofw_bus_get_name(dev));
	cdef.clkdef.parent_names = NULL;
	cdef.clkdef.parents_num = 0;
	cdef.clkdef.flags = CLK_FLAGS_STATIC;
	cdef.freq = freq;
	cdef.mult = 1;
	cdef.div = 1;
	rv = clknode_fixed_register(sc->clkdom, &cdef, &sc->mtx);
	if (rv != 0)
		goto fail;
	clkdom_finit(sc->clkdom);

	device_printf(dev, "%s clock frequency is %u HZ\n",
	    ofw_bus_get_name(dev), freq);

	return (0);
fail:
	return (ENXIO);
}

static device_method_t fixed_clock_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, fixed_clock_probe),
	DEVMETHOD(device_attach, fixed_clock_attach),

	DEVMETHOD_END
};

static driver_t fixed_clock_driver = {
	"fixedclock",
	fixed_clock_methods,
	sizeof(struct fixed_clock_softc)
};

static devclass_t fixed_clock_devclass;

EARLY_DRIVER_MODULE(fixedclock, simplebus, fixed_clock_driver,
    fixed_clock_devclass, 0, 0, BUS_PASS_CPU + BUS_PASS_ORDER_LATE);
