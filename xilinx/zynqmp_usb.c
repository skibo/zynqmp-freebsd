/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Thomas Skibo. <Thomas@Skibo.net>
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
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
	{"xlnx,zynqmp-dwc3",		1},
	{"xlnx,versal-dwc3",		1},
	{NULL,				0}
};

struct zynqmp_usb_softc {
	struct simplebus_softc	simplebus_sc;
	device_t		dev;
	struct resource		*mem_res;
};

#define WR4(sc, off, val)	bus_write_4((sc)->mem_res, (off), (val))
#define RD4(sc, off)		bus_read_4((sc)->mem_res, (off))

/* USB3 regs.
 *
 * Reference: ug1087 Zynq UltraScale+ Registers.
 *
 * https://www.xilinx.com/html_docs/registers/ug1087/ug1087-zynq-ultrascale-registers.html
 *
 */
#define USB3_CUR_PWR_ST			0x000
#define USB3_CONNECT_ST			0x004
#define USB3_COHERENCY			0x05c
#define     USB3_COHERENCY_EN			1

static int
zynqmp_usb_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ USB3 device");

	return (BUS_PROBE_DEFAULT);
}

static int
zynqmp_usb_attach(device_t dev)
{
	struct zynqmp_usb_softc *sc = device_get_softc(dev);
	int rid;
	phandle_t node;

	sc->dev = dev;

	/* Check for real dwc3 controller which is this node's child. */
	node = ofw_bus_get_node(dev);
	if (node == -1)
		return (ENXIO);
	if (OF_child(node) <= 0) {
		device_printf(dev, "missing child node.\n");
		return (ENXIO);
	}

	/* DP Module device registers. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "could not allocate memory resource.\n");
		return (ENOMEM);
	}

	/* Route transactions through CCI. */
	WR4(sc, USB3_COHERENCY, RD4(sc, USB3_COHERENCY) |
	    USB3_COHERENCY_EN);

	/*
	 * XXX TODO: certain chip versions have suspend phy quirk.
	 */

	simplebus_init(dev, node);

	/* Allow device to identify. */
	bus_generic_probe(dev);

	/* Attach child node, the generic dwc3 driver. */
	simplebus_add_device(dev, OF_child(node), 0, NULL, -1, NULL);

	return (bus_generic_attach(dev));
}

static int
zynqmp_usb_detach(device_t dev)
{
	struct zynqmp_usb_softc *sc = device_get_softc(dev);

	/* Release resources. */
	if (sc->mem_res) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);
		sc->mem_res = NULL;
	}

	return (0);
}

static device_method_t zynqmp_usb_methods[] = {
	DEVMETHOD(device_probe,		zynqmp_usb_probe),
	DEVMETHOD(device_attach,	zynqmp_usb_attach),
	DEVMETHOD(device_detach,	zynqmp_usb_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(zynqmp_usb, zynqmp_usb_driver, zynqmp_usb_methods,
    sizeof(struct zynqmp_usb_softc), simplebus_driver);

static devclass_t zynqmp_usb_devclass;

DRIVER_MODULE(zynqmp_usb, simplebus, zynqmp_usb_driver, zynqmp_usb_devclass, \
    0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
