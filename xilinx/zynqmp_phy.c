/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018-2020 Thomas Skibo
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
 * $FreeBSD$
 */

/*
 * Control for Zynq UltraScale+ PS GTR+ SERDES modules.
 *
 * Mostly, the SERDES and SIOU registers are configured  by the Vivado
 * processor configuration wizard (PCW) and the OS should leave these
 * blocks alone.   The exception is adjusting voltage and deemphasis
 * values of Display Port lanes.
 *
 * Reference: Zynq UltraScale+ Technical Reference Manual
 * UG1085 (v1.7) December 22, 2017, Chapter 29: PS-GTR Transceivers.
 *
 * Zynq UltraScale+ MPSoC Register Reference;
 * https://www.xilinx.com/html_docs/registers/ug1087/ug1087-zynq-ultrascale-registers.html
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/xilinx/zynqmp_phy.h>


#define ZPHY_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	ZPHY_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZPHY_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "phy", MTX_DEF)
#define ZPHY_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

#define NUM_PHYS	4

#define PHY_PROTO_PD		0
#define PHY_PROTO_PCIE		1
#define PHY_PROTO_SATA		2
#define PHY_PROTO_USB		3
#define PHY_PROTO_DP		4
#define PHY_PROTO_SGMII		5

#define PHY_CTLR_XXX		0
#define PHY_CTLR_USB0		1
#define PHY_CTLR_USB1		2
#define PHY_CTLR_SATA_0		3
#define PHY_CTLR_SATA_1		4
#define PHY_CTLR_PCIE_0		5
#define PHY_CTLR_PCIE_1		6
#define PHY_CTLR_PCIE_2		7
#define PHY_CTLR_PCIE_3		8
#define PHY_CTLR_DP_0		9
#define PHY_CTLR_DP_1		10
#define PHY_CTLR_SGMII0		11
#define PHY_CTLR_SGMII1		12
#define PHY_CTLR_SGMII2		13
#define PHY_CTLR_SGMII3		14

static char *phy_ctlr_name[] =
    { "-", "usb0", "usb1", "sata.0", "sata.1", "pcie.0", "pcie.1", "pcie.2",
      "pcie.3", "dp.0", "dp.1", "sgmii.0", "sgmii.1", "sgmii.2","sgmii.3" };

/*
 * Interconnect Matrix and the possible controllers for each PHY lane,
 * by protocol.
 */
static int icm_lanes[][6] = {
	{ PHY_CTLR_XXX, PHY_CTLR_PCIE_0, PHY_CTLR_SATA_0, PHY_CTLR_USB0,
	  PHY_CTLR_DP_1, PHY_CTLR_SGMII0 },
	{ PHY_CTLR_XXX, PHY_CTLR_PCIE_1, PHY_CTLR_SATA_1, PHY_CTLR_USB0,
	  PHY_CTLR_DP_0, PHY_CTLR_SGMII1 },
	{ PHY_CTLR_XXX, PHY_CTLR_PCIE_2, PHY_CTLR_SATA_0, PHY_CTLR_USB0,
	  PHY_CTLR_DP_1, PHY_CTLR_SGMII2 },
	{ PHY_CTLR_XXX, PHY_CTLR_PCIE_3, PHY_CTLR_SATA_1, PHY_CTLR_USB1,
	  PHY_CTLR_DP_0, PHY_CTLR_SGMII3 }
};

struct zynqmp_phy_softc {
	device_t	dev;
	struct mtx	sc_mtx;
	struct resource *mem_res[2];	/* Memory resource */

	struct ps_phy {
		int proto;
		int ctlr;
		phandle_t xref;
	} phy[NUM_PHYS];
};

/* SERDES Registers. */
#define SERDES_PLL_STATUS(n)		(0x23e4 + 0x4000 * (n))
#define    SERDES_PLL_STATUS_LOCKED				(1 << 4)
#define SERDES_TX_ANA_TM_18_L(n)	(0x0048 + 0x4000 * (n))
#define SERDES_TXPMD_TM_48_L(n)		(0x0cc0 + 0x4000 * (n))
#define SERDES_ICM_CFG0			0x10010
#define    SERDES_ICM_CFG0_L0_MASK				(7 << 0)
#define    SERDES_ICM_CFG0_L0_SHIFT				0
#define    SERDES_ICM_CFG0_L1_MASK				(7 << 4)
#define    SERDES_ICM_CFG0_L1_SHIFT				4
#define SERDES_ICM_CFG1			0x10014
#define    SERDES_ICM_CFG1_L2_MASK				(7 << 0)
#define    SERDES_ICM_CFG1_L2_SHIFT				0
#define    SERDES_ICM_CFG1_L3_MASK				(7 << 4)
#define    SERDES_ICM_CFG1_L3_SHIFT				4

#define WR4(sc, off, val)	bus_write_4((sc)->mem_res[0], (off), (val))
#define RD4(sc, off)		bus_read_4((sc)->mem_res[0], (off))
#define WR4_SIOU(sc, off, val)	bus_write_4((sc)->mem_res[1], (off), (val))
#define RD4_SIOU(sc, off)	bus_read_4((sc)->mem_res[1], (off))

static struct ofw_compat_data compat_data[] = {
       {"xlnx,zynqmp-psgtr-v1.1",	1},
       {"xlnx,zynqmp-psgtr",		1},
       {NULL,				0}
};

static int zynqmp_phy_attach(device_t);
static int zynqmp_phy_detach(device_t);

/* Read the static configuration of the phys. */
static void
zynqmp_phy_read_config(struct zynqmp_phy_softc *sc)
{
	uint32_t cfg01;
	uint32_t cfg23;
	int i;

	/* Get protocols. */
	cfg01 = RD4(sc, SERDES_ICM_CFG0);
	cfg23 = RD4(sc, SERDES_ICM_CFG1);

	sc->phy[0].proto = (cfg01 & SERDES_ICM_CFG0_L0_MASK) >>
	    SERDES_ICM_CFG0_L0_SHIFT;
	sc->phy[1].proto = (cfg01 & SERDES_ICM_CFG0_L1_MASK) >>
	    SERDES_ICM_CFG0_L1_SHIFT;
	sc->phy[2].proto = (cfg23 & SERDES_ICM_CFG1_L2_MASK) >>
	    SERDES_ICM_CFG1_L2_SHIFT;
	sc->phy[3].proto = (cfg23 & SERDES_ICM_CFG1_L3_MASK) >>
	    SERDES_ICM_CFG1_L3_SHIFT;

	/* For each lane, determine controller from table. */
	for (i = 0; i < NUM_PHYS; i++)
		sc->phy[i].ctlr = icm_lanes[i][sc->phy[i].proto];
}

static void
zynqmp_phy_add_sysctls(struct zynqmp_phy_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child;
	struct sysctl_oid_list *phy;
	struct sysctl_oid *tree;
	int i;
	static char *phyname[] = { "phy0", "phy1", "phy2", "phy3", NULL };

	ctx = device_get_sysctl_ctx(sc->dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	if (ctx == NULL || child == NULL) {
		device_printf(sc->dev, "could not add sysctls\n");
		return;
	}

	for (i = 0; i < NUM_PHYS; i++) {
		tree = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, phyname[i],
		    CTLFLAG_RD, NULL, "PHY configuration and stats");
		phy = SYSCTL_CHILDREN(tree);
		if (tree == NULL || phy == NULL) {
			device_printf(sc->dev, "trouble adding PHY sysctls\n");
			return;
		}

		SYSCTL_ADD_STRING(ctx, phy, OID_AUTO, "controller", CTLFLAG_RD,
		    phy_ctlr_name[sc->phy[i].ctlr], 0,
		    "PHY device controller");
	}
}

static int
zynqmp_phy_lane_from_xref(struct zynqmp_phy_softc *sc, phandle_t xref)
{
	int i;

	for (i = 0; i < NUM_PHYS; i++)
		if (sc->phy[i].xref == xref)
			return (i);
	return (-1);
}

int
zynqmp_phy_wait_pll_lock(phandle_t xref)
{
	device_t dev;
	struct zynqmp_phy_softc *sc;
	int phy;
	int tries = 1000;

	dev = OF_device_from_xref(xref);
	if (dev == NULL)
		return (ENXIO);
	sc = device_get_softc(dev);
	phy = zynqmp_phy_lane_from_xref(sc, xref);

	ZPHY_LOCK(sc);

	while (--tries > 0) {
		if (RD4(sc, SERDES_PLL_STATUS(phy)) & SERDES_PLL_STATUS_LOCKED)
			break;
		DELAY(1);
	}

	ZPHY_UNLOCK(sc);

	return (tries > 0 ? 0 : ETIMEDOUT);
}

int
zynqmp_phy_margining_factor(phandle_t xref, int p_level, int v_level)
{
	device_t dev;
	struct zynqmp_phy_softc *sc;
	int phy;
	static uint8_t vs[4][4] = { { 0x2a, 0x27, 0x24, 0x20 },
				    { 0x27, 0x23, 0x20, 0xff },
				    { 0x24, 0x20, 0xff, 0xff },
				    { 0xff, 0xff, 0xff, 0xff } };

	dev = OF_device_from_xref(xref);
	if (dev == NULL)
		return (ENXIO);
	sc = device_get_softc(dev);
	phy = zynqmp_phy_lane_from_xref(sc, xref);

	ZPHY_LOCK(sc);
	WR4(sc, SERDES_TXPMD_TM_48_L(phy), vs[p_level][v_level]);
	ZPHY_UNLOCK(sc);

	return (0);
}

int
zynqmp_phy_override_deemph(phandle_t xref, int p_level, int v_level)
{
	device_t dev;
	struct zynqmp_phy_softc *sc;
	int phy;
	static uint8_t pe[4][4] = { { 0x02, 0x02, 0x02, 0x02 },
				    { 0x01, 0x01, 0x01, 0xff },
				    { 0x00, 0x00, 0xff, 0xff },
				    { 0xff, 0xff, 0xff, 0xff } };

	dev = OF_device_from_xref(xref);
	if (dev == NULL)
		return (ENXIO);
	sc = device_get_softc(dev);
	phy = zynqmp_phy_lane_from_xref(sc, xref);

	ZPHY_LOCK(sc);
	WR4(sc, SERDES_TX_ANA_TM_18_L(phy), pe[p_level][v_level]);
	ZPHY_UNLOCK(sc);

	return (0);
}

static int
zynqmp_phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ PHY driver");
	return (0);
}

static int
zynqmp_phy_attach(device_t dev)
{
	struct zynqmp_phy_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	phandle_t child;
	int rid;
	int phy;

	sc->dev = dev;

	ZPHY_LOCK_INIT(sc);

	/* Allocate memory mapped registers. */
	rid = 0;
	sc->mem_res[0] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (sc->mem_res[0] == NULL) {
		device_printf(dev, "can not allocate memory for device");
		zynqmp_phy_detach(dev);
		return (ENOMEM);
	}
	rid = 1;
	sc->mem_res[1] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (sc->mem_res[1] == NULL) {
		device_printf(dev, "can not allocate memory for device");
		zynqmp_phy_detach(dev);
		return (ENOMEM);
	}

	/* Register node and children (phy lanes) to this driver. */
	OF_device_register_xref(OF_xref_from_node(node), dev);
	for (phy = 0, child = OF_child(node); child != 0 && phy < NUM_PHYS;
	     child = OF_peer(child), phy++) {
		sc->phy[phy].xref = OF_xref_from_node(child);
		OF_device_register_xref(sc->phy[phy].xref, dev);
	}

	zynqmp_phy_read_config(sc);

	zynqmp_phy_add_sysctls(sc);

	return (0);
}

static int
zynqmp_phy_detach(device_t dev)
{
	struct zynqmp_phy_softc *sc = device_get_softc(dev);

	if (sc->mem_res[0] != NULL) {
		/* Release memory resource. */
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res[0]), sc->mem_res[0]);
	}
	if (sc->mem_res[1] != NULL) {
		/* Release memory resource. */
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res[1]), sc->mem_res[1]);
	}

	ZPHY_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_phy_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		zynqmp_phy_probe),
	DEVMETHOD(device_attach,	zynqmp_phy_attach),
	DEVMETHOD(device_detach,	zynqmp_phy_detach),

	DEVMETHOD_END
};

static driver_t zynqmp_phy_driver = {
	"zynqmp_phy",
	zynqmp_phy_methods,
	sizeof(struct zynqmp_phy_softc),
};
static devclass_t zynqmp_phy_devclass;

DRIVER_MODULE(zynqmp_phy, simplebus, zynqmp_phy_driver,	\
	      zynqmp_phy_devclass, NULL, NULL);
MODULE_VERSION(zynqmp_phy, 1);
SIMPLEBUS_PNP_INFO(compat_data);
