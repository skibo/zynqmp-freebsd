/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Thomas Skibo.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
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

#include <dev/extres/clk/clk.h>
#include <dev/extres/clk/clk_fixed.h>
#include <arm64/xilinx/clk/zynqmp_clk_pll.h>
#include <arm64/xilinx/clk/zynqmp_clk_composite.h>

#include <arm64/xilinx/zynqmp_pm.h>

#define ZCLK_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	ZCLK_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZCLK_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "clk", MTX_DEF)
#define ZCLK_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

#ifdef CLKDEBUG
#define DPRINTF(...)	do { printf(__VA_ARGS__); } while (0)
#else
#define DPRINTF(...) do { } while (0)
#endif

MALLOC_DEFINE(M_ZCLK, "zclk", "zynqmp clock manager");

static struct ofw_compat_data compat_data[] = {
	{"xlnx,zynqmp-clk",		1},
	{NULL,				0}
};

#define MAX_CLOCKS		512
#define MAX_CLK_PARENTS		80
#define MAX_CLK_TOPO_NODES	4
#define CLK_MAX_NAME_LEN	(RETV_CNT * sizeof(uint32_t))

struct clknode;

struct zynqmp_clk_softc {
	device_t	dev;
	struct mtx	sc_mtx;

	struct clkdom	*clkdom;
};

/* Constants for communicating with platform management firmware. */
#define CLK_GET_PARENTS_RESP_NUM_WORDS		3
#define GET_ATTRS_VALID				1
#define GET_ATTRS_EXTERNAL			(1 << 2)
#define GET_PARENTS_NA_PARENT			0xffffffffu
#define GET_PARENTS_DUMMY			0xfffffffeu
#define GET_PARENTS_ID_MASK			0xffff
#define GET_PARENTS_FLAGS_MASK			(0xffffu << 16)
#define GET_PARENTS_FLAG_SELF			(0 << 16)
#define GET_PARENTS_FLAG_NODE1			(1 << 16)
#define GET_PARENTS_FLAG_NODE2			(2 << 16)
#define GET_PARENTS_FLAG_NODE3			(3 << 16)
#define GET_PARENTS_FLAG_NODE4			(4 << 16)
#define GET_PARENTS_FLAG_EXTERNAL		(5 << 16)
#define GET_PARENTS_FLAG_MIO0_77		(6 << 16)
#define CLK_GET_TOPO_RESP_NUM_WORDS		3
#define GET_TOPO_TYPE_MASK			0xf
#define GET_TOPO_TYPE_INVALID			0
#define GET_TOPO_TYPE_MUX			1
#define GET_TOPO_TYPE_PLL			2
#define GET_TOPO_TYPE_FIXED			3
#define GET_TOPO_TYPE_DIV0			4  /* DIV1 in firmware. */
#define GET_TOPO_TYPE_DIV1			5  /* DIV2 in firmware. */
#define GET_TOPO_TYPE_GATE			6
#define GET_TOPO_FLAGS_MASK			(0xffff << 8)
#define GET_TOPO_FLAGS_SET_RATE_GATE		(1 << 8)
#define GET_TOPO_FLAGS_SET_PARENT_GATE		(1 << 9)
#define GET_TOPO_FLAGS_SET_RATE_PARENT		(1 << 10)
#define GET_TOPO_FLAGS_IGNORE_UNUSED		(1 << 11)
#define GET_TOPO_FLAGS_IS_BASIC			(1 << 13)
#define GET_TOPO_FLAGS_GET_RATE_NOCACHE		(1 << 14)
#define GET_TOPO_FLAGS_SET_RATE_NO_REPARENT	(1 << 15)
#define GET_TOPO_FLAGS_GET_ACC_NOCACHE		(1 << 16)
#define GET_TOPO_FLAGS_RECALC_NEW_RATES		(1 << 17)
#define GET_TOPO_FLAGS_SET_RATE_UNGATE		(1 << 18)
#define GET_TOPO_FLAGS_IS_CRITICAL		(1 << 19)
#define GET_TOPO_FLAGS_OPS_PARENT_ENABLE	(1 << 20)
#define GET_TOPO_TYPE_FLAGS_MASK		(0xffu << 24)
#define GET_TOPO_TYPE_FLAGS_DIV_ONE_BASED	(1 << 24)
#define GET_TOPO_TYPE_FLAGS_DIV_POW_TWO	(1 << 25)
#define GET_TOPO_TYPE_FLAGS_ALLOW_ZERO		(1 << 26)
#define GET_TOPO_TYPE_FLAGS_HIWORD_MASK		(1 << 27)
#define GET_TOPO_TYPE_FLAGS_ROUND_CLOSEST	(1 << 28)
#define GET_TOPO_TYPE_FLAGS_READ_ONLY		(1 << 29)
#define GET_TOPO_TYPE_FLAGS_MAX_AT_ZERO		(1 << 30)
#define GET_TOPO_TYPE_FLAGS_FRAC		(1U << 31)

/* Get a clock name by querying the pmu firmware. */
static int
zynqmp_clk_get_name(int clock_id, char *clkname)
{
	int error;

	error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_NAME, clock_id,
	    0, 0, (uint32_t *)clkname);

	if (!clkname[0])
		return (ENXIO);

	return (error);
}

/*
 * Query the pmu firmware to retrieve clock parent names,  This requires
 * that parent_names points to a buffer large enough for MAX_CLK_PARENTS
 * character pointers and MAX_CLK_PARENTS * CLK_MAX_NAME_LEN space for clock
 * names.  It fills in the pointers and the names in manner acceptable to
 * clknode_create().
 */
static int
zynqmp_clk_get_parents(int clock_id, char **parent_names, int *count)
{
	int i;
	int index;
	int error;
	uint32_t retvs[RETV_CNT];
	char *pbuf = (char *)parent_names + MAX_CLK_PARENTS * sizeof(char *);

	*count = 0;
	index = 0;
	for (;;) {
		error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_PARENTS,
		    clock_id, index, 0, retvs);
		if (error)
			return (error);
		for (i = 1; i < CLK_GET_PARENTS_RESP_NUM_WORDS + 1; i++) {
			if (retvs[i] == GET_PARENTS_NA_PARENT) {
				*count = index;
				return (0);
			}

			if (index >= MAX_CLK_PARENTS)
				return (E2BIG);

			if (retvs[i] == GET_PARENTS_DUMMY) {
				parent_names[index] = NULL;
				continue;
			}

			parent_names[index] = pbuf + CLK_MAX_NAME_LEN * index;

			error = zynqmp_clk_get_name(retvs[i] &
			    GET_PARENTS_ID_MASK, parent_names[index]);
			if (error)
				return (error);

			index++;
		}
	}
}

/*
 * Get clock attributes by querying pmu firmware.  It's only a valid flag and
 * an external flag.
 */
static int
zynqmp_clk_get_attrs(int clock_id, uint32_t *attrs)
{
	int error;
	uint32_t retvs[RETV_CNT];

	error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_ATTRIBUTES, clock_id,
	    0, 0, retvs);

	if (!error)
		*attrs = retvs[1];

	return (error);
}

/* Get clock topology from pmu firmware. */
static int
zynqmp_clk_get_topology(int clock_id, int *count, uint32_t *topos)
{
	int error;
	int i, index;
	uint32_t retvs[RETV_CNT];

	index = 0;
	for (;;) {
		error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_TOPOLOGY,
		    clock_id, index, 0, retvs);
		if (error)
			return (error);
		for (i = 0; i < CLK_GET_TOPO_RESP_NUM_WORDS; i++) {
			if ((retvs[i + 1] & GET_TOPO_TYPE_MASK) ==
			    GET_TOPO_TYPE_INVALID) {
				*count = index;
				return (0);
			}
			if (index >= MAX_CLK_TOPO_NODES) {
				printf("%s: clkid=%d too many topo nodes!\n",
				    __func__, clock_id);
				return (E2BIG);
			}
			topos[index++] = retvs[i + 1];
		}
	}
}

/* Get fixed factor parameters for a clock by querying pmu firmware. */
static int
zynqmp_clk_get_fixed_factor_params(int clock_id, uint32_t *mult, uint32_t *div)
{
	int error;
	uint32_t retvs[RETV_CNT];

	error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_FIXEDFACTOR_PARAMS,
	    clock_id, 0, 0, retvs);

	if (!error) {
		*mult = retvs[1];
		*div = retvs[2];
	}

	return (error);
}

/* Get maximum divisor by querying pm firmware.  div_id should be 0 or 1. */
static int
zynqmp_clk_get_max_divisor(int clock_id, int div_id, uint32_t *max_div)
{
	int error;
	uint32_t retvs[RETV_CNT];

	error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_MAX_DIVISOR, clock_id,
	    div_id ? GET_TOPO_TYPE_DIV1 : GET_TOPO_TYPE_DIV0, 0, retvs);

	if (!error)
		*max_div = retvs[1];

	return (error);
}

static void
zynqmp_clk_external(struct zynqmp_clk_softc *sc, struct clk_fixed_def *clkdef)
{
       int error;
       clk_t clk;

       /*
	* Check if clock is in "clock-names" property in which case it
	* references a clock in another node (typically fixed-clock
	* devices).
	*/
       error = clk_get_by_ofw_name(sc->dev, 0, clkdef->clkdef.name, &clk);
       if (!error)
	       return;

       /* Create a dead-end clock node. */
       clknode_fixed_register(sc->clkdom, clkdef);
}


#if 0
static char *topo_type_names[] = { "invalid", "mux", "pll", "fixed",
				   "div0", "div1", "gate", "?" };
#endif

/*
 * Map PLL names to pmu node ids.  See comment in
 * zynqmp_clk_register_clocks().
 */
static struct {
	char *name;
	int node_id;
} pll_ids[] = {
	{ "apll_int", ZYNQMP_PM_NODE_APLL},
	{ "vpll_int", ZYNQMP_PM_NODE_VPLL },
	{ "dpll_int", ZYNQMP_PM_NODE_DPLL },
	{ "rpll_int", ZYNQMP_PM_NODE_RPLL },
	{ "iopll_int", ZYNQMP_PM_NODE_IOPLL },
	{ NULL, 0 }
};

/* Query the platform firmware for the entire clock tree. */
static int
zynqmp_clk_register_clocks(struct zynqmp_clk_softc *sc)
{
	int id;
	int error;
	int nclks;
	uint32_t retvs[RETV_CNT];
	uint32_t attrs;
	char name[CLK_MAX_NAME_LEN];
	uint32_t topos[MAX_CLK_TOPO_NODES];
	int nodes;
	char **parent_names;
	int nparents;
	int i;
	union {
		struct clk_fixed_def		fixed;
		struct zynqmp_clk_pll_def	pll;
		struct zynqmp_clk_composite_def	composite;
	} clkdef;

	sc->clkdom = clkdom_create(sc->dev);

	/* Get number of clocks. */
	if ((error = zynqmp_pm_query_data(PM_QID_CLOCK_GET_NUM_CLOCKS,
	    0, 0, 0, retvs)) < 0) {
		device_printf(sc->dev, "trouble querying num clocks.\n");
		return (error);
	}
	nclks = retvs[1];

	/* Sanity check number of clocks. XXX: necessary? */
	if (nclks > MAX_CLOCKS)
		return (E2BIG);
	else if (nclks < 1)
		return (EINVAL);

	/*
	 * Allocate space for parent_names parameter to clknode_create().
	 * This space is released at the end of this function.
	 */
	parent_names = malloc(MAX_CLK_PARENTS * (sizeof(char *) +
		CLK_MAX_NAME_LEN), M_ZCLK, M_NOWAIT | M_ZERO);
	if (parent_names == NULL) {
		device_printf(sc->dev, "could not allocate parent_names\n");
		error = ENOMEM;
		goto fail;
	}

	for (id = 0; id < nclks; id++) {
		/*
		 * XXX: Skip CLK_LPD_WDT because it is in the
		 * arm-trusted-firmware platform code but is not in PMU
		 * firmware so lots of calls to firmware fail.
		 *
		 * XXX: what's this magic number!? says andrew@ :-)
		 */
		if (id == 112)
			continue;

		/* Get clock attributes. */
		error = zynqmp_clk_get_attrs(id, &attrs);
		if (error) {
			device_printf(sc->dev, "could not get attrs id=%d\n",
			    id);
			goto fail;
		}
		if (!(attrs & GET_ATTRS_VALID))
			continue;

		/* Get clock name. */
		error = zynqmp_clk_get_name(id, name);
		if (error) {
			device_printf(sc->dev, "could not get name id=%d\n",
			    id);
			goto fail;
		}

		memset(&clkdef, 0, sizeof(clkdef));
		clkdef.fixed.clkdef.id = id;
		clkdef.fixed.clkdef.name = name;

		if ((attrs & GET_ATTRS_EXTERNAL) != 0) {
			zynqmp_clk_external(sc, &clkdef.fixed);
			continue;
		}

		/* Get topology. */
		error = zynqmp_clk_get_topology(id, &nodes, topos);
		if (error) {
			device_printf(sc->dev, "could not get topo id=%d\n",
			    id);
			goto fail;
		}

		/* Get parents. */
		error = zynqmp_clk_get_parents(id, parent_names, &nparents);
		if (error) {
			device_printf(sc->dev, "could not get parents id=%d\n",
			    id);
			goto fail;
		}
		clkdef.fixed.clkdef.parent_names = (const char **)(void *)
		    parent_names;
		clkdef.fixed.clkdef.parent_cnt = nparents;

#ifdef CLKDEBUG
		DPRINTF("%s: id=%d name=%s nodes=%d nparents=%d\n",
		    __func__, id, name, nodes, nparents);
		DPRINTF("%s: parents: ", __func__);
		for (i = 0; i < nparents; i++)
			DPRINTF("%s ", parent_names[i]);
		DPRINTF("\n");
#endif

		switch (topos[0] & GET_TOPO_TYPE_MASK) {
		case GET_TOPO_TYPE_FIXED:
			KASSERT(nodes == 1,
			    ("FIXED clock should only have one node.\n"));

			/* Get fixed factors from pmu firmware. */
			error = zynqmp_clk_get_fixed_factor_params(id,
			    &clkdef.fixed.mult, &clkdef.fixed.div);
			if (error) {
				/* XXX: not sure if this is a failure. */
				printf("%s: couldn't get fixed factor params: "
				    "name=%s\n", __func__,
				    clkdef.fixed.clkdef.name);
				break;
			}

			DPRINTF("%s: fixed clock: mult=%d div=%d\n", __func__,
			    clkdef.fixed.mult, clkdef.fixed.div);

			error = clknode_fixed_register(sc->clkdom,
			    &clkdef.fixed);
			break;

		case GET_TOPO_TYPE_PLL:
			KASSERT(nodes == 1,
			    ("PLL clock should only have one node.\n"));

			/*
			 * Look up PLL node ids based upon the name. XXX This
			 * is a hack necessary because I don't see any way to
			 * retrieve the PLL id from a clock id and the
			 * routines to get pll parameters through the clock
			 * id DON'T WORK.
			 */
			for (i = 0; pll_ids[i].name != NULL; i++)
				if (strcmp(pll_ids[i].name,
				    clkdef.pll.clkdef.name) == 0)
					break;
			if (pll_ids[i].name == NULL) {
				error = EINVAL;
				break;
			}
			clkdef.pll.pll_id = pll_ids[i].node_id;

			DPRINTF("%s: pll: pll_id=%d\n", __func__,
			    clkdef.pll.pll_id);

			error = zynqmp_clk_pll_register(sc->clkdom,
			    &clkdef.pll);
			break;

		case GET_TOPO_TYPE_MUX:
		case GET_TOPO_TYPE_DIV0:
		case GET_TOPO_TYPE_DIV1:
		case GET_TOPO_TYPE_GATE:

			for (i = 0; i < nodes; i++)
				switch (topos[i] & GET_TOPO_TYPE_MASK) {
				case GET_TOPO_TYPE_DIV0:
				case GET_TOPO_TYPE_DIV1:
					clkdef.composite.divisors++;
					break;
				case GET_TOPO_TYPE_GATE:
					clkdef.composite.has_gate++;
					break;
				}

			if (clkdef.composite.divisors > 0) {
				error = zynqmp_clk_get_max_divisor(id, 0,
				    &clkdef.composite.div0_max);
				if (error) {
					device_printf(sc->dev,
					    "could not get max divisor0.\n");
					goto fail;
				}
			}
			if (clkdef.composite.divisors > 1) {
				error = zynqmp_clk_get_max_divisor(id, 1,
				    &clkdef.composite.div1_max);
				if (error) {
					device_printf(sc->dev,
					    "could not get max divisor1.\n");
					goto fail;
				}
			}

			DPRINTF("%s: composite: divs=%d has_gate=%d\n",
			    __func__, clkdef.composite.divisors,
			    clkdef.composite.has_gate);

			error = zynqmp_clk_composite_register(sc->clkdom,
			    &clkdef.composite);
			break;
		default:
			device_printf(sc->dev, "unhandled topo type=%d\n",
			    topos[0] & GET_TOPO_TYPE_MASK);
			goto fail;
			break;
		}

		if (error) {
			device_printf(sc->dev,
			    "failed to register clk id=%d error=%d\n", id,
			    error);
			goto fail;
		}
	}

	error = clkdom_finit(sc->clkdom);
	if (error)
		device_printf(sc->dev, "clkdom_finit() failed. error=%d\n",
		    error);

	if (bootverbose)
		clkdom_dump(sc->clkdom);

fail:
	free(parent_names, M_ZCLK);

	return (error);
}

static int
zynqmp_clk_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ Clock Driver");

	return (BUS_PROBE_DEFAULT);
}

static int
zynqmp_clk_attach(device_t dev)
{
	struct zynqmp_clk_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	int error;

	sc->dev = dev;

	error = zynqmp_clk_register_clocks(sc);
	if (error)
		return (error);

	/* Register node to this driver. */
	OF_device_register_xref(OF_xref_from_node(node), dev);

	ZCLK_LOCK_INIT(sc);

	return (0);
}

static int
zynqmp_clk_detach(device_t dev)
{
	struct zynqmp_clk_softc *sc = device_get_softc(dev);

	ZCLK_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_clk_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		zynqmp_clk_probe),
	DEVMETHOD(device_attach,	zynqmp_clk_attach),
	DEVMETHOD(device_detach,	zynqmp_clk_detach),

	DEVMETHOD_END
};

static driver_t zynqmp_clk_driver = {
	"zynqmp_clk",
	zynqmp_clk_methods,
	sizeof(struct zynqmp_clk_softc),
};
static devclass_t zynqmp_clk_devclass;

EARLY_DRIVER_MODULE(zynqmp_clk, simplebus, zynqmp_clk_driver,
    zynqmp_clk_devclass, NULL, NULL,
    BUS_PASS_TIMER + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(zynqmp_clk, 1);
MODULE_DEPEND(zynqmp_clk, zynqmp_pm, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
