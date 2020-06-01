/*-
 * Copyright (c) 2020 Thomas Skibo. <ThomasSkibo@yahoo.com>
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
#include <dev/extres/clk/clk_div.h>
#include <dev/extres/clk/clk_mux.h>
#include <dev/extres/clk/clk_fixed.h>
#include <dev/extres/clk/clk_gate.h>

#include <arm64/xilinx/zynqmp_pm.h>

#define ZCLK_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	ZCLK_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZCLK_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "clk", MTX_DEF)
#define ZCLK_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

#ifdef ZCLK_DEBUG
static int zynqmp_clk_debug = ZCLK_DEBUG;
#define DPRINTF(...) do {		\
	if (zynqmp_clk_debug)		\
		printf(__VA_ARGS__);	\
	} while (0)
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

	int		nclks;
	struct clknode	**clknodes;
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

/* Totally for debuggin. */
static int
zynqmp_clk_dump(SYSCTL_HANDLER_ARGS)
{
	struct zynqmp_clk_softc *sc = (struct zynqmp_clk_softc *)arg1;
	int error, i;

	error = sysctl_wire_old_buffer(req, sizeof(int));
	if (error == 0) {
		i = 0;
		error = sysctl_handle_int(oidp, &i, 0, req);
	}
	if (error || req->newptr == NULL)
		return (error);

	clkdom_dump(sc->clkdom);

	return (0);
}

/* Totally for debuggin.  Get and set pl0 clock (fpga clock 0). */
static int
zynqmp_clk_test_set_pl0(SYSCTL_HANDLER_ARGS)
{
	struct zynqmp_clk_softc *sc = (struct zynqmp_clk_softc *)arg1;
	int error, freq;
	uint64_t freq64;

	error = sysctl_wire_old_buffer(req, sizeof(int));
	if (error == 0) {
		clkdom_xlock(sc->clkdom);
		(void)clknode_get_freq(sc->clknodes[71], &freq64);
		clkdom_unlock(sc->clkdom);
		freq = (int)freq64;
		error = sysctl_handle_int(oidp, &freq, 0, req);
	}
	if (error || req->newptr == NULL)
		return (error);

	freq64 = freq;
	clkdom_xlock(sc->clkdom);
	error = clknode_set_freq(sc->clknodes[71], freq64, 0, 0);
	clkdom_unlock(sc->clkdom);

	return (error);
}

/* Totally for debuggin.  Get and set dp video ref clock. */
static int
zynqmp_clk_test_set_vid(SYSCTL_HANDLER_ARGS)
{
	struct zynqmp_clk_softc *sc = (struct zynqmp_clk_softc *)arg1;
	int error, freq;
	uint64_t freq64;

	error = sysctl_wire_old_buffer(req, sizeof(int));
	if (error == 0) {
		clkdom_xlock(sc->clkdom);
		(void)clknode_get_freq(sc->clknodes[16], &freq64);
		clkdom_unlock(sc->clkdom);
		freq = (int)freq64;
		error = sysctl_handle_int(oidp, &freq, 0, req);
	}
	if (error || req->newptr == NULL)
		return (error);

	freq64 = freq;
	clkdom_xlock(sc->clkdom);
	error = clknode_set_freq(sc->clknodes[16], freq64, 0, 0);
	clkdom_unlock(sc->clkdom);

	return (error);
}

static void
zynqmp_clk_addsysctls(struct zynqmp_clk_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child;

	ctx = device_get_sysctl_ctx(sc->dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	if (ctx == NULL || child == NULL) {
		device_printf(sc->dev, "could not add sysctls\n");
		return;
	}

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "nclocks", CTLFLAG_RD,
	    &sc->nclks, 0, "Number of clocks");

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "_dump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_SECURE,
	    sc, 0, zynqmp_clk_dump, "I", "dump clock info");

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "_pl0freq",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_SECURE,
	    sc, 0, zynqmp_clk_test_set_pl0, "I", "test setting pl0 clk freq");

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "_vidfreq",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_SECURE,
	    sc, 0, zynqmp_clk_test_set_vid, "I", "test setting video clock");
}

/***************************************************************************/

/* Declare zynqmp_clk_clknode_class. */

struct zynqmp_clk_clknode_sc {
	device_t	dev;
	int		id;
	int		pll_id;
	char		name[CLK_MAX_NAME_LEN];
	int		nparents;
	uint32_t	topos[MAX_CLK_TOPO_NODES];
	int		nodes;
};

#ifdef ZCLK_DEBUG
static char *topo_type_names[] = { "invalid", "mux", "pll", "fixed",
				   "div0", "div1", "gate", "?" };
#endif

/* Map PLL names to pmu node ids.  See comment in zynqmp_clk_init(). */
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

static int
zynqmp_clk_init(struct clknode *clk, device_t dev)
{
	struct zynqmp_clk_clknode_sc *sc = clknode_get_softc(clk);
	int i, j;
	int err;
	int parent_idx = 0;

	sc->dev = dev;

#ifdef ZCLK_DEBUG
	if (zynqmp_clk_debug > 1) {
		/* Dump topology. */
		DPRINTF("%s: id=%d name=%s nodes=%d\n", __func__, sc->id,
		    sc->name, sc->nodes);
		for (i = 0; i < sc->nodes; i++)
			DPRINTF("\ttype=%s\n", topo_type_names[sc->topos[i] &
				GET_TOPO_TYPE_MASK]);
	}
#endif

	/*
	 * Look up PLL node ids based upon the name of the clock.
	 * XXX This is a hack necessary because I don't see any way to retrieve
	 * the PLL id from a clock id and the routines to get pll parameters
	 * through the clock id DON'T WORK.
	 */
	for (i = 0; i < sc->nodes; i++)
		if ((sc->topos[i] & GET_TOPO_TYPE_MASK) == GET_TOPO_TYPE_PLL) {
			for (j = 0; pll_ids[j].name != NULL; j++)
				if (strcmp(pll_ids[j].name, sc->name) == 0) {
					sc->pll_id = pll_ids[j].node_id;
					break;
				}
			break;
		}

	if (sc->nparents > 1) {
		err = zynqmp_pm_clock_get_parent(sc->id, &parent_idx);
		if (err)
			return (err);
	}

	clknode_init_parent_idx(clk, parent_idx);

	return (0);
}

static int
zynqmp_clk_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct zynqmp_clk_clknode_sc *sc = clknode_get_softc(clk);
	int i;
	int err;
	uint32_t mult;
	uint32_t div;
	int mode;
	uint32_t fbdiv;
	uint32_t frac;

	DPRINTF("%s: id=%d name=%s freq=%ld\n", __func__, sc->id, sc->name,
	    *freq);

	for (i = 0; i < sc->nodes; i++)
		switch (sc->topos[i] & GET_TOPO_TYPE_MASK) {
		case GET_TOPO_TYPE_FIXED:
			err = zynqmp_clk_get_fixed_factor_params(sc->id,
			    &mult, &div);
			if (err)
				return (err);
			DPRINTF("\tFIXED mult=%d div=%d\n", mult, div);
			*freq = (*freq * 2 * mult / div + 1) / 2;
			break;
		case GET_TOPO_TYPE_PLL:
			err = zynqmp_pm_pll_get_param(sc->pll_id,
			    PM_PLL_PARAM_FBDIV, &fbdiv);
			if (err) {
				printf("%s: can't get pll param: err=%d\n",
				    __func__, err);
				return (err);
			}

			err = zynqmp_pm_pll_get_mode(sc->pll_id, &mode);
			if (err) {
				printf("%s: can't get pll mode: err=%d\n",
				    __func__, err);
				return (err);
			}

			if (mode == PM_PLL_MODE_FRAC) {
				/* Fractional mode */
				err = zynqmp_pm_pll_get_param(sc->pll_id,
				    PM_PLL_PARAM_DATA, &frac);
				if (err) {
					printf("%s: can't get pll param: "
					    "err=%d\n", __func__, err);
					return (err);
				}

				DPRINTF("\tPLL FRAC fbdiv=%d frac=0x%x\n",
				    fbdiv, frac);

				*freq = (((*freq * fbdiv) << 16) +
				    (*freq * frac) + (1 << 15)) >> 16;
			} else {
				/* Integer mode */
				DPRINTF("\tPLL fbdiv=%d\n", fbdiv);
				*freq = (*freq * 2 * fbdiv + 1) / 2;
			}

			/* Round to nearest khz. XXX: doesn't work */
			*freq = ((*freq + 500) / 1000) * 1000;
			break;
		case GET_TOPO_TYPE_DIV0:
		case GET_TOPO_TYPE_DIV1:
			err = zynqmp_pm_clock_get_divider(sc->id, &div);
			if (err)
				return (err);

			if ((sc->topos[i] & GET_TOPO_TYPE_MASK) ==
			    GET_TOPO_TYPE_DIV1) {
				div >>= 16;
				DPRINTF("\tDIV1: div=%d\n", div);
			} else {
				div &= 0xffff;
				DPRINTF("\tDIV0: div=%d\n", div);
			}

			*freq = (*freq * 2 / div + 1) / 2;
			break;
		}

	return (0);
}

static int
zynqmp_clk_set_gate(struct clknode *clk, bool enable)
{
	struct zynqmp_clk_clknode_sc *sc = clknode_get_softc(clk);
	int err;
	int i;

	DPRINTF("%s: id=%d name=%s enable=%d\n", __func__, sc->id, sc->name,
	    enable);

	/* Ignore if clock doesn't have a gate. */
	for (i = 0; i < sc->nodes; i++)
		if ((sc->topos[i] & GET_TOPO_TYPE_MASK) == GET_TOPO_TYPE_GATE)
			break;
	if (i >= sc->nodes)
		return (0);

	if (enable)
		err = zynqmp_pm_clock_enable(sc->id);
	else
		err = zynqmp_pm_clock_disable(sc->id);

	return (err);
}

static int
zynqmp_clk_set_mux(struct clknode *clk, int index)
{
	struct zynqmp_clk_clknode_sc *sc = clknode_get_softc(clk);

	DPRINTF("%s: id=%d name=%s index=%d\n", __func__, sc->id,
	    sc->name, index);

	if (index >= sc->nparents)
		return (EINVAL);

	return zynqmp_pm_clock_set_parent(sc->id, index);
}

static int
zynqmp_clk_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct zynqmp_clk_clknode_sc *sc = clknode_get_softc(clk);
	int i, error;
	uint64_t freq_o;
	uint32_t div0_max, div1_max;
	u_int div0, div1, freq_err;
	u_int best_div0, best_div1, best_err;

	DPRINTF("%s: id=%d name=%s fin=%ld flags=0x%x\n", __func__, sc->id,
	    sc->name, fin, flags);

	for (i = 0; i < sc->nodes; i++)
		switch (sc->topos[i] & GET_TOPO_TYPE_MASK) {
		case GET_TOPO_TYPE_PLL:
			/*
			 * For now, we cannot set PLLs.  Later, we'll want to
			 * be able to modify the video PLL.
			 */
			return (EINVAL);
		case GET_TOPO_TYPE_DIV0:
			error = zynqmp_clk_get_max_divisor(sc->id, 0,
			    &div0_max);
			DPRINTF("%s: div0_max=%d err=%d\n", __func__,
			    div0_max, error);
			if (error)
				return (error);
			if (i + 1 < sc->nodes && (sc->topos[i + 1] &
			    GET_TOPO_TYPE_MASK) == GET_TOPO_TYPE_DIV1) {
				/* Two divisors. */
				error = zynqmp_clk_get_max_divisor(sc->id, 1,
				    &div1_max);
				DPRINTF("%s: div1_max=%d err=%d\n", __func__,
				    div1_max, error);
				if (error)
					return (error);

				/* Find best matching divisor pair. */
				best_div0 = 0;
				best_div1 = 0;
				best_err = ~0U;

				for (div0 = 1; div0 <= div0_max; div0++) {
					div1 = (fin * 2 / *fout / div0 + 1) /
					    2;
					if (div1 < 1 || div1 > div1_max)
						continue;

					freq_o = fin / div0 / div1;
					freq_err = freq_o > *fout ?
					    freq_o - *fout : *fout - freq_o;
					if (freq_err < best_err) {
						best_err = freq_err;
						best_div0 = div0;
						best_div1 = div1;
					}
					if (freq_err == 0)
						break;
				}

				DPRINTF("%s: fin=%ld freq=%ld best_div0=%u "
				    "best_div1=%u\n", __func__, fin, *fout,
				    best_div0, best_div1);

				if (best_div0 == 0)
					return (EINVAL);

				if ((flags & CLK_SET_DRYRUN) == 0) {
					error = zynqmp_pm_clock_set_divider(
					    sc->id, 0, best_div0);
					if (error)
						return (error);
					error = zynqmp_pm_clock_set_divider(
					    sc->id, 1, best_div1);
					if (error)
						return (error);
				}

				*done = 1;
				i++;
			} else {
				/* Single divisor. */
				div0 = (2 * fin / *fout + 1) / 2;
				if (div0 < 1)
					div0 = 1;
				else if (div0 > div0_max)
					div0 = div0_max;
				DPRINTF("%s: fin=%ld freq=%ld div0=%u\n",
				    __func__, fin, *fout, div0);
				if ((flags & CLK_SET_DRYRUN) == 0) {
					error = zynqmp_pm_clock_set_divider(
					    sc->id, 0, div0);
					if (error)
						return (error);
				}
			}

			*done = 1;
			break;
		case GET_TOPO_TYPE_DIV1:
			printf("%s: DIV1 should come right after DIV0.\n",
			    __func__);
			return (EINVAL);
		case GET_TOPO_TYPE_FIXED:
			return (EINVAL);
		}

	return (0);
}

static clknode_method_t zynqmp_clk_clknode_methods[] = {
	CLKNODEMETHOD(clknode_init,		zynqmp_clk_init),
	CLKNODEMETHOD(clknode_recalc_freq,	zynqmp_clk_recalc_freq),
	CLKNODEMETHOD(clknode_set_gate,		zynqmp_clk_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		zynqmp_clk_set_mux),
	CLKNODEMETHOD(clknode_set_freq,		zynqmp_clk_set_freq),

	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(zynqmp_clk_clknode, zynqmp_clk_clknode_class,
    zynqmp_clk_clknode_methods, sizeof(struct zynqmp_clk_clknode_sc),
    clknode_class);


/***************************************************************************/

static void
zynqmp_clk_external(struct zynqmp_clk_softc *sc,
    struct clknode_init_def *clk_def)
{
	int err;
	struct clknode *clknode;
	clk_t clk;

	/*
	 * Check if clock is in "clock-names" property in which case it
	 * references a clock in another node (typically fixed-clock
	 * devices).
	 */
	err = clk_get_by_ofw_name(sc->dev, 0, clk_def->name, &clk);
	if (!err)
		return;

	/* Create a dead-end clock node. */
	clknode = clknode_create(sc->clkdom, &clknode_class, clk_def);
	clknode_register(sc->clkdom, clknode);
}

/* Query the platform firmware for the entire clock tree. */
static int
zynqmp_clk_register_clocks(struct zynqmp_clk_softc *sc)
{
	int id;
	int err;
	uint32_t retvs[RETV_CNT];
	uint32_t attrs;
	char name[CLK_MAX_NAME_LEN];
	uint32_t topos[MAX_CLK_TOPO_NODES];
	int nodes;
	char **parent_names;
	int nparents;
	struct clknode_init_def clk_def;
	struct zynqmp_clk_clknode_sc *clksc;
	struct clknode *clknode;
	uint64_t freq;

	sc->clkdom = clkdom_create(sc->dev);

	/* Get number of clocks. */
	if ((err = zynqmp_pm_query_data(PM_QID_CLOCK_GET_NUM_CLOCKS,
	    0, 0, 0, retvs)) < 0) {
		device_printf(sc->dev, "trouble querying num clocks.\n");
		return (err);
	}
	sc->nclks = retvs[1];

	/* Sanity check number of clocks. XXX: necessary? */
	if (sc->nclks > MAX_CLOCKS)
		return (E2BIG);
	else if (sc->nclks < 1)
		return (EINVAL);

	/* Allocate a table of clknode pointers. */
	sc->clknodes = malloc(sc->nclks * sizeof(struct clknode *), M_ZCLK,
	    M_NOWAIT | M_ZERO);
	if (sc->clknodes == NULL)
		return (ENOMEM);

	/*
	 * Allocate space for parent_names parameter to clknode_create().
	 * This space is released at the end of this function.
	 */
	parent_names = malloc(MAX_CLK_PARENTS * (sizeof(char *) +
		CLK_MAX_NAME_LEN), M_ZCLK, M_NOWAIT | M_ZERO);
	if (parent_names == NULL) {
		err = ENOMEM;
		goto fail;
	}

	for (id = 0; id < sc->nclks; id++) {
		/*
		 * XXX: Skip CLK_LPD_WDT because it is in the
		 * arm-trusted-firmware platform code but is not in PMU
		 * firmware so lots of calls to firmware fail.
		 */
		if (id == 112)
			continue;

		/* Get clock attributes. */
		err = zynqmp_clk_get_attrs(id, &attrs);
		if (err)
			goto fail;
		if (!(attrs & GET_ATTRS_VALID))
			continue;

		/* Get clock name. */
		err = zynqmp_clk_get_name(id, name);
		if (err)
			goto fail;

		memset(&clk_def, 0, sizeof(clk_def));
		clk_def.id = id;
		clk_def.name = name;

		if ((attrs & GET_ATTRS_EXTERNAL) != 0) {
			zynqmp_clk_external(sc, &clk_def);
			continue;
		}

		/* Get topology. */
		err = zynqmp_clk_get_topology(id, &nodes, topos);
		if (err)
			goto fail;

		/* Get parents. */
		err = zynqmp_clk_get_parents(id, parent_names, &nparents);
		if (err)
			goto fail;
		clk_def.parent_names = (const char **)(void *)parent_names;
		clk_def.parent_cnt = nparents;

#ifdef ZCLK_DEBUG
		DPRINTF("%s: id=%d name=%s nodes=%d nparents=%d\n",
		    __func__, id, name, nodes, nparents);
		if (zynqmp_clk_debug) {
			int i;

			DPRINTF("%s: parents: ", __func__);
			for (i = 0; i < nparents; i++)
				DPRINTF("%s ", parent_names[i]);
			DPRINTF("\n");
		}
#endif

		clknode = clknode_create(sc->clkdom, &zynqmp_clk_clknode_class,
		    &clk_def);
		clksc = clknode_get_softc(clknode);
		clksc->id = id;
		memcpy(clksc->topos, topos, nodes * sizeof(uint32_t));
		memcpy(clksc->name, name, CLK_MAX_NAME_LEN);
		clksc->nodes = nodes;
		clksc->nparents = nparents;
		clknode_register(sc->clkdom, clknode);

		sc->clknodes[id] = clknode;
	}

	err = clkdom_finit(sc->clkdom);
	if (err)
		goto fail;

	/* Fill out all the frequencies and enable counts. XXX */
	clkdom_xlock(sc->clkdom);
	for (id = 0; id < sc->nclks; id++)
		if (sc->clknodes[id]) {
			if (clknode_get_freq(sc->clknodes[id], &freq) != 0)
				DPRINTF("%s: couldn't get freq: name=%s\n",
				    __func__,
				    clknode_get_name(sc->clknodes[id]));
		}

	clkdom_unlock(sc->clkdom);

fail:
	if (err) {
		free(sc->clknodes, M_ZCLK);
		sc->clknodes = NULL;
	}
	/* free on success or failure. */
	free(parent_names, M_ZCLK);
	return (err);
}

static int
zynqmp_clk_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ Clock Driver");

	return (0);
}

static int
zynqmp_clk_attach(device_t dev)
{
	struct zynqmp_clk_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	int err;

	sc->dev = dev;

	err = zynqmp_clk_register_clocks(sc);
	if (err)
		return (err);

	zynqmp_clk_addsysctls(sc);

	/* Register node to this driver. */
	OF_device_register_xref(OF_xref_from_node(node), dev);

	ZCLK_LOCK_INIT(sc);

	return (0);
}

static int
zynqmp_clk_detach(device_t dev)
{
	struct zynqmp_clk_softc *sc = device_get_softc(dev);

	if (sc->clknodes != NULL) {
		free(sc->clknodes, M_ZCLK);
		sc->clknodes = NULL;
	}

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
