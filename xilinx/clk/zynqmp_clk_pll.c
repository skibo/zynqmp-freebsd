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
#include <sys/bus.h>

#include <dev/extres/clk/clk.h>

#include <arm64/xilinx/zynqmp_pm.h>
#include <arm64/xilinx/clk/zynqmp_clk_pll.h>

#include "clkdev_if.h"

#ifdef CLKDEBUG
#define DPRINTF(...)	do { printf(__VA_ARGS__); } while (0)
#else
#define DPRINTF(...)	do { } while (0)
#endif

struct zynqmp_clk_pll_sc {
	int		id;
	int		pll_id;
	bool		fractmode;
	uint32_t	fbdiv;
	uint32_t	fract;
};

static int
zynqmp_clk_pll_init(struct clknode *clk, device_t dev)
{
	struct zynqmp_clk_pll_sc *sc = clknode_get_softc(clk);
	int mode;
	int error;

	error = zynqmp_pm_pll_get_param(sc->pll_id, PM_PLL_PARAM_FBDIV,
	    &sc->fbdiv);
	if (error) {
		device_printf(dev, "could not get pll fbdiv.\n");
		return (error);
	}

	error = zynqmp_pm_pll_get_mode(sc->pll_id, &mode);
	if (error) {
		device_printf(dev, "could not get pll mode.\n");
		return (error);
	}

	if (mode == PM_PLL_MODE_FRAC) {
		error = zynqmp_pm_pll_get_param(sc->pll_id, PM_PLL_PARAM_DATA,
		    &sc->fract);
		if (error) {
			device_printf(dev, "could not get pll fract data.\n");
			return (error);
		}
		sc->fractmode = true;
	}

	clknode_init_parent_idx(clk, 0);

	return (0);
}

static int
zynqmp_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct zynqmp_clk_pll_sc *sc = clknode_get_softc(clk);

	DPRINTF("%s: pll_id=%d freq=%ld fractmode=%d fbdiv=%d fract=0x%x\n",
	    __func__, sc->pll_id, *freq, sc->fractmode, sc->fbdiv, sc->fract);

	if (sc->fractmode)
		/* Fractional mode */
		*freq = (((*freq * sc->fbdiv) << 16) +
		    (*freq * sc->fract) + (1 << 15)) >> 16;
	else
		/* Integer mode */
		*freq = (*freq * 2 * sc->fbdiv + 1) / 2;

	/* Round to nearest khz. XXX: doesn't work */
	*freq = ((*freq + 500) / 1000) * 1000;

	return (0);
}

static int
zynqmp_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
#ifdef CLKDEBUG
	struct zynqmp_clk_pll_sc *sc = clknode_get_softc(clk);
#endif

	DPRINTF("%s: pll_id=%d fin=%ld flags=0x%x\n", __func__, sc->pll_id,
	    fin, flags);

	/*
	 * For now, we cannot set PLLs.  Later, we'll want to
	 * be able to modify the video PLL.
	 */
	return (EINVAL);
}

static clknode_method_t zynqmp_clk_pll_clknode_methods[] = {
	CLKNODEMETHOD(clknode_init,		zynqmp_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	zynqmp_clk_pll_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		zynqmp_clk_pll_set_freq),

	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(zynqmp_clk_pll_clknode, zynqmp_clk_pll_clknode_class,
    zynqmp_clk_pll_clknode_methods, sizeof(struct zynqmp_clk_pll_sc),
    clknode_class);

int
zynqmp_clk_pll_register(struct clkdom *clkdom,
    struct zynqmp_clk_pll_def *clkdef)
{
	struct clknode *clk;
	struct zynqmp_clk_pll_sc *sc;

	clk = clknode_create(clkdom, &zynqmp_clk_pll_clknode_class,
	    &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->id = clkdef->clkdef.id;
	sc->pll_id = clkdef->pll_id;

	clknode_register(clkdom, clk);

	return (0);
}
