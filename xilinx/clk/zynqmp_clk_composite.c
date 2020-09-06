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
#include <arm64/xilinx/clk/zynqmp_clk_composite.h>

#include "clkdev_if.h"

#ifdef CLKDEBUG
#define DPRINTF(...)	do { printf(__VA_ARGS__); } while (0)
#else
#define DPRINTF(...)	do { } while (0)
#endif

struct zynqmp_clk_composite_sc {
	int		id;
	int		divisors;
	int		nparents;
	bool		has_gate;
	uint32_t	div0_max;
	uint32_t	div1_max;
};

static int
zynqmp_clk_composite_init(struct clknode *clk, device_t dev)
{
	struct zynqmp_clk_composite_sc *sc = clknode_get_softc(clk);
	int idx = 0;
	int error;

	if (sc->nparents > 1) {
		error = zynqmp_pm_clock_get_parent(sc->id, &idx);
		if (error) {
			printf("%s: cannot get parent. id=%d\n", __func__,
			    sc->id);
			return (error);
		}
	}
	clknode_init_parent_idx(clk, idx);

	return (0);
}

static int
zynqmp_clk_composite_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct zynqmp_clk_composite_sc *sc = clknode_get_softc(clk);
	int error;
	uint32_t div;

	DPRINTF("%s: id=%d freq=%ld\n", __func__, sc->id, *freq);

	if (sc->divisors > 0) {
		error = zynqmp_pm_clock_get_divider(sc->id, &div);
		if (error) {
			printf("%s: cannot get divider. id=%d\n", __func__,
			    sc->id);
			return (error);
		}

		/* DIV0 */
		*freq = (*freq * 2 / (div & 0xffff) + 1) / 2;

		/* DIV1 */
		if (sc->divisors > 1)
			*freq = (*freq * 2 / (div >> 16) + 1) / 2;
	}

	return (0);
}

static int
zynqmp_clk_composite_set_gate(struct clknode *clk, bool enable)
{
	struct zynqmp_clk_composite_sc *sc = clknode_get_softc(clk);
	int error;

	DPRINTF("%s: id=%d enable=%d\n", __func__, sc->id, enable);

	/* Ignore if clock doesn't have a gate. */
	if (!sc->has_gate)
		return (0);

	if (enable)
		error = zynqmp_pm_clock_enable(sc->id);
	else
		error = zynqmp_pm_clock_disable(sc->id);

	return (error);
}

static int
zynqmp_clk_composite_set_mux(struct clknode *clk, int index)
{
	struct zynqmp_clk_composite_sc *sc = clknode_get_softc(clk);

	DPRINTF("%s: id=%d index=%d\n", __func__, sc->id, index);

	if (index >= sc->nparents)
		return (EINVAL);

	return zynqmp_pm_clock_set_parent(sc->id, index);
}


static int
zynqmp_clk_composite_set_freq(struct clknode *clk, uint64_t fin,
    uint64_t *fout, int flags, int *done)
{
	struct zynqmp_clk_composite_sc *sc = clknode_get_softc(clk);
	int error;
	uint64_t freq_o;
	u_int div0, div1, freq_err;
	u_int best_div0, best_div1, best_err;

	DPRINTF("%s: id=%d fin=%ld *fout=%ld flags=0x%x\n", __func__,
	    sc->id, fin, *fout, flags);

	*done = 0;

	if (sc->divisors == 2) {
		/* Find best matching divisor pair. */
		best_div0 = 0;
		best_div1 = 0;
		best_err = ~0U;

		for (div0 = 1; div0 <= sc->div0_max; div0++) {
			div1 = (fin * 2 / *fout / div0 + 1) / 2;
			if (div1 < 1 || div1 > sc->div1_max)
				continue;

			freq_o = fin / div0 / div1;
			freq_err = abs(freq_o - *fout);
			if (freq_err < best_err) {
				best_err = freq_err;
				best_div0 = div0;
				best_div1 = div1;
			}
			if (freq_err == 0)
				break;
		}

		DPRINTF("%s: fin=%ld freq=%ld best_div0=%u best_div1=%u\n",
		    __func__, fin, *fout, best_div0, best_div1);

		if (best_div0 == 0)
			return (EINVAL);

		if ((flags & CLK_SET_DRYRUN) == 0) {
			error = zynqmp_pm_clock_set_divider(sc->id, 0,
			    best_div0);
			if (error) {
				printf("%s: could not set divisor0.\n",
				    __func__);
				return (error);
			};
			error = zynqmp_pm_clock_set_divider(sc->id, 1,
			    best_div1);
			if (error) {
				printf("%s: could not set divisor1.\n",
				    __func__);
				return (error);
			}
		}

		*done = 1;
	} else if (sc->divisors == 1) {
		/* Single divisor. */
		div0 = (2 * fin / *fout + 1) / 2;
		if (div0 < 1)
			div0 = 1;
		else if (div0 > sc->div0_max)
			div0 = sc->div0_max;

		DPRINTF("%s: fin=%ld freq=%ld div0=%u\n", __func__, fin,
		    *fout, div0);

		if ((flags & CLK_SET_DRYRUN) == 0) {
			error = zynqmp_pm_clock_set_divider(sc->id, 0, div0);
			if (error) {
				printf("%s: could not set divisor.\n",
				    __func__);
				return (error);
			}
		}

		*done = 1;
	}

	return (0);
}


static clknode_method_t zynqmp_clk_composite_clknode_methods[] = {
	CLKNODEMETHOD(clknode_init,	    zynqmp_clk_composite_init),
	CLKNODEMETHOD(clknode_recalc_freq,  zynqmp_clk_composite_recalc_freq),
	CLKNODEMETHOD(clknode_set_gate,	    zynqmp_clk_composite_set_gate),
	CLKNODEMETHOD(clknode_set_mux,	    zynqmp_clk_composite_set_mux),
	CLKNODEMETHOD(clknode_set_freq,	    zynqmp_clk_composite_set_freq),

	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(zynqmp_clk_composite_clknode,
    zynqmp_clk_composite_clknode_class, zynqmp_clk_composite_clknode_methods,
    sizeof(struct zynqmp_clk_composite_sc), clknode_class);

int
zynqmp_clk_composite_register(struct clkdom *clkdom,
    struct zynqmp_clk_composite_def *clkdef)
{
	struct clknode *clk;
	struct zynqmp_clk_composite_sc *sc;

	clk = clknode_create(clkdom, &zynqmp_clk_composite_clknode_class,
	    &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->id = clkdef->clkdef.id;
	sc->nparents = clkdef->clkdef.parent_cnt;
	sc->has_gate = clkdef->has_gate;
	sc->divisors = clkdef->divisors;
	sc->div0_max = clkdef->div0_max;
	sc->div1_max = clkdef->div1_max;

	clknode_register(clkdom, clk);

	return (0);
}
