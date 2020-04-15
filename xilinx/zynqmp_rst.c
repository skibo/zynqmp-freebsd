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

#include <arm64/xilinx/zynqmp_pm.h>

#define ZRST_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	ZRST_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZRST_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "rst", MTX_DEF)
#define ZRST_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

struct zynqmp_rst_softc {
	device_t	dev;
	struct mtx	sc_mtx;
};

static struct ofw_compat_data compat_data[] = {
	{"xlnx,zynqmp-reset",		1},
	{NULL,				0}
};

static char *reset_names[] = {
	[ZYNQMP_PM_RESET_PCIE_CFG - ZYNQMP_PM_RESET_BASE] =	"PCIE_CFG",
	[ZYNQMP_PM_RESET_PCIE_BRIDGE - ZYNQMP_PM_RESET_BASE] =	"PCIE_BRIDGE",
	[ZYNQMP_PM_RESET_PCIE_CTRL - ZYNQMP_PM_RESET_BASE] =	"PCIE_CTRL",
	[ZYNQMP_PM_RESET_DP - ZYNQMP_PM_RESET_BASE] =		"DP",
	[ZYNQMP_PM_RESET_SWDT_CRF - ZYNQMP_PM_RESET_BASE] =	"SWDT_CRF",
	[ZYNQMP_PM_RESET_AFI_FM5 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM5",
	[ZYNQMP_PM_RESET_AFI_FM4 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM4",
	[ZYNQMP_PM_RESET_AFI_FM3 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM3",
	[ZYNQMP_PM_RESET_AFI_FM2 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM2",
	[ZYNQMP_PM_RESET_AFI_FM1 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM1",
	[ZYNQMP_PM_RESET_AFI_FM0 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM0",
	[ZYNQMP_PM_RESET_GDMA - ZYNQMP_PM_RESET_BASE] =		"GDMA",
	[ZYNQMP_PM_RESET_GPU_PP1 - ZYNQMP_PM_RESET_BASE] =	"GPU_PP1",
	[ZYNQMP_PM_RESET_GPU_PP0 - ZYNQMP_PM_RESET_BASE] =	"GPU_PP0",
	[ZYNQMP_PM_RESET_GPU - ZYNQMP_PM_RESET_BASE] =		"GPU",
	[ZYNQMP_PM_RESET_GT - ZYNQMP_PM_RESET_BASE] =		"GT",
	[ZYNQMP_PM_RESET_SATA - ZYNQMP_PM_RESET_BASE] =		"SATA",
	[ZYNQMP_PM_RESET_ACPU3_PWRON - ZYNQMP_PM_RESET_BASE] =	"ACPU3_PWRON",
	[ZYNQMP_PM_RESET_ACPU2_PWRON - ZYNQMP_PM_RESET_BASE] =	"ACPU2_PWRON",
	[ZYNQMP_PM_RESET_ACPU1_PWRON - ZYNQMP_PM_RESET_BASE] =	"ACPU1_PWRON",
	[ZYNQMP_PM_RESET_ACPU0_PWRON - ZYNQMP_PM_RESET_BASE] =	"ACPU0_PWRON",
	[ZYNQMP_PM_RESET_APU_L2 - ZYNQMP_PM_RESET_BASE] =	"APU_L2",
	[ZYNQMP_PM_RESET_ACPU3 - ZYNQMP_PM_RESET_BASE] =	"ACPU3",
	[ZYNQMP_PM_RESET_ACPU2 - ZYNQMP_PM_RESET_BASE] =	"ACPU2",
	[ZYNQMP_PM_RESET_ACPU1 - ZYNQMP_PM_RESET_BASE] =	"ACPU1",
	[ZYNQMP_PM_RESET_ACPU0 - ZYNQMP_PM_RESET_BASE] =	"ACPU0",
	[ZYNQMP_PM_RESET_DDR - ZYNQMP_PM_RESET_BASE] =		"DDR",
	[ZYNQMP_PM_RESET_APM_FPD - ZYNQMP_PM_RESET_BASE] =	"APM_FPD",
	[ZYNQMP_PM_RESET_SOFT - ZYNQMP_PM_RESET_BASE] =		"SOFT",
	[ZYNQMP_PM_RESET_GEM0 - ZYNQMP_PM_RESET_BASE] =		"GEM0",
	[ZYNQMP_PM_RESET_GEM1 - ZYNQMP_PM_RESET_BASE] =		"GEM1",
	[ZYNQMP_PM_RESET_GEM2 - ZYNQMP_PM_RESET_BASE] =		"GEM2",
	[ZYNQMP_PM_RESET_GEM3 - ZYNQMP_PM_RESET_BASE] =		"GEM3",
	[ZYNQMP_PM_RESET_QSPI - ZYNQMP_PM_RESET_BASE] =		"QSPI",
	[ZYNQMP_PM_RESET_UART0 - ZYNQMP_PM_RESET_BASE] =	"UART0",
	[ZYNQMP_PM_RESET_UART1 - ZYNQMP_PM_RESET_BASE] =	"UART1",
	[ZYNQMP_PM_RESET_SPI0 - ZYNQMP_PM_RESET_BASE] =		"SPI0",
	[ZYNQMP_PM_RESET_SPI1 - ZYNQMP_PM_RESET_BASE] =		"SPI1",
	[ZYNQMP_PM_RESET_SDIO0 - ZYNQMP_PM_RESET_BASE] =	"SDIO0",
	[ZYNQMP_PM_RESET_SDIO1 - ZYNQMP_PM_RESET_BASE] =	"SDIO1",
	[ZYNQMP_PM_RESET_CAN0 - ZYNQMP_PM_RESET_BASE] =		"CAN0",
	[ZYNQMP_PM_RESET_CAN1 - ZYNQMP_PM_RESET_BASE] =		"CAN1",
	[ZYNQMP_PM_RESET_I2C0 - ZYNQMP_PM_RESET_BASE] =		"I2C0",
	[ZYNQMP_PM_RESET_I2C1 - ZYNQMP_PM_RESET_BASE] =		"I2C1",
	[ZYNQMP_PM_RESET_TTC0 - ZYNQMP_PM_RESET_BASE] =		"TTC0",
	[ZYNQMP_PM_RESET_TTC1 - ZYNQMP_PM_RESET_BASE] =		"TTC1",
	[ZYNQMP_PM_RESET_TTC2 - ZYNQMP_PM_RESET_BASE] =		"TTC2",
	[ZYNQMP_PM_RESET_TTC3 - ZYNQMP_PM_RESET_BASE] =		"TTC3",
	[ZYNQMP_PM_RESET_SWDT_CRL - ZYNQMP_PM_RESET_BASE] =	"SWDT_CRL",
	[ZYNQMP_PM_RESET_NAND - ZYNQMP_PM_RESET_BASE] =		"NAND",
	[ZYNQMP_PM_RESET_ADMA - ZYNQMP_PM_RESET_BASE] =		"ADMA",
	[ZYNQMP_PM_RESET_GPIO - ZYNQMP_PM_RESET_BASE] =		"GPIO",
	[ZYNQMP_PM_RESET_IOU_CC - ZYNQMP_PM_RESET_BASE] =	"IOU_CC",
	[ZYNQMP_PM_RESET_TIMESTAMP - ZYNQMP_PM_RESET_BASE] =	"TIMESTAMP",
	[ZYNQMP_PM_RESET_RPU_R50 - ZYNQMP_PM_RESET_BASE] =	"RPU_R50",
	[ZYNQMP_PM_RESET_RPU_R51 - ZYNQMP_PM_RESET_BASE] =	"RPU_R51",
	[ZYNQMP_PM_RESET_RPU_AMBA - ZYNQMP_PM_RESET_BASE] =	"RPU_AMBA",
	[ZYNQMP_PM_RESET_OCM - ZYNQMP_PM_RESET_BASE] =		"OCM",
	[ZYNQMP_PM_RESET_RPU_PGE - ZYNQMP_PM_RESET_BASE] =	"RPU_PGE",
	[ZYNQMP_PM_RESET_USB0_CORERESET - ZYNQMP_PM_RESET_BASE] =
	    "USB0_CORERESET",
	[ZYNQMP_PM_RESET_USB1_CORERESET - ZYNQMP_PM_RESET_BASE] =
	    "USB1_CORERESET",
	[ZYNQMP_PM_RESET_USB0_HIBERRESET - ZYNQMP_PM_RESET_BASE] =
	    "USB0_HIBERRESET",
	[ZYNQMP_PM_RESET_USB1_HIBERRESET - ZYNQMP_PM_RESET_BASE] =
	    "USB1_HIBERRESET",
	[ZYNQMP_PM_RESET_USB0_APB - ZYNQMP_PM_RESET_BASE] =	"USB0_APB",
	[ZYNQMP_PM_RESET_USB1_APB - ZYNQMP_PM_RESET_BASE] =	"USB1_APB",
	[ZYNQMP_PM_RESET_IPI - ZYNQMP_PM_RESET_BASE] =		"IPI",
	[ZYNQMP_PM_RESET_APM_LPD - ZYNQMP_PM_RESET_BASE] =	"APM_LPD",
	[ZYNQMP_PM_RESET_RTS - ZYNQMP_PM_RESET_BASE] =		"RTS",
	[ZYNQMP_PM_RESET_SYSMON - ZYNQMP_PM_RESET_BASE] =	"SYSMON",
	[ZYNQMP_PM_RESET_AFI_FM6 - ZYNQMP_PM_RESET_BASE] =	"AFI_FM6",
	[ZYNQMP_PM_RESET_LPD_SWDT - ZYNQMP_PM_RESET_BASE] =	"LPD_SWDT",
	[ZYNQMP_PM_RESET_FPD - ZYNQMP_PM_RESET_BASE] =		"FPD",
	[ZYNQMP_PM_RESET_RPU_DBG1 - ZYNQMP_PM_RESET_BASE] =	"RPU_DBG1",
	[ZYNQMP_PM_RESET_RPU_DBG0 - ZYNQMP_PM_RESET_BASE] =	"RPU_DBG0",
	[ZYNQMP_PM_RESET_DBG_LPD - ZYNQMP_PM_RESET_BASE] =	"DBG_LPD",
	[ZYNQMP_PM_RESET_DBG_FPD - ZYNQMP_PM_RESET_BASE] =	"DBG_FPD",
	[ZYNQMP_PM_RESET_APLL - ZYNQMP_PM_RESET_BASE] =		"APLL",
	[ZYNQMP_PM_RESET_DPLL - ZYNQMP_PM_RESET_BASE] =		"DPLL",
	[ZYNQMP_PM_RESET_VPLL - ZYNQMP_PM_RESET_BASE] =		"VPLL",
	[ZYNQMP_PM_RESET_IOPLL - ZYNQMP_PM_RESET_BASE] =	"IOPLL",
	[ZYNQMP_PM_RESET_RPLL - ZYNQMP_PM_RESET_BASE] =		"RPLL",
	[ZYNQMP_PM_RESET_GPO3_PL(0) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_0",
	[ZYNQMP_PM_RESET_GPO3_PL(1) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_1",
	[ZYNQMP_PM_RESET_GPO3_PL(2) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_2",
	[ZYNQMP_PM_RESET_GPO3_PL(3) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_3",
	[ZYNQMP_PM_RESET_GPO3_PL(4) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_4",
	[ZYNQMP_PM_RESET_GPO3_PL(5) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_5",
	[ZYNQMP_PM_RESET_GPO3_PL(6) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_6",
	[ZYNQMP_PM_RESET_GPO3_PL(7) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_7",
	[ZYNQMP_PM_RESET_GPO3_PL(8) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_8",
	[ZYNQMP_PM_RESET_GPO3_PL(9) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_9",
	[ZYNQMP_PM_RESET_GPO3_PL(10) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_10",
	[ZYNQMP_PM_RESET_GPO3_PL(11) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_11",
	[ZYNQMP_PM_RESET_GPO3_PL(12) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_12",
	[ZYNQMP_PM_RESET_GPO3_PL(13) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_13",
	[ZYNQMP_PM_RESET_GPO3_PL(14) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_14",
	[ZYNQMP_PM_RESET_GPO3_PL(15) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_15",
	[ZYNQMP_PM_RESET_GPO3_PL(16) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_16",
	[ZYNQMP_PM_RESET_GPO3_PL(17) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_17",
	[ZYNQMP_PM_RESET_GPO3_PL(18) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_18",
	[ZYNQMP_PM_RESET_GPO3_PL(19) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_19",
	[ZYNQMP_PM_RESET_GPO3_PL(20) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_20",
	[ZYNQMP_PM_RESET_GPO3_PL(21) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_21",
	[ZYNQMP_PM_RESET_GPO3_PL(22) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_22",
	[ZYNQMP_PM_RESET_GPO3_PL(23) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_23",
	[ZYNQMP_PM_RESET_GPO3_PL(24) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_24",
	[ZYNQMP_PM_RESET_GPO3_PL(25) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_25",
	[ZYNQMP_PM_RESET_GPO3_PL(26) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_26",
	[ZYNQMP_PM_RESET_GPO3_PL(27) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_27",
	[ZYNQMP_PM_RESET_GPO3_PL(28) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_28",
	[ZYNQMP_PM_RESET_GPO3_PL(29) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_29",
	[ZYNQMP_PM_RESET_GPO3_PL(30) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_30",
	[ZYNQMP_PM_RESET_GPO3_PL(31) - ZYNQMP_PM_RESET_BASE] =	"GPO3_PL_31",
	[ZYNQMP_PM_RESET_RPU_LS - ZYNQMP_PM_RESET_BASE] =	"RPU_LS",
	[ZYNQMP_PM_RESET_PS_ONLY - ZYNQMP_PM_RESET_BASE] =	"PS_ONLY",
	[ZYNQMP_PM_RESET_PL - ZYNQMP_PM_RESET_BASE] =		"PL",
	[ZYNQMP_PM_RESET_LAST - ZYNQMP_PM_RESET_BASE + 1] = NULL

	/* XXX: don't know why these don't work and hang the system.
	[ZYNQMP_PM_RESET_GPIO5_EMIO_92 - ZYNQMP_PM_RESET_BASE] =
		"GPIO5_EMIO_92",
	[ZYNQMP_PM_RESET_GPIO5_EMIO_93 - ZYNQMP_PM_RESET_BASE] =
		"GPIO5_EMIO_93",
	[ZYNQMP_PM_RESET_GPIO5_EMIO_94 - ZYNQMP_PM_RESET_BASE] =
		"GPIO5_EMIO_94",
	[ZYNQMP_PM_RESET_GPIO5_EMIO_95 - ZYNQMP_PM_RESET_BASE] =
		"GPIO5_EMIO_95", */
};

/* Totally for debuggin. */
static int
zynqmp_rst_dump(SYSCTL_HANDLER_ARGS)
{
	int error, i;
	/* struct zynqmp_rst_softc *sc = (struct zynqmp_rst_softc *)arg1; */
	int status;

	error = sysctl_wire_old_buffer(req, sizeof(int));
	if (error == 0) {
		i = 0;
		error = sysctl_handle_int(oidp, &i, 0, req);
	}
	if (error || req->newptr == NULL)
		return (error);

	for (i = 0; reset_names[i] != NULL; i++) {
		if (zynqmp_pm_reset_get_status(ZYNQMP_PM_RESET_BASE + i,
			&status)) {
			printf("%s: XXX: reset_get_status err for %s.\n",
			    __func__, reset_names[i]);
			continue;
		}

		printf("\tReset %s status = %d\n", reset_names[i], status);
	}

	return (0);
}

static void
zynqmp_rst_addsysctls(struct zynqmp_rst_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child;

	ctx = device_get_sysctl_ctx(sc->dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	if (ctx == NULL || child == NULL) {
		device_printf(sc->dev, "could not add sysctls\n");
		return;
	}

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "_dump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_SECURE,
	    sc, 0, zynqmp_rst_dump, "I", "dump reset info");
}

static int
zynqmp_rst_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ Reset Driver");

	return (0);
}

static int
zynqmp_rst_attach(device_t dev)
{
	struct zynqmp_rst_softc *sc = device_get_softc(dev);

	sc->dev = dev;

	zynqmp_rst_addsysctls(sc);

	ZRST_LOCK_INIT(sc);

	return (0);
}

static int
zynqmp_rst_detach(device_t dev)
{
	struct zynqmp_rst_softc *sc = device_get_softc(dev);

	ZRST_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_rst_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		zynqmp_rst_probe),
	DEVMETHOD(device_attach,	zynqmp_rst_attach),
	DEVMETHOD(device_detach,	zynqmp_rst_detach),

	DEVMETHOD_END
};

static driver_t zynqmp_rst_driver = {
	"zynqmp_rst",
	zynqmp_rst_methods,
	sizeof(struct zynqmp_rst_softc),
};
static devclass_t zynqmp_rst_devclass;

EARLY_DRIVER_MODULE(zynqmp_rst, simplebus, zynqmp_rst_driver,
    zynqmp_rst_devclass, NULL, NULL,
    BUS_PASS_TIMER + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(zynqmp_rst, 1);
MODULE_DEPEND(zynqmp_rst, zynqmp_pm, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
