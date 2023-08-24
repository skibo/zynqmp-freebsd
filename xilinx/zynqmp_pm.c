/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018-2020 Thomas Skibo. <Thomas@skibo.net>
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

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/psci/smccc.h>

#include <arm64/xilinx/zynqmp_pm.h>

/*
 * This driver handles communication with the PMU (Platform Management Unit)
 * firmware that runs in a small Microblaze CPU in the Zynq UltraScale+.
 * It handles things like clocks, power, and resets.
 *
 * See: http://www.wiki.xilinx.com/PMU+Firmware
 *
 * Also See: Embedded Energy Management Interface.  EEMI API Reference Guide.
 * Xilinx UG1200 (v3.1) October 30, 2019.
 *
 */

#define PM_SIP_SVC			0xc2000000
#define PM_SET_SUSPEND_MODE		0xa02
#define PM_GET_TRUSTZONE_VERSION	0xa03

/* API ID's */
#define PM_GET_API_VERSION		1
#define PM_SET_CONFIGURATION		2
#define PM_GET_NODE_STATUS		3

#define PM_REQUEST_SUSPEND		6
#define PM_SELF_SUSPEND			7
#define PM_FORCE_POWERDOWN		8
#define PM_ABORT_SUSPEND		9
#define PM_REQUEST_WAKEUP		10
#define PM_SET_WAKEUP_SOURCE		11
#define PM_SYSTEM_SHUTDOWN		12

#define PM_REQUEST_NODE			13
#define PM_RELEASE_NODE			14
#define PM_SET_REQUIREMENT		15
#define PM_SET_MAX_LATENCY		16

#define PM_RESET_ASSERT			17
#define PM_RESET_GET_STATUS		18
#define PM_MMIO_WRITE			19
#define PM_MMIO_READ			20

#define PM_INIT_FINALIZE		21

#define PM_FPGA_LOAD			22
#define PM_FPGA_GET_STATUS		23
#define PM_GET_CHIPID			24

#define PM_PINCTRL_REQUEST		28

#define PM_IOCTL			34
#define PM_QUERY_DATA			35

#define PM_CLOCK_ENABLE			36
#define PM_CLOCK_DISABLE		37
#define PM_CLOCK_GETSTATE		38
#define PM_CLOCK_SETDIVIDER		39
#define PM_CLOCK_GETDIVIDER		40
#define PM_CLOCK_SETRATE		41
#define PM_CLOCK_GETRATE		42
#define PM_CLOCK_SETPARENT		43
#define PM_CLOCK_GETPARENT		44

#define PM_PLL_SET_PARAM		48
#define PM_PLL_GET_PARAM		49
#define PM_PLL_SET_MODE			50
#define PM_PLL_GET_MODE			51

#define PM_FPGA_READ			46


#define ZPM_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	ZPM_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZPM_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "pm", MTX_DEF)
#define ZPM_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

struct zynqmp_pm_softc {
	struct simplebus_softc simplebus_sc;
	device_t	dev;
	struct mtx	sc_mtx;
	bool		pm_fw_use_hvc;
};

static struct ofw_compat_data compat_data[] = {
	{"xlnx,zynqmp-firmware",	1},
	{"xlnx,zynqmp-pmu",		1},
	{NULL,				0}
};


static char *node_names[] = {
	[ZYNQMP_PM_NODE_APU] =		"APU",
	[ZYNQMP_PM_NODE_APU_0] =	"APU_0",
	[ZYNQMP_PM_NODE_APU_1] =	"APU_1",
	[ZYNQMP_PM_NODE_APU_2] =	"APU_2",
	[ZYNQMP_PM_NODE_APU_3] =	"APU_3",
	[ZYNQMP_PM_NODE_RPU] =		"RPU",
	[ZYNQMP_PM_NODE_RPU_0] =	"RPU_0",
	[ZYNQMP_PM_NODE_RPU_1] =	"RPU_1",
	[ZYNQMP_PM_NODE_PLD] =		"PLD",
	[ZYNQMP_PM_NODE_FPD] =		"FPD",
	[ZYNQMP_PM_NODE_OCM_BANK_0] =	"OCM_BANK_0",
	[ZYNQMP_PM_NODE_OCM_BANK_1] =	"OCM_BANK_1",
	[ZYNQMP_PM_NODE_OCM_BANK_2] =	"OCM_BANK_2",
	[ZYNQMP_PM_NODE_OCM_BANK_3] =	"OCM_BANK_3",
	[ZYNQMP_PM_NODE_TCM_0_A] =	"TCM_0_A",
	[ZYNQMP_PM_NODE_TCM_0_B] =	"TCM_0_B",
	[ZYNQMP_PM_NODE_TCM_1_A] =	"TCM_1_A",
	[ZYNQMP_PM_NODE_TCM_1_B] =	"TCM_1_B",
	[ZYNQMP_PM_NODE_L2] =		"L2",
	[ZYNQMP_PM_NODE_GPU_PP_0] =	"GPU_PP_0",
	[ZYNQMP_PM_NODE_GPU_PP_1] =	"GPU_PP_1",
	[ZYNQMP_PM_NODE_USB0] =		"USB0",
	[ZYNQMP_PM_NODE_USB1] =		"USB1",
	[ZYNQMP_PM_NODE_TTC_0] =	"TTC_0",
	[ZYNQMP_PM_NODE_TTC_1] =	"TTC_1",
	[ZYNQMP_PM_NODE_TTC_2] =	"TTC_2",
	[ZYNQMP_PM_NODE_TTC_3] =	"TTC_3",
	[ZYNQMP_PM_NODE_SATA] =		"SATA",
	[ZYNQMP_PM_NODE_GEM0] =		"GEM0",
	[ZYNQMP_PM_NODE_GEM1] =		"GEM1",
	[ZYNQMP_PM_NODE_GEM2] =		"GEM2",
	[ZYNQMP_PM_NODE_GEM3] =		"GEM3",
	[ZYNQMP_PM_NODE_UART0] =	"UART0",
	[ZYNQMP_PM_NODE_UART1] =	"UART1",
	[ZYNQMP_PM_NODE_SPI0] =		"SPI0",
	[ZYNQMP_PM_NODE_SPI1] =		"SPI1",
	[ZYNQMP_PM_NODE_I2C0] =		"I2C0",
	[ZYNQMP_PM_NODE_I2C1] =		"I2C1",
	[ZYNQMP_PM_NODE_SDIO0] =	"SDIO0",
	[ZYNQMP_PM_NODE_SDIO1] =	"SDIO1",
	[ZYNQMP_PM_NODE_DP] =		"DP",
	[ZYNQMP_PM_NODE_GDMA] =		"GDMA",
	[ZYNQMP_PM_NODE_ADMA] =		"ADMA",
	[ZYNQMP_PM_NODE_NAND] =		"NAND",
	[ZYNQMP_PM_NODE_QSPI] =		"QSPI",
	[ZYNQMP_PM_NODE_GPIO] =		"GPIO",
	[ZYNQMP_PM_NODE_CAN0] =		"CAN0",
	[ZYNQMP_PM_NODE_CAN1] =		"CAN1",
	[ZYNQMP_PM_NODE_EXTERN] =	"EXTERN",
	[ZYNQMP_PM_NODE_APLL] =		"APLL",
	[ZYNQMP_PM_NODE_VPLL] =		"VPLL",
	[ZYNQMP_PM_NODE_DPLL] =		"DPLL",
	[ZYNQMP_PM_NODE_RPLL] =		"RPLL",
	[ZYNQMP_PM_NODE_IOPLL] =	"IOPLL",
	[ZYNQMP_PM_NODE_DDR] =		"DDR",
	[ZYNQMP_PM_NODE_IPI_APU] =	"IPI_APU",
	[ZYNQMP_PM_NODE_IPI_RPU_0] =	"IPI_RPU_0",
	[ZYNQMP_PM_NODE_GPU] =		"GPU",
	[ZYNQMP_PM_NODE_PCIE] =		"PCIE",
	[ZYNQMP_PM_NODE_PCAP] =		"PCAP",
	[ZYNQMP_PM_NODE_RTC] =		"RTC",
	[ZYNQMP_PM_NODE_LPD] =		"LPD",
	[ZYNQMP_PM_NODE_VCU] =		"VCU",
	[ZYNQMP_PM_NODE_IPI_RPU_1] =	"IPI_RPU_1",
	[ZYNQMP_PM_NODE_IPI_PL0] =	"IPI_PL0",
	[ZYNQMP_PM_NODE_IPI_PL1] =	"IPI_PL1",
	[ZYNQMP_PM_NODE_IPI_PL2] =	"IPI_PL2",
	[ZYNQMP_PM_NODE_IPI_PL3] =	"IPI_PL3",
	[ZYNQMP_PM_NODE_PL] =		"PL",
	[ZYNQMP_PM_NODE_LAST + 1] = NULL
};

static struct zynqmp_pm_softc *zynqmp_pm_sc;

static int
zynqmp_pm_invoke_fn(uint32_t api_id, uint32_t arg0, uint32_t arg1,
		    uint32_t arg2, uint32_t arg3, uint32_t *retvs)
{
	int retv;
	struct arm_smccc_res arm_retvs;

	if (zynqmp_pm_sc == NULL)
		return (ENXIO);

	if (zynqmp_pm_sc->pm_fw_use_hvc)
		retv = arm_smccc_hvc(PM_SIP_SVC | api_id,
		    ((uint64_t)arg1 << 32) | arg0,
		    ((uint64_t)arg3 << 32) | arg2,
		    0, 0, 0, 0, 0, &arm_retvs);
	else
		retv = arm_smccc_smc(PM_SIP_SVC | api_id,
		    ((uint64_t)arg1 << 32) | arg0,
		    ((uint64_t)arg3 << 32) | arg2,
		    0, 0, 0, 0, 0, &arm_retvs);

	if (retvs != NULL) {
		retvs[0] = arm_retvs.a0 & 0xffffffffu;
		retvs[1] = arm_retvs.a0 >> 32;
		retvs[2] = arm_retvs.a1 & 0xffffffffu;
		retvs[3] = arm_retvs.a1 >> 32;
	}

	switch (retv) {
	case XST_PM_SUCCESS:
	case XST_PM_DOUBLE_REQ:
		return (0);
	case XST_PM_NO_FEATURE:
		return (ENOTSUP);
	case XST_PM_NO_ACCESS:
		return (EACCES);
	case XST_PM_INVALID_NODE:
	case XST_PM_INTERNAL:
	case XST_PM_CONFLICT:
	default:
		return (EINVAL);
	}
	/* NOTREACHED */
}

int
zynqmp_pm_get_api_version(uint32_t *version)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_GET_API_VERSION, 0, 0, 0, 0, retvs);
	if (!retv)
		*version = retvs[1];

	return (retv);
}

int
zynqmp_pm_get_chipid(uint32_t *idcode, uint32_t *version)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_GET_CHIPID, 0, 0, 0, 0, retvs);

	if (!retv) {
		*idcode = retvs[1];
		*version = retvs[2];
	}

	return (retv);
}

int
zynqmp_pm_init_finalize(void)
{

	return zynqmp_pm_invoke_fn(PM_INIT_FINALIZE, 0, 0, 0, 0, NULL);
}

int
zynqmp_pm_get_trustzone_version(uint32_t *version)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_GET_TRUSTZONE_VERSION, 0, 0, 0, 0,
	    retvs);
	if (!retv)
		*version = retvs[1];

	return (retv);
}

int
zynqmp_pm_query_data(uint32_t qid, uint32_t arg1, uint32_t arg2,
    uint32_t arg3, uint32_t *retvs)
{
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_QUERY_DATA, qid,
	    arg1, arg2, arg3, retvs);

	return (qid == PM_QID_CLOCK_GET_NAME ? 0 : retv);
}

int
zynqmp_pm_ioctl(uint32_t node_id, uint32_t ioctl_id, uint32_t arg1,
    uint32_t arg2, uint32_t *retvs)
{

	return zynqmp_pm_invoke_fn(PM_IOCTL, node_id, ioctl_id, arg1, arg2,
	    retvs);
}

int
zynqmp_pm_mmio_write(uint32_t addr, uint32_t mask, uint32_t data)
{

	return (zynqmp_pm_invoke_fn(PM_MMIO_WRITE, addr, mask, data, 0, NULL));
}

int
zynqmp_pm_mmio_read(uint32_t addr, uint32_t *data)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_MMIO_READ, addr, 0, 0, 0, retvs);
	if (!retv)
		*data = retvs[1];

	return (retv);
}

int
zynqmp_pm_fpga_load(vm_paddr_t physaddr, uint32_t size, uint32_t flags)
{

	return (zynqmp_pm_invoke_fn(PM_FPGA_LOAD,
		(uint64_t)physaddr & 0xffffffffu, (uint64_t)physaddr >> 32,
		size, flags, NULL));
}

/* Note: even if you are only reading a PCAP status register, you must
 * provide the physical address of 256 bytes of contiguous memory because
 * the PMU uses it to do an elaborate DMA to retrieve internal PCAP registers.
 */
int
zynqmp_pm_fpga_read(vm_paddr_t physaddr, uint32_t n, uint32_t read_type,
		    uint32_t *value)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_FPGA_READ, n,
	    (uint64_t)physaddr & 0xffffffffu, (uint64_t)physaddr >> 32,
	    read_type, retvs);
	if (!retv)
		*value = retvs[1];

	return (retv);
}

int
zynqmp_pm_fpga_get_status(uint32_t *status)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_FPGA_GET_STATUS, 0, 0, 0, 0, retvs);
	if (!retv)
		*status = retvs[1];

	return (retv);
}

int
zynqmp_pm_reset_action(int reset_id, int action)
{

	return (zynqmp_pm_invoke_fn(PM_RESET_ASSERT, reset_id, action,
	    0, 0, NULL));
}

int
zynqmp_pm_reset_get_status(int reset_id, int *status)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_RESET_GET_STATUS, reset_id,
	    0, 0, 0, retvs);
	if (!retv)
		*status = retvs[1];

	return (retv);
}

int
zynqmp_pm_clock_enable(int clock_id)
{

	return (zynqmp_pm_invoke_fn(PM_CLOCK_ENABLE, clock_id,
	    0, 0, 0, NULL));
}

int
zynqmp_pm_clock_disable(int clock_id)
{

	return (zynqmp_pm_invoke_fn(PM_CLOCK_DISABLE, clock_id,
	    0, 0, 0, NULL));
}

int
zynqmp_pm_clock_get_state(int clock_id, int *state)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_CLOCK_GETSTATE, clock_id,
	    0, 0, 0, retvs);
	if (!retv)
		*state = retvs[1];

	return (retv);
}

/* Set one of the two dividers.  div_id is either 0 or 1. */
int
zynqmp_pm_clock_set_divider(int clock_id, int div_id, uint32_t val)
{

	/* wack abi. */
	if (div_id)
		val = (val << 16) | 0xffff;
	else
		val |= 0xffff0000u;

	return (zynqmp_pm_invoke_fn(PM_CLOCK_SETDIVIDER, clock_id, val, 0, 0,
	    NULL));
}

/*
 * This actually gets both dividers.  div1 is returned in the upper 16 bits
 *  and div0 is in the lower 16 bits.
 */
int
zynqmp_pm_clock_get_divider(int clock_id, uint32_t *val)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_CLOCK_GETDIVIDER, clock_id, 0, 0, 0,
	    retvs);
	if (!retv)
		*val = retvs[1];

	return (retv);
}

int
zynqmp_pm_clock_set_parent(int clock_id, int parent_id)
{

	return (zynqmp_pm_invoke_fn(PM_CLOCK_SETPARENT, clock_id, parent_id,
	    0, 0, NULL));
}

int
zynqmp_pm_clock_get_parent(int clock_id, int *parent_id)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_CLOCK_GETPARENT, clock_id,
	    0, 0, 0, retvs);
	if (!retv)
		*parent_id = retvs[1];

	return (retv);
}

int
zynqmp_pm_pll_set_param(int pll_id, int param_id, uint32_t param_data)
{

	return (zynqmp_pm_invoke_fn(PM_PLL_SET_PARAM, pll_id, param_id,
	    param_data, 0, NULL));
}

int
zynqmp_pm_pll_get_param(int pll_id, int param_id, uint32_t *param_data)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_PLL_GET_PARAM, pll_id, param_id,
	    0, 0, retvs);
	if (!retv)
		*param_data = retvs[1];

	return (retv);
}

int
zynqmp_pm_pll_set_mode(int pll_id, int mode)
{

	return (zynqmp_pm_invoke_fn(PM_PLL_SET_MODE, pll_id, mode, 0, 0,
	    NULL));
}

int
zynqmp_pm_pll_get_mode(int pll_id, int *mode)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_PLL_GET_MODE, pll_id, 0, 0, 0, retvs);
	if (!retv)
		*mode = retvs[1];

	return (retv);
}

int
zynqmp_pm_request_node(int node_id, uint32_t caps, uint32_t qos, int ack)
{

	return (zynqmp_pm_invoke_fn(PM_REQUEST_NODE, node_id, caps,
	    qos, ack, NULL));
}

int
zynqmp_pm_set_requirement(int node_id, uint32_t req, uint32_t qos, int ack)
{

	return (zynqmp_pm_invoke_fn(PM_SET_REQUIREMENT, node_id, req,
	    qos, ack, NULL));
}

int
zynqmp_pm_release_node(int node_id)
{

	return (zynqmp_pm_invoke_fn(PM_RELEASE_NODE, node_id, 0, 0, 0, NULL));
}

int
zynqmp_pm_get_node_status(int node_id, uint32_t *status,
    uint32_t *requirements, uint32_t *usage)
{
	uint32_t retvs[RETV_CNT];
	int retv;

	retv = zynqmp_pm_invoke_fn(PM_GET_NODE_STATUS, node_id,
	    0, 0, 0, retvs);

	if (!retv) {
		*status = retvs[1];
		if (requirements)
			*requirements = retvs[2];
		if (usage)
			*usage = retvs[3];
	}

	return retv;
}

/*
 * What a hack!  Go through device tree looking for power-domain phandles
 * and check status of the driver.  Register nodes with active drivers.
 * Perform init-finalize which powers down unused devices and the unused RPUs
 * (Cortex-R5 cores).  More importantly, it also gives the kernel permissions
 * to modify clocks associated with the nodes.
 */
static int
zynqmp_pm_find_active_nodes(struct zynqmp_pm_softc *sc)
{
	phandle_t node;
	int error;
	phandle_t xref;
	pcell_t *cells;
	int ncells;

	node = OF_finddevice("/amba");
	if (node <= 0) {
		device_printf(sc->dev, "can't find /amba in dtb\n");
		return (-1);
	}

	for (node = OF_child(node); node > 0; node = OF_peer(node)) {
		error = ofw_bus_parse_xref_list_alloc(node, "power-domains",
		    "#power-domain-cells", 0, &xref, &ncells, &cells);

		if (error || !ofw_bus_node_status_okay(node))
			continue;

		/* printf("%s: found power-domain node=%s (%d)\n",
		   __func__, node_names[cells[0]], cells[0]); */

		/* Request and set requirements for power node. */
		error = zynqmp_pm_request_node(cells[0], 0, 0,
		    PM_REQ_ACK_BLOCKING);
		if (error) {
			device_printf(sc->dev, "failed to request node: %d "
			    "error=%d\n", cells[0], error);
			continue;
		}
		error = zynqmp_pm_set_requirement(cells[0], PM_CAP_ACCESS,
		    PM_MAX_QOS,	PM_REQ_ACK_BLOCKING);
		if (error)
			device_printf(sc->dev, "failed to set requirements: "
			    "node %d error=%d\n", cells[0], error);
		OF_prop_free(cells);
	}

	/* Power down all the unused islands.  */
	error = zynqmp_pm_init_finalize();
	if (error)
		device_printf(sc->dev, "init-finalize failed. err=%d\n",
		    error);

	return (0);
}


/* XXX: DEBUG */
static void
zynqmp_pm_dump_nodes(void)
{
	int id;
	uint32_t status;
	uint32_t req;
	uint32_t usage;
	static char *usage_str[] = { "none", "curr", "other", "both" };

	for (id = ZYNQMP_PM_NODE_FIRST; node_names[id] != NULL; id++) {
		if (zynqmp_pm_get_node_status(id, &status, &req, &usage)) {
			printf("%s:  zynqmp_pm_get_node_status err for %s.\n",
			    __func__, node_names[id]);
			continue;
		}

		printf("\tNode %s: status=%d req=%d usage=%s\n",
		    node_names[id], status, req, usage_str[usage]);
	}
}

/* XXX: DEBUG */
static int
zynqmp_pm_dump(SYSCTL_HANDLER_ARGS)
{
	int error, i;

	error = sysctl_wire_old_buffer(req, sizeof(int));
	if (error == 0) {
		i = 0;
		error = sysctl_handle_int(oidp, &i, 0, req);
	}
	if (error != 0 || req->newptr == NULL)
		return (error);

	zynqmp_pm_dump_nodes();

	return (0);
}

static char pm_api_version_str[16];
static char pm_chipid_str[48];
static char trustzone_version_str[16];

static void
zynqmp_pm_getsysctls(struct zynqmp_pm_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child;
	uint32_t version;
	uint32_t idcode;
	int retv;

	ctx = device_get_sysctl_ctx(sc->dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	if (ctx == NULL || child == NULL) {
		device_printf(sc->dev, "could not add sysctls\n");
		return;
	}

	retv = zynqmp_pm_get_api_version(&version);
	if (retv)
		device_printf(sc->dev, "trouble getting api version\n");
	else {
		snprintf(pm_api_version_str, sizeof(pm_api_version_str),
		    "v%d.%d", version >> 16, version & 0xffff);

		SYSCTL_ADD_STRING(ctx, child, OID_AUTO, "api-version",
		    CTLFLAG_RD, pm_api_version_str, 0,
		    "PMU firmware api version");
	}

	retv = zynqmp_pm_get_chipid(&idcode, &version);
	if (retv)
		device_printf(sc->dev, "trouble getting chip id\n");
	else {
		snprintf(pm_chipid_str, sizeof(pm_chipid_str),
		    "idcode 0x%08x vers 0x%08x", idcode, version);

		SYSCTL_ADD_STRING(ctx, child, OID_AUTO, "chip-id", CTLFLAG_RD,
		    pm_chipid_str, 0, "Chip ID and version");
	}

	retv = zynqmp_pm_get_trustzone_version(&version);

	if (retv)
		device_printf(sc->dev, "trouble getting trustzone version");
	else {
		snprintf(trustzone_version_str, sizeof(trustzone_version_str),
		    "v%d.%d", version >> 16, version & 0xffff);

		SYSCTL_ADD_STRING(ctx, child, OID_AUTO, "trustzone-version",
		    CTLFLAG_RD, trustzone_version_str, 0,
		    "TrustZone version");
	}

	/* XXX: DEBUG */
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "_dump",
	    CTLTYPE_INT | CTLFLAG_RW, sc, 0, zynqmp_pm_dump, "I",
	    "Dump information for debug purposes");
}

static int
zynqmp_pm_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ Platform Management Driver");

	return (BUS_PROBE_DEFAULT);
}

static int
zynqmp_pm_attach(device_t dev)
{
	struct zynqmp_pm_softc *sc = device_get_softc(dev);
	phandle_t node;
	char method[16];

	if (zynqmp_pm_sc != NULL)
		return (EBUSY);

	sc->dev = dev;
	sc->pm_fw_use_hvc = false;
	zynqmp_pm_sc = sc;

	node = ofw_bus_get_node(dev);
	if (OF_getprop(node, "method", method, sizeof(method)) > 0 &&
	    strcmp(method, "hvc") == 0)
		sc->pm_fw_use_hvc = true;

	ZPM_LOCK_INIT(sc);

	zynqmp_pm_getsysctls(sc);

	(void)zynqmp_pm_find_active_nodes(sc);

	/* Register node to this driver. */
	OF_device_register_xref(OF_xref_from_node(node), dev);

	/* Walk the fdt and add sub-nodes. */
	for (node = OF_child(node); node > 0; node = OF_peer(node))
		simplebus_add_device(dev, node, 0, NULL, -1, NULL);

	return (bus_generic_attach(dev));
}

static int
zynqmp_pm_detach(device_t dev)
{
	struct zynqmp_pm_softc *sc = device_get_softc(dev);

	zynqmp_pm_sc = NULL;

	ZPM_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_pm_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		zynqmp_pm_probe),
	DEVMETHOD(device_attach,	zynqmp_pm_attach),
	DEVMETHOD(device_detach,	zynqmp_pm_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(zynqmp_pm, zynqmp_pm_driver, zynqmp_pm_methods,
    sizeof(struct zynqmp_pm_softc), simplebus_driver);

EARLY_DRIVER_MODULE(zynqmp_pm, simplebus, zynqmp_pm_driver, 0, 0,
    BUS_PASS_TIMER + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(zynqmp_pm, 1);
SIMPLEBUS_PNP_INFO(compat_data);
