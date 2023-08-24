/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Thomas Skibo. <Thomas@skibo.net>
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
#include <sys/kernel.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/fbio.h>
#include <sys/consio.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fb/fbreg.h>
#include <dev/vt/vt.h>

#include <arm64/xilinx/zynqmp_dpdma.h>


/* Driver for Xilinx Zynq UltraScale+ DisplayPort DMA controller.
 *
 * References: Zynq UltraScale+ Device Technical Refernce Manual.
 * (v2.1) August 21, 2019.  Xilinx doc UG1085.  Ch.33.
 *
 * Zynq UltraScale+ MPSoC Register Reference;
 * https://www.xilinx.com/html_docs/registers/ug1087/ug1087-zynq-ultrascale-registers.html
 *
 */

static struct ofw_compat_data compat_data[] = {
	{"xlnx,dpdma",		1},
	{NULL,			0}
};

static int zynqmp_dpdma_attach(device_t dev);
static int zynqmp_dpdma_detach(device_t dev);

struct dpdma_desc;

struct zynqmp_dpdma_softc {
	device_t		dev;
	struct mtx		sc_mtx;
	struct resource		*mem_res;

	bus_dma_tag_t		desc_dma_tag;
	bus_dmamap_t		desc_dma_map;
	struct dpdma_desc	*desc;
	bus_addr_t		desc_paddr;
};

/* Display Port DMA Modules registers. */
#define ZYNQMP_DPDMA_ISR			0x0004
#define    ZYNQMP_DPDMA_ISR_VSYNC_INT				(1 << 27)
#define    ZYNQMP_DPDMA_ISR_AXI_RD_4K_CROSS			(1 << 26)
#define    ZYNQMP_DPDMA_ISR_WR_DATA_FIFO_FULL			(1 << 25)
#define    ZYNQMP_DPDMA_ISR_WR_CMD_FIFO_FULL			(1 << 24)
#define    ZYNQMP_DPDMA_ISR_DSCR_ERR(n)				(1 << 18 + (n))
#define    ZYNQMP_DPDMA_ISR_DATA_AXI_ERR(n)			(1 << 12 + (n))
#define    ZYNQMP_DPDMA_ISR_NO_OSTAND_TRAN(n)			(1 << 6 + (n))
#define    ZYNQMP_DPDMA_ISR_DSCR_DONE(n)			(1 << (n))
#define	   ZYNQMP_DPDMA_ISR_ALL					0x0fffffff
#define ZYNQMP_DPDMA_IMR			0x0008
#define ZYNQMP_DPDMA_IEN			0x000c
#define ZYNQMP_DPDMA_IDS			0x0010
#define ZYNQMP_DPDMA_EISR			0x0014
#define    ZYNQMP_DPDMA_EISR_RD_CMD_FIFO_FULL			(1u << 31)
#define    ZYNQMP_DPDMA_EISR_DSCR_DONE_ERR(n)			(1 << 25 + (n))
#define    ZYNQMP_DPDMA_EISR_WR_AXI_ERR(n)			(1 << 19 + (n))
#define    ZYNQMP_DPDMA_EISR_CRC_ERR(n)				(1 << 13 + (n))
#define    ZYNQMP_DPDMA_EISR_PRE_ERR(n)				(1 << 7 + (n))
#define    ZYNQMP_DPDMA_EISR_RD_AXI_ERR(n)			(1 << 1 + (n))
#define    ZYNQMP_DPDMA_EISR_INV_APB				(1 << 0)
#define ZYNQMP_DPDMA_EIMR			0x0018
#define ZYNQMP_DPDMA_EIEN			0x001c
#define ZYNQMP_DPDMA_EIDS			0x0020
#define ZYNQMP_DPDMA_GBL			0x0104
#define    ZYNQMP_DPDMA_GBL_RTRG(n)				(1 << 6 + (n))
#define    ZYNQMP_DPDMA_GBL_TRG(n)				(1 << (n))
#define ZYNQMP_DPDMA_ALC_CNTL(n)		(0x0108 + 0x18 * (n))
#define    ZYNQMP_DPDMA_ALC_CNTL_MON_ID				(0xf << 2)
#define    ZYNQMP_DPDMA_ALC_CNTL_CLEAR				(1 << 1)
#define    ZYNQMP_DPDMA_ALC_CNTL_EN				(1 << 0)
#define ZYNQMP_DPDMA_ALC_STATUS(n)		(0x010c + 0x18 * (n))
#define    ZYNQMP_DPDMA_ALC_STATUS_OFLOW			(1 << 0)
#define ZYNQMP_DPDMA_ALC_MAX(n)			(0x0110 + 0x18 * (n))
#define ZYNQMP_DPDMA_ALC_MIN(n)			(0x0114 + 0x18 * (n))
#define ZYNQMP_DPDMA_ALC_ACC(n)			(0x0118 + 0x18 * (n))
#define ZYNQMP_DPDMA_ALC_ACC_TRAN(n)		(0x011c + 0x18 * (n))
#define ZYNQMP_DPDMA_CH_DSCR_STRT_ADDRE(n)	(0x0200 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_DSCR_STRT_ADDR(n)	(0x0204 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_DSCR_NEXT_ADDRE(n)	(0x0208 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_DSCR_NEXT_ADDR(n)	(0x020c + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_PYLD_CUR_ADDRE(n)	(0x0210 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_PYLD_CUR_ADDR(n)	(0x0214 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_CNTL(n)			(0x0218 + 0x100 * (n))
#define    ZYNQMP_DPDMA_CH_CNTL_DSCR_DLY_CNT_MASK		(0x3ff << 20)
#define    ZYNQMP_DPDMA_CH_CNTL_DSCR_DLY_CNT_SHIFT		20
#define    ZYNQMP_DPDMA_CH_CNTL_DSCR_DLY_CNT(n)			((n) << 20)
#define    ZYNQMP_DPDMA_CH_CNTL_DSCR_AXCACHE_MASK		(0xf << 16)
#define    ZYNQMP_DPDMA_CH_CNTL_DSCR_AXCACHE_SHIFT		16
#define    ZYNQMP_DPDMA_CH_CNTL_DSCR_AXCACHE(n)			((n) << 16)
#define    ZYNQMP_DPDMA_CH_CNTL_AXPROT_MASK			(3 << 14)
#define    ZYNQMP_DPDMA_CH_CNTL_AXPROT_SHIFT			14
#define    ZYNQMP_DPDMA_CH_CNTL_AXPROT(n)			((n) << 14)
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DATA_RD_MASK		(0xf << 10)
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DATA_RD_SHIFT		10
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DATA_RD(n)			((n) << 10)
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_RD_MASK		(0xf << 6)
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_RD_SHIFT		6
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_RD(n)			((n) << 6)
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_WR_MASK		(0xf << 2)
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_WR_SHIFT		2
#define    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_WR(n)			((n) << 2)
#define    ZYNQMP_DPDMA_CH_CNTL_PAUSE				(1 << 1)
#define    ZYNQMP_DPDMA_CH_CNTL_EN				(1 << 0)
#define ZYNQMP_DPDMA_CH_STATUS(n)		(0x021c + 0x100 * (n))
#define    ZYNQMP_DPDMA_CH_STATUS_OTRAN_CNT_MASK		(0xf << 21)
#define    ZYNQMP_DPDMA_CH_STATUS_OTRAN_CNT_SHIFT		21
#define    ZYNQMP_DPDMA_CH_STATUS_PREAMBLE_MASK			(0xff << 13)
#define    ZYNQMP_DPDMA_CH_STATUS_PREAMBLE_SHIFT		13
#define    ZYNQMP_DPDMA_CH_STATUS_EN_DSCR_INTR			(1 << 12)
#define    ZYNQMP_DPDMA_CH_STATUS_EN_DSCR_UP			(1 << 11)
#define    ZYNQMP_DPDMA_CH_STATUS_DSCR_DONE			(1 << 10)
#define    ZYNQMP_DPDMA_CH_STATUS_IGNR_DONE			(1 << 9)
#define    ZYNQMP_DPDMA_CH_STATUS_LDSCR_FRAME			(1 << 8)
#define    ZYNQMP_DPDMA_CH_STATUS_LAST_DSCR			(1 << 7)
#define    ZYNQMP_DPDMA_CH_STATUS_EN_CRC			(1 << 6)
#define    ZYNQMP_DPDMA_CH_STATUS_MODE				(1 << 5)
#define    ZYNQMP_DPDMA_CH_STATUS_BURST_TYPE			(1 << 4)
#define    ZYNQMP_DPDMA_CH_STATUS_BURST_LEN_MASK		(0xf << 0)
#define    ZYNQMP_DPDMA_CH_STATUS_BURST_LEN_SHIFT		0
#define ZYNQMP_DPDMA_CH_VDO(n)			(0x0220 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_PYLD_SZ(n)		(0x0224 + 0x100 * (n))
#define ZYNQMP_DPDMA_CH_DESC_ID(n)		(0x0228 + 0x100 * (n))

struct dpdma_desc {
	uint32_t	ctrl;
#define ZYNQMP_DPDMA_DESC_CTRL_PREAMBLE				0xa5
#define ZYNQMP_DPDMA_DESC_CTRL_COMPL_INT			(1 << 8)
#define ZYNQMP_DPDMA_DESC_CTRL_DESC_UPDATE			(1 << 9)
#define ZYNQMP_DPDMA_DESC_CTRL_IGNORE_DONE			(1 << 10)
#define ZYNQMP_DPDMA_DESC_CTRL_AXI_FIXED_BURST			(1 << 11)
#define ZYNQMP_DPDMA_DESC_CTRL_AXCACHE_MASK			(0xf << 12)
#define ZYNQMP_DPDMA_DESC_CTRL_AXCACHE_SHIFT			12
#define ZYNQMP_DPDMA_DESC_CTRL_AXCACHE(n)			((n) << 12)
#define ZYNQMP_DPDMA_DESC_CTRL_AXPROT_MASK			(3 << 16)
#define ZYNQMP_DPDMA_DESC_CTRL_AXPROT_SHIFT			16
#define ZYNQMP_DPDMA_DESC_CTRL_AXPROT(n)			((n) << 16)
#define ZYNQMP_DPDMA_DESC_CTRL_MODE_FRAG			(1 << 18)
#define ZYNQMP_DPDMA_DESC_CTRL_LAST_DESC			(1 << 19)
#define ZYNQMP_DPDMA_DESC_CTRL_CRC_EN				(1 << 20)
#define ZYNQMP_DPDMA_DESC_CTRL_LAST_DESC_FRAME			(1 << 21)
	uint32_t	dscr_id;
	uint32_t	xfer_size;
	uint32_t	line_size_stride;
#define ZYNQMP_DPDMA_DESC_STRIDE(n)				((n) << 18)
	uint32_t	timestamp_lo;
	uint32_t	timestamp_hi;
	uint32_t	addr_ext;	/* src in hi 16 bits, next in lo */
	uint32_t	next_desc;
	uint32_t	src_addr;
	uint32_t	addr_ext_23;
	uint32_t	addr_ext_45;
	uint32_t	src_addr_2;
	uint32_t	src_addr_3;
	uint32_t	src_addr_4;
	uint32_t	src_addr_5;
	uint32_t	crc;
};

#define ZYNQMP_DPDMA_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define ZYNQMP_DPDMA_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZYNQMP_DPDMA_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)
#define ZYNQMP_DPDMA_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), "dpdma", \
	    MTX_DEF)
#define ZYNQMP_DPDMA_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->sc_mtx)

#define RD4(sc, off)		(bus_read_4((sc)->mem_res, (off)))
#define WR4(sc, off, val)	(bus_write_4((sc)->mem_res, (off), (val)))

static void
callback_getaddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (nsegs != 1 || error)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

void
zynqmp_dpdma_start(device_t dev, struct fb_info *info, int channel)
{
	struct zynqmp_dpdma_softc *sc = device_get_softc(dev);

	/*
	 * Create a descriptor that points to frame buffer memory and
	 * whose next descriptor field points at itself.
	 */
	memset(sc->desc, 0, sizeof(struct dpdma_desc));
	sc->desc->ctrl =
	    ZYNQMP_DPDMA_DESC_CTRL_PREAMBLE |
	    ZYNQMP_DPDMA_DESC_CTRL_IGNORE_DONE |
	    ZYNQMP_DPDMA_DESC_CTRL_LAST_DESC_FRAME;
	sc->desc->xfer_size = info->fb_width * info->fb_height *
	    info->fb_bpp / 8;
	sc->desc->line_size_stride = (info->fb_width * info->fb_bpp / 8) |
	    (info->fb_stride << 14);
	sc->desc->src_addr = info->fb_pbase & 0xffffffffu;
	sc->desc->next_desc = sc->desc_paddr & 0xffffffffu;
	sc->desc->addr_ext = (info->fb_pbase >> 16 & 0xffff0000u) |
	    (sc->desc_paddr >> 32);

	/* Sync cache. */
	bus_dmamap_sync(sc->desc_dma_tag, sc->desc_dma_map,
	    BUS_DMASYNC_PREWRITE);

	/* Give it the descriptor physical address and GO. */
	WR4(sc, ZYNQMP_DPDMA_CH_DSCR_STRT_ADDRE(channel),
	    sc->desc_paddr >> 32);
	WR4(sc, ZYNQMP_DPDMA_CH_DSCR_STRT_ADDR(channel),
	    sc->desc_paddr & 0xffffffffu);
	WR4(sc, ZYNQMP_DPDMA_CH_CNTL(channel),
	    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_WR(11) |
	    ZYNQMP_DPDMA_CH_CNTL_QOS_DSCR_RD(11) |
	    ZYNQMP_DPDMA_CH_CNTL_QOS_DATA_RD(11) |
	    ZYNQMP_DPDMA_CH_CNTL_EN);
	WR4(sc, ZYNQMP_DPDMA_GBL, ZYNQMP_DPDMA_GBL_TRG(channel));
}

void
zynqmp_dpdma_stop(device_t dev, int channel)
{
	struct zynqmp_dpdma_softc *sc = device_get_softc(dev);

	/* Stop DPDMA. */
	WR4(sc, ZYNQMP_DPDMA_CH_CNTL(channel), 0);
}

static int
zynqmp_dpdma_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ DisplayPort DMA controller");

	return (BUS_PROBE_DEFAULT);
}

static int
zynqmp_dpdma_attach(device_t dev)
{
	struct zynqmp_dpdma_softc *sc = device_get_softc(dev);
	int rid;
	int error;
	phandle_t node;

	sc->dev = dev;

	ZYNQMP_DPDMA_LOCK_INIT(sc);

	/* DPDMA module device registers. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "could not allocate memory resource.\n");
		error = ENOMEM;
		goto fail;
	}

	/* Disable all interrupts in DPDMA. */
	WR4(sc, ZYNQMP_DPDMA_IDS, ZYNQMP_DPDMA_ISR_ALL);

	/* Create DMA resources for DPDMA descriptor. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 1, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    sizeof(struct dpdma_desc), 1, sizeof(struct dpdma_desc), 0,
	    busdma_lock_mutex, &sc->sc_mtx, &sc->desc_dma_tag);
	if (error) {
		device_printf(dev, "bus_dma_tag_create failed.\n");
		goto fail;
	}

	error = bus_dmamem_alloc(sc->desc_dma_tag,
	    (void **)&sc->desc, BUS_DMA_NOWAIT, &sc->desc_dma_map);
	if (error) {
		device_printf(dev, "bus_dmamem_alloc failed.\n");
		goto fail;
	}

	error = bus_dmamap_load(sc->desc_dma_tag, sc->desc_dma_map,
	    (void *)sc->desc, sizeof(struct dpdma_desc), callback_getaddr,
	    &sc->desc_paddr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(dev, "bus_dmamap_load failed.\n");
		goto fail;
	}

	/* Register node to this driver. */
	node = ofw_bus_get_node(dev);
	if (node <= 0) {
		device_printf(dev, "failed to get node for this device\n");
		error = ENXIO;
		goto fail;
	}
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);
fail:
	zynqmp_dpdma_detach(dev);
	return (error);
}

static int
zynqmp_dpdma_detach(device_t dev)
{
	struct zynqmp_dpdma_softc *sc = device_get_softc(dev);

	/* Unload DMA map and resoures. */
	if (sc->desc_paddr != 0) {
		bus_dmamap_unload(sc->desc_dma_tag, sc->desc_dma_map);
		sc->desc_paddr = 0;
	}
	if (sc->desc != NULL) {
		bus_dmamem_free(sc->desc_dma_tag, sc->desc, sc->desc_dma_map);
		sc->desc = NULL;
		sc->desc_dma_map = NULL;
	}
	if (sc->desc_dma_tag != NULL) {
		bus_dma_tag_destroy(sc->desc_dma_tag);
		sc->desc_dma_tag = NULL;
	}

	/* Release resources. */
	if (sc->mem_res) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);
		sc->mem_res = NULL;
	}

	ZYNQMP_DPDMA_LOCK_DESTROY(sc);

	return (0);
}
static device_method_t zynqmp_dpdma_methods[] = {
	DEVMETHOD(device_probe,		zynqmp_dpdma_probe),
	DEVMETHOD(device_attach,	zynqmp_dpdma_attach),
	DEVMETHOD(device_detach,	zynqmp_dpdma_detach),

	DEVMETHOD_END
};

static driver_t zynqmp_dpdma_driver = {
	"zynqmp_dpdma",
	zynqmp_dpdma_methods,
	sizeof(struct zynqmp_dpdma_softc)
};

DRIVER_MODULE(zynqmp_dpdma, simplebus, zynqmp_dpdma_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
