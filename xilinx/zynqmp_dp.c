/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018-2020 Thomas Skibo.
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
#include <vm/vm.h>
#include <vm/vm_kern.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>

#include <dev/fb/fbreg.h>
#include <dev/vt/vt.h>

#include <arm64/xilinx/zynqmp_phy.h>
#include <arm64/xilinx/zynqmp_dpdma.h>

#include "fb_if.h"

/* Driver for Xilinx Zynq UltraScale+ Display Port Frame Buffer Device.
 *
 * References: Zynq UltraScale+ Device Technical Refernce Manual.
 * (v1.7) December 22, 2017.  Xilinx doc UG1085.  Ch.33.
 *
 * Zynq UltraScale+ MPSoC Register Reference;
 * https://www.xilinx.com/html_docs/registers/ug1087/ug1087-zynq-ultrascale-registers.html
 *
 */

#ifdef DPRINTF
#undef DPRINTF
#endif
#ifdef DPDEBUG
#define DPRINTF(lvl, ...) do { if ((lvl) >= DPDEBUG)	\
	    printf(__VA_ARGS__); } while (0)
#else
#define DPRINTF(...) do { } while (0)
#endif

static struct ofw_compat_data compat_data[] = {
	{"xlnx,zynqmp-dp",		1},
	{"xlnx,zynqmp-dpsub-1.7",	1},
	{NULL,				0}
};

struct dpdma_desc;

#define MAX_LANES		2
#define DPCD_RX_CAP_SIZE	16

struct zynqmp_dp_softc {
	device_t		dev;
	struct mtx		sc_mtx;
	struct resource		*mem_res[4];
	struct resource		*irq_res;
	void			*intr_hdl;

	phandle_t		phyxref[MAX_LANES];
	int			phy_ct;

	device_t		dpdma_dev;
	int			dpdma_chan;

	clk_t			vref_clk;
	int			vref_clk_freq;

	device_t		fbdev;
	struct fb_info		info;
	size_t			fb_size;
	int			fb_stride;
	bus_addr_t		fb_paddr;
	vm_offset_t		fb_vaddr;

	int			hpd_state;
	uint8_t			dpcd_caps[DPCD_RX_CAP_SIZE];
	int			link_bw;
	int			lane_ct;

	int			width;
	int			h_front_porch;
	int			h_sync;
	int			h_back_porch;

	int			height;
	int			v_front_porch;
	int			v_sync;
	int			v_back_porch;

	int			pixclk_khz;
	int			bits_per_pixel;
};

#define FB_DEPTH		24
#define FB_ALIGN		256
#define FB_STRIDE_ALIGN		256

#define DEFAULT_WIDTH		1920
#define DEFAULT_H_FRONT_PORCH	88
#define DEFAULT_H_SYNC_WIDTH	44
#define DEFAULT_H_BACK_PORCH	148

#define DEFAULT_HEIGHT		1080
#define DEFAULT_V_FRONT_PORCH	4
#define DEFAULT_V_SYNC_WIDTH	5
#define DEFAULT_V_BACK_PORCH	36

#define DEFAULT_PIX_CLK_KHZ	148500
#define DEFAULT_BITS_PER_PIXEL	FB_DEPTH

/* Display Port Module registers.  (Not complete.) */
#define ZYNQMP_DP_LINK_BW_SET			0x0000
#define    ZYNQMP_DP_LINK_BW_SET_1_62GBPS			0x06
#define    ZYNQMP_DP_LINK_BW_SET_2_7GBPS			0x0a
#define    ZYNQMP_DP_LINK_BW_SET_5_4GBPS			0x14
#define ZYNQMP_DP_LANE_COUNT_SET		0x0004
#define ZYNQMP_DP_ENHANCED_FRAME_EN		0x0008
#define ZYNQMP_DP_TRAINING_PATTERN_SET		0x000c
#define ZYNQMP_DP_SCRAMBLING_DISABLE		0x0014
#define ZYNQMP_DP_DOWNSPREAD_CTRL		0x0018
#define    ZYNQMP_DP_DOWNSPREAD_CTRL_5_0			1
#define ZYNQMP_DP_SOFTWARE_RESET		0x001c
#define    ZYNQMP_DP_SOFTWARE_RESET_STREAM(n)			(1 << (n))
#define    ZYNQMP_DP_SOFTWARE_RESET_AUX				(1 << 7)
#define    ZYNQMP_DP_SOFTWARE_RESET_ALL				0x8f
#define ZYNQMP_DP_TRANSMITTER_ENABLE		0x0080
#define ZYNQMP_DP_MAIN_STREAM_ENABLE		0x0084
#define ZYNQMP_DP_FORCE_SCRAMBLER_RESET		0x00c0
#define ZYNQMP_DP_VERSION_REGISTER		0x00f8
#define ZYNQMP_DP_CORE_ID			0x00fc
#define ZYNQMP_DP_AUX_COMMAND			0x0100
#define    ZYNQMP_DP_AUX_COMMAND_ADDR_TRANSFER_EN		(1 << 12)
#define    ZYNQMP_DP_AUX_COMMAND_CMD_MASK			(0xf << 8)
#define    ZYNQMP_DP_AUX_COMMAND_CMD_SHIFT			8
#define    ZYNQMP_DP_AUX_COMMAND_CMD_AUX_WR			(0x8 << 8)
#define    ZYNQMP_DP_AUX_COMMAND_CMD_AUX_RD			(0x9 << 8)
#define    ZYNQMP_DP_AUX_COMMAND_NUM_OF_BYTES_MASK		0xf /* n-1 */
#define ZYNQMP_DP_AUX_WRITE_FIFO		0x0104
#define ZYNQMP_DP_AUX_ADDRESS			0x0108
#define ZYNQMP_DP_AUX_CLOCK_DIVIDER		0x010c
#define    ZYNQMP_DP_AUX_CLOCK_DIV_AUX_PULSE_WIDTH_MASK		(0xff << 8)
#define    ZYNQMP_DP_AUX_CLOCK_DIV_AUX_PULSE_WIDTH_SHIFT	8
#define    ZYNQMP_DP_AUX_CLOCK_DIV_AUX_PULSE_WIDTH(n)		((n) << 8)
#define    ZYNQMP_DP_AUX_CLOCK_DIVIDER_VAL_MASK			0xff
#define ZYNQMP_DP_TX_USER_FIFO_OVERFLOW		0x0110
#define ZYNQMP_DP_INTERRUPT_SIGNAL_STATE	0x0130
#define    ZYNQMP_DP_INTERRUPT_SIGNAL_STATE_REPLY_TIMEOUT	(1 << 3)
#define    ZYNQMP_DP_INTERRUPT_SIGNAL_STATE_REPLY_STATE		(1 << 2)
#define    ZYNQMP_DP_INTERRUPT_SIGNAL_STATE_REQUEST_STATE	(1 << 1)
#define    ZYNQMP_DP_INTERRUPT_SIGNAL_STATE_HPD_STATE		(1 << 0)
#define ZYNQMP_DP_AUX_REPLY_DATA		0x0134
#define ZYNQMP_DP_AUX_REPLY_CODE		0x0138
#define    ZYNQMP_DP_AUX_REPLY_CODE_CODE1_MASK			(3 << 2)
#define    ZYNQMP_DP_AUX_REPLY_CODE_CODE0_MASK			3
#define ZYNQMP_DP_AUX_REPLY_COUNT		0x013c
#define ZYNQMP_DP_REPLY_DATA_COUNT		0x0148
#define    ZYNQMP_DP_REPLY_DATA_COUNT_MASK			0xff
#define ZYNQMP_DP_REPLY_STATUS			0x014c
#define    ZYNQMP_DP_REPLY_STATUS_AUX_REPLY_STATE_MASK		(0xff << 4)
#define    ZYNQMP_DP_REPLY_STATUS_AUX_REPLY_STATE_SHIFT		4
#define    ZYNQMP_DP_REPLY_STATUS_REPLY_ERROR			(1 << 3)
#define    ZYNQMP_DP_REPLY_STATUS_REQUEST_IN_PROGRESS		(1 << 2)
#define    ZYNQMP_DP_REPLY_STATUS_REPLY_IN_PROGRESS		(1 << 1)
#define    ZYNQMP_DP_REPLY_STATUS_REPLY_RECEIVED		(1 << 0)
#define ZYNQMP_DP_HPD_DURATION			0x0150
#define ZYNQMP_DP_MAIN_STREAM_HTOTAL		0x0180
#define ZYNQMP_DP_MAIN_STREAM_VTOTAL		0x0184
#define ZYNQMP_DP_MAIN_STREAM_POLARITY		0x0188
#define    ZYNQMP_DP_MAIN_STREAM_POLARITY_VSYNC			(1 << 1)
#define    ZYNQMP_DP_MAIN_STREAM_POLARITY_HSYNC			(1 << 0)
#define ZYNQMP_DP_MAIN_STREAM_HSWIDTH		0x018c
#define ZYNQMP_DP_MAIN_STREAM_VSWIDTH		0x0190
#define ZYNQMP_DP_MAIN_STREAM_HRES		0x0194
#define ZYNQMP_DP_MAIN_STREAM_VRES		0x0198
#define ZYNQMP_DP_MAIN_STREAM_HSTART		0x019c
#define ZYNQMP_DP_MAIN_STREAM_VSTART		0x01a0
#define ZYNQMP_DP_MAIN_STREAM_MISC0		0x01a4
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_MASK			(7 << 5)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_6			(0 << 5)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_8			(1 << 5)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_10			(2 << 5)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_12			(3 << 5)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_16			(4 << 5)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_YCBCR_COLR		(1 << 4)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_DYNC_RANGE		(1 << 3)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_COMP_FORMAT_MASK		(3 << 1)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_COMP_FORMAT_RGB		(0 << 1)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_COMP_FORMAT_YCRCB_422	(1 << 1)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_COMP_FORMAT_YCRCB_444	(2 << 1)
#define    ZYNQMP_DP_MAIN_STREAM_MISC0_SYNC_CLOCK		(1 << 0)
#define ZYNQMP_DP_MAIN_STREAM_MISC1		0x01a8
#define    ZYNQMP_DP_MAIN_STREAM_MISC1_Y_ONLY_EN		(1 << 7)
#define    ZYNQMP_DP_MAIN_STREAM_MISC1_STEREO_VID_ATTR_MASK	(3 << 1)
#define    ZYNQMP_DP_MAIN_STREAM_MISC1_STEREO_VID_ATTR_SHIFT	1
#define    ZYNQMP_DP_MAIN_STREAM_MISC1_STEREO_VID_ATTR(n)	((n) << 1)
#define ZYNQMP_DP_MAIN_STREAM_M_VID		0x01ac
#define ZYNQMP_DP_MSA_TRANSFER_UNIT_SIZE	0x01b0
#define ZYNQMP_DP_MAIN_STREAM_N_VID		0x01b4
#define ZYNQMP_DP_USER_PIX_WIDTH		0x01b8
#define ZYNQMP_DP_USER_DATA_COUNT_PER_LANE	0x01bc
#define ZYNQMP_DP_MIN_BYTES_PER_TU		0x01c4
#define ZYNQMP_DP_FRAC_BYTES_PER_TU		0x01c8
#define ZYNQMP_DP_INIT_WAIT			0x01cc
#define ZYNQMP_DP_PHY_RESET			0x0200
#define    ZYNQMP_DP_PHY_RESET_EN_8B_10B			(1 << 16)
#define    ZYNQMP_DP_PHY_RESET_PCS_RESET			(1 << 9)
#define    ZYNQMP_DP_PHY_RESET_PMA_RESET			(1 << 8)
#define    ZYNQMP_DP_PHY_RESET_GTTX_RESET			(1 << 1)
#define    ZYNQMP_DP_PHY_RESET_PHY_RESET			(1 << 0)
#define    ZYNQMP_DP_PHY_RESET_ALL				0x303
#define ZYNQMP_DP_TX_PHY_PREEMPHASIS_LANE(n)	(0x210 + 4 * (n))
#define ZYNQMP_DP_TX_PHY_VOLTAGE_DIFF_LANE(n)	(0x220 + 4 * (n))
#define ZYNQMP_DP_TRANSMIT_PRBS7		0x0230
#define ZYNQMP_DP_PHY_CLOCK_SELECT		0x0234
#define    ZYNQMP_DP_PHY_CLOCK_SELECT_5_4GBPS			5
#define    ZYNQMP_DP_PHY_CLOCK_SELECT_2_7GBPS			3
#define    ZYNQMP_DP_PHY_CLOCK_SELECT_1_62GBPS			1
#define ZYNQMP_DP_TX_PHY_POWER_DOWN		0x0238
#define    ZYNQMP_DP_TX_PHY_POWER_DOWN_LANE(n)			(1 << (n))
#define    ZYNQMP_DP_TX_PHY_POWER_DOWN_ALL			0xf
#define ZYNQMP_DP_SUB_TX_PHY_PRECURSOR_LANE(n)	(0x24c + 4 * (n))
#define ZYNQMP_DP_PHY_STATUS			0x0280
#define    ZYNQMP_DP_PHY_STATUS_PLL_LOCKED			(1 << 4)
#define    ZYNQMP_DP_PHY_STATUS_RATE_CHANGE_DONE1		(1 << 3)
#define    ZYNQMP_DP_PHY_STATUS_RATE_CHANGE_DONE0		(1 << 2)
#define    ZYNQMP_DP_PHY_STATUS_RESET_LANE1			(1 << 1)
#define    ZYNQMP_DP_PHY_STATUS_RESET_LANE0			(1 << 0)
#define    ZYNQMP_DP_PHY_STATUS_RESET_LANE10			(3 << 0)
#define    ZYNQMP_DP_PHY_STATUS_ALL_READY			0x13
#define ZYNQMP_DP_INT_STATUS			0x03a0
#define    ZYNQMP_DP_INT_STATUS_VSYNC_TS			(1u << 31)
#define    ZYNQMP_DP_INT_STATUS_EXT_VSYNC_TS			(1 << 30)
#define    ZYNQMP_DP_INT_STATUS_CUST_TS				(1 << 29)
#define    ZYNQMP_DP_INT_STATUS_CUST_TS_2			(1 << 28)
#define    ZYNQMP_DP_INT_STATUS_CHBUF_OVERFLW(n)		(1 << 27 - (n))
#define    ZYNQMP_DP_INT_STATUS_CHBUF_UNDERFLW(n)		(1 << 21 - (n))
#define    ZYNQMP_DP_INT_STATUS_PIXEL0_MATCH			(1 << 15)
#define    ZYNQMP_DP_INT_STATUS_PIXEL1_MATCH			(1 << 14)
#define    ZYNQMP_DP_INT_STATUS_VBLNK_START			(1 << 13)
#define    ZYNQMP_DP_INT_STATUS_LIV_ABUF_UNDRFLW		(1 << 12)
#define    ZYNQMP_DP_INT_STATUS_EXT_PKT_TXD			(1 << 5)
#define    ZYNQMP_DP_INT_STATUS_HPD_PULSE_DET			(1 << 4)
#define    ZYNQMP_DP_INT_STATUS_REPLY_TIMEOUT			(1 << 3)
#define    ZYNQMP_DP_INT_STATUS_REPLY_RECEIVED			(1 << 2)
#define    ZYNQMP_DP_INT_STATUS_HPD_EVENT			(1 << 1)
#define    ZYNQMP_DP_INT_STATUS_HPD_IRQ				(1 << 0)
#define    ZYNQMP_DP_INT_ALL					0xfffff03fu
#define ZYNQMP_DP_INT_EN			0x03a8
#define ZYNQMP_DP_INT_DS			0x03ac

#define ZYNQMP_V_BLEND_LAYER0_CONTROL		0x0018
#define ZYNQMP_V_BLEND_LAYER1_CONTROL		0x001c
#define    ZYNQMP_V_BLEND_LAYER_CONTROL_BYPASS			(1 << 8)
#define    ZYNQMP_V_BLEND_LAYER_CONTROL_RGB_MODE		(1 << 1)
#define    ZYNQMP_V_BLEND_LAYER_CONTROL_EN_US			(1 << 0)

#define ZYNQMP_AV_BUF_FORMAT			0x0000
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_MASK			(0xf << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_RGBA8888		(0 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_ABGR8888		(1 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_RGB888		(2 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_BGR888		(3 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_RGBA5551		(4 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_RGBA4444		(5 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_RGB565		(6 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_8BPP			(7 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_4BPP			(8 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_2BPP			(9 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_1BPP			(10 << 8)
#define    ZYNQMP_AV_BUF_FORMAT_NL_VID_MASK			0x1f
#define    ZYNQMP_AV_BUF_FORMAT_NL_VID_CbY0CrY1		0 /* 422 interleaved */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_CrY0CbY1		1 /* 422 interleaved */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_Y0CrY1Cb		2 /* 422 interleaved */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_Y0CbY1Cr		3 /* 422 interleaved */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16		4 /* 422 planar */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV24		5 /* planar */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16ci		6 /* 422 semi planar */
#define    ZYNQMP_AV_BUF_FORMAT_NL_VID_MONOCHROME	7
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16ci2		8 /* 422 semi planar */
#define    ZYNQMP_AV_BUF_FORMAT_NL_VID_YUV444		9
#define    ZYNQMP_AV_BUF_FORMAT_NL_VID_RGB888		10
#define    ZYNQMP_AV_BUF_FORMAT_NL_VID_RGBA8880		11
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_RGB888_10BPC	12
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YUV444_10BPC	13
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16CI2_10BPC	14 /* CbCr swap */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16CI_10BPC	15 /* 422 semi planar*/
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16_10BPC	16 /* 422 planar */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV24_10BPC	17
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_Y_ONLY_10BPC	18
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16_420		19 /* 420 planar */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16CI_420	20
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16CI2_420	21 /* CbCr swap */
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16_420_10BPC	22
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16CI_420_10BPC	23
#define	   ZYNQMP_AV_BUF_FORMAT_NL_VID_YV16CI2_420_10BPC 24
#define ZYNQMP_AV_CHBUF(n)			(0x0010 + 4 * (n))
#define    ZYNQMP_AV_CHBUF_BURST_LEN_MASK			(0x1f << 2)
#define    ZYNQMP_AV_CHBUF_BURST_LEN_SHIFT			2
#define    ZYNQMP_AV_CHBUF_BURST_LEN(n)				((n) << 2)
#define    ZYNQMP_AV_CHBUF_FLUSH				(1 << 1)
#define    ZYNQMP_AV_CHBUF_EN					(1 << 0)
#define ZYNQMP_AV_BUF_STC_CONTROL		0x002c
#define ZYNQMP_AV_BUF_OUT_SEL			0x0070
#define    ZYNQMP_AV_BUF_OUT_SEL_AUD_ST2			(1 << 6)
#define    ZYNQMP_AV_BUF_OUT_SEL_AUD_ST1_MASK			(3 << 4)
#define    ZYNQMP_AV_BUF_OUT_SEL_AUD_ST1_LIVE			(0 << 4)
#define    ZYNQMP_AV_BUF_OUT_SEL_AUD_ST1_MEM			(1 << 4)
#define    ZYNQMP_AV_BUF_OUT_SEL_AUD_ST1_PATT			(2 << 4)
#define    ZYNQMP_AV_BUF_OUT_SEL_AUD_ST1_NONE			(3 << 4)
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST2_MASK			(3 << 2)
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST2_DISABLE		(0 << 2)
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST2_ENABLE_MEM		(1 << 2)
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST2_ENABLE_LIVE		(2 << 2)
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST2_NONE			(3 << 2)
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST1_MASK			3
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST1_LIVE			0
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST1_MEM			1
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST1_PATT			2
#define    ZYNQMP_AV_BUF_OUT_SEL_VID_ST1_NONE			3
#define ZYNQMP_AV_BUF_AUD_VID_CLK_SRC		0x0120
#define    ZYNQMP_AV_BUF_AUD_VID_CLK_SRC_VID_TIMING_PS		(1 << 2)
#define    ZYNQMP_AV_BUF_AUD_VID_CLK_SRC_AUD_CLK_PS		(1 << 1)
#define    ZYNQMP_AV_BUF_AUD_VID_CLK_SRC_VID_CLK_PS		(1 << 0)


/* Defines for DP specification.  (incomplete.) */
#define AUX_NATIVE_REPLY_ACK			(0 << 4)

#define DP_DPCD_REV			0x000
#define DP_MAX_LINK_RATE		0x001
#define DP_MAX_LANE_COUNT		0x002
#define    DP_MAX_LANE_COUNT_MASK		0x1f
#define    DP_PATT3_SUPPORT			(1 << 6) /* v1.2 */
#define    DP_ENHANCED_FRAME_CAP		(1 << 7)
#define DP_MAX_DOWNSPREAD		0x003
#define    DP_MAX_DOWNSPREAD_0_5		(1 << 0)
#define DP_MAIN_LINK_CHANNEL_CODING	0x006
#define DP_TRAIN_AUX_RD_INTVL		0x00e
#define    DP_TRAIN_AUX_RD_INTVL_MASK		0x7f

#define DP_LINK_BW_SET			0x100
#define    DP_LINK_BW_1_62			0x06
#define    DP_LINK_BW_2_7			0x0a
#define    DP_LINK_BW_5_4			0x14
#define DP_LANE_COUNT_SET		0x101
#define    DP_LANE_COUNT_ENHANCED_FRAME_EN	(1 << 7)
#define    DP_LANE_COUNT_MASK			0x0f
#define DP_TRAINING_PATTERN_SET		0x102
#define    DP_TRAINING_PATTERN_1		1
#define    DP_TRAINING_PATTERN_2		2
#define    DP_LINK_SCRAMBLING_DISABLE		(1 << 5)
#define DP_TRAINING_LANE0_SET		0x103
#define DP_TRAINING_LANE1_SET		0x104
#define DP_TRAINING_LANE2_SET		0x105
#define DP_TRAINING_LANE3_SET		0x106
#define    DP_TRAIN_MAX_PRE_EMPHASIS_REACHED	(1 << 5)
#define    DP_TRAIN_PRE_EMPHASIS_MASK		(3 << 3)
#define    DP_TRAIN_PRE_EMPHASIS_SHIFT		3
#define    DP_TRAIN_PRE_EMPHASIS_0		(0 << 3)
#define    DP_TRAIN_PRE_EMPHASIS_3_5		(1 << 3)
#define    DP_TRAIN_PRE_EMPHASIS_6		(2 << 3)
#define    DP_TRAIN_PRE_EMPHASIS_9_5		(3 << 3)
#define    DP_TRAIN_MAX_SWING_REACHED		(1 << 2)
#define    DP_TRAIN_VOLTAGE_SWING_MASK		(3 << 0)
#define    DP_TRAIN_VOLTAGE_SWING_SHIFT		0
#define    DP_TRAIN_VOLTAGE_SWING_400		(0 << 0)
#define    DP_TRAIN_VOLTAGE_SWING_600		(1 << 0)
#define    DP_TRAIN_VOLTAGE_SWING_800		(2 << 0)
#define    DP_TRAIN_VOLTAGE_SWING_1200		(3 << 0)
#define DP_DOWNSPREAD_CTRL		0x107
#define    DP_SPREAD_AMP_0_5			(1 << 4)
#define DP_MAIN_LINK_CHANNEL_CODING_SET	0x108
#define    DP_SET_ANSI_8B10B			1

#define DP_SINK_COUNT			0x200
#define DP_LANE0_1_STATUS		0x202
#define DP_LANE2_3_STATUS		0x203
#define    DP_LANE0_CR_DONE			(1 << 0)
#define    DP_LANE0_CHANNEL_EQ_DONE		(1 << 1)
#define    DP_LANE0_SYMBOL_LOCKED		(1 << 2)
#define    DP_LANE1_CR_DONE			(1 << 4)
#define    DP_LANE1_CHANNEL_EQ_DONE		(1 << 5)
#define    DP_LANE1_SYMBOL_LOCKED		(1 << 6)
#define    DP_CHANNEL0_EQ_BITS			(7 << 0)
#define    DP_CHANNEL1_EQ_BITS			(7 << 4)
#define DP_LANE_ALIGN_STATUS_UPDATED	0x204
#define    DP_INTERLANE_ALIGN_DONE		(1 << 0)
#define DP_ADJUST_REQUEST_LANE0_1	0x206
#define DP_ADJUST_REQUEST_LANE2_3	0x207
#define    DP_ADJUST_VOLTAGE_SWING_LANE0_MASK	(3 << 0)
#define    DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT	0
#define    DP_ADJUST_PRE_EMPHASIS_LANE0_MASK	(3 << 2)
#define    DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT	2
#define    DP_ADJUST_VOLTAGE_SWING_LANE1_MASK	(3 << 4)
#define    DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT	4
#define    DP_ADJUST_PRE_EMPHASIS_LANE1_MASK	(3 << 6)
#define    DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT	6

#define DP_SET_POWER			0x600
#define    DP_SET_POWER_D0			1
#define    DP_SET_POWER_D3			2
#define    DP_SET_POWER_MASK			3

#define ZYNQMP_DP_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define ZYNQMP_DP_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZYNQMP_DP_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)
#define ZYNQMP_DP_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), "fb", MTX_DEF)
#define ZYNQMP_DP_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->sc_mtx)

static int zynqmp_dp_attach(device_t);
static int zynqmp_dp_detach(device_t);

#define RD4_DP(sc, off)		(bus_read_4((sc)->mem_res[0], (off)))
#define WR4_DP(sc, off, val)	(bus_write_4((sc)->mem_res[0], (off), (val)))
#define RD4_VB(sc, off)		(bus_read_4((sc)->mem_res[1], (off)))
#define WR4_VB(sc, off, val)	(bus_write_4((sc)->mem_res[1], (off), (val)))
#define RD4_AV(sc, off)		(bus_read_4((sc)->mem_res[2], (off)))
#define WR4_AV(sc, off, val)	(bus_write_4((sc)->mem_res[2], (off), (val)))
#define RD4_AU(sc, off)		(bus_read_4((sc)->mem_res[3], (off)))
#define WR4_AU(sc, off, val)	(bus_write_4((sc)->mem_res[3], (off), (val)))

static int
zynqmp_dp_setup_fbd(struct zynqmp_dp_softc *sc)
{
	int error;

	memset(&sc->info, 0, sizeof(sc->info));

	/* Determine stride and frame buffer size. */
	sc->fb_stride = sc->width * FB_DEPTH / 8;
	if (sc->fb_stride % FB_STRIDE_ALIGN != 0)
		sc->fb_stride += (FB_STRIDE_ALIGN -
		    sc->fb_stride % FB_STRIDE_ALIGN);
	sc->fb_size = sc->fb_stride * sc->height;

	/* Allocate frame buffer memory. */
	sc->fb_vaddr = kmem_alloc_contig(sc->fb_size, M_NOWAIT | M_ZERO, 0, ~0,
	    FB_ALIGN, 0, VM_MEMATTR_WRITE_THROUGH);
	if (sc->fb_vaddr == 0) {
		device_printf(sc->dev, "failed to allocate FB memory\n");
		return (ENOMEM);
	}
	sc->fb_paddr = pmap_kextract(sc->fb_vaddr);

	sc->info.fb_name = device_get_nameunit(sc->dev);
	sc->info.fb_vbase = (intptr_t)sc->fb_vaddr;
	sc->info.fb_pbase = sc->fb_paddr;
	sc->info.fb_size = sc->fb_size;
	sc->info.fb_stride = sc->fb_stride;
	sc->info.fb_width = sc->width;
	sc->info.fb_height = sc->height;
	sc->info.fb_depth = sc->info.fb_bpp = FB_DEPTH;

	sc->fbdev = device_add_child(sc->dev, "fbd", device_get_unit(sc->dev));
	if (sc->fbdev == NULL) {
		device_printf(sc->dev, "failed to add fbd child\n");
		return (ENOENT);
	}
	error = device_probe_and_attach(sc->fbdev);
	if (error) {
		device_printf(sc->dev, "failed to attach fbd device\n");
		return (error);
	}

	return (0);
}

static void
zynqmp_dp_teardown_fbd(struct zynqmp_dp_softc *sc)
{
	if (sc->fbdev != NULL) {
		device_delete_child(sc->dev, sc->fbdev);
		sc->fbdev = NULL;
	}
	if (sc->fb_vaddr) {
		kmem_free(sc->fb_vaddr, sc->fb_size);
		sc->fb_vaddr = 0;
	}
}

/* XXX: DEBUG */
static void zynqmp_dp_dump_settings(struct zynqmp_dp_softc *);
static int
zynqmp_dp_dump(SYSCTL_HANDLER_ARGS)
{
	int error, i;
	struct zynqmp_dp_softc *sc = (struct zynqmp_dp_softc *)arg1;

	error = sysctl_wire_old_buffer(req, sizeof(int));
	if (error == 0) {
		i = 0;
		error = sysctl_handle_int(oidp, &i, 0, req);
	}
	if (error || req->newptr == NULL)
		return (error);

	printf("Dumping registers:\n");
	printf("  ZYNQMP_DP_PHY_STATUS:\t\t0x%08x\n",
	    RD4_DP(sc, ZYNQMP_DP_PHY_STATUS));
	printf("  ZYNQMP_DP_INT_SIGNAL_STATE:\t0x%08x\n",
	    RD4_DP(sc, ZYNQMP_DP_INTERRUPT_SIGNAL_STATE));
	printf("  ZYNQMP_DP_INT_STATUS:\t\t0x%08x\n",
	    RD4_DP(sc, ZYNQMP_DP_INT_STATUS));

	printf("Dumping receiver settings:\n");
	zynqmp_dp_dump_settings(sc);

	return (0);
}

static void
zynqmp_dp_get_size(struct zynqmp_dp_softc *sc)
{

	/* For now, just use defaults. */
	sc->width =		DEFAULT_WIDTH;
	sc->h_front_porch =	DEFAULT_H_FRONT_PORCH;
	sc->h_sync =		DEFAULT_H_SYNC_WIDTH;
	sc->h_back_porch =	DEFAULT_H_BACK_PORCH;
	sc->height =		DEFAULT_HEIGHT;
	sc->v_front_porch =	DEFAULT_V_FRONT_PORCH;
	sc->v_sync =		DEFAULT_V_SYNC_WIDTH;
	sc->v_back_porch =	DEFAULT_V_BACK_PORCH;
	sc->bits_per_pixel =	DEFAULT_BITS_PER_PIXEL;
	sc->pixclk_khz =	DEFAULT_PIX_CLK_KHZ;
}

static void
zynqmp_dp_add_sysctls(struct zynqmp_dp_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child;

	ctx = device_get_sysctl_ctx(sc->dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	if (ctx == NULL || child == NULL) {
		device_printf(sc->dev, "could not add sysctls\n");
		return;
	}

	/* DEBUG */
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "_dump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_SECURE,
	    sc, 0, zynqmp_dp_dump, "I", "dump displayport regs");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "_ref_clk", CTLFLAG_RD,
	    &sc->vref_clk_freq, 0, "Reference clock frequency");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "h_back_porch", CTLFLAG_RD,
	    &sc->h_back_porch, 0, "Horizontal back porch in pixels");
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "h_sync", CTLFLAG_RD,
	    &sc->h_sync, 0, "Horizontal sync width in pixels");
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "h_front_porch", CTLFLAG_RD,
	    &sc->h_front_porch, 0, "Horizontal front porch in pixels");
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "v_back_porch", CTLFLAG_RD,
	    &sc->v_back_porch, 0, "Vertical back porch in lines");
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "v_sync", CTLFLAG_RD,
	    &sc->v_sync, 0, "Vertical sync width in lines");
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "v_front_porch", CTLFLAG_RD,
	    &sc->v_front_porch, 0, "Vertical front porch in lines");

	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "height", CTLFLAG_RD,
	    &sc->height, 0, "Frame buffer height");
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "width", CTLFLAG_RD,
	    &sc->width, 0, "Frame buffer width");
}

static int
zynqmp_dp_aux_readn(struct zynqmp_dp_softc *sc, uint32_t addr,
		    uint8_t *data, int n)
{
	int i;
	int tries;
	uint32_t reply_stat;
	uint32_t reply_code;
	uint32_t reply_count;

	WR4_DP(sc, ZYNQMP_DP_AUX_ADDRESS, addr);
	WR4_DP(sc, ZYNQMP_DP_AUX_COMMAND, ZYNQMP_DP_AUX_COMMAND_CMD_AUX_RD |
	    (n - 1));

	tries = 1000;
	while (--tries > 0) {
		reply_stat = RD4_DP(sc, ZYNQMP_DP_REPLY_STATUS);
		if ((reply_stat &
		     (ZYNQMP_DP_REPLY_STATUS_REPLY_ERROR |
		      ZYNQMP_DP_REPLY_STATUS_REPLY_RECEIVED)) != 0)
			break;
		DELAY(10);
	}

	reply_code = RD4_DP(sc, ZYNQMP_DP_AUX_REPLY_CODE) &
		ZYNQMP_DP_AUX_REPLY_CODE_CODE0_MASK;
	if (reply_code != AUX_NATIVE_REPLY_ACK) {
		device_printf(sc->dev, "aux read bad reply: 0x%02x.\n",
		    reply_code);
		return (-1);
	}

	if (tries == 0 || reply_stat & ZYNQMP_DP_REPLY_STATUS_REPLY_ERROR) {
		device_printf(sc->dev, "aux read timed out or err.\n");
		return (-1);
	}

	reply_count = RD4_DP(sc, ZYNQMP_DP_REPLY_DATA_COUNT) &
		ZYNQMP_DP_REPLY_DATA_COUNT_MASK;
	if (reply_count != n ) {
		device_printf(sc->dev, "aux read replied %d bytes not %d\n",
		    reply_count, n);
		return (-1);
	}

	for (i = 0; i < n; i++)
		data[i] = RD4_DP(sc, ZYNQMP_DP_AUX_REPLY_DATA);

	return (0);
}

/* XXX: I guess not needed.  All routines use zynqmp_dp_aux_readn() instead.
static int
zynqmp_dp_aux_read(struct zynqmp_dp_softc *sc, uint32_t addr)
{
	uint8_t byte;

	if (zynqmp_dp_aux_readn(sc, addr, &byte, 1) < 0)
		return (-1);

	return (byte);
}
*/

static int
zynqmp_dp_aux_writen(struct zynqmp_dp_softc *sc, uint32_t addr,
		     uint8_t *data, int n)
{
	int i;
	int tries;
	uint32_t reply_stat;
	uint32_t reply_code;

	WR4_DP(sc, ZYNQMP_DP_AUX_ADDRESS, addr);
	for (i = 0; i < n; i++)
		WR4_DP(sc, ZYNQMP_DP_AUX_WRITE_FIFO, data[i]);
	WR4_DP(sc, ZYNQMP_DP_AUX_COMMAND, ZYNQMP_DP_AUX_COMMAND_CMD_AUX_WR |
	    (n - 1));

	tries = 1000;
	while (--tries > 0) {
		reply_stat = RD4_DP(sc, ZYNQMP_DP_REPLY_STATUS);
		if ((reply_stat &
		     (ZYNQMP_DP_REPLY_STATUS_REPLY_ERROR |
		      ZYNQMP_DP_REPLY_STATUS_REPLY_RECEIVED)) != 0)
			break;
		DELAY(10);
	}

	reply_code = RD4_DP(sc, ZYNQMP_DP_AUX_REPLY_CODE) &
	    ZYNQMP_DP_AUX_REPLY_CODE_CODE0_MASK;
	if (reply_code != AUX_NATIVE_REPLY_ACK) {
		device_printf(sc->dev, "aux write bad reply: 0x%02x.\n",
			      reply_code);
		return (-1);
	}

	if (tries == 0 || reply_stat & ZYNQMP_DP_REPLY_STATUS_REPLY_ERROR) {
		device_printf(sc->dev, "aux write timed out or err.\n");
		return (-1);
	}

	return (0);
}

static int
zynqmp_dp_aux_write(struct zynqmp_dp_softc *sc, uint32_t addr, uint8_t data)
{

	return (zynqmp_dp_aux_writen(sc, addr, &data, 1));
}

/* XXX: DEBUG */
static void
zynqmp_dp_dump_settings(struct zynqmp_dp_softc *sc)
{
	int i;
	uint8_t data[9];

	memset(data, 0xff, sizeof(data));

	if (zynqmp_dp_aux_readn(sc, 0x100, data, 9) < 0)
		printf("%s: trouble reading aux\n", __func__);
	for (i = 0; i < 9; i++)
		printf("\taux[0x%x] = %02x\n", 0x100 + i, data[i]);
	if (zynqmp_dp_aux_readn(sc, 0x202, data, 3) < 0)
		printf("%s: trouble reading aux2\n", __func__);
	for (i = 0; i < 3; i++)
		printf("\taux[0x%x] = %02x\n", 0x202 + i, data[i]);
}

static void
zynqmp_dp_start_stream(struct zynqmp_dp_softc *sc)
{
	int words_per_lane;
	int avg_bytes_tu;
	int init_wait;
	const int xfer_unit = 64; /* XXX:not sure will ever be configurable. */

	/* Set up main stream. */
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_HTOTAL, sc->width +
	    sc->h_front_porch + sc->h_sync + sc->h_back_porch);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_HSWIDTH, sc->h_sync);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_HSTART, sc->h_sync +
	    sc->h_back_porch);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_HRES, sc->width);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_VTOTAL, sc->height +
	    sc->v_front_porch + sc->v_sync + sc->v_back_porch);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_VSWIDTH, sc->v_sync);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_VSTART, sc->v_sync +
	    sc->v_back_porch);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_VRES, sc->height);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_POLARITY,
	    ZYNQMP_DP_MAIN_STREAM_POLARITY_HSYNC |
	    ZYNQMP_DP_MAIN_STREAM_POLARITY_VSYNC);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_MISC0,
	    ZYNQMP_DP_MAIN_STREAM_MISC0_COMP_FORMAT_RGB |
	    ZYNQMP_DP_MAIN_STREAM_MISC0_BPC_8);
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_MISC1, 0);

	words_per_lane = (sc->width * sc->bits_per_pixel + 15) / 16;
	words_per_lane -= sc->lane_ct;
	words_per_lane += words_per_lane % sc->lane_ct;
	WR4_DP(sc, ZYNQMP_DP_USER_DATA_COUNT_PER_LANE, words_per_lane);

	/* WR4_DP(sc, ZYNQMP_DP_MSA_TRANSFER_UNIT_SIZE, xfer_unit); */
	/* Calculate average bytes per TU times 1000. */
	avg_bytes_tu = (sc->pixclk_khz * xfer_unit * sc->bits_per_pixel / 8) /
	    (sc->link_bw * 27 * sc->lane_ct);
	WR4_DP(sc, ZYNQMP_DP_FRAC_BYTES_PER_TU, avg_bytes_tu % 1000);
	avg_bytes_tu /= 1000;
	WR4_DP(sc, ZYNQMP_DP_MIN_BYTES_PER_TU, avg_bytes_tu);
	if (avg_bytes_tu > xfer_unit)
		init_wait = 0;
	else if (avg_bytes_tu < 4)
		init_wait = xfer_unit;
	else
		init_wait = xfer_unit - avg_bytes_tu;
	WR4_DP(sc, ZYNQMP_DP_INIT_WAIT, init_wait);

	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_ENABLE, 1);

	/* Video blender (send layer 1 (graphics) directly). */
	WR4_VB(sc, ZYNQMP_V_BLEND_LAYER0_CONTROL, 0);
	WR4_VB(sc, ZYNQMP_V_BLEND_LAYER1_CONTROL,
	    ZYNQMP_V_BLEND_LAYER_CONTROL_BYPASS |
	    ZYNQMP_V_BLEND_LAYER_CONTROL_RGB_MODE);

	/* Set up AV buffer manager. */
	WR4_AV(sc, ZYNQMP_AV_BUF_FORMAT,
	    ZYNQMP_AV_BUF_FORMAT_NL_GRAPHX_BGR888);
	WR4_AV(sc, ZYNQMP_AV_BUF_AUD_VID_CLK_SRC,
	    ZYNQMP_AV_BUF_AUD_VID_CLK_SRC_VID_TIMING_PS |
	    ZYNQMP_AV_BUF_AUD_VID_CLK_SRC_AUD_CLK_PS |
	    ZYNQMP_AV_BUF_AUD_VID_CLK_SRC_VID_CLK_PS);
	WR4_AV(sc, ZYNQMP_AV_CHBUF(sc->dpdma_chan), ZYNQMP_AV_CHBUF_FLUSH);
	WR4_AV(sc, ZYNQMP_AV_CHBUF(sc->dpdma_chan),
	    ZYNQMP_AV_CHBUF_BURST_LEN(15) | ZYNQMP_AV_CHBUF_EN);
	WR4_AV(sc, ZYNQMP_AV_BUF_OUT_SEL,
	    ZYNQMP_AV_BUF_OUT_SEL_VID_ST2_ENABLE_MEM);
}

static int
zynqmp_dp_wait_phy_ready(struct zynqmp_dp_softc *sc)
{
	int tries;
	uint32_t val;

	/* XXX: okay, this should check to see if the rate changed
	 * and check the CHANGE_DONE bits.  It should also take
	 * into account if only one lane is on.
	 */
	tries = 100;
	while (tries-- > 0) {
		if (((val = RD4_DP(sc, ZYNQMP_DP_PHY_STATUS)) &
			ZYNQMP_DP_PHY_STATUS_ALL_READY) ==
		    ZYNQMP_DP_PHY_STATUS_ALL_READY)
			break;
		DELAY(100);
	}

	return (tries < 0 ? -1 : 0);
}

static int
zynqmp_dp_set_bw(struct zynqmp_dp_softc *sc)
{

	/* Disable transmitter. */
	WR4_DP(sc, ZYNQMP_DP_TRANSMITTER_ENABLE, 0);

	switch (sc->link_bw) {
	case DP_LINK_BW_5_4:
		WR4_DP(sc, ZYNQMP_DP_PHY_CLOCK_SELECT,
		    ZYNQMP_DP_PHY_CLOCK_SELECT_5_4GBPS);
		break;
	case DP_LINK_BW_2_7:
		WR4_DP(sc, ZYNQMP_DP_PHY_CLOCK_SELECT,
		    ZYNQMP_DP_PHY_CLOCK_SELECT_2_7GBPS);
		break;
	case DP_LINK_BW_1_62:
		WR4_DP(sc, ZYNQMP_DP_PHY_CLOCK_SELECT,
		    ZYNQMP_DP_PHY_CLOCK_SELECT_1_62GBPS);
		break;
	default:
		device_printf(sc->dev, "unknown link_bw settings: 0x%x\n",
		    sc->link_bw);
		return (-1);
	}
	WR4_DP(sc, ZYNQMP_DP_LINK_BW_SET, sc->link_bw);

	/* Wait for PHY ready. */
	if (zynqmp_dp_wait_phy_ready(sc)) {
		device_printf(sc->dev, "timed out waiting for phy.\n");
		return (-1);
	}

	/* Enable transmitter. */
	WR4_DP(sc, ZYNQMP_DP_TRANSMITTER_ENABLE, 1);

	if (zynqmp_dp_aux_write(sc, DP_LINK_BW_SET, sc->link_bw) < 0)
		return (-2);

	return (0);
}

static int
zynqmp_dp_lower_bw(struct zynqmp_dp_softc *sc)
{
	switch (sc->link_bw) {
	case DP_LINK_BW_5_4:
		sc->link_bw = DP_LINK_BW_2_7;
		break;
	case DP_LINK_BW_2_7:
		sc->link_bw = DP_LINK_BW_1_62;
		break;
	case DP_LINK_BW_1_62:
		/* At lowest speed. */
		return (-1);
	default:
		device_printf(sc->dev, "unknown link_bw settings: 0x%x\n",
		    sc->link_bw);
		return (-1);
	}

	return (zynqmp_dp_set_bw(sc));
}

/* Set PHYs and training set. */
static int
zynqmp_dp_train_set(struct zynqmp_dp_softc *sc, int p_level, int v_level)
{
	int i;
	uint8_t train_set[MAX_LANES];

	/* Adjust our PHYs */
	for (i = 0; i < sc->lane_ct; i++) {
		zynqmp_phy_margining_factor(sc->phyxref[i], p_level, v_level);
		zynqmp_phy_override_deemph(sc->phyxref[i], p_level, v_level);
		WR4_DP(sc, ZYNQMP_DP_SUB_TX_PHY_PRECURSOR_LANE(i), 2);
	}

	/* Write training sets. */
	train_set[0] = v_level << DP_TRAIN_VOLTAGE_SWING_SHIFT;
	if ((v_level << DP_TRAIN_VOLTAGE_SWING_SHIFT) ==
	    DP_TRAIN_VOLTAGE_SWING_1200)
		train_set[0] |= DP_TRAIN_MAX_SWING_REACHED;
	train_set[0] |= p_level << DP_TRAIN_PRE_EMPHASIS_SHIFT;
	if ((p_level << DP_TRAIN_PRE_EMPHASIS_SHIFT) ==
	    DP_TRAIN_PRE_EMPHASIS_9_5)
		train_set[0] |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;
	for (i = 1; i < MAX_LANES; i++)
		train_set[i] = train_set[0];
	if (zynqmp_dp_aux_writen(sc, DP_TRAINING_LANE0_SET, train_set,
		MAX_LANES) < 0)
		return (-2);

	return (0);
}

/* Handle a training adjustment request.  Returns -2 if problems occur with
 * aux channel, otherwise, return the voltage set.
 */
static int
zynqmp_dp_train_adj(struct zynqmp_dp_softc *sc, uint8_t linkstat[])
{
	int v_level, v;
	int p_level, p;

	/* Parse voltage requests 0 and 1. */
	v_level = (linkstat[4] & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK) >>
	    DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
	if (sc->lane_ct > 1) {
		v = (linkstat[4] & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK) >>
		    DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
		if (v > v_level)
			v_level = v;
	}

	/* Parse preemphasis requests 0 and 1. */
	p_level = (linkstat[4] & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK) >>
	    DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
	if (sc->lane_ct > 1) {
		p = (linkstat[4] & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK) >>
		    DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;
		if (p > p_level)
			p_level = p;
	}

	if (zynqmp_dp_train_set(sc, p_level, v_level) < 0)
		return(-2);

	v_level <<= DP_TRAIN_VOLTAGE_SWING_SHIFT;
	if (v_level == DP_TRAIN_VOLTAGE_SWING_1200)
		v_level |= DP_TRAIN_MAX_SWING_REACHED;

	return (v_level);
}

static int
zynqmp_dp_training_day(struct zynqmp_dp_softc *sc)
{
	int tries;
	int voltage, prevvoltage, samevoltage;
	int statmask;
	int error = 0;
	uint8_t linkstat[6];

	/* Training pattern 1. */
	WR4_DP(sc, ZYNQMP_DP_SCRAMBLING_DISABLE, 1);
	WR4_DP(sc, ZYNQMP_DP_TRAINING_PATTERN_SET, 1);
	if (zynqmp_dp_aux_write(sc, DP_TRAINING_PATTERN_SET,
		DP_TRAINING_PATTERN_1 | DP_LINK_SCRAMBLING_DISABLE))
		return (-1);

	zynqmp_dp_train_set(sc, 0, 0);

	voltage = 0;
	prevvoltage = -1;
	samevoltage = 0;
	tries = 100;
	while (tries-- > 0) {
		DELAY(400); /* XXX: check DPCD[0xe] ... */

		/* Read link status. */
		if (zynqmp_dp_aux_readn(sc, DP_LANE0_1_STATUS,
		    linkstat, 6) < 0) {
			error = -2;
			goto fail;
		}

		DPRINTF(3,
		    "%s: patt1: status: %02x %02x %02x %02x %02x %02x\n",
		    __func__, linkstat[0], linkstat[1], linkstat[2],
		    linkstat[3], linkstat[4], linkstat[5]);

		statmask = DP_LANE0_CR_DONE |
		    (sc->lane_ct > 1 ? DP_LANE1_CR_DONE : 0);
		if ((linkstat[0] & statmask) == statmask)
			break;

		/* Have we used the same voltage 5 times? */
		if (voltage == prevvoltage)
			samevoltage++;
		else {
			prevvoltage = voltage;
			samevoltage = 0;
		}
		if (samevoltage >= 5)
			break;

		/* Done with pattern 1 if we've reached max voltage. */
		if ((voltage & DP_TRAIN_MAX_SWING_REACHED) != 0)
			break;

		/* Make adjustments. */
		if ((voltage = zynqmp_dp_train_adj(sc, linkstat)) < 0) {
			error = -2;
			goto fail;
		}
	}

	if (tries < 0) {
		device_printf(sc->dev, "training pattern 1 loop stuck.\n");
		error = -1;
		goto fail;
	}

	/* Training pattern 2. */
	WR4_DP(sc, ZYNQMP_DP_TRAINING_PATTERN_SET, 2);
	if (zynqmp_dp_aux_write(sc, DP_TRAINING_PATTERN_SET,
		DP_TRAINING_PATTERN_2 | DP_LINK_SCRAMBLING_DISABLE) < 0) {
		error = -2;
		goto fail;
	}

	tries = 5;
	while (tries-- > 0) {
		DELAY(400); /* XXX: check DPCD[0xe] ... */

		/* Read link status. */
		if (zynqmp_dp_aux_readn(sc, DP_LANE0_1_STATUS,
		    linkstat, 6) < 0) {
			error = -2;
			goto fail;
		}

		DPRINTF(3,
		    "%s: patt2: status: %02x %02x %02x %02x %02x %02x\n",
		    __func__, linkstat[0], linkstat[1], linkstat[2],
		    linkstat[3], linkstat[4], linkstat[5]);

		statmask = DP_CHANNEL0_EQ_BITS |
		    (sc->lane_ct > 1 ? DP_CHANNEL1_EQ_BITS : 0);
		if ((linkstat[0] & statmask) == statmask &&
		    (linkstat[2] & DP_INTERLANE_ALIGN_DONE))
			break;

		/* Make adjustments. */
		if (zynqmp_dp_train_adj(sc, linkstat) < 0) {
			error = -2;
			goto fail;
		}
	}

	if (tries < 0) {
		device_printf(sc->dev, "training pattern 2 failed.\n");
		error = -1;
		goto fail;
	}

	/* Done. */
fail:
	if (zynqmp_dp_aux_write(sc, DP_TRAINING_PATTERN_SET, 0) < 0)
		error = -2;
	WR4_DP(sc, ZYNQMP_DP_TRAINING_PATTERN_SET, 0);
	WR4_DP(sc, ZYNQMP_DP_SCRAMBLING_DISABLE, 0);

	return (error);
}

static void
zynqmp_dp_hpd_up(struct zynqmp_dp_softc *sc)
{
	int error;

	DPRINTF(1, "%s:\n", __func__);

	if (zynqmp_dp_aux_readn(sc, DP_DPCD_REV, sc->dpcd_caps,
		DPCD_RX_CAP_SIZE) < 0)
		return;

#ifdef DPDEBUG
	for (int i = 0; i < DPCD_RX_CAP_SIZE; i++)
		DPRINTF(1, "   DP CAPS[%2d]: 0x%02x\n", i, sc->dpcd_caps[i]);
#endif

	/* Get link bw and lane counts. */
	sc->link_bw = MIN(DP_LINK_BW_5_4, sc->dpcd_caps[DP_MAX_LINK_RATE]);
	sc->lane_ct = MIN(sc->phy_ct,
	    sc->dpcd_caps[DP_MAX_LANE_COUNT] & DP_MAX_LANE_COUNT_MASK);

	/* Set lane count and optional enhanced framing. */
	WR4_DP(sc, ZYNQMP_DP_LANE_COUNT_SET, sc->lane_ct);
	if (sc->dpcd_caps[DP_MAX_LANE_COUNT] & DP_ENHANCED_FRAME_CAP) {
		WR4_DP(sc, ZYNQMP_DP_ENHANCED_FRAME_EN, 1);
		if (zynqmp_dp_aux_write(sc, DP_LANE_COUNT_SET,
			sc->lane_ct | DP_LANE_COUNT_ENHANCED_FRAME_EN) < 0)
			return;
	} else {
		WR4_DP(sc, ZYNQMP_DP_ENHANCED_FRAME_EN, 0);
		if (zynqmp_dp_aux_write(sc, DP_LANE_COUNT_SET,
		    sc->lane_ct) < 0)
			return;
	}

	/* Set downspread control. */
	if (sc->dpcd_caps[DP_MAX_DOWNSPREAD] & DP_MAX_DOWNSPREAD_0_5) {
		if (zynqmp_dp_aux_write(sc, DP_DOWNSPREAD_CTRL,
			DP_SPREAD_AMP_0_5) < 0)
			return;
		WR4_DP(sc, ZYNQMP_DP_DOWNSPREAD_CTRL,
		    ZYNQMP_DP_DOWNSPREAD_CTRL_5_0);
	} else {
		if (zynqmp_dp_aux_write(sc, DP_DOWNSPREAD_CTRL, 0) < 0)
			return;
		WR4_DP(sc, ZYNQMP_DP_DOWNSPREAD_CTRL, 0);
	}

	/* Set 8B/10B encoding. */
	if (zynqmp_dp_aux_write(sc, DP_MAIN_LINK_CHANNEL_CODING_SET,
		DP_SET_ANSI_8B10B) < 0)
		return;

	/* Set initial link bw. */
	if (zynqmp_dp_set_bw(sc) < 0)
		return;

	if (zynqmp_dp_aux_write(sc, DP_SET_POWER, DP_SET_POWER_D0) < 0) {
		device_printf(sc->dev, " DP_SET_POWER failed.\n");
		return;
	}

	/* Perform training patterns one and two, lower bw if it fails. */
	for (;;) {
		error = zynqmp_dp_training_day(sc);
		if (!error)
			break;
		else if (error < -1)
			return;

		if(sc->link_bw > DP_LINK_BW_1_62) {
			error = zynqmp_dp_lower_bw(sc);
			if (error)
				return;
		} else
			break;
	}

	zynqmp_dp_start_stream(sc);
	zynqmp_dpdma_start(sc->dpdma_dev, &sc->info, sc->dpdma_chan);
	sc->hpd_state = 1;
}

static void
zynqmp_dp_hpd_down(struct zynqmp_dp_softc *sc)
{

	DPRINTF(1, "%s:\n", __func__);

	/* Stop DPDMA */
	zynqmp_dpdma_stop(sc->dpdma_dev, sc->dpdma_chan);

	/* Stop stream. */
	WR4_DP(sc, ZYNQMP_DP_MAIN_STREAM_ENABLE, 0);
	WR4_AV(sc, ZYNQMP_AV_CHBUF(sc->dpdma_chan), 0);

	sc->hpd_state = 0;
}

static int
zynqmp_dp_init_hw(struct zynqmp_dp_softc *sc)
{

	ZYNQMP_DP_ASSERT_LOCKED(sc);

	/* Disable interrupts. */
	WR4_DP(sc, ZYNQMP_DP_INT_DS, ZYNQMP_DP_INT_ALL);

	/* Reset PHY, disable transmitter. */
	WR4_DP(sc, ZYNQMP_DP_PHY_RESET, ZYNQMP_DP_PHY_RESET_ALL);
	WR4_DP(sc, ZYNQMP_DP_FORCE_SCRAMBLER_RESET, 1);
	WR4_DP(sc, ZYNQMP_DP_TRANSMITTER_ENABLE, 0);

	/* Set up AUX clock divider. Reference clock is TOPSW_LSBUS_CLK. */
	WR4_DP(sc, ZYNQMP_DP_AUX_CLOCK_DIVIDER, 100 |
	    ZYNQMP_DP_AUX_CLOCK_DIV_AUX_PULSE_WIDTH(48));

	/* Clear PHY reset.  Set 8B10B. */
	WR4_DP(sc, ZYNQMP_DP_PHY_RESET, ZYNQMP_DP_PHY_RESET_EN_8B_10B);

	/* Wait for RESET complete and PLL locked. */
	if (zynqmp_dp_wait_phy_ready(sc)) {
		device_printf(sc->dev, "timed out waiting for phy.\n");
		return (ETIMEDOUT);
	}

	/* Enable transmitter. */
	WR4_DP(sc, ZYNQMP_DP_TRANSMITTER_ENABLE, 1);

	/* Enable interrupts. */
	WR4_DP(sc, ZYNQMP_DP_INT_EN, ZYNQMP_DP_INT_STATUS_HPD_EVENT);

	return (0);
}

static void
zynqmp_dp_intr(void *arg)
{
	struct zynqmp_dp_softc *sc = (struct zynqmp_dp_softc *)arg;
	uint32_t status;

	ZYNQMP_DP_LOCK(sc);

	status = RD4_DP(sc, ZYNQMP_DP_INT_STATUS);
	if (status & ZYNQMP_DP_INT_STATUS_HPD_EVENT) {
		/* Hot-plug event. Detected or lost HPD signal. */
		if ((RD4_DP(sc, ZYNQMP_DP_INTERRUPT_SIGNAL_STATE) &
		     ZYNQMP_DP_INTERRUPT_SIGNAL_STATE_HPD_STATE) != 0) {
			if (!sc->hpd_state)
				zynqmp_dp_hpd_up(sc);
			else
				printf("%s: spurious HPD up int?\n", __func__);
		} else {
			if (sc->hpd_state)
				zynqmp_dp_hpd_down(sc);
			else /* XXX: this happens if HPD up fails. */
				device_printf(sc->dev,
				    "spurious HPD down int.\n");
		}

		WR4_DP(sc, ZYNQMP_DP_INT_STATUS,
		    ZYNQMP_DP_INT_STATUS_HPD_EVENT);
	}

	ZYNQMP_DP_UNLOCK(sc);
}

static struct fb_info *
zynqmp_dp_getinfo(device_t dev)
{
	struct zynqmp_dp_softc *sc = device_get_softc(dev);

	return (&sc->info);
}

static int
zynqmp_dp_get_phys(struct zynqmp_dp_softc *sc)
{
	phandle_t node = ofw_bus_get_node(sc->dev);
	int i;
	int error;
	int index;
	pcell_t *cells;
	int ncells;
	char name[16];

	for (i = 0; i < MAX_LANES; i++) {
		sprintf(name, "dp-phy%d", i);
		error = ofw_bus_find_string_index(node, "phy-names", name,
		    &index);
		if (error)
			break;

		error = ofw_bus_parse_xref_list_alloc(node, "phys",
		    "#phy-cells", index, &sc->phyxref[i], &ncells, &cells);
		if (error)
			break;
		OF_prop_free(cells);
	}

	sc->phy_ct = i;

	if (sc->phy_ct == 0)
		device_printf(sc->dev, "could not locate phys\n");

	return (sc->phy_ct == 0 ? ENXIO : 0);
}

static device_t
zynqmp_dp_getdpdma(device_t dev, int *pchannel)
{
	phandle_t node;
	phandle_t xref;
	int error;
	int ncells;
	pcell_t *cells;

	node = ofw_bus_get_node(dev);
	if (node <= 0) {
		device_printf(dev, "could not get my node!\n");
		return (NULL);
	}

	node = ofw_bus_find_child(node, "gfx-layer");
	if (node <= 0) {
		device_printf(dev, "could not find gfx-layer\n");
		return (NULL);
	}

	/* Get list of dma xrefs.  We expect only one. */
	error = ofw_bus_parse_xref_list_alloc(node, "dmas", "#dma-cells", 0,
	    &xref, &ncells, &cells);
	if (error) {
		device_printf(dev, "could not parse dmas list\n");
		return (NULL);
	}

	*pchannel = cells[0];

	OF_prop_free(cells);
	return (OF_device_from_xref(xref));
}

static int
zynqmp_dp_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ Display Port device");

	return (BUS_PROBE_DEFAULT);
}

static int
zynqmp_dp_attach(device_t dev)
{
	struct zynqmp_dp_softc *sc = device_get_softc(dev);
	int i;
	int rid;
	int error;
	uint64_t freq64;

	sc->dev = dev;

	ZYNQMP_DP_LOCK_INIT(sc);

	/* Get DPDMA device. */
	sc->dpdma_dev = zynqmp_dp_getdpdma(dev, &sc->dpdma_chan);
	if (sc->dpdma_dev == NULL) {
		device_printf(dev, "could not find dpdma device.\n");
		return (ENXIO);
	}

	/* DP Module device registers: DP, V_BLEND, AV_BUF, AUDIO. */
	for (i = 0; i < 4; i++) {
		rid = i;
		sc->mem_res[i] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &rid, RF_ACTIVE);
		if (sc->mem_res[i] == NULL) {
			device_printf(dev,
			    "could not allocate memory resource %d.\n", i);
			error = ENOMEM;
			goto fail;
		}
	}

	/* Allocate IRQ. */
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "could not allocate IRQ\n");
		error = ENOMEM;
		goto fail;
	}

	/* Activate the interrupt. */
	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, zynqmp_dp_intr, sc, &sc->intr_hdl);
	if (error) {
		device_printf(dev, "could not set up interrupt.\n");
		goto fail;
	}

	zynqmp_dp_get_size(sc);
	error = zynqmp_dp_setup_fbd(sc);
	if (error)
		goto fail;

	error = zynqmp_dp_get_phys(sc);
	if (error)
		goto fail;

	/* Retrieve clock even though we cannot change it yet. */
	error = clk_get_by_ofw_name(dev, 0, "dp_vtc_pixel_clk_in",
	    &sc->vref_clk);
	if (error)
		device_printf(dev,
		    "warning: could not get video ref clock.\n");
	else if (clk_enable(sc->vref_clk))
		device_printf(dev,
		    "warning: could not enable video ref clock.\n");
	else if (clk_get_freq(sc->vref_clk, &freq64))
		device_printf(dev,
		    "warning: could not get video ref clock frequency.\n");
	else
		sc->vref_clk_freq = (int)freq64;

	zynqmp_dp_add_sysctls(sc);

	ZYNQMP_DP_LOCK(sc);
	error = zynqmp_dp_init_hw(sc);
	if (error) {
		device_printf(sc->dev, "failed to initialize DP module\n");
		ZYNQMP_DP_UNLOCK(sc);
		goto fail;
	}
	ZYNQMP_DP_UNLOCK(sc);

	return (0);
fail:
	zynqmp_dp_detach(dev);
	return (error);
}

static int
zynqmp_dp_detach(device_t dev)
{
	struct zynqmp_dp_softc *sc = device_get_softc(dev);

	/* Teardown and release interrupt. */
	if (sc->irq_res != NULL) {
		if (sc->intr_hdl) {
			bus_teardown_intr(dev, sc->irq_res, sc->intr_hdl);
			sc->intr_hdl = NULL;
		}
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(sc->irq_res), sc->irq_res);
		sc->irq_res = NULL;
	}

	/* Release resources. */
	if (sc->mem_res[0]) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res[0]), sc->mem_res[0]);
		sc->mem_res[0] = NULL;
	}
	if (sc->mem_res[1]) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res[1]), sc->mem_res[1]);
		sc->mem_res[1] = NULL;
	}

	zynqmp_dp_teardown_fbd(sc);

	ZYNQMP_DP_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_dp_methods[] = {
	DEVMETHOD(device_probe,		zynqmp_dp_probe),
	DEVMETHOD(device_attach,	zynqmp_dp_attach),
	DEVMETHOD(device_detach,	zynqmp_dp_detach),

	DEVMETHOD(fb_getinfo,		zynqmp_dp_getinfo),

	DEVMETHOD_END
};

static devclass_t zynqmp_dp_devclass;

static driver_t zynqmp_dp_driver = {
	"fb",
	zynqmp_dp_methods,
	sizeof(struct zynqmp_dp_softc)
};

DRIVER_MODULE(zynqmp_dp, simplebus, zynqmp_dp_driver, zynqmp_dp_devclass, \
    0, 0);
MODULE_DEPEND(zynqmp_dp, zynqmp_phy, 1, 1, 1);
MODULE_DEPEND(zynqmp_dp, zynqmp_dpdma, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
