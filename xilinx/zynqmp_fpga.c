/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Thomas Skibo.
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

/*
 * Zynq UltraScale+ Fpga driver.  This allows programming the PL (FPGA)
 * section of the Zynq UltraScale+
 *
 * Reference: XXX
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/uio.h>
#include <vm/vm.h>
#include <vm/vm_kern.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/xilinx/zynqmp_pm.h>

static struct ofw_compat_data compat_data[] = {
	{ "xlnx,zynqmp-fpga",		1 },
	{ NULL,				0 },
};

struct zynqmp_fpga_softc {
	device_t	dev;
	struct mtx	sc_mtx;
	struct resource	*mem_res;

	int		is_open;
	struct cdev	*sc_ctl_dev;

	vm_offset_t	bit_vaddr;
	bus_addr_t	bit_paddr;
	size_t		bit_size;	/* bitstream size in bytes */
};

#define FPGA_SC_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	FPGA_SC_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define FPGA_SC_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "zynqmp_fpga", MTX_DEF)
#define FPGA_SC_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->sc_mtx);
#define FPGA_SC_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED);

/* FPGA bitstream sizes in bytes.  See TRM Table 11-10. */
#define ZYNQMP_ZU2_3_BITSTREAM_SZ	5568668
#define ZYNQMP_ZU4_5_BITSTREAM_SZ	7658736
#define ZYNQMP_ZU6_BITSTREAM_SZ		26510780
#define ZYNQMP_ZU7_BITSTREAM_SZ		19311092
#define ZYNQMP_ZU9_BITSTREAM_SZ		26510780
#define ZYNQMP_ZU11_BITSTREAM_SZ	23580908
#define ZYNQMP_ZU15_BITSTREAM_SZ	28700744
#define ZYNQMP_ZU17_19_BITSTREAM_SZ	36343112
#define ZYNQMP_ZU21_29_BITSTREAM_SZ	34437356 /* ZU21-ZU29 */
#define ZYNQMP_FPGA_MAX_BITSTREAM	ZYNQMP_ZU17_19_BITSTREAM_SZ

/* Extra room needed for bitstream file header, typically 120 bytes. */
#define BITSTREAM_PAD	256

/* CSU_PCAP_STATUS: all bits read-only.  Read via zynqmp_pm_mmio_read() or
 * zynqmp_pm_fpga_get_status().
 */
#define CSU_PCAP_STATUS			0xFFCA3010
#define     CSU_PCAP_STATUS_WR_IDLE			1
#define     CSU_PCAP_STATUS_RD_IDLE			(1 << 1)
#define     CSU_PCAP_STATUS_PL_DONE			(1 << 2)
#define     CSU_PCAP_STATUS_PL_INIT			(1 << 3)
#define     CSU_PCAP_STATUS_PL_EOS			(1 << 4)
#define     CSU_PCAP_STATUS_PL_SEU_ERR			(1 << 5)
#define     CSU_PCAP_STATUS_PL_CFG_RESET_B		(1 << 6)
#define     CSU_PCAP_STATUS_PL_FIRST_CFG		(1 << 7)
#define     CSU_PCAP_STATUS_GHIGH_B			(1 << 8)
#define     CSU_PCAP_STATUS_GPWRDWN_B			(1 << 9)
#define     CSU_PCAP_STATUS_GTS_CFG_B			(1 << 10)
#define     CSU_PCAP_STATUS_GTS_USR_B			(1 << 11)
#define     CSU_PCAP_STATUS_PCFG_MCAP_MODE		(1 << 12)
#define     CSU_PCAP_STATUS_PCFG_GLOBAL_WE		(1 << 13)

/* cdev entry points. */
static int zynqmp_fpga_open(struct cdev *, int, int, struct thread *);
static int zynqmp_fpga_write(struct cdev *, struct uio *, int);
static int zynqmp_fpga_close(struct cdev *, int, int, struct thread *);

struct cdevsw zynqmp_fpga_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	zynqmp_fpga_open,
	.d_write =	zynqmp_fpga_write,
	.d_close =	zynqmp_fpga_close,
	.d_name =	"fpga",
};

static int
zynqmp_fpga_prog_fpga(struct zynqmp_fpga_softc *sc)
{
	int err;

	/*
	 * XXX I need to rethink this because I program the FPGA on close()
	 * but there is no way to return an error to user-land on close()
	 * (or it's almost universally ignored).  For now, print errors to
	 * the console.
	 */
	err = zynqmp_pm_fpga_load(sc->bit_paddr, sc->bit_size, 0);
	if (err) {
		device_printf(sc->dev, "fpga_load returned err=%d.\n", err);
		return (err);
	}

	return (0);
}

static int
zynqmp_fpga_open(struct cdev *dev, int oflags, int devtype,
    struct thread *td)
{
	struct zynqmp_fpga_softc *sc = dev->si_drv1;

	FPGA_SC_LOCK(sc);
	if (sc->is_open) {
		FPGA_SC_UNLOCK(sc);
		return (EBUSY);
	}
	sc->is_open = 1;
	FPGA_SC_UNLOCK(sc);

	/* Allocate bitstream buffer. */
	sc->bit_vaddr = kmem_alloc_contig(sc->bit_size, M_ZERO, 0, ~0,
	    4, 0, VM_MEMATTR_WRITE_THROUGH);
	if (sc->bit_vaddr == 0) {
		sc->is_open = 0;
		return (ENOMEM);
	}
	sc->bit_paddr = pmap_kextract(sc->bit_vaddr);

	return (0);
}

static int
zynqmp_fpga_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct zynqmp_fpga_softc *sc = dev->si_drv1;
	char *bitmem = (char *)sc->bit_vaddr;

	if (uio->uio_offset + uio->uio_resid > sc->bit_size)
		return (ENOSPC);

	return (uiomove(bitmem + uio->uio_offset, uio->uio_resid, uio));
}

static int
zynqmp_fpga_close(struct cdev *dev, int fflag, int devtype,
    struct thread *td)
{
	struct zynqmp_fpga_softc *sc = dev->si_drv1;

	/* XXX: no way to return error? */
	zynqmp_fpga_prog_fpga(sc);

	/* Free up bitstream buffer. */
	kmem_free(sc->bit_vaddr, sc->bit_size);
	sc->bit_vaddr = 0;

	sc->is_open = 0;

	return (0);
}

static int
zynqmp_fpga_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ FPGA configuration device");
	return (0);
}

static int
zynqmp_fpga_sysctl_pl_init(SYSCTL_HANDLER_ARGS)
{
	/* struct zynqmp_fpga_softc *sc = (struct zynqmp_fpga_softc *)arg1; */
	int pl_init = 0;
	int err;
	uint32_t status;

	err = zynqmp_pm_fpga_get_status(&status);
	if (!err)
		pl_init = (status & CSU_PCAP_STATUS_PL_INIT) != 0;

	return (sysctl_handle_int(oidp, &pl_init, 0, req));
}

static int
zynqmp_fpga_sysctl_pl_done(SYSCTL_HANDLER_ARGS)
{
	/* struct zynqmp_fpga_softc *sc = (struct zynqmp_fpga_softc *)arg1; */
	int pl_done = 0;
	int err;
	uint32_t status;

	err = zynqmp_pm_fpga_get_status(&status);
	if (!err)
		pl_done = (status & CSU_PCAP_STATUS_PL_DONE) != 0;

	return (sysctl_handle_int(oidp, &pl_done, 0, req));
}

static void
zynqmp_fpga_sysctls(struct zynqmp_fpga_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child;

	ctx = device_get_sysctl_ctx(sc->dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	if (ctx == NULL || child == NULL) {
		device_printf(sc->dev, "could not add sysctls\n");
		return;
	}

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "pl_done",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_NEEDGIANT, NULL, 0,
	    zynqmp_fpga_sysctl_pl_done, "I", "PL section config DONE signal");

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "pl_init",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_NEEDGIANT, NULL, 0,
	    zynqmp_fpga_sysctl_pl_init, "I", "PL section config INIT signal");
}

static void
zynqmp_fpga_get_bitsize(struct zynqmp_fpga_softc *sc)
{
	int error;
	uint32_t chipid, chipversion;

	/* Get bitstream size. */
	error = zynqmp_pm_get_chipid(&chipid, &chipversion);
	if (error) {
		device_printf(sc->dev, "failed to get chip id\n");
		chipid = 0;
	}

	switch (chipid) {
	case PM_IDCODE_ZU2:
	case PM_IDCODE_ZU3:
		sc->bit_size = ZYNQMP_ZU2_3_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU4:
	case PM_IDCODE_ZU5:
		sc->bit_size = ZYNQMP_ZU4_5_BITSTREAM_SZ;
	case PM_IDCODE_ZU6:
		sc->bit_size = ZYNQMP_ZU6_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU7:
		sc->bit_size = ZYNQMP_ZU7_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU9:
		sc->bit_size = ZYNQMP_ZU9_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU11:
		sc->bit_size = ZYNQMP_ZU11_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU15:
		sc->bit_size = ZYNQMP_ZU15_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU17:
	case PM_IDCODE_ZU19:
		sc->bit_size = ZYNQMP_ZU17_19_BITSTREAM_SZ;
		break;
	case PM_IDCODE_ZU21:
	case PM_IDCODE_ZU25:
	case PM_IDCODE_ZU27:
	case PM_IDCODE_ZU28:
	case PM_IDCODE_ZU29:
		sc->bit_size = ZYNQMP_ZU21_29_BITSTREAM_SZ;
		break;
	default:
		device_printf(sc->dev, "unknown chip-id: 0x%x\n", chipid);
		sc->bit_size = ZYNQMP_FPGA_MAX_BITSTREAM;
		break;
	}

	/* Padding for file header. */
	sc->bit_size += BITSTREAM_PAD;
}

static int zynqmp_fpga_detach(device_t dev);

static int
zynqmp_fpga_attach(device_t dev)
{
	struct zynqmp_fpga_softc *sc = device_get_softc(dev);
	int error;

	sc->dev = dev;

	FPGA_SC_LOCK_INIT(sc);

	/* Create /dev/fpga */
	sc->sc_ctl_dev = make_dev(&zynqmp_fpga_cdevsw, 0,
	    UID_ROOT, GID_WHEEL, 0600, "fpga");
	if (sc->sc_ctl_dev == NULL) {
		device_printf(dev, "failed to create /dev/fpga\n");
		error = ENXIO;
		goto fail;
	}
	sc->sc_ctl_dev->si_drv1 = sc;

	zynqmp_fpga_get_bitsize(sc);
	zynqmp_fpga_sysctls(sc);

	return (0);
fail:
	zynqmp_fpga_detach(dev);
	return (error);
}

static int
zynqmp_fpga_detach(device_t dev)
{
	struct zynqmp_fpga_softc *sc = device_get_softc(dev);

	/* Get rid of /dev/fpga. */
	if (sc->sc_ctl_dev != NULL)
		destroy_dev(sc->sc_ctl_dev);

	FPGA_SC_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_fpga_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		zynqmp_fpga_probe),
	DEVMETHOD(device_attach,	zynqmp_fpga_attach),
	DEVMETHOD(device_detach,	zynqmp_fpga_detach),

	DEVMETHOD_END
};

static driver_t zynqmp_fpga_driver = {
	"zynqmp_fpga",
	zynqmp_fpga_methods,
	sizeof(struct zynqmp_fpga_softc),
};
static devclass_t zynqmp_fpga_devclass;

DRIVER_MODULE(zynqmp_fpga, simplebus, zynqmp_fpga_driver, \
    zynqmp_fpga_devclass, 0, 0);
MODULE_DEPEND(zynqmp_fpga, zynqmp_pm, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
