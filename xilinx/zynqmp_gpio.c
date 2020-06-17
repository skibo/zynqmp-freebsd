/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018 Thomas Skibo
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
 * A GPIO driver for Xilinx Zynq Ultrascale.
 *
 * The GPIO peripheral on Zynq Ultrascale allows controlling 174 general
 * purpose I/Os.
 *
 * 78 of the pins in the first three banks (32 bits each) sent to the MIO.
 * Any MIO pins not used by a PS peripheral are available as a GPIO pin.
 * Pins 96-191 are sent to the PL (FPGA) section of Zynq as EMIO signals.
 * Pin 191 (EMIO[95]) acts as a reset signal for the PL.
 *
 * The hardware provides a way to use IOs as interrupt sources but the
 * gpio framework doesn't seem to have hooks for this.
 *
 * Reference: Zynq UltraScale+ Device Technical Refernce Manual.
 * (v1.7) December 22, 2017.  Xilinx doc UG1085.  Ch.27.
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
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "gpio_if.h"

#define NUMBANKS	6
#define MAXPIN		(32 * NUMBANKS)

#define MIO_PIN		0	/* pins 0-25, 32-57, 64-89 go to MIO */
#define MIO_PINS_BANK	26	/* 26 MIO pins per bank in the first three */
#define MIO_PIN_LAST	89
#define EMIO_PIN	96	/* pins 64-127 go to PL */
#define NUM_EMIO_PINS	64

#define VALID_PIN(u)	(((u) >= MIO_PIN && (u) <= MIO_PIN_LAST &&	\
			  (u) % 32 < MIO_PINS_BANK) ||			\
			 ((u) >= EMIO_PIN && (u) < EMIO_PIN + NUM_EMIO_PINS))

#define ZGPIO_LOCK(sc)			mtx_lock(&(sc)->sc_mtx)
#define	ZGPIO_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZGPIO_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "gpio", MTX_DEF)
#define ZGPIO_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

struct zynqmp_gpio_softc {
	device_t	dev;
	device_t	busdev;
	struct mtx	sc_mtx;
	struct resource *mem_res;	/* Memory resource */
};

#define WR4(sc, off, val)	bus_write_4((sc)->mem_res, (off), (val))
#define RD4(sc, off)		bus_read_4((sc)->mem_res, (off))


/* Xilinx Zynq UltraScale GPIO register definitions: */
#define ZYNQMP_GPIO_MASK_DATA_LSW(b)	(0x0000 + 8 * (b)) /* maskable wr lo */
#define ZYNQMP_GPIO_MASK_DATA_MSW(b)	(0x0004 + 8 * (b)) /* maskable wr hi */
#define ZYNQMP_GPIO_DATA(b)		(0x0040 + 4 * (b)) /* in/out data */
#define ZYNQMP_GPIO_DATA_RO(b)		(0x0060 + 4 * (b)) /* input data */

#define ZYNQMP_GPIO_DIRM(b)		(0x0204 + 0x40 * (b)) /* dir mode */
#define ZYNQMP_GPIO_OEN(b)		(0x0208 + 0x40 * (b)) /* out enable */
#define ZYNQMP_GPIO_INT_MASK(b)		(0x020c + 0x40 * (b)) /* int mask */
#define ZYNQMP_GPIO_INT_EN(b)		(0x0210 + 0x40 * (b)) /* int enable */
#define ZYNQMP_GPIO_INT_DIS(b)		(0x0214 + 0x40 * (b)) /* int disable */
#define ZYNQMP_GPIO_INT_STAT(b)		(0x0218 + 0x40 * (b)) /* int status */
#define ZYNQMP_GPIO_INT_TYPE(b)		(0x021c + 0x40 * (b)) /* int type */
#define ZYNQMP_GPIO_INT_POLARITY(b)	(0x0220 + 0x40 * (b)) /* int polarity*/
#define ZYNQMP_GPIO_INT_ANY(b)		(0x0224 + 0x40 * (b)) /* any edge */

static struct ofw_compat_data compat_data[] = {
       {"xlnx,zynqmp-gpio-1.0",	1},
       {NULL,			0}
};

static device_t
zynqmp_gpio_get_bus(device_t dev)
{
	struct zynqmp_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
zynqmp_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = MAXPIN;
	return (0);
}

/* Get a specific pin's capabilities. */
static int
zynqmp_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{

	if (!VALID_PIN(pin))
		return (EINVAL);

	*caps = (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT | GPIO_PIN_TRISTATE);

	return (0);
}

/* Get a specific pin's name. */
static int
zynqmp_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{

	if (!VALID_PIN(pin))
		return (EINVAL);

	if (pin <= MIO_PIN_LAST) {
		snprintf(name, GPIOMAXNAME, "MIO_%d",
		    (pin / 32) * MIO_PINS_BANK + pin % 32);
		name[GPIOMAXNAME - 1] = '\0';
	} else {
		snprintf(name, GPIOMAXNAME, "EMIO_%d", pin - EMIO_PIN);
		name[GPIOMAXNAME - 1] = '\0';
	}

	return (0);
}

/* Get a specific pin's current in/out/tri state. */
static int
zynqmp_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(pin))
		return (EINVAL);

	ZGPIO_LOCK(sc);

	if ((RD4(sc, ZYNQMP_GPIO_DIRM(pin >> 5)) & (1U << (pin & 31))) != 0) {
		/* output */
		if ((RD4(sc, ZYNQMP_GPIO_OEN(pin >> 5)) &
		    (1U << (pin & 31))) == 0)
			*flags = (GPIO_PIN_OUTPUT | GPIO_PIN_TRISTATE);
		else
			*flags = GPIO_PIN_OUTPUT;
	} else
		/* input */
		*flags = GPIO_PIN_INPUT;

	ZGPIO_UNLOCK(sc);

	return (0);
}

/* Set a specific pin's in/out/tri state. */
static int
zynqmp_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(pin))
		return (EINVAL);

	ZGPIO_LOCK(sc);

	if ((flags & GPIO_PIN_OUTPUT) != 0) {
		/* Output.  Set or reset OEN too. */
		WR4(sc, ZYNQMP_GPIO_DIRM(pin >> 5),
		    RD4(sc, ZYNQMP_GPIO_DIRM(pin >> 5)) | (1U << (pin & 31)));

		if ((flags & GPIO_PIN_TRISTATE) != 0)
			WR4(sc, ZYNQMP_GPIO_OEN(pin >> 5),
			    RD4(sc, ZYNQMP_GPIO_OEN(pin >> 5)) &
			    ~(1U << (pin & 31)));
		else
			WR4(sc, ZYNQMP_GPIO_OEN(pin >> 5),
			    RD4(sc, ZYNQMP_GPIO_OEN(pin >> 5)) |
			    (1U << (pin & 31)));
	} else {
		/* Input.  Turn off OEN. */
		WR4(sc, ZYNQMP_GPIO_DIRM(pin >> 5),
		    RD4(sc, ZYNQMP_GPIO_DIRM(pin >> 5)) & ~(1U << (pin & 31)));
		WR4(sc, ZYNQMP_GPIO_OEN(pin >> 5),
		    RD4(sc, ZYNQMP_GPIO_OEN(pin >> 5)) & ~(1U << (pin & 31)));
	}

	ZGPIO_UNLOCK(sc);

	return (0);
}

/* Set a specific output pin's value. */
static int
zynqmp_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(pin) || value > 1)
		return (EINVAL);

	/* Fancy register tricks allow atomic set or reset. */
	if ((pin & 16) != 0)
		WR4(sc, ZYNQMP_GPIO_MASK_DATA_MSW(pin >> 5),
		    (0xffff0000U ^ (0x10000U << (pin & 15))) |
		    (value << (pin & 15)));
	else
		WR4(sc, ZYNQMP_GPIO_MASK_DATA_LSW(pin >> 5),
		    (0xffff0000U ^ (0x10000U << (pin & 15))) |
		    (value << (pin & 15)));

	return (0);
}

/* Get a specific pin's input value. */
static int
zynqmp_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *value)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(pin))
		return (EINVAL);

	*value = (RD4(sc, ZYNQMP_GPIO_DATA_RO(pin >> 5)) >> (pin & 31)) & 1;

	return (0);
}

/* Toggle a pin's output value. */
static int
zynqmp_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(pin))
		return (EINVAL);

	ZGPIO_LOCK(sc);

	WR4(sc, ZYNQMP_GPIO_DATA(pin >> 5),
	    RD4(sc, ZYNQMP_GPIO_DATA(pin >> 5)) ^ (1U << (pin & 31)));

	ZGPIO_UNLOCK(sc);

	return (0);
}

static int
zynqmp_gpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Zynq UltraScale+ GPIO driver");

	return (BUS_PROBE_DEFAULT);
}

static int zynqmp_gpio_detach(device_t dev);

static int
zynqmp_gpio_attach(device_t dev)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);
	int rid;

	sc->dev = dev;

	ZGPIO_LOCK_INIT(sc);

	/* Allocate memory. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Can't allocate memory for device");
		zynqmp_gpio_detach(dev);
		return (ENOMEM);
	}

	sc->busdev = gpiobus_attach_bus(dev);
	if (sc->busdev == NULL) {
		zynqmp_gpio_detach(dev);
		return (ENOMEM);
	}

	return (0);
}

static int
zynqmp_gpio_detach(device_t dev)
{
	struct zynqmp_gpio_softc *sc = device_get_softc(dev);

	gpiobus_detach_bus(dev);

	if (sc->mem_res != NULL) {
		/* Release memory resource. */
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);
	}

	ZGPIO_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zynqmp_gpio_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		zynqmp_gpio_probe),
	DEVMETHOD(device_attach,	zynqmp_gpio_attach),
	DEVMETHOD(device_detach,	zynqmp_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		zynqmp_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		zynqmp_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	zynqmp_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	zynqmp_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	zynqmp_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	zynqmp_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		zynqmp_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		zynqmp_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	zynqmp_gpio_pin_toggle),

	DEVMETHOD_END
};

static driver_t zynqmp_gpio_driver = {
	"gpio",
	zynqmp_gpio_methods,
	sizeof(struct zynqmp_gpio_softc),
};
static devclass_t zynqmp_gpio_devclass;

DRIVER_MODULE(zynqmp_gpio, simplebus, zynqmp_gpio_driver,	\
	      zynqmp_gpio_devclass, NULL, NULL);
SIMPLEBUS_PNP_INFO(compat_data);
