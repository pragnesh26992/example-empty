/*
 * Copyright (c) 2018 Xilinx, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       sifive_u_rproc.c
 *
 * DESCRIPTION
 *
 *       This file define Xilinx Zynq A9 platform specific remoteproc
 *       implementation.
 *
 **************************************************************************/

#include <openamp/remoteproc.h>
#include <metal/atomic.h>
#include <metal/device.h>
#include <metal/irq.h>
#include "platform_info.h"

/* SCUGIC macros */
#define GIC_DIST_SOFTINT                   0xF00
#define GIC_SFI_TRIG_CPU_MASK              0x00FF0000
#define GIC_SFI_TRIG_SATT_MASK             0x00008000
#define GIC_SFI_TRIG_INTID_MASK            0x0000000F
#define GIC_CPU_ID_BASE                    (1 << 4)

static struct remoteproc *
sifive_u_proc_init(struct remoteproc *rproc,
			struct remoteproc_ops *ops, void *arg)
{
	int ret;

	if (!rproc || !ops)
		return NULL;
	
	rproc->ops = ops;

	return rproc;
}

static void sifive_u_proc_remove(struct remoteproc *rproc)
{
}

static void *
sifive_u_proc_mmap(struct remoteproc *rproc, metal_phys_addr_t *pa,
			metal_phys_addr_t *da, size_t size,
			unsigned int attribute, struct metal_io_region **io)
{
	struct remoteproc_mem *mem;
	metal_phys_addr_t lpa, lda;
	struct metal_io_region *tmpio;

	lpa = *pa;
	lda = *da;

	if (lpa == METAL_BAD_PHYS && lda == METAL_BAD_PHYS)
		return NULL;
	if (lpa == METAL_BAD_PHYS)
		lpa = lda;
	if (lda == METAL_BAD_PHYS)
		lda = lpa;

	mem = metal_allocate_memory(sizeof(*mem));
	if (!mem)
		return NULL;
	tmpio = metal_allocate_memory(sizeof(*tmpio));
	if (!tmpio) {
		metal_free_memory(mem);
		return NULL;
	}
	remoteproc_init_mem(mem, NULL, lpa, lda, size, tmpio);
	/* va is the same as pa in this platform */
	metal_io_init(tmpio, (void *)lpa, &mem->pa, size,
		      sizeof(metal_phys_addr_t)<<3, attribute, NULL);
	remoteproc_add_mem(rproc, mem);
	*pa = lpa;
	*da = lda;
	if (io)
		*io = tmpio;
	return metal_io_phys_to_virt(tmpio, mem->pa);
}

static int sifive_u_proc_notify(struct remoteproc *rproc, uint32_t id)
{
#if 0
	unsigned long mask = 0;

	(void)id;
	if (!rproc)
		return -1;
	prproc = rproc->priv;
	if (!prproc->gic_io)
		return -1;

	mask = ((1 << (GIC_CPU_ID_BASE + prproc->cpu_id)) |
		(prproc->irq_to_notify))
	    & (GIC_SFI_TRIG_CPU_MASK | GIC_SFI_TRIG_INTID_MASK);
	/* Trigger IPI */
	metal_io_write32(prproc->gic_io, GIC_DIST_SOFTINT, mask);
#endif
	return 0;
}

/* processor operations from r5 to a53. It defines
 * notification operation and remote processor managementi operations. */
struct remoteproc_ops sifive_u_proc_ops = {
	.init = sifive_u_proc_init,
	.remove = sifive_u_proc_remove,
	.mmap = sifive_u_proc_mmap,
	.notify = sifive_u_proc_notify,
	.start = NULL,
	.stop = NULL,
	.shutdown = NULL,
};
