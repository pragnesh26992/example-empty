/*
 * Copyright (c) 2014, Mentor Graphics Corporation. All rights reserved.
 * Copyright (c) 2017 - 2018 Xilinx, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       platform_info.c
 *
 * DESCRIPTION
 *
 *       This file define platform specific data and implements APIs to set
 *       platform specific information for OpenAMP.
 *
 **************************************************************************/

#include <openamp/remoteproc.h>
#include <openamp/rpmsg_virtio.h>
#include <metal/atomic.h>
#include <metal/device.h>
#include <metal/io.h>
#include <metal/irq.h>
#include <metal/sys.h>
#include "platform_info.h"
#include "rsc_table.h"

#define KICK_DEV_NAME         "poll_dev"
#define KICK_BUS_NAME         "generic"

/* Cortex R5 memory attributes */
#define DEVICE_SHARED           0x00000001U /* device, shareable */
#define DEVICE_NONSHARED        0x00000010U /* device, non shareable */
#define NORM_NSHARED_NCACHE     0x00000008U /* Non cacheable  non shareable */
#define NORM_SHARED_NCACHE      0x0000000CU /* Non cacheable shareable */
#define PRIV_RW_USER_RW         (0x00000003U<<8U) /* Full Access */

#define SHARED_MEM_PA  0x90000000UL
#define SHARED_MEM_SIZE 0x100000UL
#define SHARED_BUF_OFFSET 0x8000UL

//#define SHARED_MEM_SIZE 0x80000UL
//#define SHARED_BUF_OFFSET 0x80000UL

#ifndef RPMSG_NO_IPI
#define _rproc_wait() asm volatile("wfi")
#endif /* !RPMSG_NO_IPI */

/* Polling information used by remoteproc operations.
 */
static metal_phys_addr_t poll_phys_addr = POLL_BASE_ADDR;

/* Place resource table in special ELF section */
  /* Redefine __section for section name with token */
#define __section_t(S)          __attribute__((__section__(#S)))
#define __resource              __section_t(.resource_table)

#define RPMSG_IPU_C0_FEATURES        1

/* VirtIO rpmsg device id */
#define VIRTIO_ID_RPMSG_             7

/* Remote supports Name Service announcement */
#define VIRTIO_RPMSG_F_NS           0

/* Resource table entries */
#define NUM_VRINGS                  0x02
#define VRING_ALIGN                 0x1000
#define RING_TX                     0x90020000
#define RING_RX                     0x90040000
#define VRING_SIZE                  256

#define NUM_TABLE_ENTRIES           1

struct remote_resource_table resources = {
        /* Version */
        1,

        /* NUmber of table entries */
        NUM_TABLE_ENTRIES,
        /* reserved fields */
        {0, 0,},

        /* Offsets of rsc entries */
        {
         offsetof(struct remote_resource_table, rpmsg_vdev),
         },

        /* Virtio device entry */
        {
         RSC_VDEV, VIRTIO_ID_RPMSG_, RSC_NOTIFY_ID_ANY, RPMSG_IPU_C0_FEATURES, 0, 0, 0,
         NUM_VRINGS, {0, 0},
        },

        /* Vring rsc entry - part of vdev rsc entry */
        {RING_TX, VRING_ALIGN, VRING_SIZE, 1, 0},
        {RING_RX, VRING_ALIGN, VRING_SIZE, 2, 0},
};

struct metal_device kick_device = {
        .name = "poll_dev",
        .bus = NULL,
        .num_regions = 1,
        .regions = {
                {
                        .virt = (void *)POLL_BASE_ADDR,
                        .physmap = &poll_phys_addr,
                        .size = 0x1000,
                        .page_shift = -1UL,
                        .page_mask = -1UL,
                        .mem_flags = DEVICE_NONSHARED | PRIV_RW_USER_RW,
                        .ops = {NULL},
                }
        },
        .node = {NULL},
#ifndef RPMSG_NO_IPI
        .irq_num = 1,
        .irq_info = (void *)IPI_IRQ_VECT_ID,
#endif /* !RPMSG_NO_IPI */
};

static struct remoteproc_priv rproc_priv = {
        .kick_dev_name = KICK_DEV_NAME,
        .kick_dev_bus_name = KICK_BUS_NAME,
#ifndef RPMSG_NO_IPI
        .ipi_chn_mask = IPI_CHN_BITMASK,
#endif /* !RPMSG_NO_IPI */
};

/* processor operations for hil_proc for A9. It defines
 * notification operation and remote processor management. */
extern struct remoteproc_ops sifive_u_proc_ops;

/* Remoteproc instance */
static struct remoteproc rproc_inst;

/* External functions */
extern int init_system(void);
extern void cleanup_system(void);

/* RPMsg virtio shared buffer pool */
static struct rpmsg_virtio_shm_pool shpool;

static struct remoteproc *
platform_create_proc(int proc_index, int rsc_index)
{
	void *rsc_table;
	int rsc_size;
	int ret;

	metal_phys_addr_t pa;

	(void) proc_index;
	rsc_table = get_resource_table(rsc_index, &rsc_size);

        /* Register IPI device */
        if (metal_register_generic_device(&kick_device))
                return NULL;

	/* Initialize remoteproc instance */
	if (!remoteproc_init(&rproc_inst, &sifive_u_proc_ops, &rproc_priv))
		return NULL;

	/*
	 * Mmap shared memories
	 * Or shall we constraint that they will be set as carved out
	 * in the resource table?
	 */
	/* mmap resource table */
	pa = (metal_phys_addr_t)rsc_table;
	(void *)remoteproc_mmap(&rproc_inst, &pa,
				NULL, rsc_size,
				(uintptr_t) NULL,
				&rproc_inst.rsc_io);
	/* mmap shared memory */
	pa = SHARED_MEM_PA;
	(void *)remoteproc_mmap(&rproc_inst, &pa,
				NULL, SHARED_MEM_SIZE,
				(uintptr_t) NULL,
				NULL);

	/* parse resource table to remoteproc */
	ret = remoteproc_set_rsc_table(&rproc_inst, rsc_table, rsc_size);
	if (ret) {
		printf("Failed to intialize remoteproc\r\n");
		remoteproc_remove(&rproc_inst);
		return NULL;
	}
	printf("Initialize remoteproc successfully.\r\n");

	return &rproc_inst;
}

int platform_init(int argc, char *argv[], void **platform)
{
	unsigned long proc_id = 0;
	unsigned long rsc_id = 0;
	struct remoteproc *rproc;

	if (!platform) {
		printf("Failed to initialize platform,"
			   "NULL pointer to store platform data.\r\n");
		return -EINVAL;
	}

	/* Initialize HW system components */
	init_system();

	if (argc >= 2) {
		proc_id = strtoul(argv[1], NULL, 0);
	}

	if (argc >= 3) {
		rsc_id = strtoul(argv[2], NULL, 0);
	}

	rproc = platform_create_proc(proc_id, rsc_id);
	if (!rproc) {
		printf("Failed to create remoteproc device.\r\n");
		return -EINVAL;
	}
	*platform = rproc;
	return 0;
}



struct  rpmsg_device *
platform_create_rpmsg_vdev(void *platform, unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_bind_cb)
{
	struct remoteproc *rproc = platform;
	struct rpmsg_virtio_device *rpmsg_vdev;
	struct virtio_device *vdev;
	void *shbuf;
	struct metal_io_region *shbuf_io;
	int ret;

	rpmsg_vdev = metal_allocate_memory(sizeof(*rpmsg_vdev));
	if (!rpmsg_vdev)
		return NULL;

	shbuf_io = remoteproc_get_io_with_pa(rproc, SHARED_MEM_PA);
	if (!shbuf_io)
		return NULL;
	shbuf = metal_io_phys_to_virt(shbuf_io,
				      SHARED_MEM_PA + SHARED_BUF_OFFSET);

	printf("creating remoteproc virtio\r\n");
	/* TODO: can we have a wrapper for the following two functions? */
	vdev = remoteproc_create_virtio(rproc, vdev_index, role, rst_cb);
	if (!vdev) {
		printf("failed remoteproc_create_virtio\r\n");
		goto err1;
	}

	printf("initializing rpmsg vdev\r\n");
	if (role == VIRTIO_DEV_MASTER) {
		/* Only RPMsg virtio master needs to initialize the
		 * shared buffers pool
		 */
		rpmsg_virtio_init_shm_pool(&shpool, shbuf,
					   (SHARED_MEM_SIZE -
					    SHARED_BUF_OFFSET));

		/* RPMsg virtio slave can set shared buffers pool
		 * argument to NULL
		 */
		ret =  rpmsg_init_vdev(rpmsg_vdev, vdev, ns_bind_cb,
				       shbuf_io, &shpool);
	} else {
		ret =  rpmsg_init_vdev(rpmsg_vdev, vdev, ns_bind_cb,
				       shbuf_io, NULL);
	}
	if (ret) {
		printf("failed rpmsg_init_vdev\r\n");
		goto err2;
	}
	printf("initializing rpmsg vdev\r\n");
	return rpmsg_virtio_get_rpmsg_device(rpmsg_vdev);
err2:
	remoteproc_remove_virtio(rproc, vdev);
err1:
	metal_free_memory(rpmsg_vdev);
	return NULL;
}

int platform_poll(void *priv)
{
        struct remoteproc *rproc = priv;
        struct remoteproc_priv *prproc;
        unsigned int flags;
        int ret;

        prproc = rproc->priv;

        while(1) {
#ifdef RPMSG_NO_IPI
                if (metal_io_read32(prproc->kick_io, 0)) {
                        ret = remoteproc_get_notification(rproc,
                                                          RSC_NOTIFY_ID_ANY);
                        if (ret)
                                return ret;
                        break;
                }
                (void)flags;
#else /* !RPMSG_NO_IPI */
                flags = metal_irq_save_disable();
                if (!(atomic_flag_test_and_set(&prproc->ipi_nokick))) {
                        metal_irq_restore_enable(flags);
                        ret = remoteproc_get_notification(rproc,
                                                          RSC_NOTIFY_ID_ANY);
                        if (ret)
                                return ret;
                        break;
                }
                _rproc_wait();
                metal_irq_restore_enable(flags);
#endif /* RPMSG_NO_IPI */
	}
	return 0;
}

void platform_release_rpmsg_vdev(struct rpmsg_device *rpdev, void *platform)
{
        struct rpmsg_virtio_device *rpvdev;
        struct remoteproc *rproc;

        rpvdev = metal_container_of(rpdev, struct rpmsg_virtio_device, rdev);
        rproc = platform;

        rpmsg_deinit_vdev(rpvdev);
        remoteproc_remove_virtio(rproc, rpvdev->vdev);
}

void platform_cleanup(void *platform)
{
	struct remoteproc *rproc = platform;

	if (rproc)
		remoteproc_remove(rproc);
	cleanup_system();
}
