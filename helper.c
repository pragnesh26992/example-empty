/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 *
 * Copyright (c) 2015 Xilinx, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <metal/sys.h>
#include <metal/irq.h>
#include "platform_info.h"

/* Main hw machinery initialization entry point, called from main()*/
/* return 0 on success */
int init_system(void)
{
	int ret;
	struct metal_init_params metal_param = METAL_INIT_DEFAULTS;
	register unsigned s5 asm("s5") = 0xffef00;

	/* Low level abstraction layer for openamp initialization */
	metal_init_2(&metal_param);

	s5++;
	return ret;
}

void cleanup_system()
{
	metal_finish();

}
