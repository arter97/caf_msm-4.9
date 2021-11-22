/*
 *
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <soc/qcom/secure_buffer.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/qseecomi.h>
#include <linux/delay.h>

#define QSEECOM_SCM_EBUSY_WAIT_MS 30
#define QSEECOM_SCM_EBUSY_MAX_RETRY 67

static dma_addr_t dma_pa;
static void *dma_va;

static void call_tz_nand_probe(uint32_t smc_id, struct scm_desc *desc)
{
    int ret = 0;
    int retry_count = 0;

    do
    {
        ret = scm_call2_noretry(smc_id, desc);
        if (ret != 0)
        {
            msleep(QSEECOM_SCM_EBUSY_WAIT_MS);
        }
        if (retry_count == 33)
            printk(KERN_ALERT "secure world has been busy for 1 second!\n");
    } while (ret != 0 && (retry_count++ < QSEECOM_SCM_EBUSY_MAX_RETRY));
    printk(KERN_ALERT "ret = %d!\n", ret);
}

int _tz_nand_probe(void)
{
    struct scm_desc desc = {0};
    uint32_t dma_pool_size=0x2000, i=0; // FLASH driver needs ~5K+ memory

    dma_va = dma_alloc_coherent(NULL, dma_pool_size, &dma_pa, GFP_KERNEL);
    desc.arginfo = TZ_OS_NAND_PROBE_STATUS_ID_PARAM_ID;
    desc.args[0] = (uint32_t)dma_pa;
    desc.args[1] = (uint32_t)dma_pool_size;

    pr_info("Inside the init function %x %x \n", (uint32_t)dma_pa, (uint32_t)dma_va, dma_pool_size);
    memset((void*)dma_va, 0x0, dma_pool_size);
    call_tz_nand_probe(TZ_OS_NAND_PROBE_STATUS_ID, &desc);
    pr_info("return %x %x\n", (uint32_t)desc.ret[0], (uint32_t)desc.ret[1]);
    dma_free_coherent(NULL, dma_pool_size, dma_va, dma_pa);

    pr_info("scm_call_done \n");
    return 0;
}

void _tz_nand_probe_exit(void)
{
    printk(KERN_ALERT "Exit kernel device ");
}
module_init(_tz_nand_probe);
module_exit(_tz_nand_probe_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. TZ NAND PROBE driver");

