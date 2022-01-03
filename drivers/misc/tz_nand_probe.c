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
    int ret = 0;
    struct scm_desc desc = {0};

    desc.arginfo = TZ_NAND_PROBE_STATUS_ID_PARAM_ID;
    call_tz_nand_probe(TZ_NAND_PROBE_STATUS_ID, &desc);
    if(desc.ret[0] != 0)
    {
        ret = -EIO;
    }
    pr_info("return %x\n", (uint32_t)desc.ret[0]);

    return ret;
}

void _tz_nand_probe_exit(void)
{
    printk(KERN_ALERT "Exit NAND PROBE\n");
}
module_init(_tz_nand_probe);
module_exit(_tz_nand_probe_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. TZ NAND PROBE driver");

