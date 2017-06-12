/*
 * Copyright (c) 2015-2016, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos - Secure Memory Key Access
 *
 * Kernel access to CAAM functionality via direct function calls, implementing
 * functionality similar to caamkeys userspace driver.
 *
 * The kernel will always use keystore unit 0, with 1+ being available to the
 * userspace driver.
 */

#include "compat.h"
#include "intern.h"
#include "desc.h"
#include "error.h"
#include "jr.h"
#include "sm.h"
#include <linux/sonos_kernel.h>

static __u32 unit, keyslot;

struct platform_device *sonos_sm_get_pdev(void)
{
	struct device_node *dev_node;
	struct platform_device *pdev;

	// Find the CAAM hardware device...
	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return (struct platform_device*) -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return (struct platform_device*) -ENODEV;

	return pdev;
}
EXPORT_SYMBOL(sonos_sm_get_pdev);


// Initialize and "open" the secure memory device for kernel access to
// the secure memory functionality.

int sonos_sm_init (struct platform_device *pdev)
{
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *kspriv;
	__u32 units;
	__u32 ret = 0;
	__s32 status;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;
	kspriv = dev_get_drvdata(ksdev);
	if (kspriv == NULL)
		return -ENODEV;

	// How many keystores?
	units = sm_detect_keystore_units(ksdev);
	if (units < 1) {
		dev_err(ksdev, "blkkey_ex: insufficient keystore units\n");
		return -ENODEV;
	}

	// Unit 0 is reserved for the kernel. The rest are available for
	// userspace access.
	unit = 0;

	dev_info(ksdev, "%d keystore units available - kernel allocating unit %d\n", units, unit);

	// establish the keystore in the unit
	if ( (status = sm_establish_keystore(ksdev, unit)) ) {
		// Something is wrong - we cannot establish a store
		ret = status;
		dev_err(ksdev, "attempt to establish keystore %d failed [%d]\n", unit, status);
	}

	// Allocate a slot...
	if ( (status = sm_keystore_slot_alloc(ksdev, unit, MAX_SLOT_SIZE, &keyslot)) != 0) {
		// Something is wrong - we cannot get a slot
		ret = status;
		dev_err(ksdev, "attempt to allocate keyslot %d in keystore unit %d failed [%d]\n",keyslot, unit, status);
	} else {
		dev_info(ksdev, "allocated keyslot %d in keystore unit %d\n",keyslot, unit);
	}
	return ret;
}
EXPORT_SYMBOL(sonos_sm_init);


int sonos_sm_encdec(struct platform_device *pdev, struct crypt_operation *op)
{
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *kspriv;
	int ret = 0;
	int status;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;
	kspriv = dev_get_drvdata(ksdev);
	if (kspriv == NULL)
		return -ENODEV;

	if ( op->cmd == ENCRYPT ) {
		status = sm_keystore_slot_load(ksdev, unit, keyslot, op->input_buffer, op->input_length);
		if ( status ) {
			dev_info(ksdev, "sm_keystore_slot_load failed [%d]\n", status);
			ret = status;
			goto fail1;
		}
		if ( op->color == USE_BLACK ) {
			status = sm_keystore_cover_key(ksdev, unit, keyslot, op->input_length, KEY_COVER_CCM);
			if ( status ) {
				dev_info(ksdev, "sm_keystore_cover_key failed [%d]\n", status);
				ret = status;
				goto fail1;
			}
		}
		status = sm_keystore_slot_export(ksdev, unit, keyslot, (op->color == USE_BLACK) ? BLACK_KEY : RED_KEY,
			       KEY_COVER_CCM, op->output_buffer, *(op->output_length), op->keymod);
		if ( status ) {
			dev_info(ksdev, "sm_keystore_slot_export failed [%d]\n", status);
			ret = status;
			goto fail1;
		}
	} else if ( op->cmd == DECRYPT ) {
		status = sm_keystore_slot_import(ksdev, unit, keyslot, (op->color == USE_BLACK) ? BLACK_KEY : RED_KEY,
						KEY_COVER_CCM, op->input_buffer, op->original_length, op->keymod);
		if ( status ) {
			dev_info(ksdev, "sm_keystore_slot_import failed [%d]\n", status);
			ret = status;
			goto fail1;
		}

		status = sm_keystore_slot_read(ksdev, unit, keyslot, op->original_length, op->output_buffer);
		if ( status ) {
			dev_info(ksdev, "sm_keystore_slot_read failed %d\n", status);
			ret = status;
			goto fail1;
		}
	} else {
		// This shouldn't happen...
		dev_err(ksdev, "error - %d is not a legal command to the sm_encdec function.\n",op->cmd);
		ret = -1;
	}


fail1:
	return ret;
}
EXPORT_SYMBOL(sonos_sm_encdec);

// Cleanup and exit...
void sonos_sm_exit(void)
{
	struct platform_device *pdev;
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;

	pdev = sonos_sm_get_pdev();
	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;

	sm_release_keystore(ksdev, unit);
}
EXPORT_SYMBOL(sonos_sm_exit);

