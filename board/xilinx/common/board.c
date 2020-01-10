// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2014 - 2019 Xilinx, Inc.
 * Michal Simek <michal.simek@xilinx.com>
 */

#include <common.h>
#include <dm/uclass.h>
#include <i2c.h>
#include <mach/sys_proto.h>
#include <string.h>
#include <stdbool.h>

unsigned char default_ethaddr[6] = {
	0x70, 0xb3, 0xd5, 0x0d, 0xb0, 0x00
};

int zynq_board_read_rom_ethaddr(unsigned char *ethaddr)
{
	int ret = 0;

	// always set to default ethaddr
	memcpy(ethaddr, default_ethaddr, 6);

#if defined(CONFIG_ZYNQ_GEM_I2C_MAC_OFFSET)
	struct udevice *dev;
	ofnode eeprom;
	unsigned char tmp_ethaddr[6] = {0};
	bool ethaddr_is_valid = true;

	eeprom = ofnode_get_chosen_node("xlnx,eeprom");
	if (!ofnode_valid(eeprom))
		return -ENODEV;

	debug("%s: Path to EEPROM %s\n", __func__,
	      ofnode_get_chosen_prop("xlnx,eeprom"));

	ret = uclass_get_device_by_ofnode(UCLASS_I2C_EEPROM, eeprom, &dev);
	if (ret)
		return ret;

	ret = dm_i2c_read(dev, CONFIG_ZYNQ_GEM_I2C_MAC_OFFSET, tmp_ethaddr, 6);
	if (ret)
		debug("%s: I2C EEPROM MAC address read failed\n", __func__);
	else
		debug("%s: I2C EEPROM MAC %pM\n", __func__, ethaddr);

	// check to see if the i2c ethaddr is within the default
	for (int i = 0; i < 6; i++)
	{
		if ((tmp_ethaddr[i] & default_ethaddr[i]) != default_ethaddr[i])
		{
			ethaddr_is_valid = false;
			break;
		}
	}

	if (ethaddr_is_valid)
	{
		// write the address read from i2c
		memcpy(ethaddr, tmp_ethaddr, 6);
	}

#endif

	return ret;
}
