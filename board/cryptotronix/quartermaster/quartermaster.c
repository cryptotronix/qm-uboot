

#include <common.h>
#include <dm/uclass.h>
#include <dm/device-internal.h>
#include <fdtdec.h>
#include <fpga.h>
#include <malloc.h>
#include <mmc.h>
#include <watchdog.h>
#include <wdt.h>
#include <zynqpl.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include <mach/sys_proto.h>
#include <i2c.h>
#include <spi.h>
#include <spi_flash.h>

DECLARE_GLOBAL_DATA_PTR;

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define BOOT_WORD_LEN           32
#define BOOT_WORD_QSPI_OFFSET   0xFF0000
#define BOOT_WORD_RAM_ADDR      0x02000000
#define FIT_LOAD_ADDR           0x02A00000
#define RECOVERY_FIT_LOAD_ADDR  0x02000000

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_WDT)
static struct udevice *watchdog_dev;
#endif

static struct spi_flash *flash;

unsigned char default_ethaddr[6] = {
	0x70, 0xb3, 0xd5, 0x0d, 0xb0, 0x00
};

unsigned char boot_word[BOOT_WORD_LEN] = {
    0xf7, 0x34, 0xc7, 0x0a, 0x2d, 0x03, 0x2d, 0x39,
    0x60, 0xbf, 0x6d, 0x47, 0xfa, 0xa6, 0xff, 0x43,
    0x56, 0x9a, 0x3b, 0x42, 0x63, 0xaa, 0xcb, 0x2c,
    0x28, 0x6f, 0xfb, 0xfe, 0x8e, 0x24, 0x9c, 0x92
};

unsigned char qspi_boot_word[BOOT_WORD_LEN] = {0};

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

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_BOARD_EARLY_INIT_F)
int board_early_init_f(void)
{
# if defined(CONFIG_WDT)
	/* bss is not cleared at time when watchdog_reset() is called */
	watchdog_dev = NULL;
# endif

	return 0;
}
#endif

int board_init(void)
{
#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_WDT)
	if (uclass_get_device_by_seq(UCLASS_WDT, 0, &watchdog_dev)) {
		debug("Watchdog: Not found by seq!\n");
		if (uclass_get_device(UCLASS_WDT, 0, &watchdog_dev)) {
			puts("Watchdog: Not found!\n");
			return 0;
		}
	}

	wdt_start(watchdog_dev, 0, 0);
	puts("Watchdog: Started\n");
# endif

	return 0;
}

int board_late_init(void)
{
	return 0;
}

#if !defined(CONFIG_SYS_SDRAM_BASE) && !defined(CONFIG_SYS_SDRAM_SIZE)
int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int dram_init(void)
{
	if (fdtdec_setup_mem_size_base() != 0)
		return -EINVAL;

	zynq_ddrc_init();

	return 0;
}
#else
int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)CONFIG_SYS_SDRAM_BASE,
				    CONFIG_SYS_SDRAM_SIZE);

	zynq_ddrc_init();

	return 0;
}
#endif

#if defined(CONFIG_WATCHDOG)
/* Called by macro WATCHDOG_RESET */
void watchdog_reset(void)
{
# if !defined(CONFIG_SPL_BUILD)
	static ulong next_reset;
	ulong now;

	if (!watchdog_dev)
		return;

	now = timer_get_us();

	/* Do not reset the watchdog too often */
	if (now > next_reset) {
		wdt_reset(watchdog_dev);
		next_reset = now + 1000;
	}
# endif
}
#endif

#ifndef CONFIG_CMDLINE
static char *fatload_argv[] = {
	"fatload",
    "mmc",
    "0:1",
    STR(FIT_LOAD_ADDR),
    "image.fit"
};

static char *bootm_argv[] = {
    "bootm",
    STR(FIT_LOAD_ADDR)
};

static char *bootm_recovery_argv[] = {
	"bootm",
	STR(RECOVERY_FIT_LOAD_ADDR)
};

int board_run_command(const char *cmdline)
{
    unsigned int bus = 0;
    unsigned int cs = 0;
    unsigned int speed = CONFIG_SF_DEFAULT_SPEED;
    unsigned int mode = CONFIG_SF_DEFAULT_MODE;
    int ret;

#ifdef CONFIG_DM_SPI_FLASH
    struct udevice *new, *bus_dev;
	speed = 0, mode = 0;

    /* Remove the old device, otherwise probe will just be a nop */
	ret = spi_find_bus_and_cs(bus, cs, &bus_dev, &new);
	if (!ret) {
		device_remove(new, DM_REMOVE_NORMAL);
	}
	flash = NULL;
	ret = spi_flash_probe_bus_cs(bus, cs, speed, mode, &new);
	if (ret) {
		printf("Failed to initialize SPI flash at %u:%u (error %d)\n",
		       bus, cs, ret);
		goto boot_recovery;
	}

	flash = dev_get_uclass_priv(new);
#else
    struct spi_flash *new;
	
    if (flash)
		spi_flash_free(flash);

	new = spi_flash_probe(bus, cs, speed, mode);
	flash = new;

	if (!new) {
		printf("Failed to initialize SPI flash at %u:%u\n", bus, cs);
		goto boot_recovery;
	}

	flash = new;
#endif

    printf("sf read\n");
    ret = spi_flash_read(flash, BOOT_WORD_QSPI_OFFSET, BOOT_WORD_LEN, qspi_boot_word);
    if (ret) {
        printf("spi_flash_read failure: %d", ret);
        goto boot_recovery;
    }

    // if the magic word exists, run recovery
    bool mismatch = false;
    for (int i = 0; i < BOOT_WORD_LEN; i++) {
        //printf("<debug> 0x%02X 0x%02X\n", qspi_boot_word[i], boot_word[i]);
        if (qspi_boot_word[i] != boot_word[i]) {
            mismatch = true;
        }
    }

    if (!mismatch) {
        goto boot_recovery;
    }

    // try to boot normal OS
    printf("%s %s %s %s %s\n", fatload_argv[0], fatload_argv[1], fatload_argv[2],
        fatload_argv[3], fatload_argv[4]);
    if (0 != do_fat_fsload(NULL, 0, 5, fatload_argv)) {
        printf("fatload failure\n");
        goto boot_recovery;
    }

    printf("%s %s\n", bootm_argv[0], bootm_argv[1]);
	do_bootm(NULL, 0, 2, bootm_argv);

boot_recovery:
    printf("%s %s\n", bootm_recovery_argv[0], bootm_recovery_argv[1]);
	do_bootm(NULL, 0, 2, bootm_recovery_argv);

	return 1;
}
#endif
