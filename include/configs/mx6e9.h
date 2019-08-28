/*
 * Copyright (C) 2019
 *
 * AUthor: Ovidiu Mihalachi <ovidiu.mihalachi@gmail.com>
 *
 * Configuration settings for iMX6Q Embedsky board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6EMBEDSKY_CONFIG_H
#define __MX6EMBEDSKY_CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE		3980
#define CONSOLE_DEV			"ttymxc0"
#define CONFIG_DEFAULT_FDT_FILE		"imx6q-embedsky.dtb"

#define CONFIG_IMX_THERMAL
#define CONFIG_DISPLAY_BOARDINFO_LATE

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
        (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
        (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END \
	(CONFIG_SYS_MEMTEST_START + 0x10000)
#define CONFIG_SYS_MEMTEST_SCRATCH	0x10800000

/* UART Configuration */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* MMC Configuration */
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SUPPORT_EMMC_BOOT

/* Environment organization */
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC3 */
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#endif
#define CONFIG_MMCROOT			"/dev/mmcblk1p2"

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000

/* USB Configs */
#define CONFIG_USBD_HS
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif

/* Ethernet Configuration */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_ETHADDR			00:1E:AC:0A:93:B5
#define CONFIG_ARP_TIMEOUT		200UL

/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

#ifdef CONFIG_SUPPORT_EMMC_BOOT
#define EMMC_ENV \
        "emmcdev=2\0" \
        "update_emmc_firmware=" \
                "if test ${ip_dyn} = yes; then " \
                        "setenv get_cmd dhcp; " \
                "else " \
                        "setenv get_cmd tftp; " \
                "fi; " \
                "if ${get_cmd} ${update_sd_firmware_filename}; then " \
                        "if mmc dev ${emmcdev} 1; then "        \
                                "setexpr fw_sz ${filesize} / 0x200; " \
                                "setexpr fw_sz ${fw_sz} + 1; "  \
                                "mmc write ${loadaddr} 0x2 ${fw_sz}; " \
                        "fi; "  \
                "fi\0"
#else
#define EMMC_ENV ""
#endif

#define VIDEO_ARGS        "${video_args}"
#define VIDEO_ARGS_SCRIPT "run video_args_script; "

#define CONFIG_PREBOOT \
        "if hdmidet; then " \
                "setenv video_interfaces hdmi lvds; " \
        "else " \
                "setenv video_interfaces lvds hdmi; " \
        "fi;"

#define CONFIG_EXTRA_ENV_SETTINGS \
        "console=" CONSOLE_DEV "\0" \
	"splashpos=m,m\0" \
        "script=boot.scr\0" \
        "image=zImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
        "fdt_addr=0x18000000\0" \
        "fdt_high=0xffffffff\0"   \
        "initrd_high=0xffffffff\0" \
        "boot_fdt=try\0" \
        "ip_dyn=yes\0" \
	"ethaddr=" __stringify(CONFIG_ETHADDR) "\0" \
        "mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
        "update_sd_firmware=" \
                "if test ${ip_dyn} = yes; then " \
                        "setenv get_cmd dhcp; " \
                "else " \
                        "setenv get_cmd tftp; " \
                "fi; " \
                "if mmc dev ${mmcdev}; then "   \
                        "if ${get_cmd} ${update_sd_firmware_filename}; then " \
                                "setexpr fw_sz ${filesize} / 0x200; " \
                                "setexpr fw_sz ${fw_sz} + 1; "  \
                                "mmc write ${loadaddr} 0x2 ${fw_sz}; " \
                        "fi; "  \
                "fi\0" \
        EMMC_ENV          \
        "video_args_hdmi=setenv video_args $video_args " \
                "video=mxcfb${fb}:dev=hdmi,1280x720M@60,if=RGB24\0" \
        "video_args_lvds=setenv video_args $video_args " \
                "video=mxcfb${fb}:dev=ldb,LDB-XGA,if=RGB666\0" \
        "video_args_lcd=setenv video_args $video_args " \
                "video=mxcfb${fb}:dev=lcd,CLAA-WVGA,if=RGB666\0" \
        "fb=0\0" \
        "video_args_script=" \
                "for v in ${video_interfaces}; do " \
                        "run video_args_${v}; " \
                        "setexpr fb $fb + 1; " \
                "done\0" \
        "mmcargs=setenv bootargs console=${console},${baudrate} " \
                "root=" CONFIG_MMCROOT " rootwait rw\0" \
                VIDEO_ARGS "\0" \
        "loadbootscript=" \
                "fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
        "bootscript=echo Running bootscript from mmc ...; " \
                "source\0" \
        "loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
        "loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "mmcboot=echo Booting from mmc ...; " \
                VIDEO_ARGS_SCRIPT \
                "run mmcargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run loadfdt; then " \
                                "bootz ${loadaddr} - ${fdt_addr}; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                        "bootz; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "bootz; " \
                "fi;\0" \
        "netargs=setenv bootargs console=${console},${baudrate} " \
                "root=/dev/nfs " \
                "ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
        "netboot=echo Booting from net ...; " \
                "run netargs; " \
                "if test ${ip_dyn} = yes; then " \
                        "setenv get_cmd dhcp; " \
                "else " \
                        "setenv get_cmd tftp; " \
                "fi; " \
                "${get_cmd} ${image}; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
                                "bootz ${loadaddr} - ${fdt_addr}; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                        "bootz; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "bootz; " \
                "fi;\0" \

#define CONFIG_BOOTCOMMAND \
        "mmc dev ${mmcdev};" \
        "if mmc rescan; then " \
                "if run loadbootscript; then " \
                "run bootscript; " \
                "else " \
                        "if run loadimage; then " \
                                "run mmcboot; " \
                        "else run netboot; " \
                        "fi; " \
                "fi; " \
        "else run netboot; fi"


#ifdef CONFIG_CMD_MMC
#define BOOT_TARGET_DEVICES_MMC(func) func(MMC, mmc, 0)
#else
#define BOOT_TARGET_DEVICES_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_DEVICES_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_DEVICES_USB(func)
#endif

#if defined(CONFIG_CMD_DHCP)
#define BOOT_TARGET_DEVICES_DHCP(func) func(DHCP, dhcp, na)
#else
#define BOOT_TARGET_DEVICES_DHCP(func)
#endif

#define BOOT_TARGET_DEVICES(func) \
        BOOT_TARGET_DEVICES_MMC(func) \
        BOOT_TARGET_DEVICES_USB(func) \
        BOOT_TARGET_DEVICES_DHCP(func)

#include <config_distro_bootcmd.h>

#endif                         /* __MX6EMBEDSKY_CONFIG_H */
