#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/packages;C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/products/tidrivers_cc13xx_cc26xx_2_20_00_08/packages;C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/products/bios_6_46_00_23/packages;C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/products/uia_2_00_06_52/packages;C:/ti/simplelink/PML-Firmware/examples/pmlahrs/pmlahrs/ccs/app/.config
override XDCROOT = C:/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/packages;C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/products/tidrivers_cc13xx_cc26xx_2_20_00_08/packages;C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/products/bios_6_46_00_23/packages;C:/ti/tirtos_cc13xx_cc26xx_2_20_00_06/products/uia_2_00_06_52/packages;C:/ti/simplelink/PML-Firmware/examples/pmlahrs/pmlahrs/ccs/app/.config;C:/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
