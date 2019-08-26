obj-$(CONFIG_QPNP_SMBCHARGER)	+= qpnp-smbcharger.o batterydata-lib.o pmic-voter.o
obj-$(CONFIG_QPNP_FG)		+= qpnp-fg.o
obj-$(CONFIG_QPNP_FG_GEN3)     += qpnp-fg-gen3.o fg-memif.o fg-util.o
obj-$(CONFIG_SMB135X_CHARGER)   += smb135x-charger.o pmic-voter.o
obj-$(CONFIG_SMB1351_USB_CHARGER) +=  battery.o smb1351-charger.o pmic-voter.o
obj-$(CONFIG_MSM_BCL_CTL)	+= msm_bcl.o
obj-$(CONFIG_MSM_BCL_PERIPHERAL_CTL) += bcl_peripheral.o
obj-$(CONFIG_BATTERY_BCL) += battery_current_limit.o
obj-$(CONFIG_QPNP_SMB2)		+= step-chg-jeita.o battery.o qpnp-smb2.o smb-lib.o pmic-voter.o storm-watch.o
obj-$(CONFIG_SMB138X_CHARGER)	+= battery.o smb138x-charger.o smb-lib.o pmic-voter.o storm-watch.o
obj-$(CONFIG_QPNP_QNOVO)	+= battery.o qpnp-qnovo.o

obj-$(CONFIG_NUBIA_CHARGE_FEATURE)	+= neo-battery.o

ifeq ($(CONFIG_NEO_DIRECT_CHARGE_SUPPORT),y)
	obj-$(CONFIG_NEO_WP502X_CHARGER)	+= wp502x-charger.o
endif

ifeq ($(CONFIG_NEO_EXTERNAL_FG_SUPPORT),y)
	obj-$(CONFIG_NEO_BQ27520_BATTERY)	+= bq27520-battery.o
endif
