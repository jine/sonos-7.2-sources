#
# Makefile for the IEEE 802.1d ethernet bridging layer.
#

obj-$(CONFIG_BRIDGE) += bridge.o

bridge-y	:= br.o br_device.o br_fdb.o br_forward.o br_if.o br_input.o \
			br_ioctl.o br_notify.o br_stp.o br_stp_bpdu.o \
			br_stp_if.o br_stp_timer.o br_tunnel.o br_mcast.o \
			br_direct.o br_priority.o br_uplink.o

# SONOS: We need to either fix this or disable it in the kernel build entirely:
# bridge-$(CONFIG_SYSFS) += br_sysfs_if.o br_sysfs_br.o

bridge-$(CONFIG_BRIDGE_NETFILTER) += br_netfilter.o

obj-$(CONFIG_BRIDGE_NF_EBTABLES) += netfilter/
