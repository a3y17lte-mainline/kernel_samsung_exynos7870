# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(src)			# needed for trace events

ifdef CONFIG_RUST
obj-$(CONFIG_ASHMEM)		+= ashmem.o ashmem_exports.o
else
obj-$(CONFIG_ASHMEM)		+= ashmem_c.o
endif
