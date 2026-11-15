# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_EXYNOS_MODEM) += exynos_modem.o
exynos_modem-y := boot.o ctrl.o link.o net.o wwan.o
exynos_modem-$(CONFIG_EXYNOS_MODEM_LEGACY_ABI) += legacy.o

obj-$(CONFIG_EXYNOS_MODEM_KUNIT_TEST) += sipc5_test.o
