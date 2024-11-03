#
# Copyright 2019 NXP
# Copyright 2023 Gilles Talis <gilles.talis@gmail.com>
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mp_debix_model_a.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o

ifdef CONFIG_TARGET_IMX8MP_DEBIX_MODEL_A
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif

ifdef CONFIG_TARGET_IMX8MP_DEBIX_MODEL_A_4G
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_4g.o
endif

endif
