obj-$(CONFIG_MTK_AW9523) += aw9523_key.o

# Titan Pocket (g66): its own key matrix, see key_map in aw9523_key.c
ifneq (,$(filter "g66v71c2k_dfl_eea" "g66v71c2k_dfl_tee",$(CONFIG_ARCH_MTK_PROJECT)))
ccflags-y += -DTITAN_POCKET_KEYMAP=1
endif
