#include <linux/kernel.h>
#include <linux/module.h>

extern int AudDrv_nxpspk_mod_init(void);

static void __exit AudDrv_nxpspk_mod_exit(void)
{
	pr_debug("+AudDrv_nxpspk_mod_exit\n");

	pr_debug("-AudDrv_nxpspk_mod_exit\n");
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mtksmartpa");
MODULE_AUTHOR("MediaTek Inc");

module_init(AudDrv_nxpspk_mod_init);
module_exit(AudDrv_nxpspk_mod_exit);
