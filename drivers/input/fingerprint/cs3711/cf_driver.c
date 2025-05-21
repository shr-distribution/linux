#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

long agold_strtol(const char *str)
{
  const char *parse_str;
  int base;

  if (strstr(str, "0x") || strstr(str, "0X")) {
    parse_str = str + 2;
    base = 16;
  } else {
    base = 10;
    parse_str = str;
  }
  return simple_strtol(parse_str, NULL, base);
}

extern int cf_driver_init(void);
module_init(cf_driver_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("zwp@chipsailing.com");
MODULE_DESCRIPTION("ChipSailing Fingerprint sensor device driver.");
