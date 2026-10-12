/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_SOC_FS16XX_I2C_H
#define _MTK_SOC_FS16XX_I2C_H

struct i2c_client;

int mtk_fs16xx_i2c_attach(struct i2c_client *client);
void mtk_fs16xx_i2c_detach(const struct i2c_client *client);

#endif
