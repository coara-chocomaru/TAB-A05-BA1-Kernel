/* drivers/i2c/chips/lis2dw.c - LIS2DW motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <cust_acc.h>
#include "lis2dw.h"
#include <accel.h>
#include <hwmsensor.h>

/* #define POWER_NONE_MACRO MT65XX_POWER_NONE */

#define COMPATIABLE_NAME "mediatek,lis2dw"

/* Maintain  cust info here */
//struct acc_hw accel_cust;
//static struct acc_hw *hw = &accel_cust;

/* For  driver get cust info */
//struct acc_hw *get_cust_acc(void)
//{
//	return &accel_cust;
//}



/*----------------------------------------------------------------------------*/
/* #define I2C_DRIVERID_LIS2DW 345 */
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_LIS2DW_LOWPASS   /*apply low pass filter on output*/
/*----------------------------------------------------------------------------*/
#define LIS2DW_AXIS_X          0
#define LIS2DW_AXIS_Y          1
#define LIS2DW_AXIS_Z          2
#define LIS2DW_AXES_NUM        3
#define LIS2DW_DATA_LEN        6
#define LIS2DW_DEV_NAME        "LIS2DW"
#define C_I2C_FIFO_SIZE	 8
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lis2dw_i2c_id[] = {{LIS2DW_DEV_NAME, 0}, {} };
static int lis2dw_init_flag = -1;

/*----------------------------------------------------------------------------*/
static int lis2dw_probe(void);
static int lis2dw_remove(void);
static int lis2dw_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lis2dw_i2c_remove(struct i2c_client *client);
/* static int lis2dw_i2c_detect(struct i2c_client *client, struct i2c_board_info *info); */
#ifndef USE_EARLY_SUSPEND
static int lis2dw_suspend(struct device *dev);
static int lis2dw_resume(struct device *dev);
#endif
/*----------------------------------------------------------------------------*/
typedef enum {
	ADX_TRC_FILTER  = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL   = 0x04,
	ADX_TRC_CALI	= 0X08,
	ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8  whole;
	u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][LIS2DW_AXES_NUM];
	int sum[LIS2DW_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct lis2dw_i2c_data {
	struct i2c_client *client;
	struct acc_hw hw;
	struct hwmsen_convert   cvt;

	/*misc*/
//	struct data_resolution *reso;
	int                     reso;
	atomic_t                trace;
	atomic_t                suspend;
	atomic_t                selftest;
	atomic_t				filter;
	s16                     cali_sw[LIS2DW_AXES_NUM + 1];

	/*data*/
	s8                      offset[LIS2DW_AXES_NUM + 1]; /*+1: for 4-byte alignment*/
	s16                     data[LIS2DW_AXES_NUM + 1];

	bool flush;
	u8 sample_delay;

#if defined(CONFIG_LIS2DW_LOWPASS)
	atomic_t                firlen;
	atomic_t                fir_en;
	struct data_filter      fir;
#endif
};

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops lis2dw_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis2dw_suspend, lis2dw_resume)
};
#endif

static struct i2c_driver lis2dw_i2c_driver = {
	.driver = {
		/* .owner          = THIS_MODULE, */
		.name           = LIS2DW_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &lis2dw_pm_ops,
#endif

#ifdef CONFIG_OF
		.of_match_table = accel_of_match,
#endif
	},
	.probe		= lis2dw_i2c_probe,
	.remove			= lis2dw_i2c_remove,
	/* .detect				= lis2dw_i2c_detect, */
	.id_table = lis2dw_i2c_id,
	/* .address_data = &lis2dw_addr_data, */
};


static struct acc_init_info lis2dw_gsensor_driver = {
	.name = LIS2DW_DEV_NAME,
	.init = lis2dw_probe,
	.uninit = lis2dw_remove,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *lis2dw_i2c_client;
/* static struct platform_driver lis2dw_gsensor_driver; */
static struct lis2dw_i2c_data *obj_i2c_data;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain;
/* static char selftestRes[10] = {0}; */
static DEFINE_MUTEX(lis2dw_i2c_mutex);
static DEFINE_MUTEX(lis2dw_op_mutex);
static bool enable_status;
static int sensor_suspend = 0;

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
#if 0
static struct data_resolution lis2dw_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 1, 0}, 4096},   /* dataformat +/-2g  in 14-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^14);  1024 = (2^12)/(2*2) */
	{{ 1, 9}, 2048},   /* dataformat +/-4g  in 14-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^14);  512 = (2^12)/(2*4) */
	{{ 3, 9}, 1024},   /* dataformat +/-8g  in 14-bit resolution;  { 1, 0} = 1.0 = (2*8*1000)/(2^14);  1024 = (2^12)/(2*8) */
};
#endif
static int lis2dw_data_resolution[4] = {
	244,488,976,1952
};

/*----------------------------------------------------------------------------*/
static struct data_resolution lis2dw_offset_resolution = {{15, 6}, 64};

/*--------------------read function----------------------------------*/
static int lis_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {{0}, {0} };

	mutex_lock(&lis2dw_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&lis2dw_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis2dw_i2c_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else
		err = 0;
	mutex_unlock(&lis2dw_i2c_mutex);
	return err;

}

static int lis_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	/*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&lis2dw_i2c_mutex);
//	printk(" ====brucelee=== C_I2C_FIFO_SIZE %d\n", C_I2C_FIFO_SIZE);
	if (!client) {
		mutex_unlock(&lis2dw_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis2dw_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
//	printk("smith-acc return value is %d\n", err);
	if (err < 0) {
//		printk("smith-acc send command error\n");
		GSE_ERR("send command error!!\n");
		mutex_unlock(&lis2dw_i2c_mutex);
		return -EFAULT;
	}
	mutex_unlock(&lis2dw_i2c_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/

static void dumpReg(struct i2c_client *client)
{
	int i = 0;
	u8 addr = 0x20;
	u8 regdata = 0;

	for (i = 0; i < 3 ; i++) {
		/* dump all */
		lis_i2c_read_block(client, addr, &regdata, 1);
		GSE_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
		addr++;
	}
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_SetDataResolution(struct lis2dw_i2c_data *obj)
{
	int err;
	u8  dat, reso;

	err = lis_i2c_read_block(obj->client, LIS2DW_REG_CTL_REG6, &dat, 0x01);
	if (err) {
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso  = (dat & 0x30) >> 4;

	if (reso < ARRAY_SIZE(lis2dw_data_resolution)) {
		obj->reso = lis2dw_data_resolution[reso];
		return 0;
	} else
		return -EINVAL;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_ReadData(struct i2c_client *client, s16 data[LIS2DW_AXES_NUM])
{
	struct lis2dw_i2c_data *priv = i2c_get_clientdata(client);
	/* u8 addr = LIS2DW_REG_DATAX0; */
	u8 buf[LIS2DW_DATA_LEN] = {0};
	int err = 0;

	if (client == NULL)
		err = -EINVAL;

	else {
		if (lis_i2c_read_block(client, LIS2DW_REG_OUT_X, &buf[0], 0x01)) {
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		if (lis_i2c_read_block(client, LIS2DW_REG_OUT_X + 1, &buf[1], 0x01)) {
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}

		data[LIS2DW_AXIS_X] = (s16)((s16)(buf[0] + (buf[1] << 8)) >> 2);

		if (lis_i2c_read_block(client, LIS2DW_REG_OUT_Y, &buf[2], 0x01)) {
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		if (lis_i2c_read_block(client, LIS2DW_REG_OUT_Y + 1, &buf[3], 0x01)) {
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}

		data[LIS2DW_AXIS_Y] = (s16)((s16)(buf[2] + (buf[3] << 8)) >> 2);

		if (lis_i2c_read_block(client, LIS2DW_REG_OUT_Z, &buf[4], 0x01)) {
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}

		if (lis_i2c_read_block(client, LIS2DW_REG_OUT_Z + 1, &buf[5], 0x01)) {
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}

		data[LIS2DW_AXIS_Z] = (s16)((s16)(buf[4] + (buf[5] << 8)) >> 2);

		/* GSE_LOG("[%08X %08X %08X %08x %08x %08x]\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]); */

#if 0
		data[LIS2DW_AXIS_X] &= 0x3fff;
		data[LIS2DW_AXIS_Y] &= 0x3fff;
		data[LIS2DW_AXIS_Z] &= 0x3fff;
#endif

		if (atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS2DW_AXIS_X], data[LIS2DW_AXIS_Y], data[LIS2DW_AXIS_Z],
				data[LIS2DW_AXIS_X], data[LIS2DW_AXIS_Y], data[LIS2DW_AXIS_Z]);
		}

#if 0
		if (data[LIS2DW_AXIS_X] & 0x2000) {
			data[LIS2DW_AXIS_X] = ~data[LIS2DW_AXIS_X];
			data[LIS2DW_AXIS_X] &= 0x3fff;
			data[LIS2DW_AXIS_X] += 1;
			data[LIS2DW_AXIS_X] = -data[LIS2DW_AXIS_X];
		}
		if (data[LIS2DW_AXIS_Y] & 0x2000) {
			data[LIS2DW_AXIS_Y] = ~data[LIS2DW_AXIS_Y];
			data[LIS2DW_AXIS_Y] &= 0x3fff;
			data[LIS2DW_AXIS_Y] += 1;
			data[LIS2DW_AXIS_Y] = -data[LIS2DW_AXIS_Y];
		}
		if (data[LIS2DW_AXIS_Z] & 0x2000) {
			data[LIS2DW_AXIS_Z] = ~data[LIS2DW_AXIS_Z];
			data[LIS2DW_AXIS_Z] &= 0x3fff;
			data[LIS2DW_AXIS_Z] += 1;
			data[LIS2DW_AXIS_Z] = -data[LIS2DW_AXIS_Z];
		}

		if (atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS2DW_AXIS_X], data[LIS2DW_AXIS_Y], data[LIS2DW_AXIS_Z],
				data[LIS2DW_AXIS_X], data[LIS2DW_AXIS_Y], data[LIS2DW_AXIS_Z]);
		}
#endif

#ifdef CONFIG_LIS2DW_LOWPASS
		if (atomic_read(&priv->filter)) {
			if (atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend)) {
				int idx, firlen = atomic_read(&priv->firlen);

				if (priv->fir.num < firlen) {
					priv->fir.raw[priv->fir.num][LIS2DW_AXIS_X] = data[LIS2DW_AXIS_X];
					priv->fir.raw[priv->fir.num][LIS2DW_AXIS_Y] = data[LIS2DW_AXIS_Y];
					priv->fir.raw[priv->fir.num][LIS2DW_AXIS_Z] = data[LIS2DW_AXIS_Z];
					priv->fir.sum[LIS2DW_AXIS_X] += data[LIS2DW_AXIS_X];
					priv->fir.sum[LIS2DW_AXIS_Y] += data[LIS2DW_AXIS_Y];
					priv->fir.sum[LIS2DW_AXIS_Z] += data[LIS2DW_AXIS_Z];
					if (atomic_read(&priv->trace) & ADX_TRC_FILTER) {
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][LIS2DW_AXIS_X], priv->fir.raw[priv->fir.num][LIS2DW_AXIS_Y], priv->fir.raw[priv->fir.num][LIS2DW_AXIS_Z],
							priv->fir.sum[LIS2DW_AXIS_X], priv->fir.sum[LIS2DW_AXIS_Y], priv->fir.sum[LIS2DW_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				} else {
					idx = priv->fir.idx % firlen;
					priv->fir.sum[LIS2DW_AXIS_X] -= priv->fir.raw[idx][LIS2DW_AXIS_X];
					priv->fir.sum[LIS2DW_AXIS_Y] -= priv->fir.raw[idx][LIS2DW_AXIS_Y];
					priv->fir.sum[LIS2DW_AXIS_Z] -= priv->fir.raw[idx][LIS2DW_AXIS_Z];
					priv->fir.raw[idx][LIS2DW_AXIS_X] = data[LIS2DW_AXIS_X];
					priv->fir.raw[idx][LIS2DW_AXIS_Y] = data[LIS2DW_AXIS_Y];
					priv->fir.raw[idx][LIS2DW_AXIS_Z] = data[LIS2DW_AXIS_Z];
					priv->fir.sum[LIS2DW_AXIS_X] += data[LIS2DW_AXIS_X];
					priv->fir.sum[LIS2DW_AXIS_Y] += data[LIS2DW_AXIS_Y];
					priv->fir.sum[LIS2DW_AXIS_Z] += data[LIS2DW_AXIS_Z];
					priv->fir.idx++;
					data[LIS2DW_AXIS_X] = priv->fir.sum[LIS2DW_AXIS_X] / firlen;
					data[LIS2DW_AXIS_Y] = priv->fir.sum[LIS2DW_AXIS_Y] / firlen;
					data[LIS2DW_AXIS_Z] = priv->fir.sum[LIS2DW_AXIS_Z] / firlen;
					if (atomic_read(&priv->trace) & ADX_TRC_FILTER) {
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
							priv->fir.raw[idx][LIS2DW_AXIS_X], priv->fir.raw[idx][LIS2DW_AXIS_Y], priv->fir.raw[idx][LIS2DW_AXIS_Z],
							priv->fir.sum[LIS2DW_AXIS_X], priv->fir.sum[LIS2DW_AXIS_Y], priv->fir.sum[LIS2DW_AXIS_Z],
							data[LIS2DW_AXIS_X], data[LIS2DW_AXIS_Y], data[LIS2DW_AXIS_Z]);
					}
				}
			}
		}
#endif
	}
	return err;
}
/*----------------------------------------------------------------------------*/
/*
static int LIS2DW_ReadOffset(struct i2c_client *client, s8 ofs[LIS2DW_AXES_NUM])
{
	int err;

	return err;
}
*/
/*----------------------------------------------------------------------------*/
static int LIS2DW_ResetCalibration(struct i2c_client *client)
{
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_ReadCalibration(struct i2c_client *client, int dat[LIS2DW_AXES_NUM])
{
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);

	dat[obj->cvt.map[LIS2DW_AXIS_X]] = obj->cvt.sign[LIS2DW_AXIS_X] * obj->cali_sw[LIS2DW_AXIS_X];
	dat[obj->cvt.map[LIS2DW_AXIS_Y]] = obj->cvt.sign[LIS2DW_AXIS_Y] * obj->cali_sw[LIS2DW_AXIS_Y];
	dat[obj->cvt.map[LIS2DW_AXIS_Z]] = obj->cvt.sign[LIS2DW_AXIS_Z] * obj->cali_sw[LIS2DW_AXIS_Z];

	return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int LIS2DW_ReadCalibrationEx(struct i2c_client *client, int act[LIS2DW_AXES_NUM], int raw[LIS2DW_AXES_NUM])
{

	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	if(err = LIS2DW_ReadOffset(client, obj->offset))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	mul = obj->reso->sensitivity/lis2dw_offset_resolution.sensitivity;
	raw[LIS2DW_AXIS_X] = obj->offset[LIS2DW_AXIS_X]*mul + obj->cali_sw[LIS2DW_AXIS_X];
	raw[LIS2DW_AXIS_Y] = obj->offset[LIS2DW_AXIS_Y]*mul + obj->cali_sw[LIS2DW_AXIS_Y];
	raw[LIS2DW_AXIS_Z] = obj->offset[LIS2DW_AXIS_Z]*mul + obj->cali_sw[LIS2DW_AXIS_Z];

	act[obj->cvt.map[LIS2DW_AXIS_X]] = obj->cvt.sign[LIS2DW_AXIS_X]*raw[LIS2DW_AXIS_X];
	act[obj->cvt.map[LIS2DW_AXIS_Y]] = obj->cvt.sign[LIS2DW_AXIS_Y]*raw[LIS2DW_AXIS_Y];
	act[obj->cvt.map[LIS2DW_AXIS_Z]] = obj->cvt.sign[LIS2DW_AXIS_Z]*raw[LIS2DW_AXIS_Z];

	return 0;
}
*/
/*----------------------------------------------------------------------------*/
static int LIS2DW_WriteCalibration(struct i2c_client *client, int dat[LIS2DW_AXES_NUM])
{
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	/* int cali[LIS2DW_AXES_NUM]; */


	GSE_FUN();
	if (!obj || !dat) {
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	} else {
		s16 cali[LIS2DW_AXES_NUM];

		cali[obj->cvt.map[LIS2DW_AXIS_X]] = obj->cvt.sign[LIS2DW_AXIS_X] * obj->cali_sw[LIS2DW_AXIS_X];
		cali[obj->cvt.map[LIS2DW_AXIS_Y]] = obj->cvt.sign[LIS2DW_AXIS_Y] * obj->cali_sw[LIS2DW_AXIS_Y];
		cali[obj->cvt.map[LIS2DW_AXIS_Z]] = obj->cvt.sign[LIS2DW_AXIS_Z] * obj->cali_sw[LIS2DW_AXIS_Z];
		cali[LIS2DW_AXIS_X] += dat[LIS2DW_AXIS_X];
		cali[LIS2DW_AXIS_Y] += dat[LIS2DW_AXIS_Y];
		cali[LIS2DW_AXIS_Z] += dat[LIS2DW_AXIS_Z];

		obj->cali_sw[LIS2DW_AXIS_X] += obj->cvt.sign[LIS2DW_AXIS_X] * dat[obj->cvt.map[LIS2DW_AXIS_X]];
		obj->cali_sw[LIS2DW_AXIS_Y] += obj->cvt.sign[LIS2DW_AXIS_Y] * dat[obj->cvt.map[LIS2DW_AXIS_Y]];
		obj->cali_sw[LIS2DW_AXIS_Z] += obj->cvt.sign[LIS2DW_AXIS_Z] * dat[obj->cvt.map[LIS2DW_AXIS_Z]];
	}

	return err;
}
/*----------------------------------------------------------------------------*/

static int LIS2DW_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = 0x0f;

	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_LIS2DW_CheckDeviceID;
	}

	udelay(500);

	databuf[0] = 0x0;
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_LIS2DW_CheckDeviceID;
	}

	if(databuf[0] != 0x44)
	{
		return LIS2DW_ERR_IDENTIFICATION;
	}

exit_LIS2DW_CheckDeviceID:
	if (res <= 0)
	{
		return LIS2DW_ERR_I2C;
	}
	return LIS2DW_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int lis2dw_flush(void);

static int LIS2DW_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	u8 addr = LIS2DW_REG_CTL_REG1;
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);

	/* GSE_LOG("enter Sensor power status is sensor_power = %d\n",sensor_power); */

	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return LIS2DW_SUCCESS;
	}

	if (lis_i2c_read_block(client, addr, databuf, 0x01)) {
		GSE_ERR("read power ctl register err!\n");
		return LIS2DW_ERR_I2C;
	}

	if (enable == true)
	{
		databuf[0] &=  ~LIS2DW_MEASURE_MODE;
		databuf[0] |=  0x04;
	}
	else
		databuf[0] = 0;

	res = lis_i2c_write_block(client, LIS2DW_REG_CTL_REG1, databuf, 0x1);

	if (res <= 0) {
		GSE_LOG("set power mode failed!\n");
		return LIS2DW_ERR_I2C;
	} else if (atomic_read(&obj->trace) & ADX_TRC_INFO)
		GSE_LOG("set power mode ok %d!\n", databuf[1]);

	sensor_power = enable;

	if (obj_i2c_data->flush) {
		if (sensor_power)
			lis2dw_flush();
		else
			obj_i2c_data->flush = false;
	}
	/* GSE_LOG("leave Sensor power status is sensor_power = %d\n",sensor_power); */
	return LIS2DW_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	u8 addr = LIS2DW_REG_CTL_REG6;
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (lis_i2c_read_block(client, addr, databuf, 0x01)) {
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS2DW_ERR_I2C;
	}

	databuf[0] &= ~0x30;
	databuf[0] |= dataformat;

	res = lis_i2c_write_block(client, LIS2DW_REG_CTL_REG6, databuf, 0x1);

	if (res <= 0)
		return LIS2DW_ERR_I2C;


	return LIS2DW_SetDataResolution(obj);
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
	u8 addr = LIS2DW_REG_CTL_REG1;
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (lis_i2c_read_block(client, addr, databuf, 0x01)) {
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS2DW_ERR_I2C;
	}

	databuf[0] &= ~0xF0;
	databuf[0] |= bwrate;

	res = lis_i2c_write_block(client, LIS2DW_REG_CTL_REG1, databuf, 0x1);

	if (res < 0)
		return LIS2DW_ERR_I2C;

	return LIS2DW_SUCCESS;
}
/*----------------------------------------------------------------------------*/
/* enalbe data ready interrupt */
static int LIS2DW_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[2];
	u8 addr = LIS2DW_REG_CTL_REG4;
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 2);

	if (lis_i2c_read_block(client, addr, databuf, 0x01)) {
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS2DW_ERR_I2C;
	}

	databuf[0] = 0x00;

	res = lis_i2c_write_block(client, LIS2DW_REG_CTL_REG4, databuf, 0x01);
	if (res < 0)
		return LIS2DW_ERR_I2C;

	return LIS2DW_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_Init(struct i2c_client *client, int reset_cali)
{

	struct lis2dw_i2c_data *obj = (struct lis2dw_i2c_data *)i2c_get_clientdata(client);
	int res = 0;
	u8 databuf[2] = {0, 0};
	res = LIS2DW_CheckDeviceID(client);
	if(res != LIS2DW_SUCCESS)
	{
		return res;
	}

	databuf[0] = 0x04;
	res = lis_i2c_write_block(client, LIS2DW_REG_CTL_REG1, databuf, 0x01);
	if (res < 0) {
		printk("smith-acc 1\n");
		return res;
	}

	res = LIS2DW_SetBWRate(client, obj->sample_delay);/* 400 or 100 no other choice */
	if (res < 0) {
		printk("smith-acc 2\n");
		return res;
	}

	res = LIS2DW_SetDataFormat(client, LIS2DW_RANGE_4G);/* 8g or 2G no oher choise */
	if (res < 0) {
		printk("smith-acc 3\n");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso;

	res = LIS2DW_SetIntEnable(client, false);
	if (res < 0) {
		printk("smith-acc 4\n");
		return res;
	}

	res = LIS2DW_SetPowerMode(client, enable_status);/* false); */
	if (res < 0) {
		printk("smith-acc 5\n");
		return res;
	}

	if (reset_cali != 0) {
		/* reset calibration only in power on */
		res = LIS2DW_ResetCalibration(client);
		if (res < 0) {
			printk("smith-acc 6\n");
			return res;
		}
	}

#ifdef CONFIG_LIS2DW_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return LIS2DW_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((buf == NULL) || (bufsize <= 30))
		return -1;

	if (client == NULL) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LIS2DW Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lis2dw_i2c_data *obj = (struct lis2dw_i2c_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LIS2DW_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (buf == NULL)
		return -1;
	if (client == NULL) {
		*buf = 0;
		return -2;
	}

	if (sensor_suspend == 1) {
		/* GSE_LOG("sensor in suspend read not data!\n"); */
		return 0;
	}
#if 0
	if (sensor_power == FALSE) {
		res = LIS2DW_SetPowerMode(client, true);
		if (res)
			GSE_ERR("Power on lis2dw error %d!\n", res);
		msleep(20);
	}
#endif
	res = LIS2DW_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	} else {
//		obj->data[LIS2DW_AXIS_X] += obj->cali_sw[LIS2DW_AXIS_X];
//		obj->data[LIS2DW_AXIS_Y] += obj->cali_sw[LIS2DW_AXIS_Y];
//		obj->data[LIS2DW_AXIS_Z] += obj->cali_sw[LIS2DW_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[LIS2DW_AXIS_X]] = obj->cvt.sign[LIS2DW_AXIS_X] * obj->data[LIS2DW_AXIS_X];
		acc[obj->cvt.map[LIS2DW_AXIS_Y]] = obj->cvt.sign[LIS2DW_AXIS_Y] * obj->data[LIS2DW_AXIS_Y];
		acc[obj->cvt.map[LIS2DW_AXIS_Z]] = obj->cvt.sign[LIS2DW_AXIS_Z] * obj->data[LIS2DW_AXIS_Z];

		/* GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LIS2DW_AXIS_X], acc[LIS2DW_AXIS_Y], acc[LIS2DW_AXIS_Z]); */

		/* Out put the mg */
		acc[LIS2DW_AXIS_X] = ((long long)acc[LIS2DW_AXIS_X] * GRAVITY_EARTH_1000 * obj->reso) / 1000000LL;
		acc[LIS2DW_AXIS_Y] = ((long long)acc[LIS2DW_AXIS_Y] * GRAVITY_EARTH_1000 * obj->reso) / 1000000LL;
		acc[LIS2DW_AXIS_Z] = ((long long)acc[LIS2DW_AXIS_Z] * GRAVITY_EARTH_1000 * obj->reso) / 1000000LL;

		sprintf(buf, "%04x %04x %04x", acc[LIS2DW_AXIS_X], acc[LIS2DW_AXIS_Y], acc[LIS2DW_AXIS_Z]);
		if (atomic_read(&obj->trace) & ADX_TRC_IOCTL) { /* atomic_read(&obj->trace) & ADX_TRC_IOCTL */
			GSE_LOG("gsensor data: %s!\n", buf);
			dumpReg(client);
		}
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int LIS2DW_ReadRawData(struct i2c_client *client, char *buf)
{
	struct lis2dw_i2c_data *obj = (struct lis2dw_i2c_data *)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return EINVAL;

	res = LIS2DW_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	} else {
		sprintf(buf, "%04x %04x %04x", obj->data[LIS2DW_AXIS_X],
			obj->data[LIS2DW_AXIS_Y], obj->data[LIS2DW_AXIS_Z]);

	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t chipinfo_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dw_i2c_client;
	char strbuf[LIS2DW_BUFSIZE];

	if (client == NULL) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	LIS2DW_ReadChipInfo(client, strbuf, LIS2DW_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t sensordata_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dw_i2c_client;
	char strbuf[LIS2DW_BUFSIZE];

	if (client == NULL) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	LIS2DW_ReadSensorData(client, strbuf, LIS2DW_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t cali_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dw_i2c_client;
	struct lis2dw_i2c_data *obj;
	int err, len, mul;
	int tmp[LIS2DW_AXES_NUM];

	len = 0;

	if (client == NULL) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);



	err = LIS2DW_ReadCalibration(client, tmp);
	if (err)
		return -EINVAL;
	else {
		mul = obj->reso / lis2dw_offset_resolution.sensitivity;
		len += snprintf(buf + len, PAGE_SIZE - len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
				obj->offset[LIS2DW_AXIS_X], obj->offset[LIS2DW_AXIS_Y], obj->offset[LIS2DW_AXIS_Z],
				obj->offset[LIS2DW_AXIS_X], obj->offset[LIS2DW_AXIS_Y], obj->offset[LIS2DW_AXIS_Z]);
		len += snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
				obj->cali_sw[LIS2DW_AXIS_X], obj->cali_sw[LIS2DW_AXIS_Y], obj->cali_sw[LIS2DW_AXIS_Z]);

		len += snprintf(buf + len, PAGE_SIZE - len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
				obj->offset[LIS2DW_AXIS_X] * mul + obj->cali_sw[LIS2DW_AXIS_X],
				obj->offset[LIS2DW_AXIS_Y] * mul + obj->cali_sw[LIS2DW_AXIS_Y],
				obj->offset[LIS2DW_AXIS_Z] * mul + obj->cali_sw[LIS2DW_AXIS_Z],
				tmp[LIS2DW_AXIS_X], tmp[LIS2DW_AXIS_Y], tmp[LIS2DW_AXIS_Z]);

		return len;
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t cali_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = lis2dw_i2c_client;
	int err, x, y, z;
	int dat[LIS2DW_AXES_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = LIS2DW_ResetCalibration(client);
		if (err)
			GSE_ERR("reset offset err = %d\n", err);
	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[LIS2DW_AXIS_X] = x;
		dat[LIS2DW_AXIS_Y] = y;
		dat[LIS2DW_AXIS_Z] = z;
		err = LIS2DW_WriteCalibration(client, dat);
		if (err)
			GSE_ERR("write calibration err = %d\n", err);
	} else
		GSE_ERR("invalid format\n");

	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t power_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dw_i2c_client;
	struct lis2dw_i2c_data *obj;
	u8 data;

	if (client == NULL) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	lis_i2c_read_block(client, LIS2DW_REG_CTL_REG1, &data, 0x01);
	return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t firlen_show(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LIS2DW_LOWPASS
	struct i2c_client *client = lis2dw_i2c_client;
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);

	if (atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);

		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for (idx = 0; idx < len; idx++)
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][LIS2DW_AXIS_X], obj->fir.raw[idx][LIS2DW_AXIS_Y], obj->fir.raw[idx][LIS2DW_AXIS_Z]);

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[LIS2DW_AXIS_X], obj->fir.sum[LIS2DW_AXIS_Y], obj->fir.sum[LIS2DW_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[LIS2DW_AXIS_X] / len, obj->fir.sum[LIS2DW_AXIS_Y] / len, obj->fir.sum[LIS2DW_AXIS_Z] / len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t firlen_store(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LIS2DW_LOWPASS
	struct i2c_client *client = lis2dw_i2c_client;
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if (1 != sscanf(buf, "%d", &firlen))
		GSE_ERR("invallid format\n");
	else if (firlen > C_MAX_FIR_LENGTH)
		GSE_ERR("exceeds maximum filter length\n");
	else {
		atomic_set(&obj->firlen, firlen);
		if (firlen == 0)
			atomic_set(&obj->fir_en, 0);
		else {
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t trace_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis2dw_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t trace_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2dw_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t status_show(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lis2dw_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
		obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id, obj->hw.power_vol);

	return len;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR_RO(chipinfo);
static DRIVER_ATTR_RO(sensordata);
static DRIVER_ATTR_RW(cali);
static DRIVER_ATTR_RW(firlen);
static DRIVER_ATTR_RW(trace);
static DRIVER_ATTR_RO(status);
static DRIVER_ATTR_RO(power);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *lis2dw_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_power,         /*show power reg*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
};
/*----------------------------------------------------------------------------*/
static int lis2dw_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(lis2dw_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, lis2dw_attr_list[idx]);
		if (err) {
			GSE_ERR("driver_create_file (%s) = %d\n", lis2dw_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis2dw_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(lis2dw_attr_list));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, lis2dw_attr_list[idx]);


	return err;
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * Function Configuration
******************************************************************************/
#if 0
static int lis2dw_open(struct inode *inode, struct file *file)
{
	file->private_data = lis2dw_i2c_client;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lis2dw_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long lis2dw_unlocked_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)

{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct lis2dw_i2c_data *obj = (struct lis2dw_i2c_data *)i2c_get_clientdata(client);
	char strbuf[LIS2DW_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	/* GSE_FUN(f); */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		LIS2DW_Init(client, 0);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		LIS2DW_ReadChipInfo(client, strbuf, LIS2DW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		LIS2DW_SetPowerMode(client, true);
		LIS2DW_ReadSensorData(client, strbuf, LIS2DW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		LIS2DW_ReadRawData(client, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			GSE_ERR("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		} else {
			cali[LIS2DW_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[LIS2DW_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[LIS2DW_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			err = LIS2DW_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = LIS2DW_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = LIS2DW_ReadCalibration(client, cali);
		if (err)
			break;

		sensor_data.x = cali[LIS2DW_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.y = cali[LIS2DW_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.z = cali[LIS2DW_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;


	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations lis2dw_fops = {
	.owner = THIS_MODULE,
	.open = lis2dw_open,
	.release = lis2dw_release,
	.unlocked_ioctl = lis2dw_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice lis2dw_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lis2dw_fops,
};
#endif
/*----------------------------------------------------------------------------*/
static int lis2dw_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs);
static int lis2dw_get_data(int *x, int *y, int *z, int *status);
static int lis2dw_enable_nodata(int en);

static int lis2dw_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = lis2dw_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		GSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = lis2dw_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int lis2dw_factory_get_data(int32_t data[3], int *status)
{
	return lis2dw_get_data(&data[0], &data[1], &data[2], status);

}
static int lis2dw_factory_get_raw_data(int32_t data[3])
{
	char strbuf[LIS2DW_BUFSIZE] = { 0 };
	LIS2DW_ReadRawData(lis2dw_i2c_client, strbuf);
	return 0;
}
static int lis2dw_factory_enable_calibration(void)
{
	return 0;
}
static int lis2dw_factory_clear_cali(void)
{
	int err = 0;

	err = LIS2DW_ResetCalibration(lis2dw_i2c_client);
	if (err) {
		GSE_ERR("lis2dw_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int lis2dw_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };

	/* obj */
	obj_i2c_data->cali_sw[LIS2DW_AXIS_X] += data[0];
	obj_i2c_data->cali_sw[LIS2DW_AXIS_Y] += data[1];
	obj_i2c_data->cali_sw[LIS2DW_AXIS_Z] += data[2];

	cali[LIS2DW_AXIS_X] = data[0] * gsensor_gain.x / GRAVITY_EARTH_1000;
	cali[LIS2DW_AXIS_Y] = data[1] * gsensor_gain.y / GRAVITY_EARTH_1000;
	cali[LIS2DW_AXIS_Z] = data[2] * gsensor_gain.z / GRAVITY_EARTH_1000;
	err = LIS2DW_WriteCalibration(lis2dw_i2c_client, cali);
	if (err) {
		GSE_ERR("lis2dw_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int lis2dw_factory_get_cali(int32_t data[3])
{
	data[0] = obj_i2c_data->cali_sw[LIS2DW_AXIS_X];
	data[1] = obj_i2c_data->cali_sw[LIS2DW_AXIS_Y];
	data[2] = obj_i2c_data->cali_sw[LIS2DW_AXIS_Z];
	return 0;
}
static int lis2dw_factory_do_self_test(void)
{
	return 0;
}

static struct accel_factory_fops lis2dw_factory_fops = {
	.enable_sensor = lis2dw_factory_enable_sensor,
	.get_data = lis2dw_factory_get_data,
	.get_raw_data = lis2dw_factory_get_raw_data,
	.enable_calibration = lis2dw_factory_enable_calibration,
	.clear_cali = lis2dw_factory_clear_cali,
	.set_cali = lis2dw_factory_set_cali,
	.get_cali = lis2dw_factory_get_cali,
	.do_self_test = lis2dw_factory_do_self_test,
};

static struct accel_factory_public lis2dw_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &lis2dw_factory_fops,
};

/*----------------------------------------------------------------------------*/
static int lis2dw_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);

	int err = 0;
	/* u8 dat; */
	GSE_FUN();
	mutex_lock(&lis2dw_op_mutex);
	/* if(msg.event == PM_EVENT_SUSPEND) */
	/* { */
	if (obj == NULL) {
		mutex_unlock(&lis2dw_op_mutex);
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	/* read old data */
	err = LIS2DW_SetPowerMode(obj->client, false);
	if (err) {
		GSE_ERR("write power control fail!!\n");
		mutex_unlock(&lis2dw_op_mutex);
		return err;
	}
	enable_status = false;
	atomic_set(&obj->suspend, 1);
	/* } */
	sensor_suspend = 1;
	mutex_unlock(&lis2dw_op_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis2dw_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis2dw_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();
	mutex_lock(&lis2dw_op_mutex);
	if (obj == NULL) {
		mutex_unlock(&lis2dw_op_mutex);
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	enable_status = true;

	err = LIS2DW_Init(client, 0);
	if (err) {
		mutex_unlock(&lis2dw_op_mutex);
		GSE_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->suspend, 0);
	sensor_suspend = 0;
	mutex_unlock(&lis2dw_op_mutex);
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int lis2dw_open_report_data(int open)
{
	return 0;
}

static int lis2dw_enable_nodata(int en)
{
	int res = 0;
	int retry = 0;

	if (en == 1)
	{
		enable_status = true;
		for (retry = 0; retry < 3; retry++) {
			res = LIS2DW_Init(obj_i2c_data->client, 0);
			if (res == 0)
				break;
			GSE_LOG("LIS2DW_Init fail\n");
		}
	}
	else if (en == 0)
	{
		enable_status = false;
		for (retry = 0; retry < 3; retry++) {
			res = LIS2DW_SetPowerMode(obj_i2c_data->client, false);
			if (res == 0)
				break;
			GSE_LOG("LIS2DW_SetPowerMode fail\n");
		}
	}

	return res;
}

static int lis2dw_set_delay(u64 ns)
{
	int value = 0;
	struct lis2dw_i2c_data *obj = obj_i2c_data;

printk("lis2dw_set_delay\n");

	value = (int)ns/1000/1000;

	if(value <= 5)
	{
		obj->sample_delay = LIS2DW_BW_200HZ;
	}
	else if(value <= 10)
	{
		obj->sample_delay = LIS2DW_BW_100HZ;
	}
	else
	{
		obj->sample_delay = LIS2DW_BW_50HZ;
	}

	LIS2DW_SetBWRate(obj->client, obj->sample_delay);

	return 0;
}

static int lis2dw_get_data(int *x, int *y, int *z, int *status)
{
	char buff[LIS2DW_BUFSIZE];
	struct lis2dw_i2c_data *priv = obj_i2c_data;

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	memset(buff, 0, sizeof(buff));
	LIS2DW_ReadSensorData(priv->client, buff, LIS2DW_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int lis2dw_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int lis2dw_flush(void)
{
	int err = 0;
	/*Only flush after sensor was enabled*/
	if (!sensor_power) {
		obj_i2c_data->flush = true;
		return 0;
	}
	err = acc_flush_report();
	if (err >= 0)
		obj_i2c_data->flush = false;

	return err;
}

/*----------------------------------------------------------------------------*/
static int lis2dw_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct i2c_client *new_client = NULL;
	struct lis2dw_i2c_data *obj = NULL;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
	int err = 0;
	int retry = 0;

	GSE_FUN();

	if(client->addr != (LIS2DW_I2C_SLAVE_ADDR >> 1))
	{
		return -1;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		GSE_ERR("get cust_baro dts info fail\n");
		goto exit_kfree;
	}

	err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->sample_delay = LIS2DW_BW_100HZ;

#ifdef CONFIG_LIS2DW_LOWPASS
	if (obj->hw.firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw.firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif

	lis2dw_i2c_client = new_client;

	for (retry = 0; retry < 3; retry++) {
		err = LIS2DW_Init(new_client, 1);
		if (err) {
			GSE_ERR("lis2dw_device init cilent fail time: %d\n", retry);
			continue;
		}
	}
	if (err != 0)
		goto exit_init_failed;

//	err = misc_register(&lis2dw_device);
//	if (err) {
//		GSE_ERR("lis2dw_device register failed\n");
//		goto exit_misc_device_register_failed;
//	}

	/* factory */
	err = accel_factory_device_register(&lis2dw_factory_device);
	if (err) {
		GSE_ERR("acc_factory register failed.\n");
		goto exit_misc_device_register_failed;
	}

	err = lis2dw_create_attr(&lis2dw_gsensor_driver.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = lis2dw_open_report_data;
	ctl.enable_nodata = lis2dw_enable_nodata;
	ctl.batch = lis2dw_batch;
	ctl.flush = lis2dw_flush;
	ctl.set_delay  = lis2dw_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw.is_batch_supported;
	err = acc_register_control_path(&ctl);
	if (err) {
		pr_err("acc_register_control_path(%d)\n", err);
		goto exit_kfree;
	}
	data.get_data = lis2dw_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		pr_err("acc_register_data_path(%d)\n", err);
		goto exit_kfree;
	}

	lis2dw_init_flag = 0;

	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
//	misc_deregister(&lis2dw_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
exit_kfree:
	kfree(obj);
exit:
	lis2dw_init_flag = -1;
	GSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int lis2dw_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = lis2dw_delete_attr(&lis2dw_gsensor_driver.platform_diver_addr->driver);
	if (err)
		GSE_ERR("lis2dw_delete_attr fail: %d\n", err);
//	misc_deregister(&lis2dw_device);
	lis2dw_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int lis2dw_probe(void)
{
	GSE_FUN();
	if (i2c_add_driver(&lis2dw_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}

	if(lis2dw_init_flag == -1)
	{
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int lis2dw_remove(void)
{

	GSE_FUN();
	i2c_del_driver(&lis2dw_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*/
static int __init lis2dw_init(void)
{
	acc_driver_add(&lis2dw_gsensor_driver);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit lis2dw_exit(void)
{
	GSE_FUN();
	/* platform_driver_unregister(&lis2dw_gsensor_driver); */
}
/*----------------------------------------------------------------------------*/
module_init(lis2dw_init);
module_exit(lis2dw_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS2DW I2C driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
