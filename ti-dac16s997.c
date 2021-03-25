// SPDX-License-Identifier: GPL-2.0
/*
 * ti-dac16s997.c - Texas Instruments 16-bit 1-channel 4-20mA DAC driver
 *
 * Datasheet:
 * https://www.ti.com/lit/ds/symlink/dac161s997.pdf?ts=1616550300980&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDAC161S997
 *
 *
 * Copyright (C) 2021 Daniel Tritscher <daniel.j.tritscher@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/workqueue.h>



#define DAC16S997_XFER_REG		        0x01
#define DAC16S997_NOP			        0x02
#define DAC16S997_WR_MODE		        0x03
#define DAC16S997_DACCODE		        0x04
#define DAC16S997_ERR_CONFIG	                0x05
#define DAC16S997_ERR_LOW		        0x06
#define DAC16S997_ERR_HIGH		        0x07
#define DAC16S997_RESET			        0x08
#define DAC16S997_STATUS		        0x09
#define DAC16S997_READ_CMD                      0x80
#define DAC16S997_RESET_CMD                     0xC33C
#define DAC16S997_ERR_CONFIG_SPI_TIMEOUT        0x000E
#define DAC16S997_ERR_CONFIG_MASK_SPI_TOUT      0x0001
#define DAC16S997_WR_MODE_MASK                  0x0001
#define DAC16S997_ERR_HIGH_LOW_MASK             0xFF00

/**
 * struct dac16s997 - Driver specific data
 * @spi:                SPI device
 * @lock:               protects read/write sequences
 * @wr_mode:            protected register access (see datasheet 8.5.1.3)
 * @spi_ping_delay      jiffies between NOP writes or 0 if disabled (see datasheet 8.3.1.2)
 * @ping_queue          work queue for periodic NOP reg writes
 * @ping_work           delayed work for periodic NOP reg writes
 */
struct dac16s997 {
	struct spi_device *spi;
	struct mutex lock;
        int wr_mode : 1;        
        unsigned long spi_ping_delay;
        struct workqueue_struct *ping_queue;
        struct delayed_work ping_work;
};




// SPI transfers
static int dac16s997_read_register(struct dac16s997 *priv, u8 reg, u16 *val)
{
	int ret;
        struct spi_transfer xfer[2] = {0};
        u8 rx_buf[3], tx_cmd_buf[3], tx_nop_buf[3];


	tx_cmd_buf[0] = reg | DAC16S997_READ_CMD;
	tx_cmd_buf[1] = 0xff;
	tx_cmd_buf[2] = 0xff;
    
        tx_nop_buf[0] = DAC16S997_NOP;
        tx_nop_buf[1] = 0xff;
        tx_nop_buf[2] = 0xff;
	
        xfer[0].tx_buf = tx_cmd_buf;
        xfer[0].len = 3;
        xfer[0].cs_change = true;
        xfer[0].bits_per_word = 8;

        xfer[1].tx_buf = tx_nop_buf;
        xfer[1].rx_buf = rx_buf;
        xfer[1].len = 3;
        xfer[1].cs_change = true;
        xfer[1].bits_per_word = 8;
    
        ret = spi_sync_transfer(priv->spi, xfer, 2);

	if(ret)
                return ret;
		
	*val = (rx_buf[1]<<8) + rx_buf[2];
	return 0;
}


static int dac16s997_write_register(struct dac16s997 *priv, u8 reg, u16 val)
{
        int ret;
        struct spi_transfer xfer[3] = {0};
        u8 rx_buf[3], tx_cmd_buf[3], tx_xfer_buf[3], tx_nop_buf[3];

	tx_cmd_buf[0] = reg;
	tx_cmd_buf[1] = ((val & 0xff00)>>8);
	tx_cmd_buf[2] = 0x00ff & val;

        if(priv->wr_mode == 0) {
	        return spi_write(priv->spi, tx_cmd_buf, 3);
        }
        else {
                tx_xfer_buf[0] = DAC16S997_XFER_REG;
                tx_xfer_buf[1] = 0x00;
                tx_xfer_buf[2] = 0xff;

                tx_nop_buf[0] = DAC16S997_NOP;
                tx_nop_buf[1] = 0xff;
                tx_nop_buf[2] = 0xff;

                xfer[0].tx_buf = tx_cmd_buf;
                xfer[0].len = 3;
                xfer[0].cs_change = true;
                xfer[0].bits_per_word = 8;

                xfer[1].tx_buf = tx_xfer_buf;
                xfer[1].len = 3;
                xfer[1].cs_change = true;
                xfer[1].bits_per_word = 8;

                xfer[2].tx_buf = tx_nop_buf;
                xfer[2].rx_buf = rx_buf;
                xfer[2].len = 3;
                xfer[2].cs_change = true;
                xfer[2].bits_per_word = 8;
            
                ret = spi_sync_transfer(priv->spi, xfer, 3);

                if(ret)
                        return ret;

                if(memcmp(rx_buf, tx_xfer_buf, 3))
                        return -EIO;
                else
                        return 0; 
        }
}


static int dac16s997_reset(struct dac16s997 *priv)
{
	int ret;
        
        ret = dac16s997_write_register(priv, DAC16S997_RESET, DAC16S997_RESET_CMD);
        if(ret)
                return ret;
        ret = dac16s997_write_register(priv, DAC16S997_NOP, 0xffff);
        if(ret)
                return ret;

	return 0;
}


static void dac16s997_ping(struct work_struct *work)
{
        struct dac16s997 *priv = container_of((struct delayed_work *)work, struct dac16s997, ping_work); 
    
        mutex_lock(&priv->lock);
        dac16s997_write_register(priv, DAC16S997_NOP, 0xffff);
        mutex_unlock(&priv->lock);

        if(priv->spi_ping_delay != 0)
                queue_delayed_work(priv->ping_queue, &priv->ping_work, priv->spi_ping_delay);
}




static ssize_t dac16s997_read_status(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, char *buf)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        u16 status;
        int ret;

        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_STATUS, &status);
        mutex_unlock(&priv->lock);
    
        if(ret)
                return -EIO;

        status = status & 0x00ff;
        return sprintf(buf, "%d\n", (int)status);
}

static ssize_t dac16s997_read_errcfg(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, char *buf)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        u16 err_cfg;
        int ret;

        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_ERR_CONFIG, &err_cfg);
        mutex_unlock(&priv->lock);
    
    if(ret)
                return -EIO;
 
    return sprintf(buf, "%d\n", (int)err_cfg);
}


static ssize_t dac16s997_write_errcfg(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, const char *buf, size_t len)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        int err_cfg;
        int ret;

        ret = kstrtouint(buf, 10, &err_cfg);
        if(ret)
                return ret;

        err_cfg = err_cfg & 0xffff;

        mutex_lock(&priv->lock);
        ret = dac16s997_write_register(priv, DAC16S997_ERR_CONFIG, (u16)err_cfg);
        mutex_unlock(&priv->lock);
        if(ret)
                return ret;

        if((err_cfg & DAC16S997_ERR_CONFIG_MASK_SPI_TOUT) == DAC16S997_ERR_CONFIG_MASK_SPI_TOUT) {
                priv->spi_ping_delay = 0;
        }
        else {
                priv->spi_ping_delay = msecs_to_jiffies((((err_cfg & DAC16S997_ERR_CONFIG_SPI_TIMEOUT)>>1) * 50 + 40));
                queue_delayed_work(priv->ping_queue, &priv->ping_work, 0);
        }
        
        return len;
}

static ssize_t dac16s997_read_wrmode(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, char *buf)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        u16 wr_mode;
        int ret;

        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_WR_MODE, &wr_mode);
        mutex_unlock(&priv->lock);
    
    if(ret)
                return -EIO;
 
    return sprintf(buf, "%d\n", (int)wr_mode);
}


static ssize_t dac16s997_write_wrmode(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, const char *buf, size_t len)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        int wr_mode;
        int ret;

        ret = kstrtouint(buf, 10, &wr_mode);
        if(ret)
                return ret;

        wr_mode = wr_mode & DAC16S997_WR_MODE_MASK;     

        mutex_lock(&priv->lock);
        ret = dac16s997_write_register(priv, DAC16S997_ERR_CONFIG, (u16)wr_mode);
        mutex_unlock(&priv->lock);
        if(ret)
                return ret;

        priv->wr_mode = wr_mode;
        return len;
}


static ssize_t dac16s997_read_errhigh(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, char *buf)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        u16 err_high;
        int ret;
        
        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_ERR_HIGH, &err_high);
        mutex_unlock(&priv->lock);
    
    if(ret)
                return -EIO;
 
    return sprintf(buf, "%d\n", (int)err_high);
}


static ssize_t dac16s997_write_errhigh(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, const char *buf, size_t len)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        int err_high;
        int ret;

        ret = kstrtouint(buf, 10, &err_high);
        if(ret)
                return ret;

        err_high = err_high & DAC16S997_ERR_HIGH_LOW_MASK;     

        if(err_high < 0x8000)
                return -EINVAL;
        
        mutex_lock(&priv->lock);
        ret = dac16s997_write_register(priv, DAC16S997_ERR_HIGH, (u16)err_high);
        mutex_unlock(&priv->lock);
        if(ret)
                return ret;

        return len;
}

static ssize_t dac16s997_read_errlow(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, char *buf)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        u16 errlow;
        int ret;

        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_ERR_LOW, &errlow);
        mutex_unlock(&priv->lock);
    
        if(ret)
                return -EIO;
 
    return sprintf(buf, "%d\n", (int)errlow);
}


static ssize_t dac16s997_write_errlow(struct iio_dev *iio_dev, uintptr_t private, struct iio_chan_spec const *chan, const char *buf, size_t len)
{
        struct dac16s997 *priv = iio_priv(iio_dev);
        int errlow;
        int ret;

        ret = kstrtouint(buf, 10, &errlow);
        if(ret)
                return ret;

        errlow = errlow & DAC16S997_ERR_HIGH_LOW_MASK;

        if(errlow > 0x8000)
                return   -EINVAL ;

        mutex_lock(&priv->lock);
        ret = dac16s997_write_register(priv, DAC16S997_ERR_LOW, (u16)errlow);
        mutex_unlock(&priv->lock);
        if(ret)
                return ret;

        return len;
}


static struct iio_chan_spec_ext_info dac16s997_ext_info[] = {
    {
        .name = "status",
        .shared = IIO_SHARED_BY_ALL,
        .read = dac16s997_read_status,
        .write = NULL,
    },
    {
        .name = "errcfg",
        .shared = IIO_SHARED_BY_ALL,
        .read = dac16s997_read_errcfg,
        .write = dac16s997_write_errcfg,
    },
    {
        .name = "wrmode",
        .shared = IIO_SHARED_BY_ALL,
        .read = dac16s997_read_wrmode,
        .write = dac16s997_write_wrmode,
    },
    {
        .name = "errhigh",
        .shared = IIO_SHARED_BY_ALL,
        .read = dac16s997_read_errhigh,
        .write = dac16s997_write_errhigh,
    },
    {
        .name = "errlow",
        .shared = IIO_SHARED_BY_ALL,
        .read = dac16s997_read_errlow,
        .write = dac16s997_write_errlow,
    },
    {},
};

static const struct iio_chan_spec dac16s997_channel = {
	.type = IIO_CURRENT,
	.channel = 0,
	.output = 1,
	.datasheet_name = "OUT",
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
        .ext_info = dac16s997_ext_info,
};






static int dac16s997_read_raw(struct iio_dev *iio_dev, const struct iio_chan_spec *chan, int *val, int *val2, long mask)
{
        struct dac16s997 *priv;
        u16 value;
        int status;
		
	switch(mask)
	{
	case IIO_CHAN_INFO_RAW:
                priv = iio_priv(iio_dev);
                mutex_lock(&priv->lock);
				status = dac16s997_read_register(priv, DAC16S997_DACCODE, &value);
                mutex_unlock(&priv->lock);
                if(!status) {
                        *val = value;
                        return IIO_VAL_INT;
                }
                else {
                        return -EIO;
                }
	
	case IIO_CHAN_INFO_SCALE:
		*val = 24;
                *val2 = 65535;
		return IIO_VAL_FRACTIONAL;
	
	default:
		return -EINVAL;
	}
}

static int dac16s997_write_raw(struct iio_dev *iio_dev, const struct iio_chan_spec *chan, int val, int val2, long mask)
{
	struct dac16s997 *priv = iio_priv(iio_dev);
	int ret;
	u16 reg_val;
	
	if(mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	reg_val = val & 0xFFFF;
	
	mutex_lock(&priv->lock);
	ret = dac16s997_write_register(priv, DAC16S997_DACCODE, reg_val);
	mutex_unlock(&priv->lock);
	
	return ret;
}

static const struct iio_info dac16s997_info = {
        .read_raw = dac16s997_read_raw,
	.write_raw = dac16s997_write_raw,
};



static int dac16s997_probe(struct spi_device *spi)
{
	struct iio_dev *iio_dev;
	struct dac16s997 *priv;
        u16 dac_reg;
        int ret;
	
	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if(!iio_dev)
		return -ENOMEM;
		
	priv = iio_priv(iio_dev);

	priv->spi = spi;

	spi_set_drvdata(spi, iio_dev);
	iio_dev->info = &dac16s997_info;
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->channels = &dac16s997_channel;
	iio_dev->num_channels = 1;
	iio_dev->name = spi_get_device_id(spi)->name;

        mutex_init(&priv->lock);

        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_WR_MODE, &dac_reg);
        mutex_unlock(&priv->lock);
        if(ret)
                goto out_free_device;

        priv->wr_mode = dac_reg;

        mutex_lock(&priv->lock);
        ret = dac16s997_reset(priv);
        mutex_unlock(&priv->lock);
        if(ret)
                goto out_free_device;

        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_ERR_CONFIG, &dac_reg);
        mutex_unlock(&priv->lock);
        if(ret)
                goto out_free_device;

        priv->spi_ping_delay = msecs_to_jiffies((((dac_reg & DAC16S997_ERR_CONFIG_SPI_TIMEOUT)>>1) * 50 + 40));
        
        mutex_lock(&priv->lock);
        ret = dac16s997_read_register(priv, DAC16S997_WR_MODE, &dac_reg);
        mutex_unlock(&priv->lock);
        if(ret)
                goto out_free_device;

        priv->wr_mode = dac_reg;
        
        priv->ping_queue = create_singlethread_workqueue("ping");

        INIT_DELAYED_WORK(&priv->ping_work, dac16s997_ping);
        queue_delayed_work(priv->ping_queue, &priv->ping_work, priv->spi_ping_delay);
	
	return devm_iio_device_register(&spi->dev, iio_dev);

out_free_device:
        devm_iio_device_free(&spi->dev, iio_dev);
        return -EIO;
}

static int dac16s997_remove(struct spi_device *spi)
{
	struct iio_dev *iio_dev = spi_get_drvdata(spi);
	struct dac16s997 *priv = iio_priv(iio_dev);

	devm_iio_device_unregister(&spi->dev, iio_dev);
        mutex_destroy(&priv->lock);
        cancel_delayed_work(&priv->ping_work);
        destroy_workqueue(priv->ping_queue);
	devm_iio_device_free(&spi->dev, iio_dev);
        return 0;
}


static const struct spi_device_id dac16s997_id[] = {
	{"ti-dac16s997"},
		{}
};
MODULE_DEVICE_TABLE(spi, dac16s997_id);

static const struct of_device_id dac16s997_of_match[] = {
	{ .compatible = "ti,dac16s997" },
	{ },
};
MODULE_DEVICE_TABLE(of, dac16s997_of_match);

static struct spi_driver dac16s977_driver = {
        .driver = {
                .name = "ti-dac16s997",
                .of_match_table = dac16s997_of_match,
                .owner = THIS_MODULE,
                },
        .probe = dac16s997_probe,
        .remove = dac16s997_remove,
        .id_table = dac16s997_id,
};
module_spi_driver(dac16s977_driver);


MODULE_AUTHOR("Daniel Tritscher <daniel.j.tritscher@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments DAC16S997 16-bit single channel 4-20mA DAC Driver");
MODULE_LICENSE("GPL v2");
