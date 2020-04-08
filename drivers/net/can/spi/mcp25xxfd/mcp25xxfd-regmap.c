// SPDX-License-Identifier: GPL-2.0
//
// mcp25xxfd - Microchip MCP25xxFD Family CAN controller driver
//
// Copyright (c) 2019, 2020 Pengutronix,
//                          Marc Kleine-Budde <kernel@pengutronix.de>
//

#include "mcp25xxfd.h"

#include <asm/unaligned.h>

static int mcp25xxfd_regmap_write(void *context, const void *data, size_t count)
{
	struct spi_device *spi = context;

	return spi_write(spi, data, count);
}

static int mcp25xxfd_regmap_gather_write(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	struct spi_device *spi = context;
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_map_buf *buf_tx = priv->map_buf_tx;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = buf_tx,
			.len = sizeof(buf_tx->cmd) + val_len,
		},
	};

	BUILD_BUG_ON(sizeof(buf_tx->cmd) != sizeof(__be16));

	if (IS_ENABLED(CONFIG_CAN_MCP25XXFD_SANITY) &&
	    reg_len != sizeof(buf_tx->cmd))
		return -EINVAL;

	memcpy(&buf_tx->cmd, reg, sizeof(buf_tx->cmd));
	memcpy(buf_tx->data, val, val_len);

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

static inline bool mcp25xxfd_update_bits_read_reg(unsigned int reg)
{
	switch (reg) {
	case MCP25XXFD_CAN_INT:
	case MCP25XXFD_CAN_TEFCON:
	case MCP25XXFD_CAN_FIFOCON(MCP25XXFD_RX_FIFO(0)):
	case MCP25XXFD_CAN_FLTCON(0):
	case MCP25XXFD_ECCSTAT:
	case MCP25XXFD_CRC:
		return false;
	case MCP25XXFD_CAN_CON:
	case MCP25XXFD_CAN_FIFOSTA(MCP25XXFD_RX_FIFO(0)):
	case MCP25XXFD_OSC:
	case MCP25XXFD_ECCCON:
		return true;
	default:
		WARN(1, "Status of reg 0x%04x unknown.\n", reg);
	}

	return true;
}

static int mcp25xxfd_regmap_update_bits(void *context, unsigned int reg,
					unsigned int mask, unsigned int val)
{
	struct spi_device *spi = context;
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_map_buf *buf_rx = priv->map_buf_rx;
	struct mcp25xxfd_map_buf *buf_tx = priv->map_buf_tx;
	__le32 orig_le32 = 0, mask_le32, val_le32, tmp_le32;
	u8 first_byte, last_byte, len;
	int err;

	BUILD_BUG_ON(sizeof(buf_rx->cmd) != sizeof(__be16));
	BUILD_BUG_ON(sizeof(buf_tx->cmd) != sizeof(__be16));

	if (IS_ENABLED(CONFIG_CAN_MCP25XXFD_SANITY) &&
	    mask == 0)
		return -EINVAL;

	first_byte = mcp25xxfd_first_byte_set(mask);
	last_byte = mcp25xxfd_last_byte_set(mask);
	len = last_byte - first_byte + 1;

	if (mcp25xxfd_update_bits_read_reg(reg)) {
		struct spi_transfer xfer[] = {
			{
				.tx_buf = buf_tx,
				.rx_buf = buf_rx,
				.len = sizeof(buf_tx->cmd) + len,
			},
		};

		mcp25xxfd_spi_cmd_read(&buf_tx->cmd, reg + first_byte);
		err = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
		if (err)
			return err;

		memcpy(&orig_le32, buf_rx->data, len);
	}

	mask_le32 = cpu_to_le32(mask >> BITS_PER_BYTE * first_byte);
	val_le32 = cpu_to_le32(val >> BITS_PER_BYTE * first_byte);

	tmp_le32 = orig_le32 & ~mask_le32;
	tmp_le32 |= val_le32 & mask_le32;

	mcp25xxfd_spi_cmd_write(&buf_tx->cmd, reg + first_byte);
	memcpy(buf_tx->data, &tmp_le32, len);

	return spi_write(spi, buf_tx, sizeof(buf_tx->cmd) + len);
}

static int mcp25xxfd_regmap_read(void *context,
				 const void *reg, size_t reg_len,
				 void *val_buf, size_t val_len)
{
	struct spi_device *spi = context;
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_map_buf *buf_rx = priv->map_buf_rx;
	struct mcp25xxfd_map_buf *buf_tx = priv->map_buf_tx;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = buf_tx,
			.rx_buf = buf_rx,
			.len = sizeof(buf_tx->cmd) + val_len,
		},
	};
	int err;

	BUILD_BUG_ON(sizeof(buf_rx->cmd) != sizeof(__be16));
	BUILD_BUG_ON(sizeof(buf_tx->cmd) != sizeof(__be16));

	if (IS_ENABLED(CONFIG_CAN_MCP25XXFD_SANITY) &&
	    reg_len != sizeof(buf_tx->cmd))
		return -EINVAL;

	memcpy(&buf_tx->cmd, reg, sizeof(buf_tx->cmd));

	err = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (err)
		return err;

	memcpy(val_buf, buf_rx->data, val_len);

	return 0;
}

static int mcp25xxfd_regmap_crc_gather_write(void *context,
					     const void *reg_p, size_t reg_len,
					     const void *val, size_t val_len)
{
	struct spi_device *spi = context;
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_map_buf_crc *buf_tx = priv->map_buf_crc_tx;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = buf_tx,
			.len = sizeof(buf_tx->cmd) + val_len +
				sizeof(buf_tx->crc),
		},
	};
	u16 reg = *(u16 *)reg_p;
	u16 crc;

	BUILD_BUG_ON(sizeof(buf_tx->cmd) != sizeof(__be16) + sizeof(u8));

	if (IS_ENABLED(CONFIG_CAN_MCP25XXFD_SANITY) &&
	    reg_len != sizeof(buf_tx->cmd.cmd))
		return -EINVAL;

	mcp25xxfd_spi_cmd_write_crc(&buf_tx->cmd, reg, val_len);
	memcpy(buf_tx->data, val, val_len);

	crc = mcp25xxfd_crc16_compute(buf_tx, sizeof(buf_tx->cmd) + val_len);
	put_unaligned_be16(crc, buf_tx->data + val_len);

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

static int mcp25xxfd_regmap_crc_write(void *context,
				      const void *data, size_t count)
{
	return mcp25xxfd_regmap_crc_gather_write(context, data, sizeof(__be16),
						 data + 4, count - 4);
}

static int
mcp25xxfd_regmap_crc_read_one(struct mcp25xxfd_priv *priv,
			      struct spi_transfer *xfer,
			      u16 reg,
			      size_t val_len)
{
	struct mcp25xxfd_map_buf_crc *buf_rx = priv->map_buf_crc_rx;
	struct mcp25xxfd_map_buf_crc *buf_tx = priv->map_buf_crc_tx;
	u16 crc_received, crc_calculated;
	int err;

	BUILD_BUG_ON(sizeof(buf_rx->cmd) != sizeof(__be16) + sizeof(u8));
	BUILD_BUG_ON(sizeof(buf_tx->cmd) != sizeof(__be16) + sizeof(u8));

	err = spi_sync_transfer(priv->spi, xfer, 1);
	if (err)
		return err;

	crc_received = get_unaligned_be16(buf_rx->data + val_len);
	crc_calculated = mcp25xxfd_crc16_compute2(&buf_tx->cmd,
						  sizeof(buf_tx->cmd),
						  buf_rx->data, val_len);
	if (crc_received != crc_calculated) {
		netdev_info(priv->ndev,
			    "CRC read error at address 0x%04x, length %d.\n",
			    reg, val_len);

		return -EBADMSG;
	}

	return 0;
}

static int mcp25xxfd_regmap_crc_read(void *context,
				     const void *reg_p, size_t reg_len,
				     void *val_buf, size_t val_len)
{
	struct spi_device *spi = context;
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_map_buf_crc *buf_rx = priv->map_buf_crc_rx;
	struct mcp25xxfd_map_buf_crc *buf_tx = priv->map_buf_crc_tx;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = buf_tx,
			.rx_buf = buf_rx,
			.len = sizeof(buf_tx->cmd) + val_len +
				sizeof(buf_tx->crc),
		},
	};
	u16 reg = *(u16 *)reg_p;
	int err;

	BUILD_BUG_ON(sizeof(buf_rx->cmd) != sizeof(__be16) + sizeof(u8));
	BUILD_BUG_ON(sizeof(buf_tx->cmd) != sizeof(__be16) + sizeof(u8));

	if (IS_ENABLED(CONFIG_CAN_MCP25XXFD_SANITY) &&
	    reg_len != sizeof(buf_tx->cmd))
		return -EINVAL;

	mcp25xxfd_spi_cmd_read_crc(&buf_tx->cmd, reg, val_len);
	err = mcp25xxfd_regmap_crc_read_one(priv, xfer, buf_rx, reg, val_len);
	if (err)
		return err;

	memcpy(val_buf, buf_rx->data, val_len);

	return 0;
}

static const struct regmap_range mcp25xxfd_reg_table_yes_range[] = {
	regmap_reg_range(0x000, 0x2ec),	/* CAN FD Controller Module SFR */
	regmap_reg_range(0x400, 0xbfc),	/* RAM */
	regmap_reg_range(0xe00, 0xe14),	/* MCP2517/18FD SFR */
};

/* On the mcp2517fd the CRC on the TBC fails, while on the mcp2518fd
 * it work, disallow it in general.
 */
static const struct regmap_range mcp25xxfd_reg_table_no_range_crc[] = {
	regmap_reg_range(0x010, 0x010),	/* TBC */
};

static const struct regmap_access_table mcp25xxfd_reg_table = {
	.yes_ranges = mcp25xxfd_reg_table_yes_range,
	.n_yes_ranges = ARRAY_SIZE(mcp25xxfd_reg_table_yes_range),
};

static const struct regmap_access_table mcp25xxfd_reg_table_crc = {
	.yes_ranges = mcp25xxfd_reg_table_yes_range,
	.n_yes_ranges = ARRAY_SIZE(mcp25xxfd_reg_table_yes_range),
	.no_ranges = mcp25xxfd_reg_table_no_range_crc,
	.n_no_ranges = ARRAY_SIZE(mcp25xxfd_reg_table_no_range_crc),
};

static const struct regmap_config mcp25xxfd_regmap = {
	.reg_bits = 16,
	.reg_stride = 4,
	.pad_bits = 0,
	.val_bits = 32,
	.max_register = 0xffc,
	.wr_table = &mcp25xxfd_reg_table,
	.rd_table = &mcp25xxfd_reg_table,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = (__force unsigned long)
		cpu_to_be16(MCP25XXFD_INSTRUCTION_READ),
	.write_flag_mask = (__force unsigned long)
		cpu_to_be16(MCP25XXFD_INSTRUCTION_WRITE),
};

static const struct regmap_bus mcp25xxfd_bus = {
	.write = mcp25xxfd_regmap_write,
	.gather_write = mcp25xxfd_regmap_gather_write,
	.reg_update_bits = mcp25xxfd_regmap_update_bits,
	.read = mcp25xxfd_regmap_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.max_raw_read = FIELD_SIZEOF(struct mcp25xxfd_map_buf, data),
	.max_raw_write = FIELD_SIZEOF(struct mcp25xxfd_map_buf, data),
};

static const struct regmap_config mcp25xxfd_regmap_crc = {
	.name = "crc",
	.reg_bits = 16,
	.reg_stride = 4,
	.pad_bits = 16,		/* keep data bits aligned */
	.val_bits = 32,
	.max_register = 0xffc,
	.wr_table = &mcp25xxfd_reg_table_crc,
	.rd_table = &mcp25xxfd_reg_table_crc,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_bus mcp25xxfd_bus_crc = {
	.write = mcp25xxfd_regmap_crc_write,
	.gather_write = mcp25xxfd_regmap_crc_gather_write,
	.read = mcp25xxfd_regmap_crc_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.max_raw_read = FIELD_SIZEOF(struct mcp25xxfd_map_buf_crc, data),
	.max_raw_write = FIELD_SIZEOF(struct mcp25xxfd_map_buf_crc, data),
};

static int
mcp25xxfd_regmap_init_nocrc(struct mcp25xxfd_priv *priv)
{
	priv->map = devm_regmap_init(&priv->spi->dev, &mcp25xxfd_bus,
				     priv->spi, &mcp25xxfd_regmap);

	return PTR_ERR_OR_ZERO(priv->map);
}

static int
mcp25xxfd_regmap_init_crc(struct mcp25xxfd_priv *priv)
{
	if (!(priv->devtype_data->quirks & MCP25XXFD_QUIRK_RX_CRC)) {
		priv->map_rx = priv->map;

		return 0;
	}

	priv->map_crc = devm_regmap_init(&priv->spi->dev, &mcp25xxfd_bus_crc,
					 priv->spi, &mcp25xxfd_regmap_crc);
	if (IS_ERR(priv->map_crc))
		return PTR_ERR(priv->map_crc);

	priv->map_buf_crc_rx = devm_kzalloc(&priv->spi->dev,
					    sizeof(*priv->map_buf_crc_rx),
					    GFP_KERNEL);
	if (!priv->map_buf_crc_rx)
		return -ENOMEM;

	priv->map_buf_crc_tx = devm_kzalloc(&priv->spi->dev,
					    sizeof(*priv->map_buf_crc_tx),
					    GFP_KERNEL);
	if (!priv->map_buf_crc_tx)
		return -ENOMEM;

	priv->map_rx = priv->map_crc;

	return 0;
}

int mcp25xxfd_regmap_init(struct mcp25xxfd_priv *priv)
{
	int err;

	err = mcp25xxfd_regmap_init_nocrc(priv);
	if (err)
		return err;

	return mcp25xxfd_regmap_init_crc(priv);
}
