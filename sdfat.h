/**
 * Written by Tim Johns.
 * Icewire Technologies
 * 
 * SD card SPI interface and FAT16 implementation library.
 *
 */

#ifndef _SDFATLIB_H
#define _SDFATLIB_H

/* Card select (P4.7) */
#define SD_SELECT() P4OUT &= ~(0x80)
/* Card deselect (P4.7) */
#define SD_DESELECT() P4OUT |= 0x80

enum SDErrCodes {
	SD_SUCCESS = 0,
	SD_TIMEOUT,
	SD_BAD_TYPE,
	SD_NOT_HC,
	SD_BAD_TOKEN
};

typedef enum {
	CMD0 = 0,		/* GO_IDLE_STATE */
	CMD8 = 8,		/* SEND_IF_COND */
	CMD13 = 13,		/* SEND_STATUS */
	CMD17 = 17,		/* READ_SINGLE_BLOCK */
	CMD24 = 24,		/* WRITE_BLOCK */
/* 	CMD25 = 25, */		/* WRITE_MULTIPLE_BLOCK */
	CMD55 = 55,		/* APP_CMD */
	CMD58 = 58,		/* READ_OCR */
	ACMD41 = 41		/* SD_SEND_OP_COND */
} SDcmd;

/* SD Card Tokens for Multiple Block Write */
/* #define START_BLK_TOK	0xFC */	/* 'Start Block' token */
/* #define STOP_TRANS_TOK	0xFD */	/* 'Stop Tran' token (stop transmission) */

/* SD Card type flags (CardType) */
enum SDCardType {
	CT_MMC = 0x01,			/* MMC ver 3 */
	CT_SD1 = 0x02,			/* SD ver 1 */
	CT_SD2 = 0x04,			/* SD ver 2 */
	CT_SDC = (CT_SD1|CT_SD2),	/* SD */
	CT_BLOCK = 0x08,			/* Block addressing */
	CT_SDHC = (CT_SD2 | CT_BLOCK)	/* SDHC */
};

enum SDTimeout {
	SD_SHORT_TIMEOUT = 10,
	SD_MED_TIMEOUT = 0x200,
	SD_LONG_TIMEOUT = 0x1000
};

enum SDTokens{
	SD_NOT_BUSY = 0xFF,
	SD_START_BLOCK = 0xFE,
	SD_BLOCK_ERR = 0xFF,
	SD_SINGLE_BLK = 0xFE,
	SD_VERIFY_TYPE = 0x01,
	SD_WRITE_BLK = 0x05
};

enum { SD_WRITE_BLK_MASK = 0x1F };

/* FAT Constants */
enum {
	BLKSIZE = 512,	/* Block size (in bytes) */
	DTE_PER_BLK = 16,	/* Directory table entries per block */
	DTESIZE = 32,	/* Directory table entry size (in bytes) */
	DTEDEL = 0xE5	/* Directory table entry has been deleted */
};

enum FATErrCodes {
	FAT_SUCCESS = 0,
	FAT_DT_FULL,
	FAT_BAD_BOOT_SECT,
	FAT_BAD_SECT_SIZE
};

struct fatstruct {	/* FAT information based on boot sector */
	uint16_t nbytesinsect;			/* Number of bytes per sector, should be 512 */
	uint8_t nsectsinclust;			/* Number of sectors per cluster */
	uint32_t nbytesinclust;			/* bytes per sector * sectors per cluster */
	uint16_t nressects;				/* Number of reserved sectors from offset 0 */
	uint16_t nsectsinfat;			/* Number of sectors per FAT  */
	uint8_t nfats;					/* Number of FATs */
	uint32_t fatsize;				/* Number of bytes per FAT  */
	uint32_t fatoffset;				/* Offset of the first FAT  */
	uint32_t dtoffset;				/* Offset of the directory table */
	uint32_t dtsize;				/* Size of directory table in bytes */
	uint32_t nsects;				/* Number of sectors in the partition */
	uint32_t fileclustoffset;		/* Offset of the first cluster for file data */
	uint32_t nhidsects;				/* Number of hidden sectors */
	/* Offset of the boot record sector, determined by number of hidden sectors */
	uint32_t bootoffset;
};

uint8_t init_sd(void);
void go_idle_sd(void);
uint8_t send_cmd_sd(SDcmd cmd, uint32_t arg);
uint8_t send_acmd_sd(SDcmd acmd, uint32_t arg);
/* uint8_t write_multiple_block(uint32_t start_offset); */
uint8_t write_block(uint8_t *data, uint32_t offset, uint16_t count);
uint8_t read_block(uint8_t *data, uint32_t offset);
uint16_t find_cluster(uint8_t *data, struct fatstruct *info);
uint32_t get_cluster_offset(uint16_t clust, struct fatstruct *info);
uint8_t valid_block(uint8_t block, struct fatstruct *info);
uint8_t update_fat(uint8_t *data, struct fatstruct *info, uint16_t index, uint16_t num);
uint8_t update_dir_table(uint8_t *data, struct fatstruct *info, uint16_t cluster, uint32_t file_size);
uint8_t valid_boot_sector(uint8_t *data, struct fatstruct *boot);
uint8_t parse_boot_sector(uint8_t *data, struct fatstruct *info);
void delete_file(uint8_t, uint32_t, uint8_t *data, struct fatstruct *info);
void format_sd(uint8_t *data, struct fatstruct *info, void (*pre_format)(), void (*during_format)(), void (*post_format)());

/*
 * Scan through directory table for highest file number suffix and return the
 * next highest number.
 */
uint16_t get_file_num(uint8_t *data, struct fatstruct *info);

#endif
