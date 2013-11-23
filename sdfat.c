/**
 * Written by Tim Johns.
 * Icewire Technologies
 *
 * SD card SPI interface and FAT16 implementation.
 *
 * In the current circuit design, the SD card is using the USCI_A1 SPI bus, thus
 * the functions spia_send() and spia_rec() are used.
 */

/*
* FAT16 Boot Sector
* 
* Field               Offset     Length
* -----               ------     ------
* Bytes Per Sector      11(0Bh)    2
* Sectors Per Cluster   13(0Dh)    1
* Reserved Sectors      14(0Eh)    2
* FATs                  16(10h)    1
* Root Entries          17(11h)    2
* Small Sectors         19(13h)    2
* Media Descriptor      21(15h)    1
* Sectors Per FAT       22(16h)    2
* Sectors Per Track     24(18h)    2
* Heads                 26(1Ah)    2
* Hidden Sectors        28(1Ch)    4
* Large Sectors         32(20h)    4
*/

#ifndef _SDFATLIB_C
#define _SDFATLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "circuit.h"
#include "sdfat.h"
#include "macro.h"
#include "const.h"

/*
 * Initialize SD Card
 */
uint8_t init_sd(void) {
	SD_DESELECT();

	/* Must supply min of 74 clock cycles with CS high */
	for (uint8_t i = 0; i < 80; i++) spia_send(DUMMY);
	
	SD_SELECT();

	/* Enter SPI mode */
	uint32_t tmr;
	for (tmr = SD_SHORT_TIMEOUT; tmr && (send_cmd_sd(CMD0, 0) != 1); tmr--);
	if (tmr == 0) {
		SD_DESELECT();
		return SD_TIMEOUT;
	}

	/* Verify SD 2.0 and 2.7-3.6V */
	if (send_cmd_sd(CMD8, 0x1AA) != SD_VERIFY_TYPE) {
		SD_DESELECT();
		return SD_BAD_TYPE;
	}
	/* Get response */
	uint8_t ocr[4];
	for (uint8_t n = 0; n < 4; n++) {
		ocr[n] = spia_rec();
	}
	/* 2.7-3.6 V */
	if (ocr[2] != 0x01 || ocr[3] != 0xAA) {
		SD_DESELECT();
		return SD_BAD_TYPE;
	}

	/* Wait for leaving idle state (ACMD41 with HCS bit) */
	for (tmr = SD_LONG_TIMEOUT; tmr && (send_acmd_sd(ACMD41, 1UL << 30) != 0); tmr--);
	if (tmr == 0) {
		SD_DESELECT();
		return SD_TIMEOUT;
	}

	/* Check High Capacity support (SDHC) */
	if (send_cmd_sd(CMD58, 0)) {
		SD_DESELECT();
		return SD_NOT_HC;
	}
	/* Get response */
	for (uint8_t n = 0; n < 4; n++) {
		ocr[n] = spia_rec();
	}
	/* SD 2.0 (HC or not) */
	/* TODO I don't get the point of this since this will always return SD_SUCCESS? */
	enum SDCardType ct = (ocr[0] & BIT6) ? CT_SDHC : CT_SD2;

	SD_DESELECT();

	if (ct == CT_SD2) {	/* SD 2.0 */
		return SD_SUCCESS;
	}

	/* SDHC */
	if (ct == CT_SDHC) {
		return SD_SUCCESS;
	}

	/* Fail: unrecognized card type */
	return SD_BAD_TYPE;
}

/*
 * Send command to enter idle state
 */
void go_idle_sd() {
	SD_SELECT();
	
	send_cmd_sd(CMD0, 0);

	/* Note: leave CS low to refrain from consuming power */
}

/*
 * Send command and return error code. Return zero for OK
 */
uint8_t send_cmd_sd(SDcmd cmd, uint32_t arg) {
	spia_send(cmd | BIT6);	/* Send command */
	
	/* Send argument */
	for (int8_t s = 24; s >= 0; s -= 8) {
		spia_send(arg >> s);
	}
	
	/* Send CRC */
	uint8_t crc = DUMMY;
	/* Correct CRC for CMD0 with arg 0 */
	if (cmd == CMD0) {
		crc = 0x95;
	}
	/* Correct CRC for CMD8 with arg 0x1AA */
	if (cmd == CMD8) {
		crc = 0x87;
	}
	spia_send(crc);
	
	/* Wait for response */
	uint8_t status;
	for (uint8_t i = 0; ((status = spia_rec()) & BIT7) && i < MAXBYTE; i++);
	
	return status;
}

/*
 * Send ACMD to SPI
 */
uint8_t send_acmd_sd(SDcmd acmd, uint32_t arg) {
	uint8_t resp = send_cmd_sd(CMD55, 0);
	if (resp > 1) {
		return resp;
	}
	return send_cmd_sd(acmd, arg);
}

/*
 * Wait for the card
 */
void wait_notbusy(void) {
	while (spia_rec() != SD_NOT_BUSY);
}

/*
 * Wait for Start Block token
 */
uint8_t wait_startblock(void) {
	for (uint16_t i = 0; i < SD_MED_TIMEOUT; i++) {
		uint8_t rec = spia_rec();
		if (rec == SD_START_BLOCK) {	/* Start Block token received */
			return SD_SUCCESS;
		}
		if (rec != SD_BLOCK_ERR) {	/* Error */
			return SD_BAD_TOKEN;
		}
	}
	return SD_TIMEOUT;
}

#if 0
/*
 * Write multiple blocks
 * Assume data buffer 2048 bytes in size and write 4 consecutive 512-byte
 * blocks, beginning at start_offset.
 */
uint8_t write_multiple_block(uint8_t *data, uint32_t start_offset) {
	SD_SELECT();

	wait_notbusy();	/* Wait for card to be ready */

	/* WRITE_MULTIPLE_BLOCK command */
	uint8_t err;
	if (err = send_cmd_sd(CMD25, start_offset)) {
		SD_DESELECT();
		return err;
	}

	/* Write data buffer to 4 blocks */
	for (uint8_t i = 0; i < 4; i++) {
		/* Send 'Start Block' token for each block */
		spia_send(START_BLK_TOK);

		for (uint16_t j = 0; j < BLKSIZE; j++) {
			spia_send(data[j]);
		}

		spia_send(DUMMY);	/* Dummy CRC */
		spia_send(DUMMY);	/* Dummy CRC */

		wait_notbusy();	/* Wait for flash programming to complete */
	}

	spia_send(STOP_TRANS_TOK);	/* Send 'Stop Tran' token (stop transmission) */

	/* Get status */
	if (err = send_cmd_sd(CMD13, 0) || spia_rec())	{
		SD_DESELECT();
		return err;
	}
	
	SD_DESELECT();

	return SD_SUCCESS;
}
#endif

/*
 * Write the first count bytes in the given data buffer starting at offset
 */
uint8_t write_block(uint8_t *data, uint32_t offset, uint16_t count) {
	SD_SELECT();
	
	/* WRITE_BLOCK command */
	uint8_t err = send_cmd_sd(CMD24, offset);
	if (err) {
		SD_DESELECT();
		return err;
	}
	spia_send(SD_SINGLE_BLK);	/* Write Single Block token */
	
	/* Write data bytes */
	if (count > BLKSIZE) count = BLKSIZE;
	uint16_t i;
	for (i = 0; i < count; i++) {
		spia_send(data[i]);
	}
	/* Padding to fill block */
	for (; i < BLKSIZE; i++) {
		spia_send(0);
	}
	
	spia_send(DUMMY);	/* Dummy CRC */
	spia_send(DUMMY);	/* Dummy CRC */
	
	if ((spia_rec() & SD_WRITE_BLK_MASK) != SD_WRITE_BLK) {
		SD_DESELECT();
		return SD_BAD_TOKEN;
	}

	/* Wait for flash programming to complete */
	wait_notbusy();
	
	/* Get status */
	if (send_cmd_sd(CMD13, 0) || spia_rec())	{
		SD_DESELECT();
		return SD_BAD_TOKEN;
	}
	
	SD_DESELECT();
	
	return SD_SUCCESS;
}

/*
 * Read 512 bytes from offset and store them in the given data buffer
 */
uint8_t read_block(uint8_t *data, uint32_t offset) {
	SD_SELECT();
	
	/* READ_SINGLE_BLOCK command with offset as argument */
	uint8_t err = send_cmd_sd(CMD17, offset);
	if (err) {
		SD_DESELECT();
		return err;
	}

	/* Wait for the start of the block */
	if (wait_startblock()) {
		SD_DESELECT();
		return SD_TIMEOUT;
	}
	
	/* Read bytes */
	for (uint16_t i = 0; i < BLKSIZE; i++) {
		data[i] = spia_rec();
	}
	
	SD_DESELECT();
	
	return SD_SUCCESS;
}

/*
 * Find and return a free cluster for writing file contents (also writes to
 * FAT)
 * Start searching incrementally, starting at start_cluster.
 * Return free cluster index (>0).
 * Return 0 on error or if there are no more free clusters.
 */
uint16_t find_cluster(uint8_t *data, struct fatstruct *info) {
	uint32_t block_offset = 0;
	for (uint32_t i = 0; i < info->fatsize; i += 2) {
		uint32_t j = i % BLKSIZE;	/* Cluster index relative to block */
				
		/* Read each new block of the FAT */
		if (j == 0) {
			block_offset = info->fatoffset + i;
			if (read_block(data, block_offset)) return 0;
		}
		
		if (data[j] == 0x00 && data[j+1] == 0x00) {
			
			/* Set cluster to 0xFFFF to indicate end of cluster chain for current file
			   (will be modified if file data continues) */
			data[j] = 0xFF;
			data[j+1] = 0xFF;

			/* Write to FAT  */
			if (write_block(data, block_offset, BLKSIZE)) return 0;

			/* Write to second FAT  */
			if (info->nfats > 1) {
				if (write_block(data, block_offset + info->fatsize, BLKSIZE))
					return 0;
			}

			/* Return free cluster index */
			return i >> 1;
		}
	}
	
	/* Failed to find a free cluster (disk may be full) */
	return 0;
}

/*
 * Return the offset of the given cluster number
 */
uint32_t get_cluster_offset(uint16_t clust, struct fatstruct *info) {
	return info->fileclustoffset + ((clust - 2) * info->nbytesinclust);
}

/*
 * Return true iff block number is less than nsectsinclust
 */
uint8_t valid_block(uint8_t block, struct fatstruct *info) {
	return block < info->nsectsinclust;
}

/*
 * Update the FAT
 * Replace the cluster word at byte offset index with num.
 */
uint8_t update_fat(uint8_t *data, struct fatstruct *info, uint16_t index, uint16_t num) {
	uint32_t block_offset = info->fatoffset + index - (index % BLKSIZE);
	
	/* Read the right block of the FAT  */
	{
		uint8_t err = read_block(data, block_offset);
		if (err) {
			return err;
		}
	}

	index = index % BLKSIZE;	/* Change index from absolute to relative */

	/* Point cluster word at index to num cluster */
	data[index] = WTOB_L(num);
	data[index+1] =  WTOB_H(num);

	/* Write to FAT  */
	{
		uint8_t err = write_block(data, block_offset, 512);
		if (err) {
			return err;
		}
	}

	/* Write to second FAT  */
	if (info->nfats > 1) {
		uint8_t err = write_block(data, block_offset + info->fatsize, 512);
		if (err) {
			return err;
		}
	}

	return FAT_SUCCESS;
}

/*
 * Update directory table
 *
 * File number is same as entry position, only write to empty entries (not
 * deleted).
 * 1,2,3,4 -> delete 2 -> 1,3,4 -> save file -> 1,3,4,5 -> delete 5 ->
 * 1,3,4 -> save file -> 1,3,4,6
 * When no more empty entries, use deleted entries.
 *
 * cluster: file's starting cluster
 * file_size: total bytes in file
 * file_num: file name number suffix
 */
/* TODO Contains project specific code, i.e., dte */
uint8_t update_dir_table(uint8_t *data, struct fatstruct *info, uint16_t cluster, uint32_t file_size) {

	uint8_t err;

	/* Directory table entry (MUST BE 32 BYTES) */
	uint8_t dte[] = "DATA000 WAV\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

	/* Read the directory table. */
	/* Find the last entry and prepare the next directory table entry. */
	uint32_t i, j;
	for (i = 0, j = 0; i < info->dtsize; i += 32) {
		j = i % info->nbytesinsect;
		if (j == 0) {
			/* Next sector */
			if (err = read_block(data, info->dtoffset + i)) return err;
		}
		
		/* Check for empty entry (not deleted file (0xE5 prefix)) */
		if (data[j] == 0x00) {
			break;	/* Found the entry offset */
		}
	}

	/* If no more empty entries, start using deleted entries */
	if (i >= info->dtsize) {
		for (i = 0, j = 0; i < info->dtsize; i += 32) {
			j = i % info->nbytesinsect;
			if (j == 0) {
				/* Next sector */
				if (err = read_block(data, info->dtoffset + i)) return err;
			}
			
			/* Check for deleted file (0xE5 prefix) */
			if (data[j] == 0xE5) {
				break;	/* Found the entry offset */
			}
		}
	}

	/* Check if directory table is full */
	if (i >= info->dtsize) return FAT_DT_FULL;
	
	/* Offset of directory table entry */
	uint32_t dir_entry_offset = info->dtoffset + i;

	/* Set filename prefix */
	dte[0] = 'D'; dte[1] = 'A'; dte[2] = 'T'; dte[3] = 'A';

	/* Set file number equal to entry number + 1 (start at 001) */
	uint16_t file_num = (i / 32) + 1;

	/* Set filename suffix (e.g., "012") */
	dte[4] = ((file_num / 100) % 10) + 0x30;
	dte[5] = ((file_num / 10) % 10) + 0x30;
	dte[6] = (file_num % 10) + 0x30;

	/* Set starting cluster */
	dte[26] = WTOB_L(cluster);
	dte[27] = WTOB_H(cluster);

	/* Set file size */
	dte[28] = DTOB_LL(file_size);
	dte[29] = DTOB_LH(file_size);
	dte[30] = DTOB_HL(file_size);
	dte[31] = DTOB_HH(file_size);

	/* Update directory table with new directory table entry */
	/* index of data at beginning of new dte */
	i = i % 512;
	for (j = 0; j < 32; i++, j++) {
		data[i] = dte[j];
	}
	
	/* We can only write blocks of nbytesinsect bytes, so make sure the offset
	   we're writing to is at the beginning of a sector */
	write_block(data,
				dir_entry_offset - (dir_entry_offset % info->nbytesinsect), 512);
	
	return FAT_SUCCESS;
}

/*
 * Find the boot sector, read it (store in data buffer), and verify its
 * validity
 */
uint8_t valid_boot_sector(uint8_t *data, struct fatstruct *boot) {
	/* Find boot sector */
	boot->nhidsects = 0;
	boot->bootoffset = 0;
	/* Read first sector */
	{
		uint8_t err = read_block(data, 0);
		if (err) {
			return err;
		}
	}
	
	/* Check if the first sector is the boot sector */
	if (data[0x00] == 0x00) {
		/* First sector is not boot sector, find location of boot sector */
		/* number of hidden sectors: 4 bytes at offset 0x1C6 */
		boot->nhidsects = BTOD(data[0x1C6], data[0x1C7], data[0x1C8], data[0x1C9]);
		/* Location of boot sector */
		boot->bootoffset = boot->nhidsects * 512;
		/* Read boot sector and store in data buffer */
		uint8_t err = read_block(data, boot->bootoffset);
		if (err) {
			return err;
		}
	}
	
	/* Verify validity of boot sector */
	if (BTOW(data[0x1FE], data[0x1FF]) != 0xAA55) {
		return FAT_BAD_BOOT_SECT;
	}

	return FAT_SUCCESS;
}

/*
 * Parse the FAT16 boot sector
 */
uint8_t parse_boot_sector(uint8_t *data, struct fatstruct *info) {
	/* Is the SD card formatted to FAT16? */
	if ( !(data[0x36] == 'F' &&
			data[0x37] == 'A' &&
			data[0x38] == 'T' &&
			data[0x39] == '1' &&
			data[0x3A] == '6') ) {
		return FAT_BAD_BOOT_SECT;
	}

	/* Fill valuable global variables */
	/* bytes per sector:			2 bytes	at offset 0x0B */
	info->nbytesinsect = BTOW(data[0x0B], data[0x0C]);
	/* sectors per cluster:			1 byte	at offset 0x0D */
	info->nsectsinclust = data[0x0D];
	info->nbytesinclust = info->nbytesinsect * info->nsectsinclust;
	/* number of reserved sectors:	2 bytes	at offset 0x0E */
	info->nressects = BTOW(data[0x0E], data[0x0F]);
	/* number of FATs:				1 byte	at offset 0x10 */
	info->nfats = data[0x10];
	/* max directory entries:		2 bytes	at offset 0x11 */
	info->dtsize = BTOW(data[0x11], data[0x12]) * 32;
	/* number of sectors per FAT:	2 bytes	at offset 0x16 */
	info->nsectsinfat = BTOW(data[0x16], data[0x17]);
	/* total sectors:				4 bytes	at offset 0x20 */
	info->nsects = BTOD(data[20], data[21], data[22], data[23]);
	
	/* Only compatible with sectors of 512 bytes */
	if (info->nbytesinsect != BLKSIZE) {
		return FAT_BAD_SECT_SIZE;
	}
	
	/* Get location of FAT */
	info->fatsize = (uint32_t)info->nbytesinsect * (uint32_t)info->nsectsinfat;
	info->fatoffset = (uint32_t)info->nressects * (uint32_t)info->nbytesinsect +
		info->bootoffset;
	
	/* Get location of directory table */
	info->dtoffset = (uint32_t)info->nbytesinsect *
		(uint32_t)info->nressects +
		BLKSIZE * (uint32_t)info->nsectsinfat *
		(uint32_t)info->nfats +
		info->bootoffset;
	
	/* Get location of first cluster to be used by file data */
	info->fileclustoffset = info->dtoffset + info->dtsize;

	return FAT_SUCCESS;
}

/*
 * Delete a file
 *
 * dten: directory table number within current block of data (0 <= dten < 16)
 * curoffset: absolute offset of current block of data
 * data: buffer containing block of data from directory table
 * info: parsed FAT information
 *
 * Free cluster chain in FAT.
 * In directory table, mark file as deleted (0xE5).
 */
void delete_file(	uint8_t dten, uint32_t curoffset, uint8_t *data, struct fatstruct *info) {

	/* Get offset of directory table entry (relative to directory table) */
	uint16_t dte_offset = dten * 32;

	/* Get starting cluster */
	uint16_t cluster = BTOW(data[dte_offset+26], data[dte_offset+27]);

	/* Free cluster chain in FAT */
	while (cluster != 0xFFFF) {
		/* Read appropriate block of FAT */
		uint32_t block_offset = info->fatoffset;
		if (cluster >= 256) {
			block_offset += (cluster * 2) % 512;
		}
		read_block(data, block_offset);

		uint16_t i = (cluster % 256) * 2;	/* Index of cluster */
		cluster = BTOW(data[i], data[i+1]);	/* Get next cluster in chain */
		data[i] = 0x00;	/* Free cluster */
		data[i+1] = 0x00;
		write_block(data, block_offset, 512);
	}

	read_block(data, curoffset);
	data[dte_offset+0] = 0xE5;	/* Mark directory table entry as deleted */
	write_block(data, curoffset, 512);
}

/*
 * Format the SD card to FAT16 (quick format)
 *
 * Clear all sectors up to directory table.
 * Initialize boot sector.
 * Initialize FAT(s).
 * Clear directory table, preserving CONFIG.INI file if found.
 *
 * Format info:
 * bytes per sector:			512
 * sectors per cluster:			64
 * number of reserved sectors:	2
 * number of FATs:				2
 * max directory entries:		512
 * number of sectors per FAT:	235
 * total sectors:				3842048
 */
void format_sd(uint8_t *data, struct fatstruct *info, void (*pre_format)(), void (*during_format)(), void (*post_format)()) {
	/* Clear block 0 up to directory table */
	for (uint32_t j = 0; j < info->dtoffset; j += 512) {
		write_block(data, j, 0);
		/* Indicate that format is happening */
		if (j % 2048 == 0) {
			during_format();
		}
	}
	
	/* Indicate that format is about to start */
	pre_format();

	/* Initialize bytes for boot sector */
	/* (temporary variable for clean initialization) */
	uint8_t tmp[] = {
			0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, \
			0x30, 0x00, 0x02, 0x40, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, \
			0x00, 0xF8, 0xEB, 0x00, 0x3F, 0x00, 0xFF, 0x00, 0x00, 0x00, \
			0x00, 0x00, 0x00, 0xA0, 0x3A, 0x00, 0x80, 0x00, 0x29, 0xFF, \
			0xFF, 0xFF, 0xFF, 0x5A, 0x41, 0x50, 0x50, 0x20, 0x20, 0x20, \
			0x20, 0x20, 0x20, 0x20, 0x46, 0x41, 0x54, 0x31, 0x36, 0x20, \
			0x20, 0x20, 0x33, 0xC9, 0x8E, 0xD1, 0xBC, 0xF0, 0x7B, 0x8E, \
			0xD9, 0xB8, 0x00, 0x20, 0x8E, 0xC0, 0xFC, 0xBD, 0x00, 0x7C, \
			0x38, 0x4E, 0x24, 0x7D, 0x24, 0x8B, 0xC1, 0x99, 0xE8, 0x3C, \
			0x01, 0x72, 0x1C, 0x83, 0xEB, 0x3A, 0x66, 0xA1, 0x1C, 0x7C, \

			0x26, 0x66, 0x3B, 0x07, 0x26, 0x8A, 0x57, 0xFC, 0x75, 0x06, \
			0x80, 0xCA, 0x02, 0x88, 0x56, 0x02, 0x80, 0xC3, 0x10, 0x73, \
			0xEB, 0x33, 0xC9, 0x8A, 0x46, 0x10, 0x98, 0xF7, 0x66, 0x16, \
			0x03, 0x46, 0x1C, 0x13, 0x56, 0x1E, 0x03, 0x46, 0x0E, 0x13, \
			0xD1, 0x8B, 0x76, 0x11, 0x60, 0x89, 0x46, 0xFC, 0x89, 0x56, \
			0xFE, 0xB8, 0x20, 0x00, 0xF7, 0xE6, 0x8B, 0x5E, 0x0B, 0x03, \
			0xC3, 0x48, 0xF7, 0xF3, 0x01, 0x46, 0xFC, 0x11, 0x4E, 0xFE, \
			0x61, 0xBF, 0x00, 0x00, 0xE8, 0xE6, 0x00, 0x72, 0x39, 0x26, \
			0x38, 0x2D, 0x74, 0x17, 0x60, 0xB1, 0x0B, 0xBE, 0xA1, 0x7D, \
			0xF3, 0xA6, 0x61, 0x74, 0x32, 0x4E, 0x74, 0x09, 0x83, 0xC7, \

			0x20, 0x3B, 0xFB, 0x72, 0xE6, 0xEB, 0xDC, 0xA0, 0xFB, 0x7D, \
			0xB4, 0x7D, 0x8B, 0xF0, 0xAC, 0x98, 0x40, 0x74, 0x0C, 0x48, \
			0x74, 0x13, 0xB4, 0x0E, 0xBB, 0x07, 0x00, 0xCD, 0x10, 0xEB, \
			0xEF, 0xA0, 0xFD, 0x7D, 0xEB, 0xE6, 0xA0, 0xFC, 0x7D, 0xEB, \
			0xE1, 0xCD, 0x16, 0xCD, 0x19, 0x26, 0x8B, 0x55, 0x1A, 0x52, \
			0xB0, 0x01, 0xBB, 0x00, 0x00, 0xE8, 0x3B, 0x00, 0x72, 0xE8, \
			0x5B, 0x8A, 0x56, 0x24, 0xBE, 0x0B, 0x7C, 0x8B, 0xFC, 0xC7, \
			0x46, 0xF0, 0x3D, 0x7D, 0xC7, 0x46, 0xF4, 0x29, 0x7D, 0x8C, \
			0xD9, 0x89, 0x4E, 0xF2, 0x89, 0x4E, 0xF6, 0xC6, 0x06, 0x96, \
			0x7D, 0xCB, 0xEA, 0x03, 0x00, 0x00, 0x20, 0x0F, 0xB6, 0xC8, \

			0x66, 0x8B, 0x46, 0xF8, 0x66, 0x03, 0x46, 0x1C, 0x66, 0x8B, \
			0xD0, 0x66, 0xC1, 0xEA, 0x10, 0xEB, 0x5E, 0x0F, 0xB6, 0xC8, \
			0x4A, 0x4A, 0x8A, 0x46, 0x0D, 0x32, 0xE4, 0xF7, 0xE2, 0x03, \
			0x46, 0xFC, 0x13, 0x56, 0xFE, 0xEB, 0x4A, 0x52, 0x50, 0x06, \
			0x53, 0x6A, 0x01, 0x6A, 0x10, 0x91, 0x8B, 0x46, 0x18, 0x96, \
			0x92, 0x33, 0xD2, 0xF7, 0xF6, 0x91, 0xF7, 0xF6, 0x42, 0x87, \
			0xCA, 0xF7, 0x76, 0x1A, 0x8A, 0xF2, 0x8A, 0xE8, 0xC0, 0xCC, \
			0x02, 0x0A, 0xCC, 0xB8, 0x01, 0x02, 0x80, 0x7E, 0x02, 0x0E, \
			0x75, 0x04, 0xB4, 0x42, 0x8B, 0xF4, 0x8A, 0x56, 0x24, 0xCD, \
			0x13, 0x61, 0x61, 0x72, 0x0B, 0x40, 0x75, 0x01, 0x42, 0x03, \

			0x5E, 0x0B, 0x49, 0x75, 0x06, 0xF8, 0xC3, 0x41, 0xBB, 0x00, \
			0x00, 0x60, 0x66, 0x6A, 0x00, 0xEB, 0xB0, 0x42, 0x4F, 0x4F, \
			0x54, 0x4D, 0x47, 0x52, 0x20, 0x20, 0x20, 0x20, 0x0D, 0x0A, \
			0x52, 0x65, 0x6D, 0x6F, 0x76, 0x65, 0x20, 0x64, 0x69, 0x73, \
			0x6B, 0x73, 0x20, 0x6F, 0x72, 0x20, 0x6F, 0x74, 0x68, 0x65, \
			0x72, 0x20, 0x6D, 0x65, 0x64, 0x69, 0x61, 0x2E, 0xFF, 0x0D, \
			0x0A, 0x44, 0x69, 0x73, 0x6B, 0x20, 0x65, 0x72, 0x72, 0x6F, \
			0x72, 0xFF, 0x0D, 0x0A, 0x50, 0x72, 0x65, 0x73, 0x73, 0x20, \
			0x61, 0x6E, 0x79, 0x20, 0x6B, 0x65, 0x79, 0x20, 0x74, 0x6F, \
			0x20, 0x72, 0x65, 0x73, 0x74, 0x61, 0x72, 0x74, 0x0D, 0x0A, \

			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAC, 0xCB, 0xD8, \
			0x55, 0xAA};
	for (uint16_t i = 0; i < 512; i++) {
		data[i] = tmp[i];
	}
	/* Write to boot sector */
	write_block(data, 0, 512);

	/* Set initial bytes for FAT  */
	data[0] = 0xF8;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	for (uint16_t i = 4; i < 512; i++) {
		data[i] = 0x00;
	}
	/* Write to first FAT  */
	write_block(data, 0x400, 512);
	/* Write to second FAT  */
	write_block(data, 0x1DA00, 512);

	/* Clear directory table */
	/* While clearing directory table, look for config file to preserve it */
	uint8_t config_found = 0;	/* Flag for config ("CONFIG.INI") file */
	uint32_t dt_end = info->dtoffset + info->dtsize;
	for (uint32_t j = info->dtoffset; j < dt_end; j += 512) {
		if (!config_found) {	/* Look for config file */
			read_block(data, j);
			/* Clear this block */
			write_block(data, j, 0);
			for (uint16_t i = 0; i < 512; i += 32) {
				if (	data[i] == 'C' &&
						data[i+1] == 'O' &&
						data[i+2] == 'N' &&
						data[i+3] == 'F' &&
						data[i+4] == 'I' &&
						data[i+5] == 'G' &&
						data[i+6] == ' ' &&
						data[i+7] == ' ' &&
						data[i+8] == 'I' &&
						data[i+9] == 'N' &&
						data[i+10] == 'I')
				{
					config_found = 1;
					/* Move config file entry to start of directory table */
					for (uint8_t k = 0; k < 32; k++) data[k] = data[i+k];
					write_block(data, info->dtoffset, 32);
					uint16_t config_clust = BTOW(data[26], data[27]);
					update_fat(data, info, config_clust * 2, 0xFFFF);
					break;
				}
			}
		} else {
			/* Clear this block */
			write_block(data, j, 0);
		}
		/* Indicate that format is happening */
		if (j % 2048 == 0) {
			during_format();
		}
	}
	
	/* Indicate that format has completed */
	post_format();
}

uint16_t get_file_num(uint8_t *data, struct fatstruct *info) {
	/* Highest file number suffix */
	uint16_t max = 0;
	/* Directory table byte count */
	uint32_t i = 0;
	/* Directory table entry address */
	uint32_t j = 0;
	
	/* TODO */
	do {
		/* Check for end of sector */
		if (i % info->nbytesinsect == 0) {
			/* Read next sector */
			if (read_block(data, info->dtoffset + i)) {
				return 1;
			}
			j = 0;
		}
		/* Convert 3 byte ASCII file number suffix to integer */
		if (data[j] != DTEDEL) {
			uint16_t first_digit = data[j+4] - 0x30;
			if (first_digit <= 9) {
				uint16_t second_digit = data[j+5] - 0x30;
				if (second_digit <= 9) {
					uint16_t third_digit = data[j+6] - 0x30;
					if (third_digit <= 9) {
						/* Current file number suffix */
						uint16_t num = (first_digit * 100) + (second_digit * 10) + third_digit;
						/* Keep track of highest file number suffix */
						if (num > max) {
							max = num;
						}
					}
				}
			}
			/* Update byte counter */
			i += 32;
			/* Address of next directory table entry */
			j = i % info->nbytesinsect;
		}
	} while (i < info->dtsize && data[j] != 0x00);
	
	/* Return the highest usable file number suffix */
	return max + 1;
}

#endif
