#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * sd_store.h — SD card mounting and space management
 *
 * Owns the SPI bus and the FAT mount at /sdcard. All calls are made from
 * can_logger_task (Core 0); nothing here is thread-safe or ISR-safe.
 */

// Mount the card. Returns false on failure; caller is expected to retry.
bool sd_store_mount(void);

// Free space on the mounted volume, in bytes. Returns 0 if unavailable.
uint64_t sd_store_free_bytes(void);

// Total volume size in bytes. Returns 0 if unavailable.
uint64_t sd_store_total_bytes(void);

// Delete the oldest (lowest-numbered) canlog_*.csv files until at least
// want_free bytes are available. keep_id is the log currently being written
// and is never deleted. Returns the number of files removed.
int sd_store_reclaim(uint64_t want_free, uint32_t keep_id);
