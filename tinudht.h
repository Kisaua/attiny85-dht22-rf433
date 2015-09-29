/*
 * TinuDHT - Tinusaur DHT11 Library
 *
 * @created: 2014-07-08
 * @author: Neven Boyanov
 *
 * Source code available at: https://bitbucket.org/tinusaur/tinudht
 *
 */

#ifndef TINUDHT_H
#define TINUDHT_H

// ----------------------------------------------------------------------------

#include <stdint.h>

// ----------------------------------------------------------------------------

#define TINUDHT_OK				0
#define TINUDHT_ERROR_CHECKSUM	-1
#define TINUDHT_ERROR_TIMEOUT	-2

// ----------------------------------------------------------------------------

typedef struct {
	uint8_t humidity_int;
	uint8_t humidity_dec;
	uint8_t temperature_int;
	uint8_t temperature_dec;
} TinuDHT;

// ----------------------------------------------------------------------------

uint8_t tinudht_read(TinuDHT *ptinudht, uint8_t dht_pin);

// ----------------------------------------------------------------------------

#endif
