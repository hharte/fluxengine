#ifndef MICROPOLIS_H
#define MICROPOLIS_H

#include "decoders/decoders.h"
#include "encoders/encoders.h"
#include "arch/micropolis/micropolis.pb.h"

/* Micropolis floppies are 16-hard sectored disks with a sector format as follows:
 *
 * |-----------------------------------------------------------------------------|
 * | SYNC Byte | Track | Sector | OS Data | Payload | Checksum | ECC | ECC Valid |
 * |-----------+-------+--------+---------+---------+----------+-----+-----------|
 * |     1     |   1   |   1    |   10    |   256   |     1    |  4  |    1      |
 * |-----------------------------------------------------------------------------|
 *
 * SYNC is always 0xFF.
 * OS Data is not used by CP/M, only by Micropolis MDOS.
 * User payload is always 256 bytes.
 * All operating systems, except MZOS use the same checksum.  MZOS uses a different algorithm.
 * ECC is not used by the original Micropolis controller.
 */

#define MICROPOLIS_OS_DATA_SIZE			(10)
#define MICROPOLIS_HEADER_SIZE			(1 + 2 + MICROPOLIS_OS_DATA_SIZE)
#define MICROPOLIS_PAYLOAD_SIZE			(256)
#define MICROPOLIS_TRAILER_SIZE			(6)
#define MICROPOLIS_ENCODED_SECTOR_SIZE	(MICROPOLIS_HEADER_SIZE + MICROPOLIS_PAYLOAD_SIZE + MICROPOLIS_TRAILER_SIZE)

/* Offsets within Micropolis header */
#define MICROPOLIS_SYNC_OFFSET			(0)
#define MICROPOLIS_TRACK_OFFSET			(1)
#define MICROPOLIS_SECTOR_OFFSET		(2)

/* Offsets within Micropolis trailer */
#define MICROPOLIS_CHECKSUM_OFFSET		(0)
#define MICROPOLIS_ECC_OFFSET			(1)
#define MICROPOLIS_ECC_VALID_OFFSET		(5)

/* Sector data is returned as userData, header, trailer. */
#define MICROPOLIS_HEADER_OFFSET		(MICROPOLIS_PAYLOAD_SIZE)
#define MICROPOLIS_TRAILER_OFFSET		(MICROPOLIS_PAYLOAD_SIZE + MICROPOLIS_HEADER_SIZE)

class Sector;
class Fluxmap;
class MicropolisDecoderProto;

class MicropolisDecoder : public AbstractDecoder
{
public:
	MicropolisDecoder(const MicropolisDecoderProto& config):
		_config(config)
	{}

	virtual ~MicropolisDecoder() {}

	RecordType advanceToNextRecord();
	void decodeSectorRecord();

private:
	const MicropolisDecoderProto& _config;
};

#endif
