#include "globals.h"
#include "fluxmap.h"
#include "decoders/fluxmapreader.h"
#include "decoders/decoders.h"
#include "sector.h"
#include "micropolis.h"
#include "bytes.h"
#include "fmt/format.h"
#include "arch/ibm/ibm.pb.h"
#include "proto.h"

/* The sector has a preamble of MFM 0x00s and uses 0xFF as a sync pattern. */
static const FluxPattern SECTOR_SYNC_PATTERN(32, 0xaaaa5555);

AbstractDecoder::RecordType MicropolisDecoder::advanceToNextRecord()
{
	_fmr->seekToIndexMark();
	const FluxMatcher* matcher = nullptr;
	_sector->clock = _fmr->seekToPattern(SECTOR_SYNC_PATTERN, matcher);
	if (matcher == &SECTOR_SYNC_PATTERN) {
		readRawBits(16);
		return SECTOR_RECORD;
	}
	return UNKNOWN_RECORD;
}

/* Adds all bytes, with carry. */
static uint8_t checksum(const Bytes& bytes) {
	ByteReader br(bytes);
	uint16_t sum = 0;
	while (!br.eof()) {
		if (sum > 0xFF) {
			sum -= 0x100 - 1;
		}
		sum += br.read_8();
	}
	/* The last carry is ignored */
	return sum & 0xFF;
}

/* MZOS disks use a different checksum than the other Vector Graphic
 * operating systems:
 *
 * The MZOS checksum starts with the checksum set to zero, then for each data
 * byte: 1) RLC the data byte, 2) XOR result of #1 into the checksum. Note
 * this is backwards from the NorthStar checksum that it seems VG was trying
 * to duplicate (XOR data into checksum, then RLC the checksum).
 *
 * Reference: https://deramp.com/downloads/vector_graphic/software/disk_images/Micropolis%20controller/MZOS%20on%20Vector%20Graphic.pdf
 *
 * NOTE: WIP: This is not working yet!
 */
 static uint8_t mzos_checksum(const Bytes& bytes) {
	ByteReader br(bytes);
	uint8_t checksum = 0;
	uint8_t databyte;
	while (!br.eof()) {
		databyte = br.read_8();
		checksum ^= ((databyte << 1) | (databyte >> 7));
	}

	return checksum;
}


void MicropolisDecoder::decodeSectorRecord()
{
	auto rawbits = readRawBits(MICROPOLIS_ENCODED_SECTOR_SIZE*16);
	auto bytes = decodeFmMfm(rawbits).slice(0, MICROPOLIS_ENCODED_SECTOR_SIZE);
	ByteReader br(bytes);
	Bytes header;
	bool _ignoreTrackByte = false;

	header = br.read(MICROPOLIS_HEADER_SIZE);

	_sector->logicalSide = _sector->physicalSide;
	_sector->logicalSector = header[MICROPOLIS_SECTOR_OFFSET];

	if (_sector->logicalSector > 15)
		return;
	if (_sector->logicalTrack > 77)
		return;

	if (_sector->physicalTrack != header[MICROPOLIS_TRACK_OFFSET]) {
		if (_config.ignore_track_byte()) {
			_sector->logicalTrack = _sector->physicalTrack;
			std::cout << fmt::format("Warning: Track {}, Sector {}: Overriding track byte in header {}->{}.",
				_sector->physicalTrack,
				_sector->logicalSector,
				header[MICROPOLIS_TRACK_OFFSET],
				_sector->physicalTrack)
				<< std::endl;
		} else {
			_sector->logicalTrack = header[MICROPOLIS_TRACK_OFFSET];
			std::cout << fmt::format("Warning: Track {}, Sector {}: Header reports track {}, consider --micropolis-ignore-track-byte",
				_sector->physicalTrack,
				_sector->logicalSector,
				header[MICROPOLIS_TRACK_OFFSET])
				<< std::endl;
		}
	}
	else {
		_sector->logicalTrack = header[MICROPOLIS_TRACK_OFFSET];
	}

	Bytes userData = br.read(MICROPOLIS_PAYLOAD_SIZE);
	Bytes trailer = br.read(MICROPOLIS_TRAILER_SIZE);
	uint8_t wantChecksum = trailer[MICROPOLIS_CHECKSUM_OFFSET];
	uint8_t gotChecksum = (_config.mzos_checksum() == false) ? checksum(bytes.slice(1, 2+266)) : mzos_checksum(bytes.slice(1, 2+266));

	std::cout << fmt::format("Checksum: want: {}, calculated {}",
		wantChecksum, gotChecksum)
		<< std::endl;

    _sector->data.clear();
    _sector->data.writer().append(userData).append(header).append(trailer);

	_sector->status = (wantChecksum == gotChecksum) ? Sector::OK : Sector::BAD_CHECKSUM;
}
