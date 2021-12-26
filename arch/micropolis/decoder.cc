#include "globals.h"
#include "fluxmap.h"
#include "decoders/fluxmapreader.h"
#include "decoders/decoders.h"
#include "sector.h"
#include "micropolis.h"
#include "bytes.h"
#include "fmt/format.h"
#include "lib/decoders/decoders.pb.h"

/* The sector has a preamble of MFM 0x00s and uses 0xFF as a sync pattern.
 *
 * 00        00        00        F         F
 * 0000 0000 0000 0000 0000 0000 0101 0101 0101 0101
 * A    A    A    A    A    A    5    5    5    5
 */
static const FluxPattern SECTOR_SYNC_PATTERN(64, 0xAAAAAAAAAAAA5555LL);

/* Standard Micropolis checksum.  Adds all bytes, with carry. */
uint8_t micropolisChecksum(const Bytes& bytes) {
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

/* Vector MZOS does not use the standard Micropolis checksum.
 * The checksum is initially 0.
 * For each data byte in the 256-byte payload, rotate left,
 * carrying bit 7 to bit 0.  XOR with the current checksum.
 *
 * Unlike the Micropolis checksum, this does not cover the 12-byte
 * header (track, sector, 10 OS-specific bytes.)
 */
uint8_t mzosChecksum(const Bytes& bytes) {
	ByteReader br(bytes);
	uint8_t checksum = 0;
	uint8_t databyte;

	while (!br.eof()) {
		databyte = br.read_8();
		checksum ^= ((databyte << 1) | (databyte >> 7));
	}

	return checksum;
}

class MicropolisDecoder : public AbstractDecoder
{
public:
	MicropolisDecoder(const DecoderProto& config):
		AbstractDecoder(config),
		_config(config.micropolis())
	{
		_checksumType = _config.checksum_type();
	}

	void beginTrack()
	{
		_hardSectorId = -2;

		/* Find index mark and calculate initial sector id. */
		auto initialPos = _fmr->tell();
		_fmr->seekToIndexMark();
		auto prevSectorNs = _fmr->tell().ns();
		_fmr->seekToIndexMark();
		int history = 0; /* 0 bit for long period, 1 bit for short period. */
		for (int periods = 1; !_fmr->eof(); periods++)
		{
			auto thisSectorNs = _fmr->tell().ns();
			history <<= 1;
			/* 12.5 ms duration between sectors and 6.25 ms duration within
			 * last sector. Use 9 ms as separation between long and short
			 * durations. */
			if (thisSectorNs - prevSectorNs < 9e6)
				history |= 1;
			if (history == 0b01 && periods > 1)
			{
				_hardSectorId = MICROPOLIS_SECTORS_PER_TRACK - periods - 1;
				if (_hardSectorId == -1)
					_hardSectorId = MICROPOLIS_SECTORS_PER_TRACK - 1;
				else if (_hardSectorId < -1)
					_hardSectorId = -2;
				break;
			}
			else if (history == 0b10)
			{
				_hardSectorId = -1;
				break;
			}
			else if (history == 0b11)
			{
				_hardSectorId = MICROPOLIS_SECTORS_PER_TRACK - 2;
				break;
			}
			prevSectorNs = thisSectorNs;
			_fmr->seekToIndexMark();
		}
		_fmr->seek(initialPos);
	}

	RecordType advanceToNextRecord()
	{
		_fmr->seekToIndexMark();
		if (_hardSectorId == -1)
			_fmr->seekToIndexMark();

		auto sectorPos = _fmr->tell();
		auto resetPos = sectorPos;
		_fmr->seekToIndexMark();
		auto nextIndexMark = _fmr->tell();
		_fmr->seek(sectorPos);
		if (nextIndexMark.ns() - sectorPos.ns() < 9e6)
		{
			_hardSectorId = MICROPOLIS_SECTORS_PER_TRACK - 1;
			resetPos = nextIndexMark;
		}
		else
		{
			if (_hardSectorId == -2)
				return UNKNOWN_RECORD;
			_hardSectorId++;
			_hardSectorId %= MICROPOLIS_SECTORS_PER_TRACK;
		}

		_fmr->seek(_fmr->tell().ns() + 500 * 1000); // seek 500uS past the sector pulse.
		const FluxMatcher* matcher = nullptr;
		_sector->clock = _fmr->seekToPattern(SECTOR_SYNC_PATTERN, matcher);

		/* Preamble is expected to be 1.280 ms. If a "sync" is found after
		 * 3.7 ms, then the "data" would overlap the next sector pulse causing
		 * that next sector to be skipped. Use 3 ms as the limit. */
		if (_fmr->tell().ns() > sectorPos.ns() + 3e6)
		{
			/* Reset to known position to stay in sync with _hardSectorId. */
			_fmr->seek(resetPos);
			return UNKNOWN_RECORD;
		}

		if (matcher == &SECTOR_SYNC_PATTERN)
		{
			return SECTOR_RECORD;
		}
		return UNKNOWN_RECORD;
	}

	void decodeSectorRecord()
	{
		readRawBits(48);
		auto rawbits = readRawBits(MICROPOLIS_ENCODED_SECTOR_SIZE*16);
		auto bytes = decodeFmMfm(rawbits).slice(0, MICROPOLIS_ENCODED_SECTOR_SIZE);
		ByteReader br(bytes);

		br.read_8();  /* sync */
		auto track = br.read_8();
		auto sector = br.read_8();
		if (sector > 15)
			return;
		if (track > 77)
			return;
		if (sector != _hardSectorId)
			return;

		br.read(10);  /* OS data or padding */
		auto data = br.read(256);
		uint8_t wantChecksum = br.read_8();

		/* If not specified, automatically determine the checksum type.
		 * Once the checksum type is determined, it will be used for the
		 * entire disk.
		 */
		if (_checksumType == 0) {
			/* Calculate both standard Micropolis (MDOS, CP/M, OASIS) and MZOS checksums */
			if (wantChecksum == micropolisChecksum(bytes.slice(1, 2+266))) {
				_checksumType = 1;
			} else if (wantChecksum == mzosChecksum(bytes.slice(13, 256))) {
				_checksumType = 2;
				std::cout << "Note: MZOS checksum detected." << std::endl;
			}
		}

		uint8_t gotChecksum;

		if (_checksumType == 2) {
			gotChecksum = mzosChecksum(bytes.slice(13, 256));
		} else {
			gotChecksum = micropolisChecksum(bytes.slice(1, 2+266));
		}

		br.read(5);  /* 4 byte ECC and ECC-present flag */

		if (_config.sector_output_size() == 256)
		{
			_sector->logicalTrack = track;
			_sector->logicalSide = _sector->physicalHead;
			_sector->logicalSector = sector;
			_sector->data = data;
		}
		else if (_config.sector_output_size() == MICROPOLIS_ENCODED_SECTOR_SIZE)
		{
			/* When saving the full sector, we can use the physical location as
			 * the consumer can still determine the logical location. This has
			 * the benefit of preserving the interleave and skew. */
			_sector->logicalTrack = _sector->physicalCylinder;
			_sector->logicalSide = _sector->physicalHead;
			_sector->logicalSector = _hardSectorId;
			_sector->data = bytes;
		}
		else
			Error() << "Sector output size may only be 256 or 275";
		_sector->status = (wantChecksum == gotChecksum) ? Sector::OK : Sector::BAD_CHECKSUM;
	}

	std::set<unsigned> requiredSectors(unsigned cylinder, unsigned head) const override
	{
		static std::set<unsigned> sectors = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
		return sectors;
	}

private:
	const MicropolisDecoderProto& _config;
	/* -2 for unsynced. -1 for synced, but in middle of first half of last
	 * sector. */
	int _hardSectorId;
	int _checksumType;	/* -1 = auto, 1 = Micropolis, 2=MZOS */
};

std::unique_ptr<AbstractDecoder> createMicropolisDecoder(const DecoderProto& config)
{
	return std::unique_ptr<AbstractDecoder>(new MicropolisDecoder(config));
}
