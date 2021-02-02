#include "globals.h"
#include "flags.h"
#include "sector.h"
#include "sectorset.h"
#include "imagewriter/imagewriter.h"
#include "fmt/format.h"
#include "decoders/decoders.h"
#include "arch/micropolis/micropolis.h"
#include "lib/imagewriter/imagewriter.pb.h"
#include <algorithm>
#include <iostream>
#include <fstream>

class VgiImageWriter : public ImageWriter
{
public:
	VgiImageWriter(const ImageWriterProto& config):
		ImageWriter(config)
	{}

	void writeImage(const SectorSet& sectors)
	{
		unsigned autoTracks;
		unsigned autoSides;
		unsigned autoSectors;
		unsigned autoBytes;
		sectors.calculateSize(autoTracks, autoSides, autoSectors, autoBytes);

		size_t trackSize = autoSectors * autoBytes;

		std::cout << fmt::format("Writing {} cylinders, {} heads, {} sectors, {} ({} bytes/sector), {} kB total",
				autoTracks, autoSides,
				autoSectors, autoBytes == 256 ? "SD" : "DD", autoBytes,
				autoTracks * trackSize / 1024)
				<< std::endl;

		std::ofstream outputFile(_config.filename(), std::ios::out | std::ios::binary);
		if (!outputFile.is_open())
			Error() << "cannot open output file";

		for (int track = 0; track < autoTracks * autoSides; track++)
		{
			int head = (track < autoTracks) ? 0 : 1;
			for (int sectorId = 0; sectorId < autoSectors; sectorId++)
			{
				const auto& sector = sectors.get(track % autoTracks, head, sectorId);
/* TODO: hharte: this is not correct, need to unpack sector metadata. */
				if (sector)
				{
					outputFile.seekp(track * trackSize + sectorId * autoBytes, std::ios::beg);
					sector->data.slice(MICROPOLIS_HEADER_OFFSET, MICROPOLIS_HEADER_SIZE).writeTo(outputFile);
					sector->data.slice(0, MICROPOLIS_PAYLOAD_SIZE).writeTo(outputFile);
					sector->data.slice(MICROPOLIS_TRAILER_OFFSET, MICROPOLIS_TRAILER_SIZE).writeTo(outputFile);
				}
			}
		}
	}
};

std::unique_ptr<ImageWriter> ImageWriter::createVgiImageWriter(
	const ImageWriterProto& config)
{
    return std::unique_ptr<ImageWriter>(new VgiImageWriter(config));
}
