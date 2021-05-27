#include "globals.h"
#include "flags.h"
#include "fluxmap.h"
#include "usb/usb.h"
#include "fluxsink/fluxsink.h"
#include "lib/fluxsink/fluxsink.pb.h"
#include "fmt/format.h"

class HardwareFluxSink : public FluxSink
{
public:
    HardwareFluxSink(const HardwareFluxSinkProto& config):
        _config(config)
    {
		if (config.has_hard_sector_count())
		{
			int rotationalSpeedMs;
			int retries = 5;
            usbSetDrive(_config.drive(), _config.high_density(), _config.index_mode(),
                _config.step_interval_ms(), _config.step_settling_ms());
			std::cout << "Measuring rotational speed... " << std::flush;

			do {
				nanoseconds_t oneRevolution = usbGetRotationalPeriod(_config.hard_sector_count());
				_hardSectorThreshold = oneRevolution * 3 / (4 * _config.hard_sector_count());
				rotationalSpeedMs = oneRevolution / 1e6;
				retries--;
			} while ((rotationalSpeedMs == 0) && (retries > 0));

			if (rotationalSpeedMs == 0) {
				Error() << "Failed\nIs a disk in the drive?";
			}
			std::cout << fmt::format("{}ms\n", rotationalSpeedMs);
		}
		else
			_hardSectorThreshold = 0;
    }

    ~HardwareFluxSink()
    {
    }

public:
    void writeFlux(int track, int side, Fluxmap& fluxmap)
    {
		usbSetDrive(_config.drive(), _config.high_density(), _config.index_mode(),
            _config.step_interval_ms(), _config.step_settling_ms());
		#if 0
		if (fluxSourceSinkFortyTrack)
		{
			if (track & 1)
				Error() << "cannot write to odd physical tracks in 40-track mode";
			usbSeek(track / 2);
		}
		else
		#endif
			usbSeek(track);

        return usbWrite(side, fluxmap.rawBytes(), _hardSectorThreshold);
    }

	operator std::string () const
	{
		return fmt::format("drive {}", _config.drive());
	}

private:
    const HardwareFluxSinkProto& _config;
    nanoseconds_t _hardSectorThreshold;
};

std::unique_ptr<FluxSink> FluxSink::createHardwareFluxSink(const HardwareFluxSinkProto& config)
{
    return std::unique_ptr<FluxSink>(new HardwareFluxSink(config));
}



