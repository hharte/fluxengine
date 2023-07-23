#include "globals.h"
#include "flags.h"
#include "fluxsource/fluxsource.h"
#include "fluxmap.h"
#include "lib/config.pb.h"
#include "proto.h"
#include "utils.h"

std::unique_ptr<FluxSource> FluxSource::create(const FluxSourceProto& config)
{
    switch (config.type())
    {
        case FluxSourceSinkType::DRIVE:
            return createHardwareFluxSource(config.drive());

        case FluxSourceSinkType::ERASE:
            return createEraseFluxSource(config.erase());

        case FluxSourceSinkType::KRYOFLUX:
            return createKryofluxFluxSource(config.kryoflux());

        case FluxSourceSinkType::TEST_PATTERN:
            return createTestPatternFluxSource(config.test_pattern());

        case FluxSourceSinkType::SCP:
            return createScpFluxSource(config.scp());

        case FluxSourceSinkType::A2R:
            return createA2rFluxSource(config.a2r());

        case FluxSourceSinkType::CWF:
            return createCwfFluxSource(config.cwf());

        case FluxSourceSinkType::FLUX:
            return createFl2FluxSource(config.fl2());

        case FluxSourceSinkType::FLX:
            return createFlxFluxSource(config.flx());

        default:
            return std::unique_ptr<FluxSource>();
    }
}

class TrivialFluxSourceIterator : public FluxSourceIterator
{
public:
    TrivialFluxSourceIterator(
        TrivialFluxSource* fluxSource, int track, int head):
        _fluxSource(fluxSource),
        _track(track),
        _head(head)
    {
    }

    bool hasNext() const override
    {
        return !!_fluxSource;
    }

    std::unique_ptr<const Fluxmap> next() override
    {
        auto fluxmap = _fluxSource->readSingleFlux(_track, _head);
        _fluxSource = nullptr;
        return fluxmap;
    }

private:
    TrivialFluxSource* _fluxSource;
    int _track;
    int _head;
};

std::unique_ptr<FluxSourceIterator> TrivialFluxSource::readFlux(
    int track, int head)
{
    return std::make_unique<TrivialFluxSourceIterator>(this, track, head);
}
