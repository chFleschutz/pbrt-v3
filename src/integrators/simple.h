#pragma once

#include "integrator.h"
#include "pbrt.h"
#include "scene.h"

namespace pbrt {

class SimpleIntegrator : public SamplerIntegrator 
{
public:
    SimpleIntegrator(std::shared_ptr<const Camera> camera,
                     std::shared_ptr<Sampler> sampler,
                     const Bounds2i &pixelBounds);

    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth) const;

private:
	const int maxDepth = 5;
};

SimpleIntegrator *CreateSimpleIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);

}  // namespace pbrt
