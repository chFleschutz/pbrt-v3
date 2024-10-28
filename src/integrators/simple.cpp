#include "integrators/simple.h"

#include "camera.h"
#include "film.h"
#include "interaction.h"
#include "paramset.h"

namespace pbrt {
SimpleIntegrator::SimpleIntegrator(std::shared_ptr<const Camera> camera, std::shared_ptr<Sampler> sampler, 
    const Bounds2i &pixelBounds)
    : SamplerIntegrator(camera, sampler, pixelBounds) 
{
}

Spectrum SimpleIntegrator::Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler, MemoryArena &arena, 
    int depth) const 
{
    SurfaceInteraction isect;
    if (!scene.Intersect(ray, &isect)) 
        return Spectrum(0.0f);

    Spectrum L(0.0f);
    const Normal3f& n = isect.shading.n;
    Vector3f wo = isect.wo;

    isect.ComputeScatteringFunctions(ray, arena);
    if (!isect.bsdf)
        return Li(isect.SpawnRay(ray.d), scene, sampler, arena, depth);

    L += isect.Le(wo);

    for (const auto& light : scene.lights) 
    {
        Vector3f wi;
        Float pdf;
        VisibilityTester visibility;
        Spectrum Li = light->Sample_Li(isect, sampler.Get2D(), &wi, &pdf, &visibility);

        if (Li.IsBlack() || std::abs(pdf) < 0.01f) 
            continue;

        Spectrum f = isect.bsdf->f(wo, wi);
        if (!f.IsBlack() && visibility.Unoccluded(scene))
            L += f * Li * AbsDot(wi, n) / pdf;
    }

    if (depth < maxDepth) 
    {
        L += SpecularReflect(ray, isect, scene, sampler, arena, depth);
        L += SpecularTransmit(ray, isect, scene, sampler, arena, depth);
    }

    return L;
}

SimpleIntegrator *CreateSimpleIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler, 
    std::shared_ptr<const Camera> camera) 
{
    auto pixelBounds = camera->film->GetSampleBounds();
    return new SimpleIntegrator(camera, sampler, pixelBounds);
}

}  // namespace pbrt
