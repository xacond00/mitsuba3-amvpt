#pragma once
/*---------------------------------------------------------------------------------------------*/
/*Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/
#include "dr_macros.h"
#include <cstdint>
#include <drjit/morton.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/fwd.h>
#include <mitsuba/core/profiler.h>
#include <mitsuba/core/progress.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/ray.h>
#include <mitsuba/core/spectrum.h>
#include <mitsuba/core/timer.h>
#include <mitsuba/core/util.h>
#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/emitter.h>
#include <mitsuba/render/film.h>
#include <mitsuba/render/integrator.h>
#include <mitsuba/render/records.h>
#include <mitsuba/render/sampler.h>
#include <mitsuba/render/sensor.h>
#include <mitsuba/render/spiral.h>
#include <nanothread/nanothread.h>
NAMESPACE_BEGIN(mitsuba)

/**!

.. _integrator-path:

Multi-view Path tracer (:monosp:`mvpath`)
----------------------------

.. pluginparameters::

 * - max_depth
   - |int|
   - Specifies the longest path depth in the generated output image (where -1
     corresponds to :math:`\infty`). A value of 1 will only render directly
     visible light sources. 2 will lead to single-bounce (direct-only)
     illumination, and so on. (Default: -1)

 * - rr_depth
   - |int|
   - Specifies the path depth, at which the implementation will begin to use
     the *russian roulette* path termination criterion. For example, if set to
     1, then path generation many randomly cease after encountering directly
     visible surfaces. (Default: 5)

 * - spp_pass_lim
   - |int|
   - Limits the number of SPP per pass to reduce RAM usage (Default: 16)

 * - hide_emitters
   - |bool|
   - Hide directly visible emitters. (Default: no, i.e. |false|)

 * - sa_reuse
   - |bool|
   - Reuse samples between sensors. (Default: no, i.e. |false|)

 * - sa_mis
   - |bool|
   - Use mis for shared samples. (Default: no, i.e. |false|)

This integrator (re-)implements a basic multi-view path tracer with sample
reuse and optional MIS. Aiming to achieve great performance and unbiased
results in multi-camera rendering situations.

This integrator iterates over all sensors, and on each iteration, it reprojects
the samples onto all other sensors. Thus the sample count is ideally Ns
times larger, depending on sensor placement and materials, with minimal runtime
cost.

The reused camera samples are optionally weighted with MIS, which aims to
eliminate bias and other image artifacts.

Reusing parts of Mitsuba 3 stock source code (PathIntegrator,SamplingIntegrator).

Based on: https://bfraboni.github.io/mvpt19/index.html.
Author: Ondrej Ac (xacond00); VUT FIT, Czechia; @2024-2025.

Note: Implementation split into multiple header files, as only single source file
can be associated with a plugin. Results in few ugly hacks to hide compiler warnings.

.. image::
../../resources/data/docs/images/integrator/integrator_path_figure.png :width:
95% :align: center


.. note:: This integrator does not handle participating media
.. note:: This integrator only works with MultiSensor instance
.. note:: This integrator is not set up to work with autodiff
.. note:: This integrator currently supports only cuda and llvm variants

.. tabs::
    .. code-tab::  xml
        :name: path-integrator

        <integrator type="path">
            <integer name="max_depth" value="8"/>
        </integrator>

    .. code-tab:: python

        'type': 'path',
        'max_depth': 8

 */

#define MVPT MVPathIntegrator<Float, Spectrum>

template <typename Float, typename Spectrum>
class MVPathIntegrator final : public MonteCarloIntegrator<Float, Spectrum> {
public:
    MI_IMPORT_BASE(MonteCarloIntegrator, m_max_depth, m_rr_depth, m_stop, m_hide_emitters, m_samples_per_pass,
                   aov_names, m_render_timer, m_timeout, m_block_size, should_stop)
    MI_IMPORT_TYPES(Scene, Sensor, Film, BSDF, BSDFPtr, ImageBlock, Sampler, Medium, Emitter, EmitterPtr, SensorPtr)

    MVPathIntegrator(const Properties &props) : Base(props) {
        m_thin_lens    = false;
        m_sa_reuse     = props.get<bool>("sa_reuse", false);
        m_force_eval   = props.get<bool>("force_eval", true);
        m_sa_mis       = props.get<bool>("sa_mis", false);
        m_debug        = props.get<bool>("debug", false);
        m_fast_mis     = props.get<bool>("fast_mis", false);
        m_spp_pass_lim = props.get<uint32_t>("spp_pass_lim", 16);
    }

    // Prefix data needed to compute suffix
    struct PrefixData {
        Ray3f ray;               // First hit outgoing ray
        SurfaceInteraction3f si; // First hit intersection
        Spectrum throughput;     // First hit throughput
        Float bsdf_pdf;          // First hit bsdf_pdf
        Float eta;               // Refraction index
        UInt32 depth;            // Ray depth
        Bool bsdf_delta;         // Previous BSDF was delta
        Bool active;             // Ray is actively traced
        DRJIT_STRUCT(PrefixData, ray, si, throughput, bsdf_pdf, eta, depth, bsdf_delta, active);
    };

    struct SampleData {
        Spectrum result; // First hit radiance
        // Temporary bsdf reflectance (for direct and indirect radiance)
        Spectrum bsdf_val;
        Vector3f wi;   // Incoming surface direction
        Vector3f wo_r; // Reflected surface direction
        Vector2f pos;  // Raster position
        Float weight;  // Final weight
        Float pdfM;    // Precomputed material pdf
        Float pdf;     // Pdf of this view
        Float pdf_lk;  // Pdf from primary to this view
        Float Jp, iJp;      // Partial Jacobian term and its inverse
        UInt32 idx;    // Sensor index
        Mask indirect; // Sample indirect lighting ?
        Mask valid;    // Is visible, has same material ?
        DRJIT_STRUCT(SampleData, result, bsdf_val, wi, wo_r, pos, weight, pdfM, pdf, pdf_lk, Jp, iJp, idx, indirect, valid);
    };

    struct BSDFData {
        BSDFPtr bsdf;
        Float alpha, sqr_a, rsqrt_a;
        Bool diffuse, reuse;
        DRJIT_STRUCT(BSDFData, bsdf, alpha, sqr_a, rsqrt_a, diffuse, reuse);
    };
    // Evaluate sample with reuse
    // 1) Surface emission
    // 2) Primary hit info
    //     a) Emitter sample and direction
    //     b) Material properties - check if delta
    // 3) Sensor selection and MIS
    //     a) Select similar cameras
    //     b) Compute MIS weights
    // 4) Evaluation of direct lighting with MIS
    // 5) Evaluation of indirect lighting with MIS
    // 6) Save the radiance to each sample
    Mask sample_multi(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor, Sampler *sampler,
                      SampleData *samples, const Ray3f &p_ray, const Point2f &p_app, Mask active) const;

    TensorXf render(Scene *scene, Sensor *init_sensor, UInt32 seed, uint32_t init_spp, bool develop,
                    bool evaluate) override;

    // Renders scene from primary sensor and reuses samples in other sensors
    void render_multisample(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor, Sampler *sampler,
                            ImageBlock *block, const Vector2f &pos, SampleData *sampleData, Mask active = true) const;
    //! @}
    // =============================================================
    void render_block_(const Scene *scene, const Sensor *sensor, Sampler *sampler, ImageBlock *block,
                       uint32_t sample_count, UInt32 seed, uint32_t block_id, uint32_t block_size) const;
    // Stock implementation
    void render_sample(const Scene *scene, const Sensor *sensor, Sampler *sampler, ImageBlock *block,
                       const Vector2f &pos, Mask active = true) const;
    // Classic path tracing algorithm (except stripped of not needed parameters)
    std::pair<Spectrum, Bool> sample_single(const Scene *scene, Sampler *sampler, const RayDifferential3f &ray_,
                                            Bool active) const;
    // Given a primary intersection data (prefix), sample suffix
    // Returns spectrum and mask
    // Nearly identical to sample_single
    Spectrum sample_suffix(const Scene *scene, Sampler *sampler, const PrefixData &pd) const;

    std::string to_string() const override {
        return tfm::format("MVPathIntegrator[\n"
                           "  max_depth = %u,\n"
                           "  rr_depth = %u\n"
                           "  sa_reuse = %u\n"
                           "  spp_pass_lim = %u\n"
                           "]",
                           m_max_depth, m_rr_depth, m_sa_reuse, m_spp_pass_lim);
    }

    template <bool primary>
    auto sensors_visible(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor, const Point2f &ap_sample, 
                         const SurfaceInteraction3f &si, Bool prim_face, const UInt32 &idx, const Mask &active) const {
        auto [sensor_ds, Jp, face, valid] = sensor->sample_surface(si, ap_sample, idx, active);

        // If camera isn't primary, check geometric visibility too...
        if (!primary && dr::any_or<true>(valid)) {
            valid &= (face == prim_face) && Jp > 0.f;
            Ray3f sensor_ray = si.spawn_ray_to(sensor_ds.p);
            valid &= !scene->ray_test(sensor_ray);
        }
        return std::tuple(sensor_ds, Jp, valid);
    }
    // Fast TV distance approximation based on plugging GGX formula into TV distance calculation
    // Very accurate if the BSDF has GGX distribution, less so for Beckmann, but not too terrible...
    static Float tv_pdf_fast(const Vector3f &wo_l, const Vector3f &wi_k, const Float &p_k, const BSDFData &bsdf,
                             const Mask &active) {
        auto p_l = dr::square(Frame3f::cos_theta(dr::normalize(wi_k + wo_l)));
        auto N   = dr::fmadd(bsdf.sqr_a, dr::maximum(p_k, p_l), 1.f);
        auto D   = dr::fmadd(bsdf.sqr_a, dr::minimum(p_k, p_l), 1.f);
        Float q  = dr::square(N * dr::rcp(D));
        // Rational approximation to pow(p, 1/a)
        Float p = dr::fmadd(q - 1.f, bsdf.rsqrt_a, 1.f);
        p       = dr::square(dr::maximum(p, 0));
        p       = dr::lerp(p, q, bsdf.alpha);
        return dr::select(active, p, 0.f);
    }
    // Compute total variation pdf of brdfs from L to K
    // Wo_l is reflection direction of primary camera (local)
    // Wi_k is incoming direction from secondary camera (local) in si_k
    // Pi_k is just precomputed pdf(wi_k, wo_k)
    // Alpha is material roughness
    static Float tv_pdf(const Vector3f &wo_l, const SurfaceInteraction3f &si_k, const Float &p_k, const BSDFData &bsdf,
                        Mask active) {
        active &= p_k > 0.f;
        if (dr::none_or<false>(active))
            return 0.f;
        BSDFContext ctx(TransportMode::Radiance, (uint32_t) BSDFFlags::Glossy);
        Float p_l = bsdf.bsdf->pdf(ctx, si_k, wo_l, active);
        active &= p_l > 0.f;
        Float p_max = dr::maximum(p_l, p_k);
        Float p_min = dr::minimum(p_l, p_k);
        // Intial pdf: q = 1 - TVD
        Float q = p_min * dr::rcp(p_max);
        // Piecewise polynomial approximation of p ^ (1 / a)
        Float p = dr::fmadd(q - 1.f, bsdf.rsqrt_a, 1.f);
        p       = dr::square(dr::maximum(p, 0));
        p       = dr::lerp(p, q, bsdf.alpha);
        return dr::select(active, p, 0.f);
    }

    /// Compute a multiple importance sampling weight using the power heuristic
    static Float mis_weight(Float pdf_a, Float pdf_b) {
        pdf_a *= pdf_a;
        pdf_b *= pdf_b;
        Float w = pdf_a / (pdf_a + pdf_b);
        return dr::detach<true>(dr::select(dr::isfinite(w), w, 0.f));
    }

    /**
     * \brief Perform a Mueller matrix multiplication in polarized modes, and a
     * fused multiply-add otherwise.
     */
    static Spectrum spec_fma(const Spectrum &a, const Spectrum &b, const Spectrum &c) {
        if constexpr (is_polarized_v<Spectrum>)
            return a * b + c;
        else
            return dr::fmadd(a, b, c);
    }
    /**
     * \brief Compute MIS weights between multiple cameras
     */
    void mis_weights(SampleData *samples, uint n_sensors, SurfaceInteraction3f &si_k, const BSDFData &bsdf_data) const;
    
    MI_DECLARE_CLASS()
    uint32_t m_spp_pass_lim;
    bool m_sa_reuse;
    bool m_force_eval;
    bool m_sa_mis;
    bool m_fast_mis;
    bool m_thin_lens;
    bool m_debug;
};

NAMESPACE_END(mitsuba)
