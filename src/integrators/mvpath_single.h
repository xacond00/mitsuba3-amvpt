#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

// This is a 'hack' to get around the plugin system being limited to single source file per plugin 
// You can safely ignore this file, as it is pretty much 1 to 1 copy of original integrator

#include "mvpath.h"
NAMESPACE_BEGIN(mitsuba)

// Stock implementation
MI_VARIANT void MVPT::render_block_(const Scene *scene, const Sensor *sensor,
                                                                 Sampler *sampler, ImageBlock *block,
                                                                 uint32_t sample_count, UInt32 seed,
                                                                 uint32_t block_id, uint32_t block_size) const {

    if constexpr (!dr::is_array_v<Float>) {
        uint32_t pixel_count = block_size * block_size;

        // Avoid overlaps in RNG seeding RNG when a seed is manually
        // specified
        seed += block_id * pixel_count;

        // Clear block (it's being reused)
        block->clear();

        for (uint32_t i = 0; i < pixel_count && !should_stop(); ++i) {
            sampler->seed(seed + i);

            Point2u pos = dr::morton_decode<Point2u>(i);
            if (dr::any(pos >= block->size()))
                continue;

            Point2f pos_f = Point2f(Point2i(pos) + block->offset());
            for (uint32_t j = 0; j < sample_count && !should_stop(); ++j) {
                render_sample(scene, sensor, sampler, block, pos_f);
                sampler->advance();
            }
        }
    } else {
        DRJIT_MARK_USED(scene);
        DRJIT_MARK_USED(sensor);
        DRJIT_MARK_USED(sampler);
        DRJIT_MARK_USED(block);
        DRJIT_MARK_USED(sample_count);
        DRJIT_MARK_USED(seed);
        DRJIT_MARK_USED(block_id);
        DRJIT_MARK_USED(block_size);
        Throw("Not implemented for JIT arrays.");
    }
}

// Stock implementation
MI_VARIANT void MVPT::render_sample(const Scene *scene, const Sensor *sensor,
                                                                 Sampler *sampler, ImageBlock *block,
                                                                 const Vector2f &pos, Mask active) const {
    const Film *film = sensor->film();
    // const bool has_alpha  = has_flag(film->flags(), FilmFlags::Alpha);
    // const bool box_filter = film->rfilter()->is_box_filter();

    ScalarVector2f scale  = 1.f / ScalarVector2f(film->crop_size()),
                   offset = -ScalarVector2f(film->crop_offset()) * scale;

    Vector2f sample_pos = pos + sampler->next_2d(active), adjusted_pos = dr::fmadd(sample_pos, scale, offset);

    Point2f aperture_sample(.5f);
    if (sensor->needs_aperture_sample())
        aperture_sample = sampler->next_2d(active);

    Float time = sensor->shutter_open();
    if (sensor->shutter_open_time() > 0.f)
        time += sampler->next_1d(active) * sensor->shutter_open_time();

    Float wavelength_sample = 0.f;
    if constexpr (is_spectral_v<Spectrum>)
        wavelength_sample = sampler->next_1d(active);

    auto [ray, ray_weight] = sensor->sample_ray(time, wavelength_sample, adjusted_pos, aperture_sample);

    auto [spec, valid] = sample_single(scene, sampler, ray, active);
    if(m_force_eval)
        dr::eval(spec, valid);
    block->put(sample_pos, ray.wavelengths, spec * ray_weight, dr::select(valid, Float(1.f), Float(0.f)), 1.f, active);
}

MI_VARIANT std::pair<Spectrum, typename MVPT::Bool> MVPT::sample_single(const Scene *scene, Sampler *sampler, const RayDifferential3f &ray_,
                                            Bool active) const {
        MI_MASKED_FUNCTION(ProfilerPhase::SamplingIntegratorSample, active);

        if (unlikely(m_max_depth == 0))
            return { 0.f, false };

        // --------------------- Configure loop state ----------------------

        Ray3f ray           = Ray3f(ray_);
        Spectrum throughput = 1.f;
        Spectrum result     = 0.f;
        Float eta           = 1.f;
        UInt32 depth        = 0;

        // If m_hide_emitters == false, the environment emitter will be visible
        Mask valid_ray = !m_hide_emitters && (scene->environment() != nullptr);

        // Variables caching information from the previous bounce
        Interaction3f prev_si = dr::zeros<Interaction3f>();
        Float prev_bsdf_pdf   = 1.f;
        Bool prev_bsdf_delta  = true;
        BSDFContext bsdf_ctx;

        /* Set up a Dr.Jit loop. This optimizes away to a normal loop in scalar
           mode, and it generates either a a megakernel (default) or
           wavefront-style renderer in JIT variants. This can be controlled by
           passing the '-W' command line flag to the mitsuba binary or
           enabling/disabling the JitFlag.LoopRecord bit in Dr.Jit.
        */
        struct LoopState {
            Ray3f ray;
            Spectrum throughput;
            Spectrum result;
            Float eta;
            UInt32 depth;
            Mask valid_ray;
            Interaction3f prev_si;
            Float prev_bsdf_pdf;
            Bool prev_bsdf_delta;
            Bool active;
            Sampler *sampler;

            DRJIT_STRUCT(LoopState, ray, throughput, result, eta, depth, valid_ray, prev_si, prev_bsdf_pdf,
                         prev_bsdf_delta, active, sampler)
        } ls = { ray,     throughput,    result,          eta,    depth,  valid_ray,
                 prev_si, prev_bsdf_pdf, prev_bsdf_delta, active, sampler };

        dr::tie(ls) = dr::while_loop(
            dr::make_tuple(ls), [](const LoopState &ls) { return ls.active; },
            [this, scene, bsdf_ctx](LoopState &ls) {
                /* dr::while_loop implicitly masks all code in the loop using
                   the 'active' flag, so there is no need to pass it to every
                   function */

                SurfaceInteraction3f si = scene->ray_intersect(ls.ray,
                                                               /* ray_flags = */ +RayFlags::All,
                                                               /* coherent = */ ls.depth == 0u);
                // ---------------------- Direct emission ----------------------

                /* dr::any_or() checks for active entries in the provided
                   boolean array. JIT/Megakernel modes can't do this test
                   efficiently as each Monte Carlo sample runs independently. In
                   this case, dr::any_or<..>() returns the template argument
                   (true) which means that the 'if' statement is always
                   conservatively taken. */
                if (dr::any_or<true>(si.emitter(scene) != nullptr)) {
                    DirectionSample3f ds(scene, si, ls.prev_si);
                    Float em_pdf = 0.f;

                    if (dr::any_or<true>(!ls.prev_bsdf_delta))
                        em_pdf = scene->pdf_emitter_direction(ls.prev_si, ds, !ls.prev_bsdf_delta);

                    // Compute MIS weight for emitter sample from previous
                    // bounce
                    Float mis_bsdf = mis_weight(ls.prev_bsdf_pdf, em_pdf);

                    // Accumulate, being careful with polarization (see
                    // spec_fma)
                    ls.result =
                        spec_fma(ls.throughput, ds.emitter->eval(si, ls.prev_bsdf_pdf > 0.f) * mis_bsdf, ls.result);
                }

                // Continue tracing the path at this point?
                Bool active_next = (ls.depth + 1 < m_max_depth) && si.is_valid();

                if (dr::none_or<false>(active_next)) {
                    ls.active = active_next;
                    return; // early exit for scalar mode
                }

                BSDFPtr bsdf = si.bsdf(ls.ray);

                // ---------------------- Emitter sampling
                // ----------------------

                // Perform emitter sampling?
                Mask active_em = active_next && has_flag(bsdf->flags(), BSDFFlags::Smooth);

                DirectionSample3f ds = dr::zeros<DirectionSample3f>();
                Spectrum em_weight   = dr::zeros<Spectrum>();
                Vector3f wo          = dr::zeros<Vector3f>();

                if (dr::any_or<true>(active_em)) {
                    // Sample the emitter
                    std::tie(ds, em_weight) =
                        scene->sample_emitter_direction(si, ls.sampler->next_2d(), true, active_em);
                    active_em &= (ds.pdf != 0.f);

                    /* Given the detached emitter sample, recompute its
                       contribution with AD to enable light source optimization.
                     */
                    if (dr::grad_enabled(si.p)) {
                        ds.d            = dr::normalize(ds.p - si.p);
                        Spectrum em_val = scene->eval_emitter_direction(si, ds, active_em);
                        em_weight       = dr::select(ds.pdf != 0, em_val / ds.pdf, 0);
                    }

                    wo = si.to_local(ds.d);
                }

                // ------ Evaluate BSDF * cos(theta) and sample direction
                // -------

                Float sample_1   = ls.sampler->next_1d();
                Point2f sample_2 = ls.sampler->next_2d();

                auto [bsdf_val, bsdf_pdf, bsdf_sample, bsdf_weight] =
                    bsdf->eval_pdf_sample(bsdf_ctx, si, wo, sample_1, sample_2);

                // --------------- Emitter sampling contribution
                // ----------------

                if (dr::any_or<true>(active_em)) {
                    bsdf_val = si.to_world_mueller(bsdf_val, -wo, si.wi);

                    // Compute the MIS weight
                    Float mis_em = dr::select(ds.delta, 1.f, mis_weight(ds.pdf, bsdf_pdf));

                    // Accumulate, being careful with polarization (see
                    // spec_fma)
                    ls.result[active_em] = spec_fma(ls.throughput, bsdf_val * em_weight * mis_em, ls.result);
                }

                // ---------------------- BSDF sampling ----------------------

                bsdf_weight = si.to_world_mueller(bsdf_weight, -bsdf_sample.wo, si.wi);

                ls.ray = si.spawn_ray(si.to_world(bsdf_sample.wo));

                /* When the path tracer is differentiated, we must be careful
                   that the generated Monte Carlo samples are detached (i.e.
                   don't track derivatives) to avoid bias resulting from the
                   combination of moving samples and discontinuous visibility.
                   We need to re-evaluate the BSDF differentiably with the
                   detached sample in that case. */
                if (dr::grad_enabled(ls.ray)) {
                    ls.ray = dr::detach<true>(ls.ray);

                    // Recompute 'wo' to propagate derivatives to cosine term
                    Vector3f wo_2                 = si.to_local(ls.ray.d);
                    auto [bsdf_val_2, bsdf_pdf_2] = bsdf->eval_pdf(bsdf_ctx, si, wo_2, ls.active);
                    bsdf_weight[bsdf_pdf_2 > 0.f] = bsdf_val_2 / dr::detach(bsdf_pdf_2);
                }

                // ------ Update loop variables based on current interaction
                // ------

                ls.throughput *= bsdf_weight;
                ls.eta *= bsdf_sample.eta;
                ls.valid_ray |= ls.active && si.is_valid() && !has_flag(bsdf_sample.sampled_type, BSDFFlags::Null);

                // Information about the current vertex needed by the next
                // iteration
                ls.prev_si         = si;
                ls.prev_bsdf_pdf   = bsdf_sample.pdf;
                ls.prev_bsdf_delta = has_flag(bsdf_sample.sampled_type, BSDFFlags::Delta);

                // -------------------- Stopping criterion ---------------------

                dr::masked(ls.depth, si.is_valid()) += 1;

                Float throughput_max = dr::max(unpolarized_spectrum(ls.throughput));

                Float rr_prob  = dr::minimum(throughput_max * dr::square(ls.eta), .95f);
                Mask rractive_ = ls.depth >= m_rr_depth, rr_continue = ls.sampler->next_1d() < rr_prob;

                /* Differentiable variants of the renderer require the the
                   russian roulette sampling weight to be detached to avoid
                   bias. This is a no-op in non-differentiable variants. */
                ls.throughput[rractive_] *= dr::rcp(dr::detach(rr_prob));

                ls.active = active_next && (!rractive_ || rr_continue) && (throughput_max != 0.f);
            });

        return {dr::select(ls.valid_ray, ls.result, 0.f), ls.valid_ray };
    }
    // Dummy function to hide compiler warnings
    inline void dummy_function2() {}

NAMESPACE_END(mitsuba)