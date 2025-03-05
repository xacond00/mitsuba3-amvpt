#pragma once

// This is a 'hack' to get around the plugin system being limited to single source file per plugin

#include "mvpath.h"

NAMESPACE_BEGIN(mitsuba)

MI_VARIANT void MVPT::render_multisample(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor,
                                         Sampler *sampler, ImageBlock *block, const Vector2f &pos,
                                         SampleData *sampleData, Mask active) const {
    const Film *film   = sensor->film();
    uint32_t n_sensors = sensor->n_sensors();

    ScalarVector2f scale  = 1.f / ScalarVector2f(film->crop_size()),
                   offset = -ScalarVector2f(film->crop_offset()) * scale;

    Vector2f sample_pos = pos + sampler->next_2d(active);
    Vector2f adjusted_pos = dr::fmadd(sample_pos, scale, offset);

    Point2f aperture_sample(.5f);
    if (sensor->needs_aperture_sample())
        aperture_sample = sampler->next_2d(active);
    Float time = sensor->shutter_open();
    if (sensor->shutter_open_time() > 0.f)
        time += sampler->next_1d(active) * sensor->shutter_open_time();

    Float wavelength_sample = 0.f;
    if constexpr (is_spectral_v<Spectrum>)
        wavelength_sample = sampler->next_1d(active);
    // Reset sample data
    auto [ray, ray_weight, p_idx] = sensor->sample_ray_idx(time, wavelength_sample, adjusted_pos, aperture_sample);
    // Create a sequence of sample data (1 for each camera)
    for (uint s = 0; s < n_sensors; s++) {
        sampleData[s]     = dr::zeros<SampleData>();
        auto idx          = p_idx + s;
        // Camera index for given sample is (primary_idx + s) % n_sensors 
        idx               = dr::select(idx < n_sensors, idx, idx - n_sensors);
        sampleData[s].idx = idx;
    }
    sampleData[0].pos = sample_pos;

    Mask valid_ray      = sample_multi(scene, sensor, sampler, sampleData, ray, aperture_sample, active);
    Float alpha         = dr::select(valid_ray, Float(1.f), Float(0.f));
    ScalarVector2u grid = sensor->grid_dim();
    ScalarVector2u sres = film->size() / grid;
    // Set the block as coalesced at the start
    block->set_coalesce(true);
    // Check whether the axis of quilt are reversed
    auto [rev_x, rev_y] = sensor->reverse_axis();
    for (uint32_t i = 0; i < n_sensors; i++) {
        auto &sample = sampleData[i];
        // Reorganize the sample positions into grid
        if (i > 0) {
            auto [y, x] = dr::idivmod(sample.idx, grid[0]);
            if(rev_x) x           = (grid[0] - 1) - x;
            if(rev_y) y           = (grid[1] - 1) - y;
            sample.pos += Vector2u(x, y) * sres;
            block->set_coalesce(false);
        }
        if(m_debug)
            block->put(sample.pos, ray.wavelengths, Float(sample.valid), alpha, 1.f, true);
        else if(dr::any_or<true>(sample.valid))
            block->put(sample.pos, ray.wavelengths, sample.weight * ray_weight * sample.result, alpha, sample.weight,
                       sample.valid);
    }
    //sample_single(scene, sampler, ray, );
}

MI_VARIANT typename MVPT::Mask MVPT::sample_multi(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor,
                                                  Sampler *sampler, SampleData *samples, const Ray3f &p_ray,
                                                  const Point2f &p_app, Mask active) const {
    if (unlikely(m_max_depth == 0))
        return false;
    // Primary sensor and sample
    uint32_t n_sensors = sensor->n_sensors();
    auto &p_sample     = samples[0];
    PrefixData pd;
    // Valid ray is used to determine alpha values
    Mask valid_ray = !m_hide_emitters && (scene->environment() != nullptr);
    pd.depth       = 0;
    auto prev_si   = dr::zeros<Interaction3f>();

    // Find prefix (primary hit point)
    SurfaceInteraction3f si = scene->ray_intersect(p_ray, +RayFlags::All, true);

    // ---------------------- Surface emission ---------------------------------------------------------
    // -------------------------------------------------------------------------------------------------
    Mask direct_em = si.emitter(scene) != nullptr;
    if (dr::any_or<true>(direct_em)) {
        DirectionSample3f ds(scene, si, prev_si);
        // As Mitsuba allows emitters having a bsdf, we just assign the
        // result to all samples and hope the weight is close to 1. We could
        // also detach the direct emissivity into a separate accumulator,
        // but this works perfectly fine, if the emitter doesnt have bsdf,
        // as the weight is always 1.
        auto emis = ds.emitter->eval(si, direct_em);
        for (uint32_t k = 0; k < n_sensors; k++)
            samples[k].result[direct_em] = emis;
    }
    // Continue tracing from this point
    Bool p_hit = si.is_valid();
    if (dr::none_or<false>(p_hit)) {
        return valid_ray; // early exit for scalar mode
    }
    // Get the BSDF ptr from primary hit
    BSDFPtr bsdf = si.bsdf(p_ray);
    // ---------------------- Emitter sampling
    // ----------------------
    BSDFContext bsdf_ctx;
    BSDFContext glossy_ctx(TransportMode::Radiance, (uint32_t) BSDFFlags::Glossy);
    Bool bsdf_smooth = has_flag(bsdf->flags(), BSDFFlags::Smooth);
    Mask active_em   = p_hit && bsdf_smooth;

    DirectionSample3f ds = dr::zeros<DirectionSample3f>();
    Spectrum em_weight   = dr::zeros<Spectrum>();
    Vector3f wo          = dr::zeros<Vector3f>();

    // ---------------------- Emitter sampling ---------------------------------------------------------
    // -------------------------------------------------------------------------------------------------

    if (dr::any_or<true>(active_em)) {
        // Sample the emitter
        std::tie(ds, em_weight) = scene->sample_emitter_direction(si, sampler->next_2d(), true, active_em);
        active_em &= (ds.pdf != 0.f);

        if (dr::grad_enabled(si.p)) {
            ds.d            = dr::normalize(ds.p - si.p);
            Spectrum em_val = scene->eval_emitter_direction(si, ds, active_em);
            em_weight       = dr::select(ds.pdf != 0, em_val / ds.pdf, 0);
        }
        // Emitter wo is same for all samples
        wo = si.to_local(ds.d);
    }

    // ------ Evaluate BSDF * cos(theta) and sample direction--------------------------------------------
    // --------------------------------------------------------------------------------------------------

    Float rand_1   = sampler->next_1d();
    Point2f rand_2 = sampler->next_2d();
    // Sampling from primary view
    // Based on brdf sample we can get actual info (ie is this particular
    // interaction strictly diffuse/delta etc.)... Bsdf_val and bsdf_pdf are
    // related to direct lighting (each sample has its own bdsf_val)
    auto [bsdf_val, direct_pdf, bsdf_sample, bsdf_weight] =
        bsdf->eval_pdf_sample(bsdf_ctx, si, wo, rand_1, rand_2);


    Bool not_delta = !has_flag(bsdf_sample.sampled_type, BSDFFlags::Delta) &&
                     !has_flag(bsdf_sample.sampled_type, BSDFFlags::Null);
    Bool delta   = !not_delta;
    Bool p_not_delta = not_delta && p_hit;
    Bool reuse   = !has_flag(bsdf_sample.sampled_type, BSDFFlags::Empty) && p_not_delta && bsdf_smooth;
    Bool diffuse = has_flag(bsdf_sample.sampled_type, BSDFFlags::Diffuse);
    BSDFData bsdf_data = dr::zeros<BSDFData>();
    bsdf_data.bsdf = bsdf;
    // eval_roughness() is extension to stock BSDF class
    bsdf_data.alpha = bsdf->eval_roughness(si, active);
    bsdf_data.sqr_a = dr::fmsub(bsdf_data.alpha, bsdf_data.alpha, 1.f);
    bsdf_data.rsqrt_a = dr::rsqrt(bsdf_data.alpha);
    bsdf_data.diffuse = diffuse;
    bsdf_data.reuse = reuse;
    // In order to reuse a sample in the first place, it has to be on a
    // surface, the bsdf has to be partially smooth, and the brdf must not
    // be delta nor null
    bool should_reuse = dr::any_or<true>(reuse);
    bool sensor_mis   = m_sa_mis && should_reuse;
    //  Temporary interaction used for computing pdfs from different views
    SurfaceInteraction3f si_k = si;

    // ------ Camera selection-------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------
    // Temporarily set validity of primary sample to valid intersection.
    // It is set to true at the end, to accumulate background samples too...
    p_sample.valid = p_hit;
    if (should_reuse) {
        // Get primary face orientation and pdf
        Bool p_face                = Frame3f::cos_theta(si.wi) > 0.f;
        auto [p_ds, p_Jp, p_valid] = sensors_visible<true>(scene, sensor, p_app, si, p_face, p_sample.idx, p_hit);
        p_sample.pdf               = p_ds.pdf;
        p_sample.pdf_lk            = p_ds.pdf;
        p_sample.Jp                = p_Jp;
        p_sample.iJp = dr::rcp(p_Jp);
        p_sample.wi                = si.wi;
        p_sample.wo_r              = reflect(si.wi);
        p_sample.indirect = p_hit;
        p_sample.bsdf_val = bsdf_val;
        p_sample.pdfM =
            m_fast_mis
                ? dr::square(Frame3f::cos_theta(dr::normalize(p_sample.wi + p_sample.wo_r)))
                : bsdf->pdf(glossy_ctx, si, p_sample.wo_r, p_hit);
        Float n_direct = 1.f;
        Float n_indir  = 2.f;
        // It isn't practical to move this block into a separate function,
        // because number of params would be absolutely ridiculous
        for (uint32_t k = 1; k < n_sensors; k++) {
            // Primary camera starts with valid hit, and pdf_l->k = pdf_l, so it must be skipped.
            auto &sample_k = samples[k];
            // Checks for visibility of sample.
            auto [ds, Jp, valid] = sensors_visible<false>(scene, sensor, p_app, si, p_face, sample_k.idx, reuse);
            // If MIS is used to weight samples (amvpt algorithm).
            if (m_sa_mis) {
                sample_k.wi   = si.to_local(ds.d);
                sample_k.wo_r = reflect(sample_k.wi);
                si_k.wi       = sample_k.wi;
                sample_k.pdfM = m_fast_mis ? 
                                    dr::square(Frame3f::cos_theta(dr::normalize(si_k.wi + sample_k.wo_r))):
                                    bsdf->pdf(glossy_ctx, si_k, sample_k.wo_r, valid);
                // Material selection pdf based on primary and secondary sample
                Float pdf_Mat =
                    m_fast_mis
                        ? tv_pdf_fast(p_sample.wo_r, si_k.wi, sample_k.pdfM, bsdf_data, valid)
                        : tv_pdf(p_sample.wo_r, si_k, sample_k.pdfM, bsdf_data, valid);
                dr::masked(pdf_Mat, diffuse) = 1.f;
                // Jacobian and its propability
                Float J     = Jp * p_sample.iJp;
                Float pdf_J = dr::select(J > 1.f, dr::rcp(J), J);
                // Selection propability proportional to Jacobian and Pmat
                Float pdf_Sel = pdf_Mat * pdf_J;
                // Valid represents combination of visibility and material +
                // jacobian selection. In primary sample it is true, if
                // intersection is valid
                valid &= sampler->next_1d() < pdf_Sel;
                if (dr::none_or<false>(valid))
                    continue;

                sample_k.Jp  = Jp;
                sample_k.iJp  = dr::rcp(Jp);
                sample_k.pos = ds.uv;
                // p(k)
                sample_k.pdf = dr::select(valid, ds.pdf, 0.f);
                // p(l->k) = p(l) * J(l->k) * V(k) * p(J(l->k)) * p(M(l->k))
                sample_k.pdf_lk = dr::select(valid, p_sample.pdf * J * pdf_Sel, 0.f);
                sample_k.valid  = valid;
                Bool indirect   = valid;
                Bool direct     = valid;
                // The selection of bsdf wo, as opposed to the original
                // paper, is done right here. We reject the selection when the
                // sampled bsdf is not the same type as the initial one. On
                // top of that we evaluate mixture pdf of direct light sampling

                // Propability to replace wo sample
                Bool replace = n_indir * sampler->next_1d() < 1.f;
                auto [bsdf_val_k, bsdf_pdf_k, bsdf_sample_k, bsdf_weight_k] =
                    bsdf->eval_pdf_sample(bsdf_ctx, si_k, wo, rand_1, rand_2, valid);
                direct &= bsdf_pdf_k > 0.f;
                //  Direct contribution mixture pdf
                sample_k.bsdf_val = bsdf_val_k;
                direct_pdf += dr::select(direct, bsdf_pdf_k, 0.f);
                n_direct += Float(direct);
                // Indirect contribution - replace wo with certain
                // chance. Consider only same lobes (ie when viewing
                // angle changes too much)
                indirect &= bsdf_sample_k.sampled_type == bsdf_sample.sampled_type;
                dr::masked(bsdf_sample.wo, indirect && replace) = bsdf_sample_k.wo;

                n_indir += Float(indirect);

                sample_k.indirect = indirect;
            } else {
                sample_k.valid = valid;
                sample_k.pos   = ds.uv;
            }
        }
        direct_pdf /= n_direct;
    }

    // ------ Compute MIS weights----------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------
    if (sensor_mis) {
        mis_weights(samples, n_sensors, si_k, bsdf_data);
    }
    // --------------- Emitter sampling contribution---------------------------------------------
    // ------------------------------------------------------------------------------------------

    if (dr::any_or<true>(active_em)) {
        // Compute the MIS weight from bsdf_pdf mixture
        Float mis_em = dr::select(ds.delta, 1.f, mis_weight(ds.pdf, direct_pdf));
        // Emitter contribution
        Spectrum emis_mis = em_weight * mis_em;
        // Reuse with MIS
        if (sensor_mis) {
            for (uint32_t k = 0; k < n_sensors; k++) {
                auto &sample_k = samples[k];
                if (dr::none_or<false>(sample_k.valid))
                    continue;
                // Evaluate direct lighting for camera k
                sample_k.bsdf_val                            = si.to_world_mueller(sample_k.bsdf_val, -wo, sample_k.wi);
                sample_k.result[active_em && sample_k.valid] = spec_fma(sample_k.bsdf_val, emis_mis, sample_k.result);
            }
        } else {
            bsdf_val     = si.to_world_mueller(bsdf_val, -wo, si.wi);
            uint32_t total = should_reuse ? n_sensors : 1;
            for (uint32_t k = 0; k < total; k++) {
                samples[k].result[active_em] = spec_fma(bsdf_val, emis_mis, samples[k].result);
                samples[k].weight            = 1;
            }
        }
    }

    // ---------------------- BSDF sampling -----------------------------------------
    // ------------------------------------------------------------------------------

    pd.ray = si.spawn_ray(si.to_world(bsdf_sample.wo));
    // Multiview BSDF mixture pdf
    if (sensor_mis) {
        // Primary sensor - if delta or miss: keep old bsdf_val; else: evaluate
        auto [bsdf_val_t, bsdf_pdf_t] = bsdf->eval_pdf(bsdf_ctx, si, bsdf_sample.wo, p_sample.indirect);
        Bool p_valid = bsdf_pdf_t > 0.f;
        p_sample.bsdf_val = dr::select(p_valid, bsdf_val_t, bsdf_weight);
        // If the sample was delta, it was valid previously
        p_sample.indirect &= delta || p_valid;

        Float indir_pdf = bsdf_pdf_t;
        Float n_indir = Float(p_valid);

        for (uint32_t k = 1; k < n_sensors; k++) {
            auto &sample_k = samples[k];
            if (dr::none_or<false>(sample_k.valid))
                continue;
            si_k.wi             = sample_k.wi;
            auto [bsdf_val_k, bsdf_pdf_k] = bsdf->eval_pdf(bsdf_ctx, si_k, bsdf_sample.wo, sample_k.indirect);
            Mask valid = bsdf_pdf_k > 0.f;
            sample_k.bsdf_val = valid & bsdf_val_k;
            sample_k.indirect &= valid;
            indir_pdf += bsdf_pdf_k;
            n_indir += Float(valid);
        }
        
        // Mixture pdf -> average when not delta, otherwise primary delta pdf
        dr::masked(bsdf_sample.pdf, p_not_delta) = indir_pdf / n_indir;
    } // Normal BSDF case
    else {
        bsdf_weight = si.to_world_mueller(bsdf_weight, -bsdf_sample.wo, si.wi);
        if (dr::grad_enabled(pd.ray)) {
            pd.ray = dr::detach<true>(pd.ray);
            // Recompute 'wo' to propagate derivatives to cosine term
            Vector3f wo_2                   = si.to_local(pd.ray.d);
            auto [bsdf_val_2, bsdf_pdf_2]   = bsdf->eval_pdf(bsdf_ctx, si, wo_2, p_hit);
            bsdf_weight[bsdf_pdf_2 > 0.f] = bsdf_val_2 / dr::detach(bsdf_pdf_2);
        }
    }
    
    // ---------------------- Suffix sampling ---------------------------------------
    // ------------------------------------------------------------------------------

    pd.throughput = sensor_mis ? 1.f : bsdf_weight;
    pd.eta        = bsdf_sample.eta;
    valid_ray |= p_hit && !has_flag(bsdf_sample.sampled_type, BSDFFlags::Null);

    // Information about the current vertex needed by the next iteration
    pd.si         = si;
    pd.bsdf_pdf   = bsdf_sample.pdf;
    pd.bsdf_delta = has_flag(bsdf_sample.sampled_type, BSDFFlags::Delta);

    // -------------------- Stopping criterion ---------------------

    pd.depth += p_hit;
    pd.active = p_hit;
    if (!sensor_mis) {
        Float throughput_max = dr::max(unpolarized_spectrum(pd.throughput));
        pd.active &= (throughput_max != 0.f);
    }
    // Sample indirect contribution (once for all views)
    Spectrum indirect = sample_suffix(scene, sampler, pd);
    if (m_force_eval)
        dr::eval(indirect);

    if (sensor_mis) {
        // In case of delta reflection, the bsdf_weight is already weighted by pdf
        Float pdfW = dr::select(p_not_delta, dr::rcp(bsdf_sample.pdf), 1.f);
        for (uint32_t k = 0; k < n_sensors; k++) {
            auto &sample_k = samples[k];
            if (dr::any_or<true>(sample_k.indirect)) {
                si_k.wi                            = sample_k.wi;
                auto bsdf_we                       = si_k.to_world_mueller(sample_k.bsdf_val, -bsdf_sample.wo, si_k.wi);
                sample_k.result[sample_k.indirect] = spec_fma(bsdf_we * pdfW, indirect, sample_k.result);
            }
        }

    } else {
        uint32_t total = should_reuse ? n_sensors : 1;
        for (uint32_t k = 0; k < total; k++) {
            samples[k].result += indirect;
        }
    }
    p_sample.weight = dr::select(p_hit, p_sample.weight, 1.f);
    // Set primary view back to active (background)
    p_sample.valid  = active;
    return valid_ray;
}

MI_VARIANT void MVPT::mis_weights(SampleData *samples, uint n_sensors, SurfaceInteraction3f &si_k, const BSDFData &bsdf_data)const{
    #if 0
    dr::if_stmt(dr::make_tuple(samples, n_sensors, si_k, bsdf_data, m_fast_mis), bsdf_data.reuse, 
    [](SampleData *samples, uint n_sensors, SurfaceInteraction3f &si_k, const BSDFData &bsdf_data, bool m_fast_mis){

        for (uint32_t k = 0; k < n_sensors; k++) {
            // Sample k already contains the conditional propability
            // nominator p(l->k)
            auto &sample_k = samples[k];
            // Skip invalid samples (for scalar variant)
            if (dr::none_or<false>(sample_k.valid))
                continue;
            si_k.wi      = sample_k.wi;
            Float inv_Jk = dr::rcp(sample_k.Jp);
            Float pdfMk  = sample_k.pdfM;

            Float pdfSum = sample_k.pdf_lk;
            // P(j->k) is just p(k), if k == j
            if (k > 0)
                pdfSum += sample_k.pdf;
            Bool cond = k > 0 ? sample_k.valid : bsdf_data.reuse;

            //  Sum propability of all pdf j to k
            //  We are avoiding masking as much as possible in the nested
            //  loops (ie the pdfs are already set to 0 before)
            for (uint32_t j = 1; j < n_sensors; j++) {
                auto &sample_j = samples[j];
                // Valid = selection + visibility
                Bool valid = sample_j.valid;
                // Skip known/invalid samples
                if (j == k || dr::none_or<false>(valid)) {
                    continue;
                }
                // Sum only valid samples (visible, and after selection)
                Float J = sample_j.Jp * inv_Jk;
                Float pdf_Mat =
                    m_fast_mis
                        ? tv_pdf_fast(sample_j.wo_r, si_k.wi, pdfMk, bsdf_data,
                                      valid)
                        : tv_pdf(sample_j.wo_r, si_k, pdfMk, bsdf_data, valid);
                dr::masked(pdf_Mat, bsdf_data.diffuse) = 1.f;
                // p(j) * J(j->k) * V(k) * p(J(j->k)) * p(M(j->k))
                // Optimized J(j->k) * pdf(J(j->k)) is just min(J^2, 1)
                pdfSum +=
                    sample_j.pdf * dr::minimum(dr::square(J), 1.f) * pdf_Mat;
            }
            sample_k.weight = dr::select(cond, sample_k.pdf_lk / pdfSum, 1);
        }
        return true;

    }, [](SampleData*, uint , SurfaceInteraction3f &, const BSDFData &, bool){return false;});
    #else 
    for (uint32_t k = 0; k < n_sensors; k++) {
        bool not_first = k > 0;
        auto &sample_k = samples[k];
        // Skip invalid samples (for scalar variant)
        if (dr::none_or<false>(sample_k.valid))
            continue;
        si_k.wi      = sample_k.wi;
        Float pdfSum = sample_k.pdf_lk;
        // P(j->k) is just p(k), if k == j
        if (not_first)
            pdfSum += sample_k.pdf;
        Bool cond = not_first ? sample_k.valid : bsdf_data.reuse;
        // Use dr::is_stmt to potentially skip the inner loop.
        pdfSum += DR_IF(
            cond, (m_fast_mis, si_k, samples, n_sensors, k, bsdf_data),
            (Float pdfSum = 0.f;
            const auto &sample_k = samples[k];

             //  Sum propability of all pdf j to k
             //  We are avoiding masking as much as possible in the nested
             //  loops (ie the pdfs are already set to 0 before)
             for (uint32_t j = 1; j < n_sensors; j++) {
                 const auto &sample_j = samples[j];
                 // Valid = selection + visibility
                 const Bool &valid = sample_j.valid;
                 // Skip known/invalid samples
                 if (j == k || dr::none_or<false>(valid)) {
                     continue;
                 }
                 // Sum only valid samples (visible, and after selection)
                 Float J = sample_j.Jp * sample_k.iJp;
                 Float pdf_Mat =
                     m_fast_mis
                         ? tv_pdf_fast(sample_j.wo_r, si_k.wi, sample_k.pdfM, bsdf_data, valid)
                         : tv_pdf(sample_j.wo_r, si_k, sample_k.pdfM, bsdf_data, valid);
                 dr::masked(pdf_Mat, bsdf_data.diffuse) = 1.f;
                 // p(j) * J(j->k) * V(k) * p(J(j->k)) * p(M(j->k))
                 // Optimized J(j->k) * pdf(J(j->k)) is just min(J^2, 1)
                 pdfSum += sample_j.pdf * dr::minimum(dr::square(J), 1.f) * pdf_Mat;
             } return pdfSum;),
            (return dr::zeros<Float>();));

        sample_k.weight = sample_k.pdf_lk / pdfSum;
    }
    #endif
}


/*Nearly stock PathIntegrator Mitsuba 3 implementation*/
MI_VARIANT Spectrum MVPT::sample_suffix(const Scene *scene, Sampler *sampler, const PrefixData &pd) const {
    // MI_MASKED_FUNCTION(ProfilerPhase::SamplingIntegratorSample,
    // pd.active);

    if (unlikely(m_max_depth <= 1) || dr::none_or<false>(pd.active))
        return 0.f;

    // --------------------- Configure loop state ----------------------

    BSDFContext bsdf_ctx;

    /* Set up a Dr.Jit loop. This optimizes away to a normal loop in scalar
       mode, and it generates either a a megakernel (default) or
       wavefront-style renderer in JIT variants. This can be controlled by
       passing the '-W' command line flag to the mitsuba binary or
       enabling/disabling the JitFlag.LoopRecord bit in Dr.Jit.
    */
    Spectrum result = 0.f;
    struct LoopState {
        Ray3f ray;
        Spectrum throughput;
        Spectrum result;
        Float eta;
        UInt32 depth;
        Interaction3f prev_si;
        Float prev_bsdf_pdf;
        Bool prev_bsdf_delta;
        Bool active;
        Sampler *sampler;

        DRJIT_STRUCT(LoopState, ray, throughput, result, eta, depth, prev_si, prev_bsdf_pdf, prev_bsdf_delta, active,
                     sampler)
    } ls = { pd.ray, pd.throughput, result, pd.eta, pd.depth, pd.si, pd.bsdf_pdf, pd.bsdf_delta, pd.active, sampler };

    dr::tie(ls) = dr::while_loop(
        dr::make_tuple(ls), [](const LoopState &ls) { return ls.active; },
        [this, scene, bsdf_ctx](LoopState &ls) {
            SurfaceInteraction3f si = scene->ray_intersect(ls.ray,
                                                           /* ray_flags = */ +RayFlags::All,
                                                           /* coherent = */ false);

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
                ls.result = spec_fma(ls.throughput, ds.emitter->eval(si, ls.prev_bsdf_pdf > 0.f) * mis_bsdf, ls.result);
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
                std::tie(ds, em_weight) = scene->sample_emitter_direction(si, ls.sampler->next_2d(), true, active_em);
                active_em &= (ds.pdf != 0.f);

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

    return ls.result;
}

inline void dummy_function1(){}

NAMESPACE_END(mitsuba)