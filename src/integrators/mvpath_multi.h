#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

// This being a header, is a 'hack' to get around the plugin system being limited to single source file per plugin
#include "mvpath.h"

NAMESPACE_BEGIN(mitsuba)

MI_VARIANT void MVPT::render_amvpt(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor, Sampler *sampler,
                                   ImageBlock *block, const Vector2f &pos, SampleData *sampleData, uint32_t n_samples,
                                   Mask active) const {
    const Film *film      = sensor->film();
    ScalarVector2f scale  = 1.f / ScalarVector2f(film->crop_size()),
                   offset = -ScalarVector2f(film->crop_offset()) * scale;

    Vector2f sample_pos   = pos + sampler->next_2d(active);
    Vector2f adjusted_pos = dr::fmadd(sample_pos, scale, offset);

    Point2f aperture_sample(.5f);
    if (sensor->needs_aperture_sample())
        aperture_sample = sampler->next_2d(active);
    Float time = sensor->shutter_open();
    if (sensor->shutter_open_time() > 0.f)
        time = dr::fmadd(sampler->next_1d(active), sensor->shutter_open_time(), time);

    Float wavelength_sample = 0.f;
    if constexpr (is_spectral_v<Spectrum>)
        wavelength_sample = sampler->next_1d(active);

    // Reset sample data
    auto [ray, ray_weight, p_idx] = sensor->sample_ray_idx(time, wavelength_sample, adjusted_pos, aperture_sample);
    // Compute max camera index in each group
    UInt32 max_idx = n_samples * (dr::idiv(p_idx, n_samples) + 1U);
    // Create a sequence of sample data (1 SampleData for each camera in group)
    for (uint32_t i = 0; i < n_samples; i++) {
        UInt32 idx    = p_idx + i;
        sampleData[i] = dr::zeros<SampleData>();
        // Camera indices for given sample in agroup
        sampleData[i].idx = dr::select(idx < max_idx, idx, idx - n_samples);
    }
    sampleData[0].pos = sample_pos;

    // Adaptive doesn't work on translucent materials for some reason, that's what the mask is for
    auto [valid_ray, adapt_mask] =
        sample_mvpt(scene, sensor, sampler, sampleData, n_samples, ray, aperture_sample, active);
    Float alpha            = dr::select(valid_ray, Float(1.f), Float(0.f));
    ScalarVector2u grid    = sensor->grid_dim();
    ScalarVector2u sub_res = film->size() / grid;
    // Set the block as coalesced at the start
    block->set_coalesce(true);
    // Check whether the axis of quilt are reversed
    auto [rev_x, rev_y] = sensor->reverse_axis();
    // Adaptive samples have to be weighted with primary camera samples
    uint32_t n_adapt = std::min(m_adaptive, n_samples - 1);
    float adapt_w    = 1.f / (n_adapt + 1);

    if (m_debug) { // Debug write of adaptive mask to film
        block->put(sampleData[0].pos, ray.wavelengths, Float(adapt_mask), alpha, 1.f, true);
        return;
    } else if (n_adapt) {
        dr::masked(sampleData[0].weight, adapt_mask) = sampleData[0].weight * adapt_w;
    }

    // Accumulate all samples
    for (uint32_t i = 0; i < n_samples; i++) {
        auto &sample = sampleData[i];
        if (dr::none_or<false>(sample.valid))
            continue;
        if (i > 0) { // Compute individual view offsets on film
            auto [y, x] = dr::idivmod(sample.idx, grid[0]);
            // Handle reversed grid axes.
            // The if is deliberate here, since we 
            // can skip vector computation if false.
            if (rev_x)
                x = (grid[0] - 1) - x;
            if (rev_y)
                y = (grid[1] - 1) - y;
            sample.pos = dr::fmadd(Vector2u(x, y), sub_res, sample.pos);
            block->set_coalesce(false); // We are writing to random positions
        }
        // Accumulate to film
        block->put(sample.pos, ray.wavelengths, sample.weight * ray_weight * sample.result, alpha, sample.weight,
                   sample.valid);
    }

    // Fill in missing samples - only delta (not null) or not visible at all
    if (n_adapt) {
        if constexpr (dr::is_array_v<Float>) {
            UInt32 idx = dr::compress(adapt_mask);
            idx        = dr::repeat(idx, n_adapt);
            dr::make_opaque(idx);
            size_t wavefront = dr::width(idx);

            if (wavefront > 0) {
                ref<Sampler> sampler2 = sampler->fork();
                sampler2->seed(wavefront, wavefront);
                Log(LogLevel::Info, "Adaptive samples: %d", wavefront);

                if (sensor->needs_aperture_sample())
                    nested_gather(aperture_sample, idx);

                if (sensor->shutter_open_time() > 0.f)
                    nested_gather(time, idx);

                if constexpr (is_spectral_v<Spectrum>)
                    wavelength_sample = sampler2->next_1d();

                nested_gather(sample_pos, idx);
                adjusted_pos = dr::fmadd(sample_pos, scale, offset);
                // We need to rebuild all rays, otherwise memory consumption goes off the roof
                auto [ray, ray_weight] = sensor->sample_ray(time, wavelength_sample, adjusted_pos, aperture_sample);
                // If we do this inside sample_mvpt(), we can reuse primary hit and other data, BUT:
                //      a) Gathering primary SurfaceInteraction is memory expensive (10x the normal)
                //      b) Kernel with different size is launched directly inbetween other kernel
                //      c) It's much slower anyways - by 50%
                auto [spec, valid] = sample_single(scene, sampler2.get(), ray, true);
                //  Weight samples to average with existing primary samples
                block->put(sample_pos, ray.wavelengths, adapt_w * spec * ray_weight, 1.f, adapt_w, true);
            }
        }
    }
}

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

MI_VARIANT std::pair<typename MVPT::Mask, typename MVPT::Mask>
MVPT::sample_mvpt(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor, Sampler *sampler, SampleData *samples,
                  uint32_t n_samples, const Ray3f &p_ray, const Point2f &p_app, Mask active) const {
    Mask adapt_mask = false;
    if (unlikely(m_max_depth == 0))
        return { false, adapt_mask };
    // Primary sensor and sample
    auto &p_sample = samples[0];
    // Valid ray is used to determine alpha values
    Mask valid_ray = !m_hide_emitters && (scene->environment() != nullptr);
    auto prev_si   = dr::zeros<Interaction3f>();

    // Find prefix (primary hit point)
    SurfaceInteraction3f si = scene->ray_intersect(p_ray, +RayFlags::All, true);
    Bool p_hit              = si.is_valid();

    // ---------------------- Surface emission ---------------------------------------------------------
    // -------------------------------------------------------------------------------------------------
    Mask direct_em = si.emitter(scene) != nullptr;
    if (dr::any_or<true>(direct_em)) {
        DirectionSample3f ds(scene, si, prev_si);
        // Accumulate direct emmisivity only into primary sample
        // However if the emitter also has BSDF,
        // its reuse computation is skipped (unlikely situation)
        auto emis                  = ds.emitter->eval(si, direct_em);
        p_sample.result[direct_em] = emis;
    }
    // Continue tracing from this point
    if (dr::none_or<false>(p_hit)) {
        return { valid_ray, adapt_mask }; // early exit for scalar mode
    }
    // Get the BSDF ptr from primary hit
    BSDFPtr bsdf = si.bsdf(p_ray);
    BSDFContext bsdf_ctx;
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
    // interaction strictly diffuse/delta etc.)... bsdf_val and bsdf_pdf are
    // related to direct lighting (each sample has its own bdsf_val)
    auto [bsdf_val, direct_pdf, bsdf_sample, bsdf_weight] = bsdf->eval_pdf_sample(bsdf_ctx, si, wo, rand_1, rand_2);
    Bool flag_delta                                       = has_flag(bsdf_sample.sampled_type, BSDFFlags::Delta);
    Bool flag_null                                        = has_flag(bsdf_sample.sampled_type, BSDFFlags::Null);
    Bool flag_diff                                        = has_flag(bsdf_sample.sampled_type, BSDFFlags::Diffuse);

    Bool delta     = flag_delta || flag_null;
    Bool not_delta = !delta;
    // Combined flag of primary hit and not delta brdf
    Bool p_not_delta = not_delta && p_hit;
    // Initial mask for reusing samples
    Bool reuse = !direct_em && p_not_delta && bsdf_smooth;
    // In order to reuse a sample in the first place, it has to be on a surface,
    // the bsdf has to be partially smooth, and the brdf must not be delta nor null
    bool should_reuse = n_samples > 1 && dr::any_or<true>(reuse);
    bool should_mis   = m_sa_mis && should_reuse;
    //  Temporary surface interaction used for evaluating BRDFs from different views
    SurfaceInteraction3f si_k = si;

    // ------ Camera selection-------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------

    if (should_mis) { // MVPT algorithm
        // Fill in required BSDFData for selection and MIS
        BSDFData bsdf_data;
        // eval_roughness() is custom extension to stock BSDF class
        bsdf_data.alpha   = bsdf->eval_roughness(si, active);
        bsdf_data.sqr_a   = dr::fmsub(bsdf_data.alpha, bsdf_data.alpha, 1.f);
        bsdf_data.rsqrt_a = dr::rsqrt(bsdf_data.alpha);
        bsdf_data.diffuse = flag_diff;
        bsdf_data.reuse   = reuse;
        bsdf_data.bsdf    = bsdf;
        p_sample.bsdf_val = bsdf_val;
        // Select suitable cameras (modifies bsdf_sample and direct_pdf)
        select_views(scene, sensor, sampler, samples, n_samples, si, si_k, bsdf_data, wo, p_app, rand_2, rand_1, p_hit,
                     bsdf_sample, direct_pdf);
        // Weight the selected cameras
        mis_weights(samples, n_samples, si_k, bsdf_data);

    } else if (should_reuse) { // Normal sampling
        p_sample.valid = p_hit;
        Bool p_face    = Frame3f::cos_theta(si.wi) > 0.f;
        for (uint32_t k = 1; k < n_samples; k++) {
            auto &sample_k       = samples[k];
            auto [ds, Jp, valid] = sensors_visible<false>(scene, sensor, p_app, si, p_face, sample_k.idx, reuse);
            sample_k.pos         = ds.uv;
            sample_k.valid       = valid;
        }
    }

    // --------------- Direct lighting contribution ---------------------------------------------
    // ------------------------------------------------------------------------------------------

    if (dr::any_or<true>(active_em)) {
        // Compute the MIS weight from bsdf_pdf mixture
        Float mis_em = dr::select(ds.delta, 1.f, mis_weight(ds.pdf, direct_pdf));
        // Emitter contribution
        Spectrum emis_mis = em_weight * mis_em;
        // Reuse with MIS
        if (should_mis) {
            for (uint32_t k = 0; k < n_samples; k++) {
                auto &sample_k = samples[k];
                if (dr::none_or<false>(sample_k.valid))
                    continue;
                // Evaluate direct lighting for camera k
                sample_k.bsdf_val                            = si.to_world_mueller(sample_k.bsdf_val, -wo, sample_k.wi);
                sample_k.result[active_em && sample_k.valid] = spec_fma(sample_k.bsdf_val, emis_mis, sample_k.result);
            }
        } else { // Non MIS
            bsdf_val                   = si.to_world_mueller(bsdf_val, -wo, si.wi);
            p_sample.result[active_em] = spec_fma(bsdf_val, emis_mis, p_sample.result);
        }
    }

    // ---------------------- BSDF sampling -----------------------------------------
    // ------------------------------------------------------------------------------

    PrefixData pd;
    // Generated ray for suffix sampling
    pd.ray = si.spawn_ray(si.to_world(bsdf_sample.wo));

    // The implementation is probably more complicated than it needs to be,
    // however any attempt of rewriting this, results in the algorithm straight up breaking
    if (should_mis) { // Multiview BSDF mixture pdf
        Float n_indir = 0.f;
        Float pdf     = 0.f;
        for (uint32_t k = 0; k < n_samples; k++) {
            auto &sample_k = samples[k];
            Mask valid     = sample_k.indirect;
            if (dr::none_or<false>(valid))
                continue;
            si_k.wi                       = sample_k.wi;
            auto [bsdf_val_k, bsdf_pdf_k] = bsdf->eval_pdf(bsdf_ctx, si_k, bsdf_sample.wo, valid);
            // Special case for primary camera
            Mask pvalid = bsdf_pdf_k > 0.f;
            valid &= k > 0 ? pvalid : (pvalid | delta);
            // Accumulate mixture pdf
            if (dr::any_or<true>(valid)) {
                bsdf_pdf_k        = dr::select(valid, bsdf_pdf_k, 0.f);
                sample_k.bsdf_val = dr::select(valid, bsdf_val_k, 0.f);
                pdf += bsdf_pdf_k;
                n_indir += Float(valid);
            }
            sample_k.indirect &= valid;
        }
        // Mixture pdf -> average when not delta, otherwise primary delta pdf
        bsdf_sample.pdf     = dr::select(p_not_delta, pdf / n_indir, bsdf_sample.pdf);
        samples[0].bsdf_val = dr::select(p_not_delta, samples[0].bsdf_val, bsdf_weight);
        // Set adaptive mask based on number of collected samples (just primary)
        adapt_mask = p_hit && !flag_null && (n_indir <= 1.f);
    } else { // Normal single pdf
        bsdf_weight = si.to_world_mueller(bsdf_weight, -bsdf_sample.wo, si.wi);
        if (dr::grad_enabled(pd.ray)) {
            pd.ray = dr::detach<true>(pd.ray);
            // Recompute 'wo' to propagate derivatives to cosine term
            Vector3f wo_2                 = si.to_local(pd.ray.d);
            auto [bsdf_val_2, bsdf_pdf_2] = bsdf->eval_pdf(bsdf_ctx, si, wo_2, p_hit);
            bsdf_weight[bsdf_pdf_2 > 0.f] = bsdf_val_2 / dr::detach(bsdf_pdf_2);
        }
    }

    // ---------------------- Suffix sampling ---------------------------------------
    // ------------------------------------------------------------------------------
    // This needs to work in both cases of MIS/non-MIS
    pd.throughput = should_mis ? 1.f : bsdf_weight;
    pd.eta        = bsdf_sample.eta;
    valid_ray |= p_hit && !flag_null;
    // Use adaptive, only when hit is valid, isn't null interaction (transparency)
    // and only primary camera was sampled.
    // Information about the current vertex needed by the next iteration
    pd.si         = si;
    pd.bsdf_pdf   = bsdf_sample.pdf;
    pd.bsdf_delta = flag_delta;
    pd.depth      = UInt32(p_hit);
    pd.active     = p_hit;
    if (!should_mis) { // Russian roulette if not using MIS
        Float throughput_max = dr::max(unpolarized_spectrum(pd.throughput));
        pd.active &= (throughput_max != 0.f);
    }
    // Sample indirect contribution (once for all views)
    auto [indirect, valid] = sample_suffix(scene, sampler, pd);
    valid_ray |= valid;
    // Forced evaluation can greatly improve rendering speed
    // at the cost of memory usage
    if (m_force_eval)
        dr::eval(indirect);

    // -------------------- Indirect lighting ---------------------
    if (should_mis) {
        // In case of delta reflection, the bsdf_weight is already weighted by pdf
        Float pdfW = dr::select(p_not_delta, dr::rcp(bsdf_sample.pdf), 1.f);
        for (uint32_t k = 0; k < n_samples; k++) {
            auto &sample_k = samples[k];
            if (dr::any_or<true>(sample_k.indirect)) {
                sample_k.bsdf_val = si.to_world_mueller(sample_k.bsdf_val, -bsdf_sample.wo, sample_k.wi);
                sample_k.result[sample_k.indirect] = spec_fma(sample_k.bsdf_val * pdfW, indirect, sample_k.result);
            }
        }
    } else {
        p_sample.result += indirect;
        p_sample.weight = 1.f;
        if (should_reuse) {
            for (uint32_t k = 1; k < n_samples; k++) {
                samples[k].weight = 1.f;
                samples[k].result = p_sample.result;
            }
        }
    }

    // Set primary view back to active mask (because of background)
    p_sample.weight = dr::select(p_hit, p_sample.weight, 1.f);
    p_sample.valid  = active;
    return { valid_ray, adapt_mask };
}

MI_VARIANT void MVPT::select_views(const Scene *scene, const MultiSensor<Float, Spectrum> *sensor, Sampler *sampler,
                                   SampleData *samples, uint32_t n_samples, const SurfaceInteraction3f &si,
                                   SurfaceInteraction3f &si_k, const BSDFData &bsdf, const Vector3f &wo,
                                   const Point2f &p_app, const Point2f &rand_2, const Float &rand_1, const Bool &p_hit,
                                   BSDFSample3f &bsdf_sample, Float &direct_pdf) const {

    static BSDFContext bsdf_ctx = BSDFContext(); // BSDF contexts for choosing sampling methods
    static BSDFContext glossy_ctx(TransportMode::Radiance, (uint32_t) BSDFFlags::Glossy);

    auto &p_sample = samples[0];
    // Get primary face orientation and camera pdf
    Bool p_face                = Frame3f::cos_theta(si.wi) > 0.f;
    auto [p_ds, p_Jp, p_valid] = sensors_visible<true>(scene, sensor, p_app, si, p_face, p_sample.idx, p_hit);
    p_sample.pdf               = p_ds.pdf;
    p_sample.pdf_lk            = p_ds.pdf;
    p_sample.Jp                = p_Jp;
    p_sample.iJp               = dr::select(p_hit, dr::rcp(p_Jp), 0.f);
    p_sample.wi                = si.wi;
    p_sample.wr                = reflect(si.wi);
    // Temporarily set validity of primary sample to valid intersection.
    // It is set to true at the end, to accumulate background samples too...
    p_sample.valid    = p_hit;
    p_sample.indirect = p_hit;
    // Material pdf approximation (fits GGX perfectly) or stock primary BSDF pdf
    p_sample.pdfM = m_fast_mis ? dr::square(Frame3f::cos_theta(dr::normalize(p_sample.wi + p_sample.wr)))
                               : bsdf.bsdf->pdf(glossy_ctx, si, p_sample.wr, p_hit);
    dr::masked(p_sample.pdfM, bsdf.diffuse) = 1.f; // Handle diffuse
    Float n_direct                          = 1.f; // Number of valid direct samples
    Float n_indir                           = 2.f; // Number of valid indirect samples
    // Loop over all the samples
    for (uint32_t k = 1; k < n_samples; k++) {
        // Primary camera starts with valid hit, and pdf_l->k = pdf_l, so it must be skipped.
        auto &sample_k = samples[k];
        // Checks for visibility of sample.
        auto [ds, Jp, valid] = sensors_visible<false>(scene, sensor, p_app, si, p_face, sample_k.idx, bsdf.reuse);
        // If MIS is used to weight samples (amvpt algorithm).
        sample_k.wi   = si.to_local(ds.d);
        sample_k.wr   = reflect(sample_k.wi);
        si_k.wi       = sample_k.wi;
        sample_k.pdfM = m_fast_mis ? dr::square(Frame3f::cos_theta(dr::normalize(si_k.wi + sample_k.wr)))
                                   : bsdf.bsdf->pdf(glossy_ctx, si_k, sample_k.wr, valid);
        // Material selection pdf based on primary and secondary sample
        Float pdf_Mat                     = tv_pdf(p_sample.wr, si_k, sample_k.pdfM, bsdf, valid, m_fast_mis);
        dr::masked(pdf_Mat, bsdf.diffuse) = 1.f;
        // Jacobian and its propability
        Float J     = Jp * p_sample.iJp;
        Float pdf_J = dr::select(J > 1.f, dr::rcp(J), J);
        // Selection propability proportional to Jacobian and Pmat
        Float pdf_Sel = pdf_Mat * pdf_J;
        // Valid represents combination of visibility and material + jacobian selection.
        // In primary sample it is true, if intersection is valid
        valid &= sampler->next_1d() < pdf_Sel;
        if (dr::none_or<false>(valid))
            continue;

        sample_k.Jp  = Jp;
        sample_k.iJp = dr::select(valid, dr::rcp(Jp), 0.f);
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
        // top of that we evaluate mixture pdf of direct light sampling.

        // Propability to replace wo of suffix
        Bool replace = n_indir * sampler->next_1d() < 1.f;
        // Evaluate bsdf for the outgoing direction (this doubles as direct lighting evaluation)
        // Compared to thesis, we must use the same random numbers as primary sample, instead
        // of knowing the sampled type beforehand. But the end result is the same.
        auto [bsdf_val_k, bsdf_pdf_k, bsdf_sample_k, bsdf_weight_k] =
            bsdf.bsdf->eval_pdf_sample(bsdf_ctx, si_k, wo, rand_1, rand_2, valid);
        direct &= bsdf_pdf_k > 0.f;
        //  Direct contribution value and mixture pdf
        sample_k.bsdf_val = bsdf_val_k;
        direct_pdf += dr::select(direct, bsdf_pdf_k, 0.f);
        n_direct += Float(direct);
        // Indirect contribution - replace wo with certain chance.
        // Consider only same lobes (ie. when viewing angle changes too much)
        indirect &= bsdf_sample_k.sampled_type == bsdf_sample.sampled_type;
        dr::masked(bsdf_sample.wo, indirect && replace) = bsdf_sample_k.wo;
        // For indirect sampling we need to use separate flag, because "valid" keeps track
        // of both: valid samples for direct lighting and valid samples in general.
        n_indir += Float(indirect);
        sample_k.indirect = indirect;
    }
    direct_pdf /= n_direct;
}

MI_VARIANT void MVPT::mis_weights(SampleData *samples, uint32_t n_sensors, SurfaceInteraction3f &si_k,
                                  const BSDFData &bsdf_data) const {
    // Loop over all cameras/samples
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
        // Use dr::is_stmt to potentially skip the inner loop on some threads.
        // Run first condition if sample IS valid and NOT diffuse
        // Run second condition if sample is either NOT valid or IS diffuse
        // Makes the default MVPT (without fast MIS) about 1.5x faster during compile 
        pdfSum = pdfSum + dr::if_stmt(
                              dr::make_tuple(si_k, samples, n_sensors, k, bsdf_data, cond, m_fast_mis),
                              cond && !bsdf_data.diffuse,
                              [](const SurfaceInteraction3f &si_k, SampleData *samples, uint32_t n_sensors, uint32_t k,
                                 const BSDFData &bsdf_data, const Mask &, bool fast_mis) {
                                  Float pdfSum         = 0.f;
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
                                      //  Optimized J(j->k) * pdf(J(j->k)) is just min(J^2, 1)
                                      Float pdf_J = dr::minimum(dr::square(sample_j.Jp * sample_k.iJp), 1.f);
                                      Float pdf_Mat =
                                          tv_pdf(sample_j.wr, si_k, sample_k.pdfM, bsdf_data, valid, fast_mis);
                                      //  p(j) * J(j->k) * V(k) * p(J(j->k)) * p(M(j->k))
                                      pdfSum = dr::fmadd(sample_j.pdf, pdf_J * pdf_Mat, pdfSum);
                                  }
                                  return pdfSum;
                              },
                              [](const SurfaceInteraction3f &, SampleData *samples, uint32_t n_sensors, uint32_t k,
                                 const BSDFData &, const Mask &cond, bool) {
                                  Float pdfSum         = 0.f;
                                  const auto &sample_k = samples[k];
                                  for (uint32_t j = 1; j < n_sensors; j++) {
                                      const auto &sample_j = samples[j];
                                      if (j == k || dr::none_or<false>(sample_j.valid)) {
                                          continue;
                                      }
                                      Float pdf_J = dr::minimum(dr::square(sample_j.Jp * sample_k.iJp), 1.f);
                                      pdfSum      = dr::fmadd(sample_j.pdf, pdf_J, pdfSum);
                                  }
                                  return dr::select(cond, pdfSum, 0.f);
                              });
        sample_k.weight = sample_k.pdf_lk / pdfSum;
    }
}

/*Nearly stock PathIntegrator Mitsuba 3 implementation, just with PrefixData structure for input*/
MI_VARIANT std::pair<Spectrum, typename MVPT::Bool> MVPT::sample_suffix(const Scene *scene, Sampler *sampler,
                                                                        const PrefixData &pd) const {
    // MI_MASKED_FUNCTION(ProfilerPhase::SamplingIntegratorSample,
    // pd.active);

    if (unlikely(m_max_depth <= 1) || dr::none_or<false>(pd.active))
        return { 0.f, false };

    // --------------------- Configure loop state ----------------------

    BSDFContext bsdf_ctx;

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
        Bool valid_ray;
        Sampler *sampler;

        DRJIT_STRUCT(LoopState, ray, throughput, result, eta, depth, prev_si, prev_bsdf_pdf, prev_bsdf_delta, active,
                     valid_ray, sampler)
    } ls = { pd.ray,      pd.throughput, result,    pd.eta, pd.depth, pd.si,
             pd.bsdf_pdf, pd.bsdf_delta, pd.active, false,  sampler };
    // Setup looop state from given prefix data

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
            ls.valid_ray |= ls.active && si.is_valid() && !has_flag(bsdf_sample.sampled_type, BSDFFlags::Null);

            /* Differentiable variants of the renderer require the the
               russian roulette sampling weight to be detached to avoid
               bias. This is a no-op in non-differentiable variants. */
            ls.throughput[rractive_] *= dr::rcp(dr::detach(rr_prob));

            ls.active = active_next && (!rractive_ || rr_continue) && (throughput_max != 0.f);
        });

    return { ls.result, ls.valid_ray };
}

inline void dummy_function1() {}

NAMESPACE_END(mitsuba)