#include "mvpath.h"
#include "mvpath_multi.h"
#include "mvpath_single.h"

NAMESPACE_BEGIN(mitsuba)

MI_VARIANT typename MVPT::TensorXf MVPT::render(Scene *scene, Sensor *init_sensor, UInt32 seed, uint32_t spp,
                                                bool develop, bool evaluate) {
    // Hide unused header warnings
    dummy_function1();
    dummy_function2();
    ScopedPhase sp(ProfilerPhase::Render);
    m_stop = false;
    // Primary multisensor
    MultiSensor<Float, Spectrum> *sensor = dynamic_cast<MultiSensor<Float, Spectrum> *>(init_sensor);
    if (!sensor)
        Throw("This integrator can only be used with MultiSensor !");
    // Get subsensors
    std::vector<Sensor *> sensors = sensor->sensors();
    uint32_t n_sensors            = sensors.size();
    m_sa_reuse &= n_sensors > 1;

    const std::string &sensor_type = sensors[0]->class_()->name();
    m_thin_lens                    = sensor_type == "ThinLensCamera";
    if (!m_thin_lens && sensor_type != "PerspectiveCamera")
        Throw("Subsensor must be of type ThinLensCamera or PerspectiveCamera !");
    // Render on a larger film if the 'high quality edges' feature is enabled
    Film *film               = sensor->film();
    ScalarVector2u film_size = film->crop_size();
    if (film->sample_border())
        film_size += 2 * film->rfilter()->border_size();

    // Potentially adjust the number of samples per pixel if spp != 0
    Sampler *sampler = sensor->sampler();
    if (!spp)
        spp = sampler->sample_count();
    uint32_t spp_per_pass = m_spp_pass_lim ? std::min(m_spp_pass_lim, spp) : spp;

    uint32_t n_passes = spp / spp_per_pass;
    // Round SPP to multiple of spp_per_pass 
    spp               = n_passes * spp_per_pass;
    sampler->set_sample_count(spp);
    // Check if the sampler is independent.
    bool independent = sampler->class_()->name() == "IndependentSampler";

    if(!aov_names().empty())
        Throw("This integrator cannot be used with AOVs !");
    // Determine output channels and prepare the film with this information
    //size_t n_channels = 
    film->prepare(aov_names());
    // Start the render timer (used for timeouts & log messages)
    m_render_timer.reset();

    TensorXf result;
    if constexpr (!dr::is_jit_v<Float>) {
        // Render on the CPU using a spiral pattern
        uint32_t n_threads = (uint32_t) Thread::thread_count();

        Log(Info, "Starting render job (%ux%u, %u sample%s,%s %u thread%s)", film_size.x(), film_size.y(), spp,
            spp == 1 ? "" : "s", n_passes > 1 ? tfm::format(" %u passes,", n_passes) : "", n_threads,
            n_threads == 1 ? "" : "s");

        if (m_timeout > 0.f)
            Log(Info, "Timeout specified: %.2f seconds.", m_timeout);

        // If no block size was specified, find size that is good for parallelization
        uint32_t block_size = m_block_size;
        if (block_size == 0) {
            block_size = MI_BLOCK_SIZE; // 32x32
            while (true) {
                // Ensure that there is a block for every thread
                if (block_size == 1 || dr::prod((film_size + block_size - 1) / block_size) >= n_threads)
                    break;
                block_size /= 2;
            }
        }

        Spiral spiral(film_size, film->crop_offset(), block_size, n_passes);

        std::mutex mutex;
        ref<ProgressReporter> progress;
        Logger *logger = mitsuba::Thread::thread()->logger();
        if (logger && Info >= logger->log_level())
            progress = new ProgressReporter("Rendering");

        // Total number of blocks to be handled, including multiple passes.
        uint32_t total_blocks = spiral.block_count() * n_passes, blocks_done = 0;

        // Grain size for parallelization
        uint32_t grain_size = std::max(total_blocks / (4 * n_threads), 1u);

        // Avoid overlaps in RNG seeding RNG when a seed is manually specified
        seed *= dr::prod(film_size);

        ThreadEnvironment env;
        dr::parallel_for(
            dr::blocked_range<uint32_t>(0, total_blocks, grain_size), [&](const dr::blocked_range<uint32_t> &range) {
                ScopedSetThreadEnvironment set_env(env);
                // Fork a non-overlapping sampler for the current worker
                ref<Sampler> sampler = sensor->sampler()->fork();

                ref<ImageBlock> block =
                    film->create_block(ScalarVector2u(block_size) /* size */, false /* normalize */, true /* border */);

                // Render up to 'grain_size' image blocks
                for (uint32_t i = range.begin(); i != range.end() && !should_stop(); ++i) {
                    auto [offset, size, block_id] = spiral.next_block();
                    Assert(dr::prod(size) != 0);

                    if (film->sample_border())
                        offset -= film->rfilter()->border_size();

                    block->set_size(size);
                    block->set_offset(offset);

                    render_block_(scene, sensor, sampler, block, spp_per_pass, seed, block_id, block_size);

                    film->put_block(block);

                    /* Critical section: update progress bar */
                    if (progress) {
                        std::lock_guard<std::mutex> lock(mutex);
                        blocks_done++;
                        progress->update(blocks_done / (float) total_blocks);
                    }
                }
            });

        if (develop)
            result = film->develop();
    } else {
        size_t wavefront_size       = (size_t) film_size.x() * (size_t) film_size.y() * (size_t) spp_per_pass,
               wavefront_size_limit = 0xffffffffu;

        if (wavefront_size > wavefront_size_limit) {
            spp_per_pass /= (uint32_t) ((wavefront_size + wavefront_size_limit - 1) / wavefront_size_limit);
            n_passes       = spp / spp_per_pass;
            wavefront_size = (size_t) film_size.x() * (size_t) film_size.y() * (size_t) spp_per_pass;

            Log(Warn,
                "The requested rendering task involves %zu Monte Carlo "
                "samples, which exceeds the upper limit of 2^32 = 4294967296 "
                "for this variant. Mitsuba will instead split the rendering "
                "task into %zu smaller passes to avoid exceeding the limits.",
                wavefront_size, n_passes);
        }

        dr::sync_thread(); // Separate from scene initialization (for timings)

        Log(Info, "Starting render job (%ux%u, %u sample%s%s)", film_size.x(), film_size.y(), spp, spp == 1 ? "" : "s",
            n_passes > 1 ? tfm::format(", %u passes", n_passes) : "");

        if (n_passes > 1 && !evaluate) {
            Log(Warn, "render(): forcing 'evaluate=true' since multi-pass "
                      "rendering was requested.");
            evaluate = true;
        }

        // Inform the sampler about the passes (needed in vectorized modes)
        sampler->set_samples_per_wavefront(spp_per_pass);

        // Seed the underlying random number generators, if applicable
        sampler->seed(seed, (uint32_t) wavefront_size);

        // Allocate a large image block that will receive the entire rendering
        ref<ImageBlock> block = film->create_block(0, m_sa_reuse && 0);
        block->set_offset(film->crop_offset());

        // Only use the ImageBlock coalescing feature when rendering enough samples
        block->set_coalesce(block->coalesce() && spp_per_pass >= 4);

        // Compute discrete sample position
        UInt32 idx = dr::arange<UInt32>((uint32_t) wavefront_size);

        // Try to avoid a division by an unknown constant if we can help it
        uint32_t log_spp_per_pass = dr::log2i(spp_per_pass);
        if ((1u << log_spp_per_pass) == spp_per_pass)
            idx >>= dr::opaque<UInt32>(log_spp_per_pass);
        else
            idx /= dr::opaque<UInt32>(spp_per_pass);

        // Compute the position on the image plane
        Vector2i pos;
        pos.y() = idx / film_size[0];
        pos.x() = dr::fnmadd(film_size[0], pos.y(), idx);

        if (film->sample_border())
            pos -= film->rfilter()->border_size();

        pos += film->crop_offset();

        std::unique_ptr<SampleData[]> sampleData(new SampleData[n_sensors]);
        Timer timer;
        // Potentially render multiple passes
        for (size_t i = 0; i < n_passes; i++) {
            ref<Sampler> sampler_ref;
            Sampler *sampler_i = sampler;
            // Optimization to greatly reduce memory usage
            if(independent){
                sampler_ref = sampler->fork();
                sampler_i = sampler_ref.get();
                sampler_i->seed(i + seed, wavefront_size);
            }

            if (m_sa_reuse) {
                render_multisample(scene, sensor, sampler_i, block, pos, sampleData.get());
            } else {
                render_sample(scene, sensor, sampler_i, block, pos);
            }
            // Evaluate each pass
            if (n_passes > 1) {
                if(!independent && i + 1 < n_passes){
                    sampler_i->advance(); // Will trigger a kernel launch of size 1
                    sampler_i->schedule_state();
                }
                dr::eval(block->tensor());
            }
        }

        film->put_block(block);
        bool measure_gen = !m_force_eval && n_passes == 1 && jit_flag(JitFlag::VCallRecord) && jit_flag(JitFlag::LoopRecord);
        if (measure_gen) {
            Log(Info, "Computation graph recorded. (took %.3f s)", 0.001f * timer.reset());
        }

        if (develop) {
            result = film->develop();
            dr::schedule(result);
        } else {
            film->schedule_storage();
        }

        if (evaluate) {
            dr::eval();

            if (measure_gen) {
                Log(Info, "Code generation finished. (took %.3f s)", 0.001f * timer.reset());
                m_render_timer.reset();
            }

            dr::sync_thread();
        }
    }

    if (!m_stop && (evaluate || !dr::is_jit_v<Float>) )
        Log(Info, "Rendering finished. (took %.3f s)", 0.001f * m_render_timer.value());

    return result;
}

MI_IMPLEMENT_CLASS_VARIANT(MVPathIntegrator, MonteCarloIntegrator)
MI_EXPORT_PLUGIN(MVPathIntegrator, "MVintegrator");
NAMESPACE_END(mitsuba)