#include <mitsuba/core/fwd.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/render/fwd.h>
#include <mitsuba/render/sensor.h>
#include <mitsuba/render/wrap.h>

NAMESPACE_BEGIN(mitsuba)

/**!

.. _sensor-grid:

Grid sensor (:monosp:`grid`)
--------------------------------------------

.. pluginparameters::

 * - srf
   - |spectrum|
   - Sensor Response Function that defines the :ref:`spectral sensitivity
<explanation_srf_sensor>` of the sensor (Default: :monosp:`none`)

This meta-sensor generates multiple sensors along a given direction,
so that they can be rendered simultaneously.
This reduces tracing overheads in applications that need to
render many viewpoints, particularly in the context of differentiable rendering.

It works similiarly to batch sensor, except for being able to organize the
sensors to output into grid instead of array. The sensor behaves as expected in
normal path integrators, however it is specifically tailored to work with
MVPathIntegrator. This sensor relies on "wrap" implementation to duplicate a
sensor from single definition along a given axis.

.. tabs::
    .. code-tab:: xml
        :name: grid-sensor

        <sensor type="grid">
            <integer name="grid_x" value="3" />
            <vector name="cam_dir" value="1,0,0" />
            <float name="cam_dist" value="0.2" />
            <wrap type="any">
                <string name="wrap_class" value="sensor"/>
                <string name="wrap_type" value="perspective"/>
                <float name="fov" value="45" />
                <ref id="m_film"/>
                <ref id="m_sampler"/>
            </wrap>
            <ref id="m_film"/>
            <ref id="m_sampler"/>
        </sensor>

    .. code-tab:: python

        'type': 'grid',
        # Two perpendicular viewing directions
        'sensor1': {
            'type': 'perspective',
            'fov': 45,
            'to_world': mi.ScalarTransform4f.look_at(
                origin=[0, 0, 1],
                target=[0, 0, 0],
                up=[0, 1, 0]
            )
        },

        'film_id': {
            'type': '<film_type>',
            # ...
        },
        'sampler_id': {
            'type': '<sampler_type>',
            # ...
        }
*/

MI_VARIANT class GridSensor final : public MultiSensor<Float, Spectrum> {
public:
    MI_IMPORT_BASE(MultiSensor, m_film, m_shape, m_needs_sample_3,
                   sample_wavelengths)
    MI_IMPORT_TYPES(Shape, SensorPtr)
    using Sensor_t = Sensor<Float, Spectrum>;

    GridSensor(const Properties &props) : Base(props) {
        ScalarTransform4f to_world = Base::m_to_world.scalar();
        /*Load camera distribution parameters*/
        m_reverse  = std::pair<bool, bool>(props.get<bool>("reverse_x", false),
                                           props.get<bool>("reverse_y", true));
        m_grid_dim = ScalarVector2u(props.get<uint32_t>("grid_x", 1),
                                    props.get<uint32_t>("grid_y", 1));
        m_film_res = ScalarVector2u(props.get<uint32_t>("res_x", 0),
                                    props.get<uint32_t>("res_y", 0));

        m_film_res[0] =
            m_film_res[0] ? m_film_res[0] : m_film->size()[0] * m_grid_dim[0];
        m_film_res[1] =
            m_film_res[1] ? m_film_res[1] : m_film->size()[1] * m_grid_dim[1];

        if (m_film_res[0] % m_grid_dim[0] || m_film_res[1] % m_grid_dim[1])
            Throw("Film size must be divisible by grid dimensions !");
        auto sub_res = m_film_res / m_grid_dim;
        double sub_asp = sub_res.x() / double(sub_res.y());
        m_grid_size = m_grid_dim[0] * m_grid_dim[1];
        m_used_cone = false;
        m_cam_center = false;

        // Camera generation props
        if (props.has_property("cone_deg")) {
            // Interpolate on a line based on angle with known focal plane
            m_used_cone = true;
            m_cone_deg = props.get<ScalarFloat>("cone_deg");
        } 
        else if (props.has_property("cam_dir")) {
            // Interpolate towards a direction (centered by default)
            m_cam_dir = props.get<ScalarVector3f>("cam_dir");
            m_cam_dist    = dr::norm(m_cam_dir);
            m_cam_dist    = props.get<ScalarFloat>("cam_dist", m_cam_dist);
            m_cam_center    = props.get<bool>("cam_center", true);
        } else if (props.has_property("cam_end")) {
            // Interpolate towards end world space position
            auto m_cam_beg  = to_world.translation();
            auto m_cam_end  = props.get<ScalarVector3f>("cam_end");
            m_cam_dir  = to_world.inverse() * (m_cam_end - m_cam_beg);
            m_cam_dist = dr::norm(m_cam_dir);
            m_cam_dir  = m_cam_dir / m_cam_dist;
            m_cam_center = false;
        } else {
            m_cam_dir  = ScalarVector3f(1, 0, 0);
            m_cam_dist = 0.1f;
        }
        m_cam_off = props.get<ScalarVector3f>("cam_off", ScalarVector3f(0,0,0));
        m_cam_off.y() = -m_cam_off.y();
        m_cam_off.z() = -m_cam_off.z();
        /*Load wrappers*/
        using Wrap_t   = Wrap<Float, Spectrum>;
        Wrap_t *w_sens = nullptr;
        Wrap_t *w_film = nullptr;
        Wrap_t *w_samp = nullptr;
        // Get wraps for sensor, film and sampler
        for (auto [unused, o] : props.objects()) {
            Wrap_t *wrap(dynamic_cast<Wrap_t *>(o.get()));
            if (wrap->wrap_class() == "sensor") {
                if (w_sens)
                    Throw("Only one Wrap of type Sensor can be specified !");
                w_sens = wrap;
            } else if (wrap->wrap_class() == "film") {
                if (w_film)
                    Throw("Only one Wrap of type Film can be specified !");
                w_film = wrap;
            } else if (wrap->wrap_class() == "sampler") {
                if (w_samp)
                    Throw("Only one Wrap of type Sampler can be specified !");
                w_samp = wrap;
            }
        }
        if (!w_sens || !w_film || !w_samp)
            Throw("Need to specify wraps for sensor, film and sampler !");
        Properties &sub_props = w_sens->properties();
        // Remove films and samplers from within nested sensor definition
        for (auto [name, o] : sub_props.objects()) {
            Wrap_t *wrap(dynamic_cast<Wrap_t *>(o.get()));
            auto *film(dynamic_cast<Film<Float, Spectrum> *>(o.get()));
            auto *samp(dynamic_cast<Sampler<Float, Spectrum> *>(o.get()));
            if (wrap || film || samp) {
                sub_props.remove_property(name);
            }
        }
        double fov_x = parse_fov((props.has_property("fov") ? props : sub_props), sub_asp);
        m_foc_dist = sub_props.get<float>("focus_distance", 1);
        m_foc_dist = props.get<float>("focus_distance", m_foc_dist);
        //ScalarFloat fov = dr::tan(dr::deg_to_rad(fov_x) * 0.5) / sub_asp;
        
        // Override default parameters
        sub_props.remove_property("focal_length");
        sub_props.set_float("fov", fov_x, false);
        sub_props.set_string("fov_axis", "x", false);
        sub_props.set_float("focus_distance", (double)m_foc_dist, false);
        // Set film and sampler identical to the one in the body
        sub_props.set_object("film_wrap", w_film);
        sub_props.set_object("samp_wrap", w_samp);
        
        if(m_used_cone){
            for (uint32_t i = 0; i < m_grid_size; ++i) {
                ScalarFloat dt = i / ScalarFloat(m_grid_size - 1);
                ScalarFloat tan_off = dr::tan((dt - 0.5f) * dr::deg_to_rad(m_cone_deg));
                ScalarFloat offset = m_foc_dist * tan_off;
                double shift = 0.5 * double(tan_off) / dr::tan(dr::deg_to_rad(fov_x) * 0.5);
                auto itrafo = to_world.inverse_transpose;
                itrafo.entry(3,0) += offset + m_cam_off.x();
                itrafo.entry(3,1) += m_cam_off.y();
                itrafo.entry(3,2) += m_cam_off.z();
                auto ntrafo = dr::inverse(dr::transpose(itrafo));
                ScalarTransform4f trafo(ntrafo, itrafo);
                //trafo.inverse_transpose = dr::inverse_transpose(trafo.matrix);
                sub_props.set_transform("to_world", trafo, false);
                sub_props.set_float("lens_shift", shift, false);
                m_sensors.emplace_back(dynamic_cast<Sensor_t *>(w_sens->create_instance().get()));
                Sensor_t* inst = m_sensors.back();
                if(!inst){
                    Throw("Sub-sensor instance wasn't created successfully !!");
                }
                inst->film()->set_size(sub_res);
                inst->parameters_changed();
            }
        }
        // Construct m_grid_size instances of a sensor on a line (for animations)
        else {
            for (uint32_t i = 0; i < m_grid_size; ++i) {
                ScalarFloat dt = i / ScalarFloat(m_grid_size - 1);
                ScalarVector3f offset   = m_cam_off + m_cam_dir * (m_cam_dist * (dt - 0.5f * m_cam_center));
                auto itrafo = to_world.inverse_transpose;
                itrafo.entry(3,0) += offset.x();
                itrafo.entry(3,1) += offset.y();
                itrafo.entry(3,2) += offset.z();
                auto ntrafo = dr::inverse(dr::transpose(itrafo));
                ScalarTransform4f trafo(ntrafo, itrafo);
                sub_props.set_transform("to_world", trafo, false);
                m_sensors.emplace_back(dynamic_cast<Sensor_t *>(w_sens->create_instance().get()));
                Sensor_t* inst = m_sensors.back();
                if(!inst){
                    Throw("Sub-sensor instance wasn't created successfully !!");
                }
                inst->film()->set_size(sub_res);
                inst->parameters_changed();
            }
        }

        m_needs_sample_3 = m_sensors[0]->needs_aperture_sample();
        m_film->set_size(m_film_res);
        this->parameters_changed();
        for (const auto &s : props.unqueried()) {
            props.mark_queried(s);
        }
        m_sensors_dr = dr::load<DynamicBuffer<SensorPtr>>(m_sensors.data(),
                                                          m_sensors.size());
    }

    std::pair<Ray3f, Spectrum> sample_ray(Float time, Float wavelength_sample,
                                          const Point2f &position_sample,
                                          const Point2f &aperture_sample,
                                          Mask active = true) const override {
        MI_MASKED_FUNCTION(ProfilerPhase::EndpointSampleRay, active);
        Vector2f idx_f = position_sample * Vector2f(m_grid_dim);
        Vector2u idx_u = Vector2f(idx_f);
        Vector2u idx   = idx_u;
        if (m_reverse.first)
            idx[0] = (m_grid_dim[0] - 1) - idx[0];
        if (m_reverse.second)
            idx[1] = (m_grid_dim[1] - 1) - idx[1];
        UInt32 index   = idx[0] + m_grid_dim[0] * idx[1];
        index          = dr::minimum(index, (uint32_t) (m_sensors.size() - 1));
        SensorPtr sensor = dr::gather<SensorPtr>(m_sensors_dr, index, active);
        Point2f position_sample_2 = idx_f - Vector2f(idx_u);

        auto [ray, spec] =
            sensor->sample_ray(time, wavelength_sample, position_sample_2,
                               aperture_sample, active);

        /* The `m_last_index` variable **needs** to be updated after the
         * virtual function call above. In recorded JIT modes, the tracing will
         * also cover this function and hence overwrite `m_last_index` as part
         * of that process. To "undo" that undesired side_effect, we must
         * update `m_last_index` after that virtual function call. */
        m_last_index = index;

        return { ray, spec };
    }

    std::tuple<Ray3f, Spectrum, UInt32> sample_ray_idx(
        Float time, Float wavelength_sample, const Point2f &position_sample,
        const Point2f &aperture_sample, Mask active = true) const override {
        // MI_MASKED_FUNCTION(ProfilerPhase::EndpointSampleRay, active);
        Vector2f idx_f = position_sample * Vector2f(m_grid_dim);
        Vector2u idx_u = Vector2f(idx_f);
        Vector2u idx   = idx_u;
        if (m_reverse.first)
            idx[0] = (m_grid_dim[0] - 1) - idx[0];
        if (m_reverse.second)
            idx[1] = (m_grid_dim[1] - 1) - idx[1];
        UInt32 index = idx[0] + m_grid_dim[0] * idx[1];
        index        = dr::minimum(index, (uint32_t) (m_sensors.size() - 1));
        SensorPtr sensor = dr::gather<SensorPtr>(m_sensors_dr, index, active);
        Point2f position_sample_2 = idx_f - Vector2f(idx_u);

        auto [ray, spec] =
            sensor->sample_ray(time, wavelength_sample, position_sample_2,
                               aperture_sample, active);

        /* The `m_last_index` variable **needs** to be updated after the
         * virtual function call above. In recorded JIT modes, the tracing will
         * also cover this function and hence overwrite `m_last_index` as part
         * of that process. To "undo" that undesired side_effect, we must
         * update `m_last_index` after that virtual function call. */
        m_last_index = index;

        return { ray, spec, index };
    }

    std::pair<RayDifferential3f, Spectrum> sample_ray_differential(
        Float time, Float wavelength_sample, const Point2f &position_sample,
        const Point2f &aperture_sample, Mask active) const override {

        MI_MASKED_FUNCTION(ProfilerPhase::EndpointSampleRay, active);

        Vector2f idx_f = position_sample * Vector2f(m_grid_dim);
        Vector2u idx_u = Vector2f(idx_f);
        Vector2u idx   = idx_u;
        if (m_reverse.first)
            idx[0] = (m_grid_dim[0] - 1) - idx[0];
        if (m_reverse.second)
            idx[1] = (m_grid_dim[1] - 1) - idx[1];
        UInt32 index   = idx[0] + m_grid_dim[0] * idx[1];
        index          = dr::minimum(index, (uint32_t) (m_sensors.size() - 1));
        SensorPtr sensor = dr::gather<SensorPtr>(m_sensors_dr, index, active);
        Point2f position_sample_2 = idx_f - Vector2f(idx_u);

        auto [ray, spec] = sensor->sample_ray_differential(
            time, wavelength_sample, position_sample_2, aperture_sample,
            active);

        /* The `m_last_index` variable **needs** to be updated after the
         * virtual function call above. In recorded JIT modes, the tracing will
         * also cover this function and hence overwrite `m_last_index` as part
         * of that process. To "undo" that undesired side_effect, we must
         * update `m_last_index` after that virtual function call. */
        m_last_index = index;

        return { ray, spec };
    }

    std::tuple<DirectionSample3f, Float, Bool, Mask>
    sample_surface(const Interaction3f &it, const Point2f &sample,
                   const UInt32 &idx, Mask active) const override {
        SensorPtr sensor = dr::gather<SensorPtr>(m_sensors_dr, idx, active);
        return sensor->sample_surface(it, sample, idx, active);
    }

    std::pair<DirectionSample3f, Spectrum>
    sample_direction(const Interaction3f &it, const Point2f &sample,
                     Mask active) const override {
        DirectionSample3f result_1 = dr::zeros<DirectionSample3f>();
        Spectrum result_2          = dr::zeros<Spectrum>();

        // The behavior of random sampling a sensor instead of
        // querying `m_last_index` is useful for ptracer rendering. But it
        // is not desired if we call `sensor.sample_direction()` to
        // re-attach gradients. We detect the latter case by checking if
        // `it` has gradient tracking enabled.

        if (dr::grad_enabled(it)) {
            for (size_t i = 0; i < m_sensors.size(); ++i) {
                Mask active_i = active && (m_last_index == i);
                auto [rv_1, rv_2] =
                    m_sensors[i]->sample_direction(it, sample, active_i);
                rv_1.uv.x() += i * m_sensors[i]->film()->size().x();
                result_1[active_i] = rv_1;
                result_2[active_i] = rv_2;
            }
        } else {
            // Randomly sample a valid connection to a sensor
            Point2f sample_(sample);
            UInt32 valid_count(0u);

            for (size_t i = 0; i < m_sensors.size(); ++i) {
                auto [rv_1, rv_2] =
                    m_sensors[i]->sample_direction(it, sample_, active);
                rv_1.uv.x() += i * m_sensors[i]->film()->size().x();

                Mask active_i = active && (rv_1.pdf != 0.f);
                valid_count += dr::select(active_i, 1u, 0u);

                // Should we put this sample into the reservoir?
                Float idx_f  = sample_.x() * valid_count;
                UInt32 idx_u = UInt32(idx_f);
                Mask accept  = active_i && (idx_u == valid_count - 1u);

                // Reuse sample_.x
                sample_.x() = dr::select(active_i, idx_f - idx_u, sample_.x());

                // Update the result
                result_1[accept] = rv_1;
                result_2[accept] = rv_2;
            }

            // Account for reservoir sampling probability
            Float reservoir_pdf =
                dr::select(valid_count > 0u, valid_count, 1u) /
                (ScalarFloat) m_sensors.size();
            result_1.pdf /= reservoir_pdf;
            result_2 *= reservoir_pdf;
        }

        return { result_1, result_2 };
    }

    Float pdf_direction(const Interaction3f &it, const DirectionSample3f &ds,
                        Mask active) const override {
        Float result = dr::zeros<Float>();
        for (size_t i = 0; i < m_sensors.size(); ++i) {
            Mask active_i = active && (m_last_index == i);
            dr::masked(result, active_i) =
                m_sensors[i]->pdf_direction(it, ds, active_i);
        }
        return result;
    }

    Spectrum eval(const SurfaceInteraction3f &si, Mask active) const override {
        Spectrum result = dr::zeros<Spectrum>();
        for (size_t i = 0; i < m_sensors.size(); ++i) {
            Mask active_i    = active && (m_last_index == i);
            result[active_i] = m_sensors[i]->eval(si, active_i);
        }
        return result;
    }

    ScalarBoundingBox3f bbox() const override {
        ScalarBoundingBox3f result;
        for (size_t i = 0; i < m_sensors.size(); ++i)
            result.expand(m_sensors[i]->bbox());
        return result;
    }

    void traverse(TraversalCallback *callback) override {
        Base::traverse(callback);
        std::string id;
        for (size_t i = 0; i < m_sensors.size(); i++) {
            id = m_sensors.at(i)->id();
            if (id.empty() || string::starts_with(id, "_unnamed_"))
                id = "sensor" + std::to_string(i);
            callback->put_object(id, m_sensors.at(i).get(),
                                 +ParamFlags::NonDifferentiable);
        }
    }

    std::pair<bool, bool> reverse_axis() const override { return m_reverse; }

    uint32_t n_sensors() const override { return m_grid_size; }

    ScalarVector2u grid_dim() const override { return m_grid_dim; }

    std::vector<Sensor_t *> sensors() override {
        std::vector<Sensor_t *> res;
        for (auto &sens : m_sensors)
            res.emplace_back(sens.get());
        return res;
    }

    std::vector<const Sensor_t *> sensors() const override {
        std::vector<const Sensor_t *> res;
        for (auto &sens : m_sensors)
            res.emplace_back(sens.get());
        return res;
    }

    SensorPtr gather(const UInt32 &idx, Mask active = true) const override {
        return dr::gather<SensorPtr>(m_sensors_dr, idx, active);
    }

    MI_DECLARE_CLASS()
private:
    std::vector<ref<Sensor_t>> m_sensors;
    DynamicBuffer<SensorPtr> m_sensors_dr;
    ScalarVector3f m_cam_dir;
    ScalarVector3f m_cam_off;
    ScalarVector2u m_grid_dim;
    ScalarVector2u m_film_res;
    ScalarFloat m_cam_dist;
    ScalarFloat m_cone_deg;
    ScalarFloat m_foc_dist;
    uint32_t m_grid_size;
    mutable UInt32 m_last_index;
    std::pair<bool, bool> m_reverse;
    bool m_cam_center = true;
    bool m_used_cone = false;
};

MI_IMPLEMENT_CLASS_VARIANT(GridSensor, MultiSensor)
MI_EXPORT_PLUGIN(GridSensor, "GridSensor");
NAMESPACE_END(mitsuba)
