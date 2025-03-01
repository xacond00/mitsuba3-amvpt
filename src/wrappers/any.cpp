#include <mitsuba/core/properties.h>
#include <mitsuba/render/fwd.h>
#include <mitsuba/render/wrap.h>

NAMESPACE_BEGIN(mitsuba)

/**!

.. _sensor-multi:

Any wrapper (:monosp:`multi`)
--------------------------------------------

.. pluginparameters::

 * - none
   - |none|
   - Pure nothing

.. tabs::
    .. code-tab:: xml
        :name: multi-sensor

        <wrap type="uni">
            <!-- Two perpendicular viewing directions -->
            <sensor name="sensor_1" type="perspective">
                <float name="fov" value="45"/>
                <transform name="to_world">
                    <lookat origin="0, 0, 1" target="0, 0, 0" up="0, 1, 0"/>
                </transform>
            </sensor>
            <sensor name="sensor_2" type="perspective">
                <float name="fov" value="45"/>
                <transform name="to_world">
                    <look_at origin="1, 0, 0" target="1, 2, 1" up="0, 1, 0"/>
                </transform>
            </sensor>
            <!-- film -->
            <!-- sampler -->
        </sensor>

    .. code-tab:: python

        'type': 'multi',
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
        'sensor2': {
            'type': 'perspective',
            'fov': 45,
            'to_world': mi.ScalarTransform4f.look_at(
                origin=[1, 0, 0],
                target=[0, 0, 0],
                up=[0, 1, 0]
            ),
        }
        'film_id': {
            'type': '<film_type>',
            # ...
        },
        'sampler_id': {
            'type': '<sampler_type>',
            # ...
        }
*/

MI_VARIANT class AnyWrap final : public Wrap<Float, Spectrum> {
public:
    MI_IMPORT_BASE(Wrap)

    AnyWrap(const Properties &props) : Base(props) {}

    
    void traverse(TraversalCallback *callback) override {
        Base::traverse(callback);
    }
    
    MI_DECLARE_CLASS()
private:
};

MI_IMPLEMENT_CLASS_VARIANT(AnyWrap, Wrap)
MI_EXPORT_PLUGIN(AnyWrap, "AnyWrap");
NAMESPACE_END(mitsuba)
