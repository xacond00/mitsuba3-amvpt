#include <mitsuba/core/properties.h>
#include <mitsuba/render/fwd.h>
#include <mitsuba/render/wrap.h>
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

NAMESPACE_BEGIN(mitsuba)

/**!



Any wrapper (:monosp:`any`)
--------------------------------------------

.. pluginparameters::

 * - none
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
