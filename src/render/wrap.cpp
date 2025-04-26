/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include <mitsuba/render/wrap.h>


NAMESPACE_BEGIN(mitsuba)

MI_VARIANT Wrap<Float, Spectrum>::Wrap(const Properties &props) : Object(), m_props(props) {
    m_wrap_class = m_props.string("wrap_class");
    m_wrap_type = m_props.string("wrap_type");
    m_props.remove_property("wrap_class");
    m_props.remove_property("wrap_type");
    m_class_ptr = xml::detail::find_class(m_wrap_class, detail::get_variant<Float, Spectrum>());
    m_props.set_plugin_name(m_wrap_type);
    for(const auto &prop : props.unqueried()){
        props.mark_queried(prop);
    }
}

MI_VARIANT Wrap<Float, Spectrum>::~Wrap() { }

MI_VARIANT ref<Object> Wrap<Float, Spectrum>::create_instance() const {
    return PluginManager::instance()->create_object(Properties(m_props), m_class_ptr);
}

MI_VARIANT void Wrap<Float, Spectrum>::traverse(TraversalCallback *) {}

MI_VARIANT void Wrap<Float, Spectrum>::parameters_changed(const std::vector<std::string> &) {}

MI_VARIANT std::string Wrap<Float, Spectrum>::to_string() const {
    std::ostringstream oss;
    oss << "Wrap: " << m_wrap_class << '.' <<  m_wrap_type << "\n";
    oss << m_props;
    return oss.str();
}


MI_IMPLEMENT_CLASS_VARIANT(Wrap, Object, "wrap")
MI_INSTANTIATE_CLASS(Wrap)

NAMESPACE_END(mitsuba)
