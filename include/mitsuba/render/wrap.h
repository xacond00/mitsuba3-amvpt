#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include <mitsuba/core/object.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/properties.h>

NAMESPACE_BEGIN(mitsuba)

/** \brief An abstract wrapper class used to temporarily store properties of objects. 
 * Can defer instantiation of stored objects into contructors of other objects.
 */
template <typename Float, typename Spectrum>
class MI_EXPORT_LIB Wrap : public Object {
public:
    /// Destructor
    ~Wrap();
    /// Returns a new instance of stored object
    ref<Object> create_instance() const;
    // =============================================================
    //! @{ \name Accessor functions
    // =============================================================

    /// Return the stored properties (const version)
    const Properties &properties() const {return m_props;}
    /// Return the stored properties (non const version)
    Properties &properties() {return m_props;}
    /// Return the stored class ptr (const version)
    const Class *class_ptr() const {return m_class_ptr;}
    /// Return the stored class name (const version)
    const std::string &wrap_class() const {return m_wrap_class;}
    /// Return the stored class name (non const version)
    std::string &wrap_class() {return m_wrap_class;}
    /// Return the stored plugin name (const version)
    const std::string &wrap_type() const {return m_wrap_type;}
    /// Return the stored plugin name (non const version)
    std::string &wrap_type() {return m_wrap_type;}
    void traverse(TraversalCallback *callback) override;
    void parameters_changed(const std::vector<std::string> &/*keys*/ = {}) override;

    //! @}
    // =============================================================

    std::string to_string() const override;

    MI_DECLARE_CLASS()
protected:
    /// Create a Wrap
    Wrap(const Properties &props);
    inline Wrap(){}

    Properties m_props;
    const Class *m_class_ptr;
    std::string m_wrap_class;
    std::string m_wrap_type;
};

MI_EXTERN_CLASS(Wrap)
NAMESPACE_END(mitsuba)
