set(MI_PLUGIN_PREFIX "integrators")

add_plugin(aov        aov.cpp)
add_plugin(depth      depth.cpp)
add_plugin(direct     direct.cpp)
add_plugin(moment     moment.cpp)
add_plugin(path       path.cpp)
add_plugin(mvpath       mvpath.cpp)
add_plugin(ptracer    ptracer.cpp)
add_plugin(stokes     stokes.cpp)
add_plugin(volpath    volpath.cpp)
add_plugin(volpathmis volpathmis.cpp)

set(MI_PLUGIN_TARGETS "${MI_PLUGIN_TARGETS}" PARENT_SCOPE)
