# The QRS plugin. Hosts key saved sessions on the manufacturer and plugin codes: never change them
# once released. Rflx is shared by every reflecticon plugin (the 224 first used it).
add_machine_plugin(qrs
    PRODUCT_NAME "ReflecticonQRS"
    PLUGIN_NAME "Reflecticon QRS"
    COMPANY_NAME "joelanders"
    BUNDLE_ID "net.joelanders.reflecticon.qrs"
    MANUFACTURER_CODE Rflx
    PLUGIN_CODE Qr82
    VST3_CATEGORIES Fx Reverb
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/plugin/qrs_plugin.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/plugin/qrs_machine.cpp)

# check_machine ROM_DIR: readouts, the factory programs' reverb times, an impulse through the host,
# the panel, the kit's host checks, a live change, cost. JUCE-free.
add_executable(qrs_check_machine tests/check_machine.cpp plugin/qrs_machine.cpp)
target_compile_options(qrs_check_machine PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/O2,-O2 -g -Wall>)
