// The Quantec QRS as a kit::Machine: qrs::Unit (core/qrs.hpp, the real-time port of the QRS project's
// model) behind a rate bridge at 80 kHz (4 x the unit's 20 kHz), with the front panel's eight
// parameters, FREEZE / ENTER / MUTE, and the factory programs of files 7 and 8 as presets.
//
// The unit's controller (COP) turns the panel into register values; here the same mapping is done in
// C++ (qrs::record_settings, gate-checked in the project against the COP's own microcode), so the
// COP ROM is not needed. ENHANCE is not modelled (its key is drawn only).
#pragma once
#include "../../shared/core/machine.hpp"

namespace qrsplug {

const kit::MachineType& machine_type();

}  // namespace qrsplug
