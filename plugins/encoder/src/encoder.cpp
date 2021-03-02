/*
 * encoder.cpp
 * Copyright (C) 2009-2015 by MegaMol Team
 * Alle Rechte vorbehalten.
 */

// MegaMol includes
#include "mmcore/api/MegaMolCore.std.h"
#include "mmcore/utility/plugins/Plugin200Instance.h"
#include "mmcore/utility/plugins/PluginRegister.h"
#include "mmcore/versioninfo.h"
#include "vislib/vislibversion.h"

// Module includes
#include "CpuVideoEncoder.h"

namespace megamol::encoder {
    /** Implementing the instance class of this plugin */
    class plugin_instance : public ::megamol::core::utility::plugins::Plugin200Instance {
        REGISTERPLUGIN(plugin_instance)
    public:
        /** ctor */
        plugin_instance(void)
            : ::megamol::core::utility::plugins::Plugin200Instance(

                /* machine-readable plugin assembly name */
                "Encoder",

                /* human-readable plugin description */
                "Contains different CPU and GPU based video encoder implementations such as videolan x265, x264, "
                "NVENC, Intel Media SDK and AMD Advanced Media Framework SDK ") {

            // here we could perform addition initialization
        };
        /** Dtor */
        virtual ~plugin_instance(void) {
            // here we could perform addition de-initialization
        }
        /** Registers modules and calls */
        virtual void registerClasses(void) {

            // register modules here:
            this->module_descriptions.RegisterAutoDescription<megamol::encoder::CpuVideoEncoder>();
        }
    };
} // namespace megamol::encoder
