/*
 * MegaMolPlugin.cpp
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
#include "MpiIntraCommunicator.h"

// Call includes
#include "MpiIntraCall.h"

namespace megamol::MegaMolPlugin {
    /** Implementing the instance class of this plugin */
    class plugin_instance : public ::megamol::core::utility::plugins::Plugin200Instance {
        REGISTERPLUGIN(plugin_instance)
    public:
        /** ctor */
        plugin_instance(void)
            : ::megamol::core::utility::plugins::Plugin200Instance(

                /* machine-readable plugin assembly name */
                "Network",

                /* human-readable plugin description */
                "Provides an MPI module that allows communications between modules on different "
                "machines without using calls. Additionally an UDP based ethernet module that "
                "can send and receive data via IPv4.") {

            // here we could perform addition initialization
        };
        /** Dtor */
        virtual ~plugin_instance(void) {
            // here we could perform addition de-initialization
        }
        /** Registers modules and calls */
        virtual void registerClasses(void) {
            // register modules here:
            this->module_descriptions.RegisterAutoDescription<megamol::network::MpiIntraCommunicator>();

            // register calls here:
            this->call_descriptions.RegisterAutoDescription<megamol::network::MpiIntraCall>();
        }
    };
} // namespace megamol::MegaMolPlugin
