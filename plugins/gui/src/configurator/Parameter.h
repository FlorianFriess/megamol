/*
 * Parameter.h
 *
 * Copyright (C) 2019 by Universitaet Stuttgart (VIS).
 * Alle Rechte vorbehalten.
 */

#ifndef MEGAMOL_GUI_GRAPH_PARAMETER_H_INCLUDED
#define MEGAMOL_GUI_GRAPH_PARAMETER_H_INCLUDED


#include "mmcore/param/BoolParam.h"
#include "mmcore/param/ButtonParam.h"
#include "mmcore/param/ColorParam.h"
#include "mmcore/param/EnumParam.h"
#include "mmcore/param/FilePathParam.h"
#include "mmcore/param/FlexEnumParam.h"
#include "mmcore/param/FloatParam.h"
#include "mmcore/param/IntParam.h"
#include "mmcore/param/StringParam.h"
#include "mmcore/param/TernaryParam.h"
#include "mmcore/param/TransferFunctionParam.h"
#include "mmcore/param/Vector2fParam.h"
#include "mmcore/param/Vector3fParam.h"
#include "mmcore/param/Vector4fParam.h"

#include "vislib/sys/Log.h"

#include <map>
#include <variant>
#include <vector>

#include "GUIUtils.h"
#include "TransferFunctionEditor.h"

// Used for platform independent clipboard (ImGui so far only provides windows implementation)
#ifdef GUI_USE_GLFW
#    include "GLFW/glfw3.h"
#endif


namespace megamol {
namespace gui {
namespace configurator {


// Forward declaration
class Parameter;

// Pointer types to classes
typedef std::shared_ptr<Parameter> ParamPtrType;

/**
 * Defines parameter data structure for graph.
 */
class Parameter {
public:
    enum ParamType {
        BOOL,
        BUTTON,
        COLOR,
        ENUM,
        FILEPATH,
        FLEXENUM,
        FLOAT,
        INT,
        STRING,
        TERNARY,
        TRANSFERFUNCTION,
        VECTOR2F,
        VECTOR3F,
        VECTOR4F,
        UNKNOWN
    };

    struct StockParameter {
        std::string class_name;
        std::string description;
        Parameter::ParamType type;
        std::string value_string;
    };

    typedef std::variant<std::monostate,             // default  BUTTON
        bool,                                        // BOOL
        megamol::core::param::ColorParam::ColorType, // COLOR
        float,                                       // FLOAT
        int,                                         // INT      ENUM
        std::string,                                 // STRING   TRANSFERFUNCTION    FILEPATH    FLEXENUM
        vislib::math::Ternary,                       // TERNARY
        glm::vec2,                                   // VECTOR2F
        glm::vec3,                                   // VECTOR3F
        glm::vec4                                    // VECTOR4F
        >
        ValueType;

    typedef std::map<int, std::string> EnumStorageType;

    enum Presentations : size_t { DEFAULT = 0, SIMPLE = 1, _COUNT_ = 2 };

    Parameter(int uid, ParamType type);
    ~Parameter() {}

    const int uid;
    const ParamType type;

    std::string class_name;
    std::string description;

    std::string full_name;

    // Get ----------------------------------
    std::string GetValueString(void);

    ValueType& GetValue(void) { return this->value; }

    template <typename T> const T& GetMinValue(void) const {
        try {
            return std::get<T>(this->minval);
        } catch (std::bad_variant_access&) {
            /// XXX
        }
    }

    template <typename T> const T& GetMaxValue(void) const {
        try {
            return std::get<T>(this->maxval);
        } catch (std::bad_variant_access&) {
            /// XXX
        }
    }

    template <typename T> const T& GetStorage(void) const {
        try {
            return std::get<T>(this->storage);
        } catch (std::bad_variant_access&) {
            /// XXX
        }
    }

    // SET ----------------------------------
    template <typename T> void SetValue(T val) {
        if (std::holds_alternative<T>(this->value)) {
            this->value = val;
        } else {
            vislib::sys::Log::DefaultLog.WriteError(
                "Bad variant access. [%s, %s, line %d]\n", __FILE__, __FUNCTION__, __LINE__);
        }
    }

    template <typename T> void SetMinValue(T min) {
        if (std::holds_alternative<T>(this->minval)) {
            this->minval = min;
        } else {
            vislib::sys::Log::DefaultLog.WriteError(
                "Bad variant access. [%s, %s, line %d]\n", __FILE__, __FUNCTION__, __LINE__);
        }
    }

    template <typename T> void SetMaxValue(T max) {
        if (std::holds_alternative<T>(this->maxval)) {
            this->maxval = max;
        } else {
            vislib::sys::Log::DefaultLog.WriteError(
                "Bad variant access. [%s, %s, line %d]\n", __FILE__, __FUNCTION__, __LINE__);
        }
    }

    template <typename T> void SetStorage(T store) {
        if (std::holds_alternative<T>(this->storage)) {
            this->storage = store;
        } else {
            vislib::sys::Log::DefaultLog.WriteError(
                "Bad variant access. [%s, %s, line %d]\n", __FILE__, __FUNCTION__, __LINE__);
        }
    }

    // GUI Presentation -------------------------------------------------------

    bool GUI_Present(void) { return this->present.Present(*this); }
    void GUI_SetLabelVisibility(bool visible) { this->present.visible = visible; }
    void GUI_SetPresentation(Parameter::Presentations present) { this->present.presentations = present; }
    void GUI_SetReadOnly(bool readonly) { this->present.read_only = readonly; }

    static bool GUI_PresentationButton(Parameter::Presentations& inout_present, std::string label = "") {
        return Presentation::PresentationButton(inout_present, label);
    }

private:
    typedef std::variant<std::monostate, // default (unused/unavailable)
        float,                           // FLOAT
        int,                             // INT
        glm::vec2,                       // VECTOR_2f
        glm::vec3,                       // VECTOR_3f
        glm::vec4                        // VECTOR_4f
        >
        MinType;

    typedef std::variant<std::monostate, // default (unused/unavailable)
        float,                           // FLOAT
        int,                             // INT
        glm::vec2,                       // VECTOR_2f
        glm::vec3,                       // VECTOR_3f
        glm::vec4                        // VECTOR_4f
        >
        MaxType;

    typedef std::variant<std::monostate,               // default (unused/unavailable)
        megamol::core::view::KeyCode,                  // BUTTON
        EnumStorageType,                               // ENUM
        megamol::core::param::FlexEnumParam::Storage_t // FLEXENUM
        >
        StroageType;

    MinType minval;
    MaxType maxval;
    StroageType storage;
    ValueType value;

    /**
     * Defines GUI parameter presentation.
     */
    class Presentation {
    public:
        Presentation(void);

        ~Presentation(void);

        bool Present(Parameter& param);

        Parameter::Presentations presentations;
        bool read_only;
        bool visible;

        static bool PresentationButton(Parameter::Presentations& inout_present, std::string label = "");

    private:
        std::string help;
        GUIUtils utils;
        TransferFunctionEditor tf_editor;
        bool show_tf_editor;

        std::map<std::string, std::string> widgtmap_text;
        std::map<std::string, int> widgtmap_int;
        std::map<std::string, float> widgtmap_float;
        std::map<std::string, glm::vec2> widgtmap_vec2;
        std::map<std::string, glm::vec3> widgtmap_vec3;
        std::map<std::string, glm::vec4> widgtmap_vec4;

        void present_prefix(Parameter& param);
        void present_value(Parameter& param);
        void present_postfix(Parameter& param);

        void transfer_function_edit(Parameter& param);

    } present;
};


} // namespace configurator
} // namespace gui
} // namespace megamol

#endif // MEGAMOL_GUI_GRAPH_PARAMETER_H_INCLUDED