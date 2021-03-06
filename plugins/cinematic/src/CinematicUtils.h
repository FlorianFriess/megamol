/*
 * CinematicUtils.h
 *
 * Copyright (C) 2019 by VISUS (Universitaet Stuttgart).
 * Alle Rechte vorbehalten.
 */

#ifndef MEGAMOL_CINEMATIC_CINEMATICUTILS_H_INCLUDED
#define MEGAMOL_CINEMATIC_CINEMATICUTILS_H_INCLUDED


#include "mmcore/view/RenderUtils.h"


// #### Utility minimal camera state ################################### //

typedef megamol::core::thecam::camera<cam_maths_type>::minimal_state_type camera_state_type;

static bool operator==(const camera_state_type& ls, const camera_state_type& rs) {
    return ((ls.centre_offset == rs.centre_offset) && (ls.convergence_plane == rs.convergence_plane) &&
            (ls.eye == rs.eye) && (ls.far_clipping_plane == rs.far_clipping_plane) && (ls.film_gate == rs.film_gate) &&
            (ls.gate_scaling == rs.gate_scaling) &&
            (ls.half_aperture_angle_radians == rs.half_aperture_angle_radians) &&
            (ls.half_disparity == rs.half_disparity) && (ls.image_tile == ls.image_tile) &&
            (ls.near_clipping_plane == rs.near_clipping_plane) && (ls.orientation == rs.orientation) &&
            (ls.position == rs.position) && (ls.projection_type == rs.projection_type) &&
            (ls.resolution_gate == rs.resolution_gate));
}

static bool operator!=(const camera_state_type& ls, const camera_state_type& rs) {
    return !(ls == rs);
}

namespace megamol {
namespace cinematic {

// ##################################################################### //
/*
 * Cinematic utility functionality (colors, text, menu, ...).
 */
class CinematicUtils : public core::view::RenderUtils {

public:
    CinematicUtils(void);

    ~CinematicUtils(void);

    enum Colors {
        BACKGROUND,
        FOREGROUND,
        KEYFRAME,
        KEYFRAME_DRAGGED,
        KEYFRAME_SELECTED,
        KEYFRAME_SPLINE,
        MENU,
        FONT,
        FONT_HIGHLIGHT,
        LETTER_BOX,
        FRAME_MARKER,
        MANIPULATOR_X,
        MANIPULATOR_Y,
        MANIPULATOR_Z,
        MANIPULATOR_VECTOR,
        MANIPULATOR_ROTATION,
        MANIPULATOR_CTRLPOINT
    };

    bool Initialise(megamol::core::CoreInstance* core_instance);

    const glm::vec4 Color(CinematicUtils::Colors color) const;

    void SetBackgroundColor(glm::vec4 bc) { this->background_color = bc; };

    void PushMenu(const std::string& left_label, const std::string& middle_label, const std::string& right_label,
        float viewport_width, float viewport_height);

    void PushHotkeyList(float viewport_width, float viewport_height);

    void PushText(const std::string& text, float x, float y, float z);

    void DrawAll(glm::mat4& mat_mvp, glm::vec2 dim_vp);

    float GetTextLineHeight(void);

    float GetTextLineWidth(const std::string& text_line);

    void SetTextRotation(float a, float x, float y, float z);

    bool Initialized(void) { return this->init_once; }

private:
    // VARIABLES ------------------------------------------------------- //

    megamol::core::utility::SDFFont font;
    const float font_size;
    bool init_once;
    glm::vec4 background_color;

    // FUNCTIONS ------------------------------------------------------- //

    const float lightness(glm::vec4 background) const;
};

} /* end namespace cinematic */
} /* end namespace megamol */

#endif // MEGAMOL_CINEMATIC_CINEMATICUTILS_H_INCLUDED
