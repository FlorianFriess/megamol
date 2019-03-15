/*
 * TransferFunctionEditor.cpp
 *
 * Copyright (C) 2019 by Universitaet Stuttgart (VIS).
 * Alle Rechte vorbehalten.
 */

#include "stdafx.h"
#include "TransferFunctionEditor.h"


using namespace megamol::gui;
using namespace megamol::core;


/**
 * Ctor
 */
megamol::gui::TransferFunctionEditor::TransferFunctionEditor(void)
    : data()
    , interpol_mode(param::TransferFunctionParam::InterpolationMode::LINEAR)
    , tex_size(128)
    , tex_data()
    , tex_modified(false)
    , plot_channels{false, false, false, true}
    , point_select_node(0)
    , point_select_chan(0)
    , imm_apply(false) {

    // Init transfer function colors
    if (this->data.size() < 2) {
        this->data.clear();
        std::array<float, 5> zero = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        std::array<float, 5> one = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        this->data.emplace_back(zero);
        this->data.emplace_back(one);
        this->tex_modified = true;
    }
}


/**
 * Dtor
 */
megamol::gui::TransferFunctionEditor::~TransferFunctionEditor(void) {

    // nothing to do here ...
}


/**
 * TransferFunctionEditor::SetTransferFunction
 */
bool megamol::gui::TransferFunctionEditor::SetTransferFunction(const std::string& in_tfs) {

    return megamol::core::param::TransferFunctionParam::ParseTransferFunction(
        in_tfs, this->data, this->interpol_mode, this->tex_size);
}


/**
 * TransferFunctionEditor::GetTransferFunction
 */
bool megamol::gui::TransferFunctionEditor::GetTransferFunction(std::string& in_tfs) {

    return megamol::core::param::TransferFunctionParam::DumpTransferFunction(
        in_tfs, this->data, this->interpol_mode, this->tex_size);
}


/**
 * TransferFunctionEditor::DrawTransferFunctionEditor
 */
bool megamol::gui::TransferFunctionEditor::DrawTransferFunctionEditor(void) {

    assert(ImGui::GetCurrentContext() != nullptr);
    ImGuiIO& io = ImGui::GetIO();
    ImGuiStyle& style = ImGui::GetStyle();

    const float tfw_height = 28.0f;
    const float tfw_item_width = ImGui::GetContentRegionAvailWidth() * 0.75f;
    const float canvas_height = 150.0f;
    const float canvas_width = tfw_item_width;
    ImGui::PushItemWidth(tfw_item_width); // set general proportional item width

    // Check for required initial node data
    assert(this->data.size() > 1);

    // Select color channels
    ImGui::Checkbox("Red", &this->plot_channels[0]);
    ImGui::SameLine();
    ImGui::Checkbox("Green", &this->plot_channels[1]);
    ImGui::SameLine();
    ImGui::Checkbox("Blue", &this->plot_channels[2]);
    ImGui::SameLine();
    ImGui::Checkbox("Alpha", &this->plot_channels[3]);
    ImGui::SameLine(tfw_item_width + style.ItemSpacing.x + style.ItemInnerSpacing.x);
    ImGui::Text("Color Channels");

    // Plot
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos(); // ImDrawList API uses screen coordinates!
    ImVec2 canvas_size = ImVec2(canvas_width, canvas_height);
    if (canvas_size.x < 50.0f) canvas_size.x = 100.0f;
    if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
    ImVec2 mouse_cur_pos = io.MousePos; // current mouse position

    ImVec4 tmp_frame_back_col = style.Colors[ImGuiCol_FrameBg];
    tmp_frame_back_col.w = 1.0f;
    ImU32 frame_back_col = ImGui::ColorConvertFloat4ToU32(tmp_frame_back_col);
    ImU32 frame_border_col = ImGui::ColorConvertFloat4ToU32(style.Colors[ImGuiCol_Border]);

    const float point_radius = 10.0f;
    const float point_border = 4.0f;
    const int circle_subdiv = 12;
    ImVec2 delta_border = style.ItemInnerSpacing;

    draw_list->AddRectFilledMultiColor(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        frame_back_col, frame_back_col, frame_back_col, frame_back_col);
    draw_list->AddRect(
        canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), frame_border_col);

    draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        true); // clip lines within the canvas (if we resize it, etc.)

    int selected_node = -1;
    int selected_chan = -1;
    for (int i = 0; i < this->data.size(); ++i) {
        ImU32 point_col = ImGui::ColorConvertFloat4ToU32(
            ImVec4(this->data[i][0], this->data[i][1], this->data[i][2], this->data[i][3]));

        // For each color channel
        ImU32 line_col = IM_COL32(255, 255, 255, 255);
        for (int c = 0; c < 4; ++c) {
            switch (c) {
            case (0):
                if (!this->plot_channels[0]) {
                    continue;
                }
                line_col = IM_COL32(255, 0, 0, 255);
                break;
            case (1):
                if (!this->plot_channels[1]) {
                    continue;
                }
                line_col = IM_COL32(0, 255, 0, 255);
                break;
            case (2):
                if (!this->plot_channels[2]) {
                    continue;
                }
                line_col = IM_COL32(0, 0, 255, 255);
                break;
            case (3):
                if (!this->plot_channels[3]) {
                    continue;
                }
                line_col = IM_COL32(255, 255, 255, 255);
                break;
            }

            ImVec2 point_cur_pos = ImVec2(canvas_pos.x + this->data[i][4] * canvas_size.x,
                canvas_pos.y + (1.0f - this->data[i][c]) * canvas_size.y);

            if (i < (this->data.size() - 1)) {
                ImVec2 point_next_pos = ImVec2(canvas_pos.x + this->data[i + 1][4] * canvas_size.x,
                    canvas_pos.y + (1.0f - this->data[i + 1][c]) * canvas_size.y);

                if (this->interpol_mode == param::TransferFunctionParam::InterpolationMode::LINEAR) {
                    draw_list->AddLine(point_cur_pos, point_next_pos, line_col, 4.0f);
                } else if (this->interpol_mode == param::TransferFunctionParam::InterpolationMode::GAUSS) {


                    // TODO: Implement ...
                }
            }

            ImU32 point_border_col = ImGui::ColorConvertFloat4ToU32(style.Colors[ImGuiCol_TextDisabled]);
            if (i == this->point_select_node) {
                point_border_col = ImGui::ColorConvertFloat4ToU32(style.Colors[ImGuiCol_ButtonActive]);
            }

            draw_list->AddCircleFilled(point_cur_pos, point_radius, frame_back_col, circle_subdiv);
            float point_radius_full = point_radius + point_border - 2.0f;
            draw_list->AddCircle(point_cur_pos, point_radius_full, point_border_col, circle_subdiv, point_border);
            draw_list->AddCircleFilled(point_cur_pos, point_radius, point_col, 12);

            ImVec2 delta_vec = ImVec2(point_cur_pos.x - mouse_cur_pos.x, point_cur_pos.y - mouse_cur_pos.y);
            if (sqrtf((delta_vec.x * delta_vec.x) + (delta_vec.y * delta_vec.y)) <= point_radius_full) {
                selected_node = i;
                selected_chan = c;
            }
        }
    }
    draw_list->PopClipRect();

    ImGui::InvisibleButton("plot", canvas_size);
    if (ImGui::IsItemHovered() && (mouse_cur_pos.x > (canvas_pos.x - delta_border.x)) &&
        (mouse_cur_pos.y > (canvas_pos.y - delta_border.y)) &&
        (mouse_cur_pos.x < (canvas_pos.x + canvas_size.x + delta_border.x)) &&
        (mouse_cur_pos.y < (canvas_pos.y + canvas_size.y + delta_border.y))) {

        if (io.MouseClicked[0]) { // Left Click -> Change selected node selecteed node
            if (selected_node >= 0) {
                this->point_select_node = selected_node;
                this->point_select_chan = selected_chan;
            }
        } else if (io.MouseDown[0]) { // Left Move -> Move selecteed node

            float new_x = (mouse_cur_pos.x - canvas_pos.x) / canvas_size.x;
            new_x = std::max(0.0f, std::min(new_x, 1.0f));
            if (this->point_select_node == 0) {
                new_x = 0.0f;
            } else if (this->point_select_node == (this->data.size() - 1)) {
                new_x = 1.0f;
            } else if ((new_x <= this->data[this->point_select_node - 1][4]) ||
                       (new_x >= this->data[this->point_select_node + 1][4])) {
                new_x = this->data[this->point_select_node][4];
            }
            this->data[this->point_select_node][4] = new_x;

            float new_y = 1.0f - ((mouse_cur_pos.y - canvas_pos.y) / canvas_size.y);
            new_y = std::max(0.0f, std::min(new_y, 1.0f));

            if (this->plot_channels[0] && (this->point_select_chan == 0)) {
                this->data[this->point_select_node][0] = new_y;
            }
            if (this->plot_channels[1] && (this->point_select_chan == 1)) {
                this->data[this->point_select_node][1] = new_y;
            }
            if (this->plot_channels[2] && (this->point_select_chan == 2)) {
                this->data[this->point_select_node][2] = new_y;
            }
            if (this->plot_channels[3] && (this->point_select_chan == 3)) {
                this->data[this->point_select_node][3] = new_y;
            }
            this->tex_modified = true;

        } else if (io.MouseClicked[1]) { // Right Click -> Add/delete Node

            if (selected_node < 0) { // Add new at current position
                float new_x = (mouse_cur_pos.x - canvas_pos.x) / canvas_size.x;
                new_x = std::max(0.0f, std::min(new_x, 1.0f));

                float new_y = 1.0f - ((mouse_cur_pos.y - canvas_pos.y) / canvas_size.y);
                new_y = std::max(0.0f, std::min(new_y, 1.0f));

                for (auto it = this->data.begin(); it != this->data.end(); ++it) {
                    if (new_x < (*it)[4]) {
                        // New nodes can only be inserted between two exisintng ones,
                        // so there is always a node before and after
                        std::array<float, 5> prev_col = (*(it - 1));
                        std::array<float, 5> fol_col = (*it);
                        std::array<float, 5> new_col = {(prev_col[0] + fol_col[0]) / 2.0f,
                            (prev_col[1] + fol_col[1]) / 2.0f, (prev_col[2] + fol_col[2]) / 2.0f,
                            (prev_col[3] + fol_col[3]) / 2.0f, new_x};

                        if (this->plot_channels[0]) {
                            new_col[0] = new_y;
                        }
                        if (this->plot_channels[1]) {
                            new_col[1] = new_y;
                        }
                        if (this->plot_channels[2]) {
                            new_col[2] = new_y;
                        }
                        if (this->plot_channels[3]) {
                            new_col[3] = new_y;
                        }
                        this->data.insert(it, new_col);
                        this->tex_modified = true;
                        break;
                    }
                }
            } else { // Delete currently hovered
                if ((selected_node > 0) && (selected_node < (this->data.size() - 1))) {
                    this->data.erase(this->data.begin() + selected_node);
                    this->point_select_node = (unsigned int)std::max(0, (int)this->point_select_node - 1);
                    this->tex_modified = true;
                }
            }
        }
    }
    if (this->tex_modified) {
        std::sort(this->data.begin(), this->data.end(),
            [](const std::array<float, 5>& lhs, const std::array<float, 5>& rhs) { return (lhs[4] < rhs[4]); });
    }
    ImGui::SameLine(0.0f, style.ItemInnerSpacing.x);
    ImGui::Text("Transfer Function");
    this->HelpMarkerToolTip("[Left-Click] Select Node\n[Left-Drag] Move Node\n[Right-Click] Add/Delete Node");

    // Value slider
    float value = this->data[this->point_select_node][4];
    if (ImGui::SliderFloat("Selected Value", &value, 0.0f, 1.0f)) {
        float new_x = value;
        new_x = std::max(0.0f, std::min(new_x, 1.0f));
        if (this->point_select_node == 0) {
            new_x = 0.0f;
        } else if (this->point_select_node == (this->data.size() - 1)) {
            new_x = 1.0f;
        } else if ((new_x <= this->data[this->point_select_node - 1][4]) ||
                   (new_x >= this->data[this->point_select_node + 1][4])) {
            new_x = this->data[this->point_select_node][4];
        }
        this->data[this->point_select_node][4] = new_x;
        this->tex_modified = true;
    }
    std::string help = "[Ctrl-Click] for keyboard input";
    this->HelpMarkerToolTip(help);

    // Edit Color of selected node
    float edit_col[4] = {this->data[this->point_select_node][0], this->data[this->point_select_node][1],
        this->data[this->point_select_node][2], this->data[this->point_select_node][3]};
    if (ImGui::ColorEdit4("Selected Color", edit_col)) {
        this->data[this->point_select_node][0] = edit_col[0];
        this->data[this->point_select_node][1] = edit_col[1];
        this->data[this->point_select_node][2] = edit_col[2];
        this->data[this->point_select_node][3] = edit_col[3];
        this->tex_modified = true;
    }
    help = "[Click] on the colored square to open a color picker.\n"
           "[CTRL+Click] on individual component to input value.\n"
           "[Right-Click] on the individual color widget to show options.";
    this->HelpMarkerToolTip(help);

    // Create current texture data
    bool imm_apply_tex_changed = this->tex_modified;
    if (this->tex_modified) {
        this->tex_data.resize(this->tex_size);
        std::array<float, 5> cx1 = this->data[0];
        std::array<float, 5> cx2 = this->data[0];
        int p1 = 0;
        int p2 = 0;
        size_t col_cnt = this->data.size();
        float r, g, b, a;
        // CHOOSE LINEAR INTERPOLATION
        if (this->interpol_mode == param::TransferFunctionParam::InterpolationMode::LINEAR) {
            for (size_t i = 0; i < col_cnt; ++i) {
                cx1 = cx2;
                p1 = p2;
                cx2 = this->data[i];
                assert(cx2[4] <= 1.0f + 1e-5f);
                p2 = static_cast<int>(cx2[4] * static_cast<float>(this->tex_size - 1));
                assert(p2 < static_cast<int>(this->tex_size));
                assert(p2 >= p1);

                for (int p = p1; p <= p2; p++) {
                    float al = static_cast<float>(p - p1) / static_cast<float>(p2 - p1);
                    float be = 1.0f - al;

                    r = cx1[0] * be + cx2[0] * al;
                    g = cx1[1] * be + cx2[1] * al;
                    b = cx1[2] * be + cx2[2] * al;
                    a = cx1[3] * be + cx2[3] * al;
                    this->tex_data[p] = ImVec4(r, g, b, a);
                }
            }
        } else if (this->interpol_mode == param::TransferFunctionParam::InterpolationMode::GAUSS) {
            // TODO: Implement ....
        }

        this->tex_modified = false;
    }

    // Draw current transfer function texture
    const float texture_height = 30.0f;
    size_t tex_cnt = this->tex_data.size();
    ImVec2 texture_pos = ImGui::GetCursorScreenPos();
    ImVec2 rect_size = ImVec2(tfw_item_width / (float)tex_cnt, texture_height);
    ImGui::InvisibleButton("texture", ImVec2(tfw_item_width, rect_size.y));
    for (int i = 0; i < tex_cnt; ++i) {
        ImU32 rect_col = ImGui::ColorConvertFloat4ToU32(this->tex_data[i]);
        ImVec2 rect_pos_a = ImVec2(texture_pos.x + (float)i * rect_size.x, texture_pos.y);
        ImVec2 rect_pos_b = ImVec2(rect_pos_a.x + rect_size.x, rect_pos_a.y + rect_size.y);
        draw_list->AddRectFilled(rect_pos_a, rect_pos_b, rect_col, 0.0f, 10);
    }
    ImGui::SameLine(0.0f, style.ItemInnerSpacing.x);
    ImGui::Text("1D Texture");

    // Get texture size
    int tfw_texsize = (int)this->tex_size;
    if (ImGui::InputInt("Texture Size", &tfw_texsize, 1, 10, ImGuiInputTextFlags_EnterReturnsTrue)) {
        this->tex_size = (size_t)std::max(1, tfw_texsize);
        this->tex_modified = true; /// Changes are applied in next frame
    }

    // Select interpolation mode (Linear, Gauss, ...)
    std::map<param::TransferFunctionParam::InterpolationMode, std::string> opts;
    opts[param::TransferFunctionParam::InterpolationMode::LINEAR] = "Linear";
    // opts[InterpolMode::GAUSS] = "Gauss";
    if (ImGui::BeginCombo("Interpolation", opts[this->interpol_mode].c_str())) {
        for (int i = 0; i < opts.size(); ++i) {
            if (ImGui::Selectable(opts[(param::TransferFunctionParam::InterpolationMode)i].c_str(),
                    (this->interpol_mode == (param::TransferFunctionParam::InterpolationMode)i))) {
                this->interpol_mode = (param::TransferFunctionParam::InterpolationMode)i;
                this->tex_modified = true; /// Changes are applied in next frame
            }
        }
        ImGui::EndCombo();
    }

    // Apply current changes
    bool ret_val = false;
    if (ImGui::Button("Apply Changes")) {
        ret_val = true;
    }
    ImGui::SameLine();
    // Auto apply changes
    ImGui::Checkbox("Immediately Apply all Changes", &this->imm_apply);
    if (this->imm_apply && imm_apply_tex_changed) {
        ret_val = true;
    }

    return ret_val;
}
