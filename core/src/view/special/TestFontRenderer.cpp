/*
* TestFontRenderer.cpp
*
* Copyright (C) 2010 by VISUS (Universitaet Stuttgart)
* Alle Rechte vorbehalten.
*/

#include "stdafx.h"

#include "mmcore/view/special/TestFontRenderer.h"

#include "mmcore/utility/ResourceWrapper.h"
#include "mmcore/misc/PngBitmapCodec.h"
#include "mmcore/view/CallRender3D.h"
#include "mmcore/CoreInstance.h"
#include "mmcore/view/AbstractView3D.h"
#include "mmcore/view/special/Verdana.inc"

#include "mmcore/param/FilePathParam.h"
#include "mmcore/param/FloatParam.h"
#include "mmcore/param/ButtonParam.h"

#include "vislib/graphics/gl/Verdana.inc"
#include "vislib/graphics/gl/IncludeAllGL.h"
#include "vislib/sys/Log.h"
#include "vislib/sys/File.h"
#include "vislib/sys/ASCIIFileBuffer.h"

using namespace megamol;
using namespace megamol::core;
using namespace megamol::core::view;
using namespace megamol::core::view::special;


/*
* TestFontRenderer::TestFontRenderer
*/
TestFontRenderer::TestFontRenderer(void) : Renderer3DModule(),
    simpleFont(),
    outlineFont(vislib::graphics::gl::FontInfo_Verdana, vislib::graphics::gl::OutlineFont::RENDERTYPE_FILL),
    sdfFont(SDFFont::BitmapFont::VERDANA),
    //sdfFont(SDFFont::BitmapFont::EVOLVENTA),
    paramRenderMode1("01_RenderMode-01", "... ."),
    paramRenderMode2("02_RenderMode-02", "... ."),
    paramRenderMode3("03_RenderMode-03", "... ."),
    paramRenderMode4("04_RenderMode-04", "... ."),
    paramRenderMode5("05_RenderMode-05", "... ."),
    paramRenderMode6("06_RenderMode-06", "... ."),
    paramRenderMode7("07_RenderMode-07", "... ."),
    paramRenderMode8("08_RenderMode-08", "... ."),
    paramRenderMode9("09_RenderMode-09", "... .")
    {

    //this->sdfFont.SetRenderType(SDFFont::RenderType::RENDERTYPE_OUTLINE);
    this->sdfFont.SetRenderType(SDFFont::RenderType::RENDERTYPE_FILL);
    //this->sdfFont.SetRenderType(SDFFont::RenderType::RENDERTYPE_NONE);

    this->paramRenderMode1.SetParameter(new param::ButtonParam('1'));
    this->MakeSlotAvailable(&this->paramRenderMode1);

    this->paramRenderMode2.SetParameter(new param::ButtonParam('2'));
    this->MakeSlotAvailable(&this->paramRenderMode2);

    this->paramRenderMode3.SetParameter(new param::ButtonParam('3'));
    this->MakeSlotAvailable(&this->paramRenderMode3);

    this->paramRenderMode4.SetParameter(new param::ButtonParam('4'));
    this->MakeSlotAvailable(&this->paramRenderMode4);

    this->paramRenderMode5.SetParameter(new param::ButtonParam('5'));
    this->MakeSlotAvailable(&this->paramRenderMode5);

    this->paramRenderMode6.SetParameter(new param::ButtonParam('6'));
    this->MakeSlotAvailable(&this->paramRenderMode6);

    this->paramRenderMode7.SetParameter(new param::ButtonParam('7'));
    this->MakeSlotAvailable(&this->paramRenderMode7);

    this->paramRenderMode8.SetParameter(new param::ButtonParam('8'));
    this->MakeSlotAvailable(&this->paramRenderMode8);

    this->paramRenderMode9.SetParameter(new param::ButtonParam('9'));
    this->MakeSlotAvailable(&this->paramRenderMode9);

    this->renderMode = 7;
    this->testtext.Clear();
}


/*
* TestFontRenderer::TestFontRenderer
*/
TestFontRenderer::~TestFontRenderer(void) {
    this->Release();
}


/*
* TestFontRenderer::release
*/
void TestFontRenderer::release(void) {


}


/*
* TestFontRenderer::create
*/
bool TestFontRenderer::create(void) {

    // Initialise simple font
    if (!this->simpleFont.Initialise()) {
        vislib::sys::Log::DefaultLog.WriteError("[TestFontRenderer] [create] Couldn't initialize the simple font.");
        return false;
    }
    // Initialise outline font
    if (!this->outlineFont.Initialise()) {
        vislib::sys::Log::DefaultLog.WriteError("[TestFontRenderer] [create] Couldn't initialize the outline font.");
        return false;
    }
    // Initialise sdf font
    if (!this->sdfFont.Initialise()) {
        vislib::sys::Log::DefaultLog.WriteError("[TestFontRenderer] [create] Couldn't initialize the sdf font.");
        return false;
    }

    // Load file with test text -----------------------------------------------
    vislib::StringA filename = "sdffont_TESTTEXT";
    vislib::sys::ASCIIFileBuffer file;
    if (!file.LoadFile(filename)) {
        vislib::sys::Log::DefaultLog.WriteError("[TestFontRenderer] [create] Could not load file as ascii buffer: \"%s\". \n", filename.PeekBuffer());
        return false;
    }
    SIZE_T lineCnt = 0;
    // Read info file line by line
    this->testtext.Clear();
    while (lineCnt < file.Count()) {
        this->testtext += static_cast<vislib::StringA>(file.Line(lineCnt));
        lineCnt++;
    }
    file.Clear();
    this->testtext.Append('\0');

    return true;
}

/*
* TestFontRenderer::GetCapabilities
*/
bool TestFontRenderer::GetCapabilities(Call& call) {
    CallRender3D *cr3d = dynamic_cast<CallRender3D*>(&call);
    if (cr3d == NULL) return false;


    return true;
}

/*
* TestFontRenderer::GetExtents
*/
bool TestFontRenderer::GetExtents(core::Call& call) {

    megamol::core::view::CallRender3D *cr2d = dynamic_cast<view::CallRender3D*>(&call);
    if (cr2d == NULL) {
        vislib::sys::Log::DefaultLog.WriteError("[TestFontRenderer] [GetExtents] Call is NULL.");
        return false;
    }

    // Unused

    return true;
}


/*
* TestFontRenderer::Render
*/
bool TestFontRenderer::Render(core::Call& call) {

    megamol::core::view::CallRender3D *cr2d = dynamic_cast<megamol::core::view::CallRender3D*>(&call);
    if (cr2d == NULL) {
        vislib::sys::Log::DefaultLog.WriteError("[TestFontRenderer] [Render] Call is NULL.");
        return false;
    }

    if (this->paramRenderMode1.IsDirty()) {
        this->renderMode = 1;
        this->paramRenderMode1.ResetDirty();
    }
    if (this->paramRenderMode2.IsDirty()) {
        this->renderMode = 2;
        this->paramRenderMode2.ResetDirty();
    }
    if (this->paramRenderMode3.IsDirty()) {
        this->renderMode = 3;
        this->paramRenderMode3.ResetDirty();
    }
    if (this->paramRenderMode4.IsDirty()) {
        this->renderMode = 4;
        this->paramRenderMode4.ResetDirty();
    }
    if (this->paramRenderMode5.IsDirty()) {
        this->renderMode = 5;
        this->paramRenderMode5.ResetDirty();
    }
    if (this->paramRenderMode6.IsDirty()) {
        this->renderMode = 6;
        this->paramRenderMode6.ResetDirty();
    }
    if (this->paramRenderMode7.IsDirty()) {
        this->renderMode = 7;
        this->paramRenderMode7.ResetDirty();
    }
    if (this->paramRenderMode8.IsDirty()) {
        this->renderMode = 8;
        this->paramRenderMode8.ResetDirty();
    }
    if (this->paramRenderMode9.IsDirty()) {
        this->renderMode = 9;
        this->paramRenderMode9.ResetDirty();
    }

    // Get the foreground color (inverse background color)
    float bgColor[4];
    float fgColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glGetFloatv(GL_COLOR_CLEAR_VALUE, bgColor);
    for (unsigned int i = 0; i < 3; i++) {
        fgColor[i] -= bgColor[i];
    }
    glColor4fv(fgColor);

    // OpenGl setup -----------------------------------------------------------

    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisable(GL_CULL_FACE);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_1D);

    //glEnable(GL_CULL_FACE);
    //glCullFace(GL_BACK);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glDisable(GL_BLEND);

    //glEnable(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);

    vislib::StringA hallo = "AV Wa WA \n�������\n... Hallo Welt ...\n1234567890\n   Qualle  \n| � ^ � � |\n�������";
    float fontSize = 0.1f;
    float lineWidth = 8.0f;
    unsigned int lineCount = 0;

    // ------------------------------------------------------------------------
    if (this->renderMode == 1) {

        // Get current viewport
        int vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        int   vpWidth = vp[2] - vp[0];
        int   vpHeight = vp[3] - vp[1];
        float vpH      = static_cast<float>(vpHeight);
        float vpW      = static_cast<float>(vpWidth);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0.0f, vpW, 0.0f, vpH, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
 
        float fontSize = vpH*0.5f; // 50% of viewport height
        float nol      = 6.0f; // number of lines

        vislib::StringA simpleString  = "The Simple Font. ";
        vislib::StringA outlineString = "The Outline Font. ";
        vislib::StringA filledString  = "The Filled Font. ";
        vislib::StringA sdfString     = "The Filled Font. ";  // "The SDF Font. ";

        // Adapt font size
        vislib::StringA tmpString = "--------------------------";
        // Adapt to width
        float maxWidthFontSize = fontSize;
        while (this->simpleFont.LineWidth(maxWidthFontSize, tmpString) > vpW) {
            //maxWidthFontSize -= 0.1f;
            maxWidthFontSize -= 1.0f;
        }
        // Adapt to height 
        float maxHeightFontSize = fontSize;
        while (nol*this->simpleFont.LineHeight(maxHeightFontSize) > vpH) {
            //maxHeightFontSize -= 0.1f;
            maxHeightFontSize -= 1.0f;
        }
        fontSize = (maxWidthFontSize < maxHeightFontSize) ? (maxWidthFontSize) : (maxHeightFontSize);
        fontSize = (fontSize < 0.0f) ? (0.1f) : (fontSize);


        // SIMPLE FONT 
        float simpleWidth = this->simpleFont.LineWidth(fontSize, simpleString);
        this->simpleFont.DrawString(0.0f, vpH, simpleWidth, 1.0f, fontSize, true, simpleString, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);


        // OUTLINE FONT 
        this->outlineFont.SetRenderType(vislib::graphics::gl::OutlineFont::RENDERTYPE_OUTLINE);
        float outlineWidth = this->outlineFont.LineWidth(fontSize, outlineString);
        glDisable(GL_LINE_SMOOTH);
        this->outlineFont.DrawString(0.0f, vpH - (fontSize*1.0f), outlineWidth, 1.0f, fontSize, true, outlineString, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);

        
        outlineString += "AA ";
        outlineWidth = this->outlineFont.LineWidth(fontSize, outlineString);
        glEnable(GL_LINE_SMOOTH);
        this->outlineFont.DrawString(0.0f, vpH - (fontSize*2.0f), outlineWidth, 1.0f, fontSize, true, outlineString, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
        glDisable(GL_LINE_SMOOTH);

        // FILLED FONT 
        this->outlineFont.SetRenderType(vislib::graphics::gl::OutlineFont::RENDERTYPE_FILL);
        outlineWidth = this->outlineFont.LineWidth(fontSize, outlineString);
        glDisable(GL_POLYGON_SMOOTH);
        this->outlineFont.DrawString(0.0f, vpH - (fontSize*3.0f), outlineWidth, 1.0f, fontSize, true, filledString, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
        
        filledString += "AA ";
        outlineWidth = this->outlineFont.LineWidth(fontSize, filledString);
        glEnable(GL_POLYGON_SMOOTH);
        this->outlineFont.DrawString(0.0f, vpH - (fontSize*4.0f), outlineWidth, 1.0f, fontSize, true, filledString, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
        glDisable(GL_POLYGON_SMOOTH);

        // SDF FONT
        float sdfWidth = vpW; // this->sdfFont.LineWidth(fontSize, sdfString);
        this->sdfFont.DrawString(0.0f, vpH - (fontSize*5.0f), sdfWidth, 1.0f, fontSize, true, sdfString, megamol::core::view::special::AbstractFont::ALIGN_LEFT_TOP);

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 2) {

        lineCount = this->simpleFont.BlockLines(lineWidth, fontSize, testtext);

        this->simpleFont.DrawString(-(lineWidth/2.0f), ((lineCount*fontSize)/2.0f) - 1.0f, lineWidth, 1.0f, fontSize, true, testtext, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 3) {

        lineCount = this->outlineFont.BlockLines(lineWidth, fontSize, testtext);

        glEnable(GL_POLYGON_SMOOTH);
        //glDisable(GL_POLYGON_SMOOTH);
        this->outlineFont.DrawString(-(lineWidth / 2.0f), ((lineCount*fontSize) / 2.0f) - 1.0f, lineWidth, 1.0f, fontSize, true, testtext, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
        
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 4) {
        
        lineCount = this->sdfFont.BlockLines(lineWidth, fontSize, testtext);

        //glColor4fv(fgColor);
        //this->sdfFont.SetRenderType(SDFFont::RenderType::RENDERTYPE_FILL);

        this->sdfFont.DrawString(-(lineWidth / 2.0f), ((lineCount*fontSize) / 2.0f) - 1.0f, lineWidth, 1.0f, fontSize, true, testtext, megamol::core::view::special::AbstractFont::ALIGN_LEFT_TOP);

        /*
        glColor4f(0.0f, 0.0f, 1.0f, 1.0f); 
        this->sdfFont.SetRenderType(SDFFont::RenderType::RENDERTYPE_OUTLINE);
        this->sdfFont.DrawString(-(lineWidth / 2.0f), ((lineCount*fontSize) / 2.0f) - 1.0f, lineWidth, 1.0f, fontSize, true, testtext, megamol::core::view::special::AbstractFont::ALIGN_LEFT_TOP);
        */
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 5) {

       this->simpleFont.DrawString(-0.5f, -0.5f, lineWidth, 1.0f, fontSize, true, hallo, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 6) {

        glEnable(GL_POLYGON_SMOOTH);
        //glDisable(GL_POLYGON_SMOOTH);
        this->outlineFont.DrawString(-0.5f, -0.5f, lineWidth, 1.0f, fontSize, true, hallo, vislib::graphics::AbstractFont::ALIGN_LEFT_TOP);
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 7) {

        this->sdfFont.DrawString(-0.5f, -0.5f, lineWidth, 1.0f, fontSize, true, hallo, megamol::core::view::special::AbstractFont::ALIGN_LEFT_TOP);
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 8) {
    }
    // ------------------------------------------------------------------------
    else if (this->renderMode == 9) {
    }
    // ------------------------------------------------------------------------
    else {
        vislib::sys::Log::DefaultLog.WriteWarn("[TestFontRenderer] [render] Unknown render mode ...");
    }


    // Reset opengl -----------------------------------------------------------
    glDisable(GL_BLEND);
    glDisable(GL_POLYGON_SMOOTH);
    glDisable(GL_LINE_SMOOTH);

    return true;
}

