/**
 * Entrypoint for Newton's Cradle simulation
 */

#include <chrono>
#include <bgfx/bgfx.h>

#include <bx/readerwriter.h>
#include <bx/fpumath.h>
#include <bx/string.h>

#include "common.h"

#include "bgfx_utils.h"
#include "bx/timer.h"
#include "imgui/imgui.h"

#include "math/matrix4.h"

using namespace altertum;

struct Settings
{
    uint32_t n_balls;

};

union M4
{
    Matrix4 m4;
    float f16[16];
};

static const bgfx::Memory* loadMem(bx::FileReaderI* _reader, const char* _filePath)
{
    if (0 == bx::open(_reader, _filePath) )
    {
        uint32_t size = (uint32_t)bx::getSize(_reader);
        const bgfx::Memory* mem = bgfx::alloc(size+1);
        bx::read(_reader, mem->data, size);
        bx::close(_reader);
        mem->data[mem->size-1] = '\0';
        return mem;
    }

    return NULL;
}

static bool is_running = false;
static char * run_text[4] = {"Running\0\0\0", "Running.\0\0", "Running..\0", "Running..."}; 
static double frametime;

Matrix4 * worlds;
size_t n_worlds = 5;

int _main_(int /* argc */, char** /* *argv[] */)
{
    /* windowing variables */
    uint32_t width = 1280;
    uint32_t height = 720;
    uint32_t debug = BGFX_DEBUG_TEXT;
    uint32_t reset = BGFX_RESET_VSYNC;
    uint32_t _frame_count = 0;

    /* set up bgfx */
    bgfx::init();
    bgfx::reset(width, height, reset);

    /* debug and clear info */
    bgfx::setDebug(debug);

    bgfx::setViewClear(0
        , BGFX_CLEAR_COLOR|BGFX_CLEAR_DEPTH
        , 0x303030ff
        , 1.0f
        );

    /* initialize imgui */
    imguiCreate();

    float duration = 10.0f;
    int64_t last = 0;
    int balls = 5;
    char duration_text[5] = "5";
    entry::MouseState mouseState;

    Mesh * mesh = meshLoad("newton.bin");

    bgfx::ShaderHandle vsh = bgfx::createShader(loadMem(entry::getFileReader(), "./vs_mesh.bin") );
    bgfx::ShaderHandle fsh = bgfx::createShader(loadMem(entry::getFileReader(), "./fs_mesh.bin") );
    bgfx::ProgramHandle program = bgfx::createProgram( vsh, fsh, true );

    Matrix4 view;
    float eye[3] = { 0.0f, -1.0f, -2.5f };
    float at[3]  = { 0.0f, -1.0f,  0.0f };
    bx::mtxLookAt((float *)&view, eye, at);

    Matrix4 proj;
    bx::mtxProj((float *)&proj, 60.0f, float(width)/float(height), 0.1f, 100.0f);

    bgfx::setViewTransform(0, &view, &proj);

    Matrix4 mtx;
    bx::mtxScale((float *)&mtx, 1.0f, 1.0f, 1.0f);

    bgfx::UniformHandle u_time = bgfx::createUniform("u_time", bgfx::UniformType::Vec4);
    float time = 0.0f;

    while ( !entry::processEvents(width, height, debug, reset, &mouseState) )
    {
        if (++_frame_count > 119) _frame_count = 0;

        /* calculate frame time */
        int64_t now = bx::getHPCounter();
        static int64_t last = now;
        const int64_t frameTime = now - last;
        last = now;
        const double freq = double(bx::getHPFrequency() );
        const double toMs = 1000.0/freq;

        /* begin imgui frame */
        imguiBeginFrame(mouseState.m_mx
            , mouseState.m_my
            , (mouseState.m_buttons[entry::MouseButton::Left  ] ? IMGUI_MBUT_LEFT   : 0)
            | (mouseState.m_buttons[entry::MouseButton::Right ] ? IMGUI_MBUT_RIGHT  : 0)
            | (mouseState.m_buttons[entry::MouseButton::Middle] ? IMGUI_MBUT_MIDDLE : 0)
            , mouseState.m_mz
            , width
            , height
            );

        /* settings area */
        static int32_t rightScrollArea = 0;
        imguiBeginScrollArea("Settings", width - 256 - 10, 10, 256, 540, &rightScrollArea);

        imguiInput("Simulation Duration:", duration_text, 5, true, ImguiAlign::Left, 
                ImGuiInputTextFlags_CharsDecimal | 
                ImGuiInputTextFlags_AutoSelectAll |
                ImGuiInputTextFlags_CharsNoBlank);
        // {
        //     duration = atof(duration_text);
        // }
        imguiSlider("# of Spheres", balls, 5, 11, true, ImguiAlign::LeftIndented);

        if ( imguiButton(is_running ? run_text[_frame_count / 30] : "Run", !is_running) )
        {
            _frame_count = 0;
            is_running = !is_running;
        }

        /* submit imgui */
        imguiEndScrollArea();
        imguiEndFrame();

        /* Clear and print debug text */
        bgfx::dbgTextClear();
        bgfx::dbgTextPrintf(0, 1, 0x4f, "Cradle");
        bgfx::dbgTextPrintf(0, 2, 0x6f, "Newton's Cradle simulation.");
        bgfx::dbgTextPrintf(0, 3, 0x0f, "Frame: % 7.3f[ms]", double(frameTime)*toMs );

        /* Set view 0 default viewport. */
        bgfx::setViewRect(0, 0, 0, width, height);

        // This dummy draw call is here to make sure that view 0 is cleared
        // if no other draw calls are submitted to view 0.
        // bgfx::touch(0);

        bgfx::setUniform(u_time, &time);

        bx::mtxTranslate((float *)&mtx, -(n_worlds / float(2)), 0.0f, 0.0f);
        Matrix4 move;
        bx::mtxTranslate((float *)&move, 1.0f, 0.0f, 0.0f);
        Matrix4 _mtx = mtx;
        for ( size_t i = 0; i < n_worlds; i++ )
        {
            // meshSubmit(mesh, 0, program, worlds[i]);
            _mtx *= move;
            meshSubmit(mesh, 0, program, (float *)&_mtx);
        }

        /* advance to next frame (uses seperate thread) */
        bgfx::frame();
    }

    meshUnload(mesh);

    // Cleanup.
    bgfx::destroyProgram(program);

    bgfx::destroyUniform(u_time);

    /* clean up */
    imguiDestroy();

    /* shutdown bgfx */
    bgfx::shutdown();

    /* code */
    return 0;
}
