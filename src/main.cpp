/**
 * Entrypoint for Newton's Cradle simulation
 */

#include <chrono>
#include <vector>

#include <bgfx/bgfx.h>

#include <bx/readerwriter.h>
#include <bx/fpumath.h>
#include <bx/string.h>

#include "common.h"

#include "bgfx_utils.h"
#include "bx/timer.h"
#include "imgui/imgui.h"

#include "physics/entity.h"
#include "physics/resolver.h"

#include "math/matrix4.h"

#include "render.h"

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

// Matrix4 * worlds;
size_t n_worlds = 5;

std::vector<PhysicsBody> bodies;
std::vector<CollisionPair> pairs;

void create_bodies(size_t n_bodies)
{
    bodies = std::vector<PhysicsBody>(n_bodies);

    n_worlds = n_bodies;

    for ( size_t i = 0; i < n_bodies; i++ )
    {
        Vector3 adjust = vector3::vector3(1.0f * i, 0.0f, 0.0f);
        bodies[i].init_body(adjust,
                            10.0f, 
                            0.0f, 
                            0.4f,
                            2.25f
                        );
    }

    for ( size_t i = 0; i < n_bodies; i++ )
    {
        CollisionPair p;
        if ( i < n_bodies - 1 )
        {
            p.bodyA = &bodies[i];
            p.bodyB = &bodies[i + 1];

            pairs.push_back(p);
        }
    }
}

int32_t left_used;
bool   use_left;

int32_t right_used;
bool   use_right;

float starting_degree = 30.0f;

void update_starting_degrees()
{
    if ( is_running ) return;
    for ( std::vector<PhysicsBody>::iterator i = bodies.begin(); i != bodies.end(); i++ )
    {
        (*i).angle = 0.0f;
    }
    if ( use_left )
    {
        for ( std::vector<PhysicsBody>::iterator i = bodies.begin(); i != bodies.begin() + left_used; i++ )
        {
            (*i).angle = starting_degree;
        }
    }
    if ( use_right )
    {

        for ( std::vector<PhysicsBody>::iterator i = bodies.end() - 1; i != bodies.end() - 1 - right_used; i-- )
        {
            (*i).angle = -starting_degree;
        }
    }
}

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

    s_uniforms.init();

    /**
     * SET UP LIGHTING
     */

    // Vertex declarations.
    PosColorTexCoord0Vertex::init();

    LightProbe lightProbe;
    lightProbe.load("./grace");

    bgfx::UniformHandle u_mtx        = bgfx::createUniform("u_mtx",        bgfx::UniformType::Mat4);
    bgfx::UniformHandle u_params     = bgfx::createUniform("u_params",     bgfx::UniformType::Vec4);
    bgfx::UniformHandle u_flags      = bgfx::createUniform("u_flags",      bgfx::UniformType::Vec4);
    bgfx::UniformHandle u_camPos     = bgfx::createUniform("u_camPos",     bgfx::UniformType::Vec4);
    bgfx::UniformHandle s_texCube    = bgfx::createUniform("s_texCube",    bgfx::UniformType::Int1);
    bgfx::UniformHandle s_texCubeIrr = bgfx::createUniform("s_texCubeIrr", bgfx::UniformType::Int1);

    /**
     * PER-FRAME SETTINGS
     */

    struct Settings
    {
        float m_speed;
        float m_glossiness;
        float m_exposure;
        float m_diffspec;
        float m_rgbDiff[3];
        float m_rgbSpec[3];
        bool m_diffuse;
        bool m_specular;
        bool m_diffuseIbl;
        bool m_specularIbl;
        bool m_showDiffColorWheel;
        bool m_showSpecColorWheel;
        ImguiCubemap::Enum m_crossCubemapPreview;
    };

    Settings settings;
    settings.m_speed = 0.37f;
    settings.m_exposure = 0.0f;
    settings.m_diffuse = true;
    settings.m_specular = true;
    settings.m_diffuseIbl = true;
    settings.m_specularIbl = true;
    settings.m_showDiffColorWheel = true;
    settings.m_showSpecColorWheel = false;
    settings.m_crossCubemapPreview = ImguiCubemap::Cross;
    
    /* Steel */
    settings.m_glossiness = 0.82f;
    settings.m_diffspec   = 1.0f;
    settings.m_rgbDiff[0] = 0.0f;
    settings.m_rgbDiff[1] = 0.0f;
    settings.m_rgbDiff[2] = 0.0f;
    settings.m_rgbSpec[0] = 0.77f;
    settings.m_rgbSpec[1] = 0.78f;
    settings.m_rgbSpec[2] = 0.77f;

    float duration = 10.0f;
    int64_t last = 0;
    int balls = 5;
    char duration_text[5] = "5";
    entry::MouseState mouseState;

    Mesh * mesh = meshLoad("newton.bin");

    bgfx::ShaderHandle vsh = bgfx::createShader(loadMem(entry::getFileReader(), "./vs_ibl_mesh.bin") );
    bgfx::ShaderHandle fsh = bgfx::createShader(loadMem(entry::getFileReader(), "./fs_ibl_mesh.bin") );
    bgfx::ProgramHandle programMesh = bgfx::createProgram( vsh, fsh, true );
    
    vsh = bgfx::createShader(loadMem(entry::getFileReader(), "./vs_ibl_skybox.bin") );
    fsh = bgfx::createShader(loadMem(entry::getFileReader(), "./fs_ibl_skybox.bin") );
    bgfx::ProgramHandle programSky = bgfx::createProgram( vsh, fsh, true );

    Matrix4 view;
    float eye[3] = { 0.0f, 0.0f, -3.0f };
    float at[3]  = { 0.0f, 0.0f,  0.0f };
    bx::mtxLookAt((float *)&view, eye, at);

    Matrix4 proj;
    bx::mtxProj((float *)&proj, 60.0f, float(width)/float(height), 0.1f, 100.0f);

    bgfx::setViewTransform(0, &view, &proj);

    Matrix4 mtx;
    bx::mtxScale((float *)&mtx, 1.0f, 1.0f, 1.0f);

    bx::mtxTranslate((float *)&mtx, -(n_worlds / float(2)), 0.0f, 0.0f);
    Matrix4 move;
    bx::mtxTranslate((float *)&move, 1.0f, 0.0f, 0.0f);

    float deg = 0.0f;
    float time = 0.0f;
    float lastTime = 0.0f;

    create_bodies(n_worlds);

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
        const double toS  = 10.0 / freq;

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

        // imguiInput("Simulation Duration:", duration_text, 5, true, ImguiAlign::Left, 
        //         ImGuiInputTextFlags_CharsDecimal | 
        //         ImGuiInputTextFlags_AutoSelectAll |
        //         ImGuiInputTextFlags_CharsNoBlank);
        // {
        //     duration = atof(duration_text);
        // }
        imguiSlider("# of Spheres", balls, 5, 9, true, ImguiAlign::LeftIndented);
        if ( !is_running && balls != n_worlds )
        {
            create_bodies( balls );
            bx::mtxTranslate((float *)&mtx, -(n_worlds / float(2)), 0.0f, 0.0f);
        }
        if ( imguiSlider("Starting Degrees:", starting_degree, 30.0f, 80.0f, true) )
        {
            update_starting_degrees();
        }

        imguiSeparatorLine();

        imguiLabel( "Left Balls:" );
        if ( imguiCheck( "Left", use_left, true ) )
        {
            use_left = !use_left;
        }
        if ( imguiSlider( "Left Side:", left_used, 1, n_worlds, true, ImguiAlign::LeftIndented ) )
        {
            update_starting_degrees();
        }

        imguiSeparatorLine();

        imguiLabel( "Right Balls:" );
        if ( imguiCheck( "Right", use_right, true ) )
        {
            use_right = !use_right;
        }
        if ( imguiSlider( "Right Side:", right_used, 1, n_worlds, true, ImguiAlign::LeftIndented ) )
        {
            update_starting_degrees();
        }

        imguiSeparatorLine();

        if ( imguiButton(is_running ? run_text[_frame_count / 30] : "Run", true ) )
        {
            _frame_count = 0;
            is_running = !is_running;
            update_starting_degrees();
        }

        /* submit imgui */
        imguiEndScrollArea();
        imguiEndFrame();

        /* ibl settings */
        s_uniforms.m_glossiness = settings.m_glossiness;
        s_uniforms.m_exposure = settings.m_exposure;
        s_uniforms.m_diffspec = settings.m_diffspec;
        s_uniforms.m_flags[0] = float(settings.m_diffuse);
        s_uniforms.m_flags[1] = float(settings.m_specular);
        s_uniforms.m_flags[2] = float(settings.m_diffuseIbl);
        s_uniforms.m_flags[3] = float(settings.m_specularIbl);
        memcpy(s_uniforms.m_rgbDiff, settings.m_rgbDiff, 3*sizeof(float) );
        memcpy(s_uniforms.m_rgbSpec, settings.m_rgbSpec, 3*sizeof(float) );

        /* submit uniforms */
        s_uniforms.submitPerFrameUniforms();

        time = double(frameTime) * toS;
        s_uniforms.m_camPosTime[3] = time;

        {
            float view[16];
            float proj[16];

            bx::mtxIdentity(view);
            bx::mtxOrtho(proj, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 100.0f);
            bgfx::setViewTransform(0, view, proj);

            bx::mtxLookAt(view, eye, at);
            memcpy(s_uniforms.m_camPosTime, eye, 3*sizeof(float) );
            bx::mtxProj(proj, 60.0f, float(width)/float(height), 0.1f, 100.0f);
            bgfx::setViewTransform(1, view, proj);

            bgfx::setViewRect(0, 0, 0, width, height);
            bgfx::setViewRect(1, 0, 0, width, height);
        }

        // View 0.
        bgfx::setTexture(0, s_texCube, lightProbe.m_tex);
        bgfx::setState(BGFX_STATE_RGB_WRITE|BGFX_STATE_ALPHA_WRITE);
        screenSpaceQuad( (float)width, (float)height, true);
        s_uniforms.submitPerDrawUniforms();
        bgfx::submit(0, programSky);

        // // View 1.
        // bx::mtxSRT((float *)&mtx
        //         , 1.0f
        //         , 1.0f
        //         , 1.0f
        //         , 0.0f
        //         , bx::pi+time
        //         , 0.0f
        //         , 0.0f
        //         , -1.0f
        //         , 0.0f
        //         );

        bgfx::setTexture(0, s_texCube,    lightProbe.m_tex);
        bgfx::setTexture(1, s_texCubeIrr, lightProbe.m_texIrr);
        // meshSubmit(mesh, 1, programMesh, (float *)&mtx);

        /* Clear and print debug text */
        bgfx::dbgTextClear();
        bgfx::dbgTextPrintf(0, 1, 0x4f, "Cradle");
        bgfx::dbgTextPrintf(0, 2, 0x6f, "Newton's Cradle simulation.");
        bgfx::dbgTextPrintf(0, 3, 0x0f, "Frame: % 7.3f[ms]", double(frameTime)*toMs );
        bgfx::dbgTextPrintf(0, 4, 0x0f, "Time: % 7.3f[s]", double(frameTime) * toS );

        Matrix4 _mtx = mtx;
        _mtx.a.x = 1.0f;
        _mtx.b.y = 1.0f;
        _mtx.c.z = 1.0f;
        _mtx.d.w = 1.0f;

        if ( is_running )
        {
            for ( size_t i = 0; i < bodies.size(); i++ )
            {
                bodies[i].applyGravity();
                bodies[i].update(time, time / lastTime);
                bodies[i].solve_constraint();
                bodies[i].postsolve_constraint();
                bodies[i].clearForces();
            }
        }

        Matrix4 rot;
        for ( size_t i = 0; i < n_worlds; i++ )
        {
            _mtx *= move;

            bx::mtxRotateZ((float *)&rot, ( bodies[i].angle * M_PI) / 180 );

            Matrix4 s_mtx = _mtx;
            s_mtx *= rot;
            
            meshSubmit(mesh, 1, programMesh, (float *)&s_mtx);
        }

        deg += 0.01;

        /* advance to next frame (uses seperate thread) */
        bgfx::frame();

        lastTime = time;
    }

    meshUnload(mesh);

    // Cleanup.
    bgfx::destroyProgram(programMesh);
    bgfx::destroyProgram(programSky);

    bgfx::destroyUniform(u_camPos);
    bgfx::destroyUniform(u_flags);
    bgfx::destroyUniform(u_params);
    bgfx::destroyUniform(u_mtx);

    bgfx::destroyUniform(s_texCube);
    bgfx::destroyUniform(s_texCubeIrr);

    /* clean up */
    imguiDestroy();

    /* shutdown bgfx */
    bgfx::shutdown();

    /* code */
    return 0;
}
