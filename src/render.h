static float s_texelHalf = 0.0f;

struct Uniforms
{
    void init()
    {
        m_time = 0.0f;
        bx::mtxIdentity(m_mtx);

        u_mtx     = bgfx::createUniform("u_mtx",     bgfx::UniformType::Mat4);
        u_params  = bgfx::createUniform("u_params",  bgfx::UniformType::Vec4);
        u_flags   = bgfx::createUniform("u_flags",   bgfx::UniformType::Vec4);
        u_camPos  = bgfx::createUniform("u_camPos",  bgfx::UniformType::Vec4);
        u_rgbDiff = bgfx::createUniform("u_rgbDiff", bgfx::UniformType::Vec4);
        u_rgbSpec = bgfx::createUniform("u_rgbSpec", bgfx::UniformType::Vec4);
    }

    // Call this once per frame.
    void submitPerFrameUniforms()
    {
        bgfx::setUniform(u_mtx,     m_mtx);
        bgfx::setUniform(u_flags,   m_flags);
        bgfx::setUniform(u_camPos,  m_camPosTime);
        bgfx::setUniform(u_rgbDiff, m_rgbDiff);
        bgfx::setUniform(u_rgbSpec, m_rgbSpec);
    }

    // Call this before each draw call.
    void submitPerDrawUniforms()
    {
        bgfx::setUniform(u_params, m_params);
    }

    void destroy()
    {
        bgfx::destroyUniform(u_rgbSpec);
        bgfx::destroyUniform(u_rgbDiff);
        bgfx::destroyUniform(u_camPos);
        bgfx::destroyUniform(u_flags);
        bgfx::destroyUniform(u_params);
        bgfx::destroyUniform(u_mtx);
    }

    union
    {
        struct
        {
            float m_glossiness;
            float m_exposure;
            float m_diffspec;
            float m_time;
        };

        float m_params[4];
    };

    union
    {
        struct
        {
            float m_diffuse;
            float m_specular;
            float m_diffuseIbl;
            float m_specularIbl;
        };

        float m_flags[4];
    };

    float m_mtx[16];
    float m_camPosTime[4];
    float m_rgbDiff[4];
    float m_rgbSpec[4];

    bgfx::UniformHandle u_mtx;
    bgfx::UniformHandle u_params;
    bgfx::UniformHandle u_flags;
    bgfx::UniformHandle u_camPos;
    bgfx::UniformHandle u_rgbDiff;
    bgfx::UniformHandle u_rgbSpec;
};

static Uniforms s_uniforms;

struct PosColorTexCoord0Vertex
{
    float m_x;
    float m_y;
    float m_z;
    uint32_t m_rgba;
    float m_u;
    float m_v;

    static void init()
    {
        ms_decl
            .begin()
            .add(bgfx::Attrib::Position,  3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0,    4, bgfx::AttribType::Uint8, true)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
    }

    static bgfx::VertexDecl ms_decl;
};

bgfx::VertexDecl PosColorTexCoord0Vertex::ms_decl;

void screenSpaceQuad(float _textureWidth, float _textureHeight, bool _originBottomLeft = false, float _width = 1.0f, float _height = 1.0f)
{
    if (bgfx::checkAvailTransientVertexBuffer(3, PosColorTexCoord0Vertex::ms_decl) )
    {
        bgfx::TransientVertexBuffer vb;
        bgfx::allocTransientVertexBuffer(&vb, 3, PosColorTexCoord0Vertex::ms_decl);
        PosColorTexCoord0Vertex* vertex = (PosColorTexCoord0Vertex*)vb.data;

        const float zz = 0.0f;

        const float minx = -_width;
        const float maxx =  _width;
        const float miny = 0.0f;
        const float maxy = _height*2.0f;

        const float texelHalfW = s_texelHalf/_textureWidth;
        const float texelHalfH = s_texelHalf/_textureHeight;
        const float minu = -1.0f + texelHalfW;
        const float maxu =  1.0f + texelHalfW;

        float minv = texelHalfH;
        float maxv = 2.0f + texelHalfH;

        if (_originBottomLeft)
        {
            std::swap(minv, maxv);
            minv -= 1.0f;
            maxv -= 1.0f;
        }

        vertex[0].m_x = minx;
        vertex[0].m_y = miny;
        vertex[0].m_z = zz;
        vertex[0].m_rgba = 0xffffffff;
        vertex[0].m_u = minu;
        vertex[0].m_v = minv;

        vertex[1].m_x = maxx;
        vertex[1].m_y = miny;
        vertex[1].m_z = zz;
        vertex[1].m_rgba = 0xffffffff;
        vertex[1].m_u = maxu;
        vertex[1].m_v = minv;

        vertex[2].m_x = maxx;
        vertex[2].m_y = maxy;
        vertex[2].m_z = zz;
        vertex[2].m_rgba = 0xffffffff;
        vertex[2].m_u = maxu;
        vertex[2].m_v = maxv;

        bgfx::setVertexBuffer(&vb);
    }
}

struct LightProbe
{
    enum Enum
    {
        Wells,
        Uffizi,
        Pisa,
        Ennis,
        Grace,

        Count
    };

    void load(const char* _name)
    {
        char filePath[512];

        strcpy(filePath, _name);
        strcat(filePath, "_lod.dds");
        m_tex = loadTexture(filePath, BGFX_TEXTURE_U_CLAMP|BGFX_TEXTURE_V_CLAMP|BGFX_TEXTURE_W_CLAMP);

        strcpy(filePath, _name);
        strcat(filePath, "_irr.dds");
        m_texIrr = loadTexture(filePath, BGFX_TEXTURE_U_CLAMP|BGFX_TEXTURE_V_CLAMP|BGFX_TEXTURE_W_CLAMP);
    }

    void destroy()
    {
        bgfx::destroyTexture(m_tex);
        bgfx::destroyTexture(m_texIrr);
    }

    bgfx::TextureHandle m_tex;
    bgfx::TextureHandle m_texIrr;
};
