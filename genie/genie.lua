--
-- Copyright (c) 2015 Jonathan Howard
--

CRADLE_DIR = (path.getabsolute("..") .. "/")

local CRADLE_EXT_DIR   = (CRADLE_DIR .. "ext/")
local CRADLE_BUILD_DIR = (CRADLE_DIR .. "build/")

BGFX_DIR    = (CRADLE_EXT_DIR .. "bgfx/") 
BX_DIR      = (CRADLE_EXT_DIR .. "bx/")

function copyLib()
end

newoption {
    trigger = "with-tools",
    description = "Build with tools."
}

solution "cradle"
    configurations
    {
        "debug",
        "development",
        "release"
    }

    platforms 
    {
        "x64",
        "native"
    }

    language "C++"

    configuration {}

dofile ("toolchain.lua")
dofile ("cradle.lua")
dofile (BGFX_DIR .. "scripts/bgfx.lua")

toolchain (CRADLE_BUILD_DIR, CRADLE_EXT_DIR)

group "libs"
bgfxProject("", "StaticLib", os.is("windows") and { "BGFX_CONFIG_RENDERER_DIRECT3D9=1" } or {})
dofile (BGFX_DIR .. "scripts/example-common.lua")

if _OPTIONS["with-tools"] then
    dofile ( BGFX_DIR .. "scripts/shaderc.lua" )
    dofile ( BGFX_DIR .. "scripts/geometryc.lua" )
    dofile ( BGFX_DIR .. "scripts/texturec.lua" )
end

group "cradle"
cradle_project("cradle", "ConsoleApp", {})

