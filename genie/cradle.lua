--
-- Copyright (c) 2015 Jonathan Howard.
--

function cradle_project( _name, _kind, _defines )
project ( _name )
    kind (_kind)

    includedirs 
    {
        CRADLE_DIR .. "src/",
        BGFX_DIR .. "examples/common",
        BGFX_DIR .. "3rdparty"
    }

    defines
    {
        _defines
    }

    links
    {
        "example-common",
        "bgfx",
    }

    files 
    {
        CRADLE_DIR .. "src/**.h",
        CRADLE_DIR .. "src/**.cpp",
        path.join(BGFX_DIR, "examples/common/**.mm"),
    }

    excludes
    {
        CRADLE_DIR .. "src/foundation/unit_test.cpp"
    }

    configuration { "debug or development" }
        flags {
            "Symbols"
        }
        defines {
            "_DEBUG",
        }

    configuration { "release" }
        defines {
            "NDEBUG"
        }

    configuration {}

    strip()

    configuration {}
end
