#pragma once

#include <godot_cpp/core/class_db.hpp>

namespace godot
{

	void initialize_game_module(ModuleInitializationLevel p_level);
	void uninitialize_game_module(ModuleInitializationLevel p_level);

    extern "C"
    {
        GDExtensionBool GDE_EXPORT extension_library_init(
            GDExtensionInterfaceGetProcAddress addr,
            GDExtensionClassLibraryPtr lib,
            GDExtensionInitialization* init);
    }
}

