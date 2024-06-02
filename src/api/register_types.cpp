#include "register_types.h"

#include "ship/ship.h"
#include "ship/thruster.h"
#include "ship/landing_strut.h"
#include "environment/surface_mesh2d.h"
#include "player/player_ship_controller.h"
#include "gui/thruster_widget.h"
#include "gui/thrusters_monitor.h"
#include "gui/ship_gui.h"
#include "gui/linear_acc_widget.h"
#include "gui/torque_acc_widget.h"
#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

//using namespace godot;
namespace godot
{

	void initialize_game_module(godot::ModuleInitializationLevel p_level) {
		//using namespace godot;
		if (p_level != godot::MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
		}

		ClassDB::register_class<SurfaceMesh2D>();
		SurfaceMesh2D::CalcTemplateArrays();
		ClassDB::register_class<Thruster>();
		ClassDB::register_class<LandingStrut>();
		ClassDB::register_class<PlayerShipController>();
		ClassDB::register_class<Ship>();
		ClassDB::register_class<LinearAccWidget>();
		ClassDB::register_class<TorqueAccWidget>();
		ClassDB::register_class<ThrustersMonitor>();
		ClassDB::register_class<ThrusterWidget>();
		ClassDB::register_class<ShipGUI>();
		ThrusterWidget::OnClassRegister();
	}

	void uninitialize_game_module(godot::ModuleInitializationLevel p_level) {
		if (p_level != godot::MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
		}

	}

	extern "C"
	{
		GDExtensionBool GDE_EXPORT example_library_init(
			GDExtensionInterfaceGetProcAddress p_get_proc_address,
			const GDExtensionClassLibraryPtr p_library,
			GDExtensionInitialization* r_initialization) {
			using namespace godot;

			GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

			init_obj.register_initializer(initialize_game_module);
			init_obj.register_terminator(uninitialize_game_module);
			init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

			return init_obj.init();
		}
	}
}