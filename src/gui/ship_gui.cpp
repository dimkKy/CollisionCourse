// by Dmitry Kolontay

#include "ship_gui.h"
#include "ship/ship.h"
#include "gui/thrusters_monitor.h"
#include "gui/torque_acc_widget.h"
#include "gui/linear_acc_widget.h"
#include "utils.h"

ShipGUI::ShipGUI()
{
	set_anchors_preset(Control::LayoutPreset::PRESET_FULL_RECT);
}

void ShipGUI::_bind_methods()
{
	//godot::TextureProgressBar
}

void ShipGUI::_enter_tree()
{
	RetreiveChildren(get_owner());
}

void ShipGUI::_ready()
{
	RetreiveChildren(get_owner());
}

void ShipGUI::OnUnposess(Ship& ship)&
{
	DBG_PRINT("ShipGUI::OnUnposess");
	ship.onLinearAccChanged = nullptr;
	ship.onTorqueAccChanged = nullptr;
}

void ShipGUI::RetreiveChildren(godot::Node* owner)&
{
	Utils::RetreiveTheeseChildren(*this, owner, 
		monitor, Utils::thrusterMonitorName,
		linearAccWidget, Utils::linearAccWidgetName,
		torqueAccWidget, Utils::torqueAccWidgetName
	);
}

void ShipGUI::SetShip(Ship& ship)&
{
	DBG_PRINT("ShipGUI::SetShip");
	monitor->SetShip(ship, get_owner());
	ship.onLinearAccChanged = std::bind(&LinearAccWidget::UpdateWidgetStatic, std::placeholders::_1, linearAccWidget);
	ship.onTorqueAccChanged = std::bind(&TorqueAccWidget::UpdateWidgetStatic, std::placeholders::_1, torqueAccWidget);
}

