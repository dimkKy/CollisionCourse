// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/control.hpp>

class ThrustersMonitor;
class LinearAccWidget;
class TorqueAccWidget;
class Ship;

class ShipGUI : public godot::Control
{
	GDCLASS(ShipGUI, godot::Control)

protected:
	
	ThrustersMonitor* monitor{ nullptr };
	LinearAccWidget* linearAccWidget{ nullptr };
	TorqueAccWidget* torqueAccWidget{ nullptr };

	void RetreiveChildren(godot::Node* owner)&;

public:
	ShipGUI();
	static void _bind_methods();
	virtual void _enter_tree() override;
	virtual void _ready() override;
	void OnUnposess(Ship& ship)&;
	void SetShip(Ship& ship)&;
};

