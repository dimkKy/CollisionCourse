// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/texture_rect.hpp>

class Ship;
class Thruster;
class ThrusterWidget;

namespace godot {
	class String;
}

class ThrustersMonitor : public godot::TextureRect
{
	GDCLASS(ThrustersMonitor, godot::TextureRect)

protected:
	static void _bind_methods();

	using pairType = std::pair<const Thruster*, ThrusterWidget*>;

	std::vector<std::vector<pairType>> wgGroups;
	std::vector<pairType> widgetPairs;

	static void UpdateThWidget(const pairType& pair);
	static void UpdateThWidgets(const std::vector<pairType>& v);

	static void UpdateThPower(const pairType& pair);
	static void UpdateThPowers(const std::vector<pairType>& v);

	std::vector<ThrusterWidget*> RetreiveChildren()&;

public:
	void UpdateThPowers()&;
	void UpdateThPowers(size_t thGroup)&;

	void UpdateThWidgets()&;
	void UpdateThWidget(const Thruster* thruster)&;

	void SetShip(const Ship& ship, godot::Node* owner)&;
};

