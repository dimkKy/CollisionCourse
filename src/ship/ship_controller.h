// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include "ship/input_directions.h"

#include <array>
#include <vector>
#include <tuple>

class Ship;
class ShipGUI;

namespace godot {
	template <class T>
	class Ref;
	class Viewport;
	struct Vector2;
	struct Vector3;
}


class ShipController : public godot::Node2D
{
	//GDCLASS(ShipController, godot::Node2D)
public:
	ShipController();
	virtual void Posess(Ship& newShip)&;
	virtual void Unposess()&;

	//virtual void _enter_tree() override;
	virtual void _exit_tree() override;
	virtual void _ready() override;
	virtual void _process(double deltatime) override;
	//virtual void _unhandled_key_input(const godot::Ref<godot::InputEvent>& event) override;

	
	using vec2 = std::pair<double, double>;
	using vec3 = std::array<double, 3>;

	static void Solve(const vec2& weights, const vec3& desired, std::vector<double>& init,
		const std::vector<const double>& relPos, const std::vector<const vec2>& constr);

	//static double RotationDerivative(const vec2& weights, const vec3& desired,
		//double y, double relX, double relY, double sinX, double cosX);

	static double RotationDerivative(const vec2& weights, const vec3& sumsMindes,
		double ycosX, double ysinX, double relX, double relY);

	static double ThrustDerivative(const vec2& weights, const vec3& sumsMindes,
		double sinX, double cosX, double relX, double relY);
protected:
	Ship* ship{ nullptr };

	//need to ensure sync with solver
	std::vector<double> relPos;

};

