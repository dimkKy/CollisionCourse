// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/rigid_body2d.hpp>
#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include "utils.h"

namespace godot {
	class Sprite2D;
	class CollisionShape2D;
	class PinJoint2D;
	class GrooveJoint2D;
	//class PhysicsDirectBodyState2D;
}

class LandingStrut : public godot::RigidBody2D
{
	GDCLASS(LandingStrut, godot::RigidBody2D)
private:
	using Super = godot::RigidBody2D;
protected:
	godot::Sprite2D* legBaseSprite{ nullptr };

	godot::GrooveJoint2D* grooveJoint{ nullptr };

	godot::Sprite2D* bodySprite{ nullptr };
	godot::CollisionShape2D* bodyShape{ nullptr };

	//godot::PinJoint2D* wheelJoint{ nullptr };

	//godot::RigidBody2D* wheel{ nullptr };
	//godot::Sprite2D* wheelSprite{ nullptr };
	//godot::CollisionShape2D* wheelShape{ nullptr };

	void RetreiveChildren(godot::Node* owner)&;

	godot::Vector2 grooveUnitVec;
	godot::Vector2 zeroLengthPos;
	godot::Vector2 maxLengthPos;

	static inline constinit float minOffset{ 1.f - Utils::kindaSmallFloat };

public:
	virtual void _enter_tree() override;
	virtual void _ready() override;
	virtual void _physics_process(double deltatime) override;
	virtual void _integrate_forces(godot::PhysicsDirectBodyState2D* state) override;
	static void _bind_methods();

	LandingStrut();
	static godot::String GetNamePattern();

	[[nodiscard]] float CalcOffsetFromZero() const&;
	[[nodiscard]] float CalcOffsetFromMax() const&;
	[[nodiscard]] const godot::Vector2& GetZeroPos() const&;
	[[nodiscard]] const godot::Vector2& GetStrutAxis() const&;
	void RetreiveChildrenFromShip(godot::Node* owner)&;
	void SetStrutLength(double length);
	double GetStrutLength() const;
};

