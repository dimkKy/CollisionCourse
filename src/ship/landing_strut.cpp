#include "landing_strut.h"
#include <godot_cpp/classes/collision_shape2d.hpp>
#include <godot_cpp\classes\rectangle_shape2d.hpp>
#include <godot_cpp\classes\circle_shape2d.hpp>
#include <godot_cpp/classes/sprite2d.hpp>
#include <godot_cpp/classes/pin_joint2d.hpp>
#include <godot_cpp/classes/groove_joint2d.hpp>
//#include "utils.h"
#include "ship/ship.h"
#include <numbers>
#include <cassert>
#include <algorithm>
#include <godot_cpp\classes\physics_server2d.hpp>

LandingStrut::LandingStrut()
{
	set_gravity_scale(0.);
	//set_physics_process(false);
}

godot::String LandingStrut::GetNamePattern()
{
	return "landingStrut_?*";
}

void LandingStrut::RetreiveChildrenFromShip(godot::Node* owner)&
{
	DBG_PRINT("LandingStrut::RetreiveChildrenFromShip");
	RetreiveChildren(owner);
	if (auto* ship{ Object::cast_to<Ship>(get_parent()) }) {
		grooveJoint->set_node_b(ship->get_path());

		set_angular_damp(ship->get_angular_damp());
		set_linear_damp(ship->get_linear_damp());

		grooveUnitVec = godot::Vector2::from_angle(grooveJoint->get_rotation() + std::numbers::pi_v<double> / 2. );
		auto thisPos{ get_position() };
		auto offset{ grooveJoint->get_initial_offset() };

		zeroLengthPos = thisPos + grooveUnitVec * offset;
		maxLengthPos = thisPos - grooveUnitVec * (grooveJoint->get_length() - offset);

		grooveUnitVec *= -1.f;
		//DBG_PRINT(zeroLengthPos);
		//DBG_PRINT(maxLengthPos);

		assert(std::abs(offset - grooveJoint->get_length()) <= Utils::kindaSmallDouble);
	}
}

void LandingStrut::RetreiveChildren(godot::Node* owner)&
{
	using namespace Utils;
	set_owner(owner);

	RetreiveTheeseChildren(*this, owner,
		legBaseSprite, landingStrutLegBaseSpriteName,
		bodySprite, bodySpriteName,
		grooveJoint, grooveJointName/*,
		wheelJoint, landingStrutBodyName*/);

	RetreiveThisChild(bodyShape, *this, owner, bodyShapeName, [](auto* bodyShapeT)
		{ bodyShapeT->set_shape(memnew(godot::RectangleShape2D)); } );
	
	grooveJoint->set_exclude_nodes_from_collision(true);
	grooveJoint->set_rotation(std::numbers::pi);
	if (is_inside_tree()) {
		grooveJoint->set_node_a(get_path());
	}
	/*RetreiveThisChild(wheel, *wheelJoint, owner, Utils::bodySpriteName);

	RetreiveThisChild(wheelSprite, *wheel, owner, Utils::bodySpriteName);

	RetreiveThisChild(wheelShape, *wheel, owner, bodyShapeName, [](auto* bodyShapeT)
		{ bodyShapeT->set_shape(memnew(godot::CircleShape2D)); });*/
}

void LandingStrut::SetStrutLength(double length)
{
	if (!grooveJoint) {
		return;
	}
	grooveJoint->set_length(length);
	grooveJoint->set_initial_offset(length);
}

double LandingStrut::GetStrutLength() const
{
	return grooveJoint ? grooveJoint->get_length() : 0.;
}

float LandingStrut::CalcOffsetFromZero() const&
{
	return std::clamp((get_position() - zeroLengthPos).length() / grooveJoint->get_length(), 0., 1.);
}

float LandingStrut::CalcOffsetFromMax() const&
{
	return std::clamp((get_position() - maxLengthPos).length() / grooveJoint->get_length(), 0., 1.);
}

const godot::Vector2& LandingStrut::GetZeroPos() const&
{
	return zeroLengthPos;
}

const godot::Vector2& LandingStrut::GetStrutAxis() const&
{
	return grooveUnitVec;
}

void LandingStrut::_enter_tree()
{
	DBG_PRINT("LandingStrut::_enter_tree");
	auto* parent{ get_parent() };
	RetreiveChildren(parent ? parent->get_owner() : get_owner());
	/*if (auto* ship{ Object::cast_to<Ship>(parent) }) {
		grooveJoint->set_node_b(ship->get_path());
		grooveJoint->set_node_a(get_path());
	}*/
}

void LandingStrut::_ready()
{
	DBG_PRINT("LandingStrut::_ready");
	/*auto* parent{ get_parent() };
	RetreiveChildren(parent ? parent->get_owner() : get_owner());
	if (auto * ship{ Object::cast_to<Ship>(parent) }) {
		grooveJoint->set_node_b(ship->get_path());
		grooveJoint->set_node_a(get_path());
	}
	print_tree_pretty();*/
	
	//grooveJoint.get
}

void LandingStrut::_physics_process(double deltatime)
{
	//godot::Vector2::from_angle(get_global_rotation() + grooveJoint->get_rotation()) * grooveJoint->get_length()
	//auto deltaLength{ (get_position() - zeroLengthPos).length() };
	//DBG_PRINT(CalcRelativeOffser());

	auto offset{ CalcOffsetFromZero() };
	if (offset <= minOffset) {
		auto* physServ{ godot::PhysicsServer2D::get_singleton() };
		auto* state{ physServ->body_get_direct_state(get_rid()) };
		//DBG_PRINT(CalcRelativeOffset());
		auto globalGrooveUnit{ grooveUnitVec.rotated(get_global_rotation()) };
		//auto dampForce{ state->get_total_linear_damp() * state->get_linear_velocity().project(globalGrooveUnit) };
		//TODO REDO
		auto forceMod{ 1.f - offset };
		//physServ->body_apply_central_force(get_rid(), globalGrooveUnit * std::min(1000.f, dampForce.length() * 1.5f));
		physServ->body_apply_central_force(get_rid(), globalGrooveUnit * 400.f * forceMod);
		//physServ.body_get
	}
}

void LandingStrut::_integrate_forces(godot::PhysicsDirectBodyState2D* state)
{
	if (auto* ship{ Object::cast_to<Ship>(get_parent()) }) {
		state->set_angular_velocity(ship->get_angular_velocity());
		//state.get
		auto tform{ state->get_transform() };
		tform.set_rotation(ship->get_global_rotation());
		state->set_transform(tform);
		//state.
		//state.set
		//state.getn
	}
	Super::_integrate_forces(state);
	//DBG_PRINT("LandingStrut::_integrate_forces");
}

void LandingStrut::_bind_methods()
{
	using namespace godot;
	ClassDB::bind_method(D_METHOD("set_strut_length", "length"), &LandingStrut::SetStrutLength);
	ClassDB::bind_method(D_METHOD("get_strut_length"), &LandingStrut::GetStrutLength);
	//get_class_static()
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length"), "set_strut_length", "get_strut_length");
}
