// by Dmitry Kolontay

#include "Thruster.h"
#include <godot_cpp/classes/animated_sprite2d.hpp>
#include <godot_cpp/classes/collision_shape2d.hpp>
//#include <godot_cpp/classes/gpu_particles2d.hpp>
#include <cassert>
#include <algorithm>
#include <numbers>
#include "utils.h"
#include <godot_cpp\classes\rectangle_shape2d.hpp>

void Thruster::_bind_methods()
{

}

float Thruster::DrawFunc(float inLevel) const
{
	//TODO PROBABLY INLEVEL ^ (M/N)
	assert(inLevel >= 0.f && inLevel <= 1.f);
	return inLevel;
}


Thruster::Thruster()
{
	//bodySprite.set_name("bodySprite");
	//add_child(&bodySprite);
	
	//et_process_input(false);
	//set_physics_process(false);
}

void Thruster::_enter_tree()
{
	RetreiveChildren(get_parent()->get_owner());
	//emissionParticles.
}

godot::String Thruster::GetNamePattern()
{
	return "thruster_g?_?*";
}

void Thruster::RetreiveChildren(godot::Node* owner)&
{
	using namespace godot;
	using namespace std;
	assert(owner);
	set_owner(owner);
	double rotation{ numbers::pi * 0.5 };

	Utils::RetreiveTheeseChildren(*this, owner,
		bodySprite, Utils::bodySpriteName,
		flameSprite, Utils::flameSpriteName);
	bodySprite->set_rotation(rotation);
	flameSprite->set_rotation(rotation);
	flameSprite->pause();
	flameSprite->hide();
	flameSprite->set_process_mode(PROCESS_MODE_DISABLED);

	Utils::RetreiveThisChild(bodyShape, *this, owner, Utils::bodyShapeName, 
		[](auto* bodyShapeT) { bodyShapeT->set_shape(memnew(RectangleShape2D)); }
	);
	bodyShape->set_rotation(rotation);
}

int Thruster::GetThGroup() const&
{
	return get_name().substr(10, 1).to_int();
}

void Thruster::SetThrusterName(int group, int number)
{
	std::string base{ "thruster_g" + std::to_string(group) };
	set_name(base.append("_n" + std::to_string(number)).c_str());
}

void Thruster::SetPowerRelative(float multiplier) &
{
	assert(multiplier >= 0.f && multiplier <= 1.f);
	if (multiplier >= 0.f) {
		SetPowerLevel(powerLevel * multiplier);
	}
}

void Thruster::SetPowerFromThrust(double thrust)&
{
	SetPowerLevel(thrust / thrustMultiplier);
}

void Thruster::SetPowerLevel(float inLevel)&
{
	powerLevel = std::clamp(inLevel, 0.f, 1.f);
	if (powerLevel <= Utils::kindaSmallFloat) {
		flameSprite->pause();
		flameSprite->hide();
		flameSprite->set_process_mode(PROCESS_MODE_DISABLED);
	}
	else {
		float temp{ powerLevel - Utils::kindaSmallFloat };
		int animNo{ static_cast<int>(std::floor(temp / powerPerFlameAnim)) };
		
		auto frame{ flameSprite->get_frame() };
		auto progress{ flameSprite->get_frame_progress() };

		std::string base{ std::to_string(animNo) };
		flameSprite->show();
		flameSprite->set_process_mode(PROCESS_MODE_INHERIT);
		flameSprite->play(base.c_str());
		flameSprite->set_frame_and_progress(frame, progress);
	}
}

void Thruster::ChangePowerLevel(float levelTerm)&
{
	SetPowerLevel(powerLevel + levelTerm);
}

void Thruster::SetZeroPower()&
{
	SetPowerLevel(0.f);
}

float Thruster::GetDraw() const&
{
	return maxDraw * DrawFunc(powerLevel);
}

float Thruster::GetThrust() const&
{
	assert(powerLevel >= 0.f && powerLevel <= 1.f);
	return thrustMultiplier * powerLevel;
}

float Thruster::GetMaxThrust() const&
{
	return thrustMultiplier;
}

float Thruster::GetPowerLevel() const&
{
	return powerLevel;
}
