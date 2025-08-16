// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/area2d.hpp>

namespace godot{
	class AnimatedSprite2D;
	class CollisionShape2D;
	//class GPUParticles2D;
}

class Thruster : public godot::Area2D
{
	GDCLASS(Thruster, godot::Area2D)

protected:
	static void _bind_methods();

	//float maxThrust{ 100000.f };
	float maxDraw{ 100.f };
	//max - 1.
	float powerLevel{ 0.f };
	float thrustMultiplier{ 10.f };
	godot::AnimatedSprite2D* flameSprite{ nullptr };
	godot::AnimatedSprite2D* bodySprite{ nullptr };
	godot::CollisionShape2D* bodyShape{ nullptr };
	int lastPlayedAnim{ 0 };
	float powerPerFlameAnim{ 0.1f };
	//godot::GPUParticles2D* emissionParticles = nullptr;

	float DrawFunc(float inLevel) const;
	float ThrustFunc(float inLevel) const;
	void SetPowerLevel(float inLevel)&;
public:
	Thruster();
	virtual void _enter_tree() override;

	static godot::String GetNamePattern();

	void RetreiveChildren(godot::Node* owner)&;
	int GetThGroup() const&;

	void SetThrusterName(int group, int number);
	void SetPowerRelative(float multiplier) &;
	void SetPowerFromThrust(double thrust) &;
	void ChangePowerLevel(float levelTerm) &;
	void SetZeroPower()&;

	float GetDraw() const &;
	float GetThrust() const &;
	float GetMaxThrust() const &;
	float GetPowerLevel() const &;
	void GetThrustDelta(float delta, double& low, double& max) const &;
};

