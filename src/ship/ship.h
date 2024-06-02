// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/rigid_body2d.hpp>
#include <ship/input_directions.h>
//#include <godot_cpp/classes/input_event.hpp>
#include <bitset>
#include <functional>

class Thruster;
class LandingStrut;

namespace godot {
	template <class T>
	class Ref;

	class AnimatedSprite2D;
	class CollisionPolygon2D;
	class CollisionShape2D;
	struct Vector2;
	struct Vector3;
}

class Ship : public godot::RigidBody2D
{
    GDCLASS(Ship, godot::RigidBody2D)
private:
	using Super = RigidBody2D;
public:
	/*static constexpr int maxThGroupCount{ 3 };
	static_assert(maxThGroupCount > 0 && maxThGroupCount < 9, "invalid maxThGroupCount");

	using thGroupsArrayType = std::array<std::vector<Thruster*>, maxThGroupCount>;
	static_assert(std::is_same_v<thGroupsArrayType::value_type::value_type, 
		Thruster*>, "invalid array type");*/
protected:
	static void _bind_methods();

	// (4) input info (all -> thGroup):
	// left/up/right/down
	//static constexpr int inputInfoBitsize{ 4 * (maxThGroupCount + 1) };

	//std::bitset<inputInfoBitsize + 1> stateInfo;

	//thGroupsArrayType thGroups;
	std::vector<Thruster*> thrusters;

	godot::AnimatedSprite2D* bodySprite{ nullptr };
	godot::CollisionPolygon2D* bodyShape{ nullptr };

	godot::CollisionShape2D* visibleEnclosingRect{ nullptr };
	godot::Vector2 visibleEnclosingRectLUrel{ 0.f, 0.f };
	godot::Vector2 visibleEnclosingRectLLrel{ 0.f, 0.f };

	//LandingStrut* landingStrut1{ nullptr };
	//LandingStrut* landingStrut2{ nullptr };

	[[nodiscard]] float CalcPowerDraw() const& noexcept;
	[[nodiscard]] float CalcTorque() const&;
	[[nodiscard]] godot::Vector2 CalcForce() const&;

	
	//torque as z param
	[[nodiscard]] godot::Vector3 CalcForceTorque() const&;
	//[[nodiscard]] godot::Vector3 CalcStrutsPush() const&;

	//[[nodiscard]] int GetVerticalInput() const&;
	//[[nodiscard]] int GetVerticalInput(size_t thGroup) const&;

	//[[nodiscard]] int GetHorizontalInput() const&;
	//[[nodiscard]] int GetHorizontalInput(size_t thGroup) const&;
public:
	Ship();
	virtual void _enter_tree() override;
	virtual void _exit_tree() override;
	virtual void _ready() override;
	virtual void _physics_process(double deltatime) override;
	virtual void _process(double delta) override;

	void RetreiveChildren(godot::Node* owner)&;

	//void ChangeInputState(InputDirection dir, bool isPressed)&;
	//I allow -1 here to be consistent in inputMap
	//void ChangeInputState(InputDirection dir, int thGroup, bool isPressed)&;

	//void AddThruster(size_t thGroup);
	//void RemoveThruster(size_t thGroup)&;
	//const thGroupsArrayType GetThGroups() const&;
	int DummyGetter() const noexcept;

	float GetSpriteRadius() const&;	

	void RotateThrusters(double delta) &;
	void RotateThruster(double delta, size_t thNum)&;
	void RotateThrusterNoNotify(double delta, size_t thNum)&;
	//void RotateThrusters(double delta, size_t thGroup)&;

	void AddThrust(double deltaLevel)&;
	void AddThrust(double deltaLevel, size_t thNum)&;
	void AddThrustNoNotify(double deltaLevel, size_t thNum)&;
	//void AddThrust(double deltaLevel, size_t thGroup)&;

	[[nodiscard]] godot::Vector2 GetVisibleEnclosingRect() const&;
	[[nodiscard]] godot::Vector2 GetVisibleEnclosingRectOffset() const&;

	[[nodiscard]] godot::Vector2 GetVisibleEnclosingHalfSize() const&;
	[[nodiscard]] real_t GetVisibleEnclosingRadius() const&;
	[[nodiscard]] godot::Vector2 GetVisibleEnclosingHalfSize(real_t angl) const&;

	std::function<void(int)> onThrusterStateChanged;
	std::function<void(const godot::Vector2&)> onLinearAccChanged;
	std::function<void(float)> onTorqueAccChanged;

	const std::vector<Thruster*>& GetThrusters() const&;
	//void (*onLinearAccChanged)(const godot::Vector2&);

	//std::function<void(const int&)> onLinearAccChanged;

	//std::mem_fn<
};

