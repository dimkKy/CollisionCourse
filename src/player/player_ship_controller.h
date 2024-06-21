// by Dmitry Kolontay

#pragma once

#include "ship/ship_controller.h"
#include <godot_cpp/classes/input_event.hpp>
#include "ship/input_directions.h"

#include <vector>
#include <array>
#include <bitset>

class Ship;
class ShipGUI;

namespace godot {
	template <class T>
	class Ref;
	class Viewport;
}

enum class ShipControlMode {
	Direct, 
	WASD,

};

namespace ShipCtrlMode {
	constexpr inline const ShipControlMode Direct{ ShipControlMode::Direct };
	constexpr inline const ShipControlMode WASD{ ShipControlMode::WASD };
}

enum class SpecialInputs {
	IssueStop,
};


class PlayerShipController : public ShipController
{
	GDCLASS(PlayerShipController, ShipController)
public:
	//static constexpr int maxThGroupCount{ 3 };
	//static_assert(maxThGroupCount > 0 && maxThGroupCount < 9, "invalid maxThGroupCount");

protected:
	static inline constinit float velScaleCoef{ 0.001f };
	//static inline constinit float velOffsetCoef{ 0.4f };
	static inline constinit float velShiftThreshhold{ 35.f };
	static inline constinit float baseScale{ 2.5f };

	static inline std::pair<float, float>
		scaleConstraint{ baseScale, baseScale * 5.f };
	
	static inline constinit float cameraDeadZone{ 0.2f };
	//static inline constinit float cameraLimitZone{ 0.75f };
	static inline constinit float scaleDeadZone{ 0.01f };

	static inline constinit float slowShiftSheed{ 50.0f };
	
	//godot::Vector2 prevShipPos{};
	//godot::Vector2 prevShipVel{};

	godot::Vector2 prevCameraPos{};
	godot::Vector2 prevEnclosingHalfRect{};
	godot::Vector2 prevCameraOffset{};
	float prevScale{ baseScale };

	godot::Viewport* viewport{ nullptr };
	ShipGUI* shipGUI{ nullptr };

	float shipSizeRadius{ 0.f };

	//vector instead of unordered?
	//https://quick-bench.com/q/u6UPzc8ORwKuRJjVAZksA0-GknU
	static inline std::unordered_map<godot::Key, std::pair<InputDirection, int>> inputMap{
		{godot::KEY_A, {InDir::Left, -1}},
		{godot::KEY_W, {InDir::Up, -1}},
		{godot::KEY_D, {InDir::Right, -1}},
		{godot::KEY_S, {InDir::Down, -1}},

		{godot::KEY_F, {InDir::Left, 0}},
		{godot::KEY_T, {InDir::Up, 0}},
		{godot::KEY_H, {InDir::Right, 0}},
		{godot::KEY_G, {InDir::Down, 0}},

		{godot::KEY_J, {InDir::Left, 1}},
		{godot::KEY_I, {InDir::Up, 1}},
		{godot::KEY_L, {InDir::Right, 1}},
		{godot::KEY_K, {InDir::Down, 1}},
	};

	static inline std::unordered_map<godot::Key, SpecialInputs> specialInputMap{
		{godot::KEY_M, SpecialInputs::IssueStop},
	};

	// (4) input info (all -> thGroup):
	// left/up/right/down
	//static constexpr int inputInfoBitsize{ 4 * (maxThGroupCount + 1) };

	//std::bitset<inputInfoBitsize + 1> inputStates;

	std::vector<std::bitset<4>> inputStates;

	void ChangeInputState(InputDirection dir, bool isPressed)&;
	//I allow -1 here to be consistent in inputMap
	void ChangeInputState(InputDirection dir, int thGroup, bool isPressed)&;

	static float CalcBaseScale(float shipAbsVel);
	static godot::Vector2 CalcSmoothed(const godot::Vector2& newVal, const godot::Vector2& oldVal);
	static float CalcSmoothed(float newVal, float oldVal);

	void _notification(int p_what);

	void UpdateCamera(double deltatime);

	godot::Transform2D GetCameraTransform(double deltatime)&;

	godot::Vector2 GetCameraScreenSize() const&;

	ShipGUI* RetreiveShipGUI() const&;

	void OnThrusterGroupsChanged();

public:
	PlayerShipController();
	virtual void Posess(Ship& newShip)& override;
	virtual void Unposess()& override;

	[[nodiscard]] int GetVerticalInput() const&;
	[[nodiscard]] int GetVerticalInput(size_t thGroup) const&;

	[[nodiscard]] int GetHorizontalInput() const&;
	[[nodiscard]] int GetHorizontalInput(size_t thGroup) const&;

	virtual void _enter_tree() override;
	virtual void _exit_tree() override;
	virtual void _ready() override;
	virtual void _process(double deltatime) override;
	virtual void _physics_process(double deltatime) override;
	virtual void _unhandled_key_input(const godot::Ref<godot::InputEvent>& event) override;

	//void RotateThrusters(double delta)&;
	//void RotateThrusters(double delta, size_t thGroup)&;

	//void AddThrust(double delta)&;
	//void AddThrust(double delta, size_t thGroup)&;

protected:
	static void _bind_methods();

	ShipControlMode controlMode{ ShipCtrlMode::Direct };

	std::vector<size_t> thGroups;
};

