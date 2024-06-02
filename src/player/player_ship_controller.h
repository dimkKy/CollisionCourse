// by Dmitry Kolontay

#pragma once

#include "ship/ship_controller.h"
#include <godot_cpp/classes/input_event.hpp>
#include "ship/input_directions.h"

#include <vector>
#include <array>

class Ship;
class ShipGUI;

namespace godot {
	template <class T>
	class Ref;
	class Viewport;
}


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
	Ship* ship{ nullptr };
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

	static float CalcBaseScale(float shipAbsVel);
	static godot::Vector2 CalcSmoothed(const godot::Vector2& newVal, const godot::Vector2& oldVal);
	static float CalcSmoothed(float newVal, float oldVal);

	void _notification(int p_what);

	void UpdateCamera(double deltatime);

	godot::Transform2D GetCameraTransform(double deltatime)&;

	godot::Vector2 GetCameraScreenSize() const&;

	ShipGUI* RetreiveShipGUI() const&;
public:
	PlayerShipController();
	virtual void Posess(Ship& newShip)& override;
	virtual void Unposess()& override;

	virtual void _enter_tree() override;
	virtual void _exit_tree() override;
	virtual void _ready() override;
	virtual void _process(double deltatime) override;
	virtual void _unhandled_key_input(const godot::Ref<godot::InputEvent>& event) override;

	void RotateThrusters(double delta)&;
	void RotateThrusters(double delta, size_t thGroup)&;

	void AddThrust(double delta)&;
	void AddThrust(double delta, size_t thGroup)&;

protected:
	static void _bind_methods();

	std::vector<size_t> thGroups;
};

