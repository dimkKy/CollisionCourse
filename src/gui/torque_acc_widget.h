// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/texture_progress_bar.hpp>

class TorqueAccWidget : public godot::TextureProgressBar
{
	GDCLASS(TorqueAccWidget, godot::TextureProgressBar)

protected:
	static inline constexpr double maxAngle{ 360. };
	static inline constexpr float halfAngle{ maxAngle / 2.f };
	//static inline constexpr double minAngle{ 0. };
	static inline constexpr float minDisplayedAngle{ 0.f };
	static inline constexpr float modifier{ 0.002f };
	//TODO
	static inline constexpr double minSpeedScale{ 20. };

	static float GetValueAbs(float absTorque);
public:
	TorqueAccWidget();
	static void _bind_methods();
	//virtual void _enter_tree() override;
	//virtual void _ready() override;

	void static UpdateWidgetStatic(float torque, TorqueAccWidget* widget);
	void UpdateWidget(float torque)&;
};

