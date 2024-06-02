#include "torque_acc_widget.h"
#include <godot_cpp\classes\animated_texture.hpp>
#include "utils.h"
#include <cassert>
#include <algorithm>

float TorqueAccWidget::GetValueAbs(float absTorque)
{
	assert(absTorque > 0.f);

	return std::clamp(absTorque * modifier, minDisplayedAngle, halfAngle);
}

TorqueAccWidget::TorqueAccWidget()
{
	set_fill_mode(FILL_CLOCKWISE);
	set_max(maxAngle);
}

void TorqueAccWidget::_bind_methods()
{
}

void TorqueAccWidget::UpdateWidgetStatic(float torque, TorqueAccWidget* widget)
{
	assert(widget);
	widget->UpdateWidget(torque);
}

void TorqueAccWidget::UpdateWidget(float torque)&
{
	using namespace godot;
	if (std::abs(torque) <= Utils::kindaSmallFloat) {
		set_value(0.);
		if (auto* texture{ Object::cast_to<AnimatedTexture>(*get_progress_texture()) }) [[likely]] {
			texture->set_pause(true);
		}
	}
	else {
		if (torque < 0.) {
			set_fill_mode(FILL_COUNTER_CLOCKWISE);
			set_value(GetValueAbs(-torque));
		}
		else {
			set_fill_mode(FILL_CLOCKWISE);
			set_value(GetValueAbs(torque));
		}
		if (auto* texture{ Object::cast_to<AnimatedTexture>(*get_progress_texture()) }) [[likely]] {
			texture->set_pause(false);
			texture->set_speed_scale(minSpeedScale);
		}
	}
}
