#include "linear_acc_widget.h"
#include "utils.h"
#include <cassert>

void LinearAccWidget::_bind_methods()
{

}

void LinearAccWidget::_enter_tree()
{
	RetreiveChildren(get_owner());
}

void LinearAccWidget::_ready()
{
	RetreiveChildren(get_owner());
}

void LinearAccWidget::RetreiveChildren(godot::Node* owner)&
{
	Utils::RetreiveThisChild(marker, *this, owner, Utils::linearAccMarkerName);
	centerOffset = get_size() / 2.f;
	centerOffset -= marker->get_size() / 2.f;
	markerMaxRadius = std::min(centerOffset.x, centerOffset.y);
	UpdateWidget({0.f, 0.f});
}

void LinearAccWidget::UpdateWidget(const godot::Vector2& accel)&
{
	auto length{ accel.length() };
	if (length > Utils::kindaSmallFloat) {
		marker->set_position(centerOffset + 
			accel.limit_length(std::min(std::sqrtf(length) * scaleCoef, markerMaxRadius)));

			/*marker->set_position(centerOffset +
			accel.limit_length(std::min(std::logf(length + 1.) * scaleCoef, markerMaxRadius)));*/
	}
	else {
		//godot::Vector2::ze
		marker->set_position(centerOffset);
	}
}

void LinearAccWidget::UpdateWidgetStatic(const godot::Vector2& accel, LinearAccWidget* widget)
{
	assert(widget);
	widget->UpdateWidget(accel);
}

void LinearAccWidget::ResetWidget()&
{
	UpdateWidget({});
}
