// by Dmitry Kolontay

#include "thruster_widget.h"
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/variant/string.hpp>
#include <cassert>
#include "utils.h"
#include <ship\thruster.h>

void ThrusterWidget::_bind_methods()
{

}

void ThrusterWidget::GenerateLevelLabes()
{
	levelLabels.resize(levelsNum);
	std::string base;
	for (int i{ maxLevel }; i >= minLevel; --i) {
		base = std::to_string(i);
		levelLabels[i - minLevel] = base.c_str();
	}
	levelLabels.shrink_to_fit();
}

ThrusterWidget::ThrusterWidget()
{

}

void ThrusterWidget::OnClassRegister()
{
	GenerateLevelLabes();
}

void ThrusterWidget::RetreiveChildren(godot::Node* owner)&
{
	Utils::RetreiveThisChild(label, *this, owner, Utils::thrusterLabelName);
	//label.s
}

void ThrusterWidget::UpdateWidget(const Thruster& thruster)&
{
	UpdateDisplayedLevel(thruster);
}

void ThrusterWidget::UpdateDisplayedLevel(const Thruster& thruster)&
{
	SetDisplayedLevel(std::round(thruster.GetPowerLevel()));
}

void ThrusterWidget::SetDisplayedLevel(int level)&
{
	assert(label);
	assert(level - minLevel >= 0);
	//decltype(levelLabels)::v
	label->set_text(levelLabels[level - minLevel]);
}
