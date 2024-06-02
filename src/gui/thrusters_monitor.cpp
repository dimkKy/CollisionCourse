#include "thrusters_monitor.h"
#include "gui/thruster_widget.h"
#include "ship/ship.h"
#include <algorithm>
#include <execution>
#include <cassert>
#include "utils.h"

void ThrustersMonitor::_bind_methods()
{

}

void ThrustersMonitor::UpdateThWidget(const pairType& pair)
{
	pair.second->UpdateWidget(*pair.first);
}

void ThrustersMonitor::UpdateThWidgets(const std::vector<pairType>& v)
{
	std::for_each(std::execution::unseq, v.begin(), v.end(),
		static_cast<void (*)(const pairType&)>(&ThrustersMonitor::UpdateThWidget));
}

void ThrustersMonitor::UpdateThPower(const pairType& pair)
{
	pair.second->UpdateDisplayedLevel(*pair.first);
}

void ThrustersMonitor::UpdateThPowers(const std::vector<pairType>& v)
{
	std::for_each(std::execution::unseq, v.begin(), v.end(),
		static_cast<void (*)(const pairType&)>(&ThrustersMonitor::UpdateThPower));
}

/*void ThrustersMonitor::OnClassRegister()
{

}*/

std::vector<ThrusterWidget*> ThrustersMonitor::RetreiveChildren()&
{
	using namespace godot;
	static const String className{ ThrusterWidget::get_class_static() };

	auto children{ find_children("*", className, false) };
	std::vector<ThrusterWidget*> widgets;
	widgets.reserve(children.size());
	//no iterators ffs
	for (int i{ 0 }; i < children.size(); ++i) {
		if (auto* widget{ Object::cast_to<ThrusterWidget>(children[i]) }) [[likely]] {
			widgets.push_back(widget);
		}
	}
	return widgets;
}

void ThrustersMonitor::UpdateThPowers()&
{
	std::for_each(std::execution::unseq, widgetPairs.begin(), widgetPairs.end(),
		static_cast<void (*)(const pairType&)>(&ThrustersMonitor::UpdateThPower));
}

void ThrustersMonitor::UpdateThPowers(size_t thGroup)&
{
	assert(thGroup < wgGroups.size());
	std::for_each(std::execution::unseq, wgGroups[thGroup].begin(), wgGroups[thGroup].end(),
		static_cast<void (*)(const pairType&)>(&ThrustersMonitor::UpdateThPower));
}

void ThrustersMonitor::UpdateThWidgets()&
{
	/*static auto updater = [](const std::vector<pairType>& v) -> void 
		{ std::for_each(std::execution::unseq, v.begin(), v.end(), 
			static_cast<void (*)(const pairType&)>(&ThrustersMonitor::UpdateThWidget)); };*/

	std::for_each(std::execution::unseq, wgGroups.begin(), wgGroups.end(), 
		static_cast<void (*)(const std::vector<pairType>&)>(&ThrustersMonitor::UpdateThWidgets));
}

void ThrustersMonitor::UpdateThWidget(const Thruster* thruster)&
{
	auto finder = [thruster](const pairType& p) 
		noexcept -> bool { return p.first == thruster; };

	auto pair{ std::find_if(std::execution::unseq,
		widgetPairs.begin(), widgetPairs.begin(), finder) };

	if (pair != widgetPairs.end()) [[likely]] {
		UpdateThWidget(*pair);
	}
}

void ThrustersMonitor::SetShip(const Ship& ship, godot::Node* owner)&
{
	auto widgets{ RetreiveChildren() };
	const auto thGroups{ ship.GetThGroups() };
	static auto adder = [](size_t a, const decltype(thGroups)::value_type& vec)
		noexcept(noexcept(vec.size())) -> size_t { return a + vec.size(); };
	const size_t thrustersNum{ std::reduce(std::execution::unseq, thGroups.begin(),
		thGroups.end(), static_cast<size_t>(0), adder) };

	{
	const size_t widgetsNum{ widgets.size() };
	if (widgetsNum >= thrustersNum) {
		//hide widgets
		for (size_t i{ thrustersNum }; i < widgetsNum; ++i) {
			//https://forum.godotengine.org/t/how-to-disable-enable-a-node/22387
			widgets[i]->hide();
			widgets[i]->set_process_mode(PROCESS_MODE_DISABLED);
		}
		widgets.resize(thrustersNum);
	}
	else {
		//spawn additional widgets
		widgets.reserve(thrustersNum);
		auto* owner{ get_owner() };
		for (size_t i{ widgetsNum }; i < thrustersNum; ++i) {
			widgets.push_back(Utils::NewChild<ThrusterWidget>(*this, owner));
		}
	}
	}

	widgetPairs.reserve(thrustersNum);
	auto thGroupsSize{ thGroups.size() };
	widgetPairs.clear();
	wgGroups.resize(thGroupsSize);
	int widgetsUsed{ 0 };
	for (size_t i{ 0 }; i < thGroupsSize; ++i) {
		wgGroups[i].clear();
		wgGroups[i].reserve(thGroups[i].size());//move up to improve exception safety?
		for (auto thruster : thGroups[i]) {
			wgGroups[i].emplace_back(thruster, widgets[widgetsUsed]);
			widgetPairs.emplace_back(thruster, widgets[widgetsUsed]);
			++widgetsUsed;
		}
		wgGroups[i].shrink_to_fit();
	}
	assert(widgetsUsed == widgets.size() && widgets.size() == thrustersNum);
}
