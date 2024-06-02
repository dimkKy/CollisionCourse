// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/texture_rect.hpp>
#include <vector>

namespace godot {
	class Label;
	class String;
}
class Thruster;
//class Thru

class ThrusterWidget : public godot::TextureRect
{
	GDCLASS(ThrusterWidget, godot::TextureRect)

protected:
	static void _bind_methods();
	
	static constexpr inline int minLevel{ -100 };
	static constexpr inline int maxLevel{ 100 };
	static_assert(maxLevel >= minLevel, "invalid range");
	static constexpr inline int levelsNum{ 1 + maxLevel - minLevel };

	static void GenerateLevelLabes();
	
	static inline std::vector<godot::String> levelLabels;
	static_assert(std::is_same_v<decltype(levelLabels)::value_type, godot::String>);

	godot::Label* label = nullptr;

public:
	ThrusterWidget();
	static void OnClassRegister();
	void RetreiveChildren(godot::Node* owner)&;
	
	void UpdateWidget(const Thruster& thruster)&;
	void UpdateDisplayedLevel(const Thruster& thruster)&;
	void SetDisplayedLevel(int level)&;
};

