// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/texture_rect.hpp>


class LinearAccWidget : public godot::TextureRect
{
	GDCLASS(LinearAccWidget, godot::TextureRect)

protected:
	static void _bind_methods();

	godot::TextureRect* marker{ nullptr };
	godot::Vector2 centerOffset{ 0.f, 0.f };

	static inline constinit float scaleCoef{ 7.f };
	static inline constinit float markerMaxRadius{ 0.f };

public:
	virtual void _enter_tree() override;
	virtual void _ready() override;
	void RetreiveChildren(godot::Node* owner)&;

	void UpdateWidget(const godot::Vector2& accel)&;
	void static UpdateWidgetStatic(const godot::Vector2& accel, LinearAccWidget* widget);
	void ResetWidget()&;
};

