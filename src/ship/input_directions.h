// by Dmitry Kolontay
#pragma once

enum class InputDirection
{
	Left,
	Up,
	Right,
	Down,
};

namespace InDir {
	constexpr inline const InputDirection Left{ InputDirection::Left };
	constexpr inline const InputDirection Up{ InputDirection::Up };
	constexpr inline const InputDirection Right{ InputDirection::Right };
	constexpr inline const InputDirection Down{ InputDirection::Down };
}
