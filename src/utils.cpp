// by Dmitry Kolontay

#include "utils.h"
#include <godot_cpp/variant/vector2.hpp>
#include <cassert>
#include <numbers>

bool Utils::IsInRect(const godot::Vector2& p, const godot::Vector2& rLU, const godot::Vector2& rRD)
{
    return p.x >= rLU.x && p.x <= rRD.x &&
        p.y >= rLU.y && p.y <= rRD.y;
}

bool Utils::IsInRect(const godot::Vector2& relLoc, const godot::Vector2& bound)
{
    return std::abs(relLoc.x) <= bound.x &&
        std::abs(relLoc.y) <= bound.y;
}

template<>
float Utils::KindaSmallReal() {
    return kindaSmallFloat;
}

template<>
double Utils::KindaSmallReal() {
    return kindaSmallDouble;
}

godot::Vector2 Utils::SqrtVec(const godot::Vector2& vec)
{
    /*using namespace std;
    return { vec.x >= static_cast<real_t>(0.f) ? sqrt(vec.x) : -sqrt(-vec.x), 
        vec.y >= static_cast<real_t>(0.f) ? sqrt(vec.y) : -sqrt(-vec.y)};*/
    if (auto length{ vec.length() }; length < KindaSmallReal<decltype(length)>()) {
        return vec;
    }
    else {
        return vec.normalized() * std::sqrt(length);
    }

}

godot::Vector2 Utils::SqrtVec(const godot::Vector2& vec, real_t length)
{
    assert(std::abs(vec.length() - length) <= kindaSmallFloat);
    if (length < KindaSmallReal<decltype(length)>()) {
        return vec;
    }
    else {
        return vec.normalized() * std::sqrt(length);
    }
}

double Utils::RadiansReminder(double radAngle)
{
    return std::remainder(radAngle, std::numbers::pi * 2.);
}

double Utils::RadiansFmod(double radAngle)
{
    return std::fmod(radAngle, std::numbers::pi * 2.);
}

double Utils::Square(double in)
{
    return in * in;
}
