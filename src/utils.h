// by Dmitry Kolontay

#pragma once
#include <string_view>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/core/object.hpp>

#ifdef NDEBUG
#define DBG_PRINT(...) ((void)0)
#else
#define DBG_PRINT(...)  godot::UtilityFunctions::print(__VA_ARGS__) 
#endif

namespace Godot {
	class Vector2;
}

namespace Utils
{
	constexpr inline const double kindaSmallDouble{ 0.000001 };
	constexpr inline const float kindaSmallFloat{ static_cast<float>(kindaSmallDouble) };

	template<class R>
		requires std::is_floating_point_v<R>
	R KindaSmallReal() {
		return static_cast<R>(kindaSmallDouble);
	}
	
	template<>
	double KindaSmallReal();

	template<>
	float KindaSmallReal();

	constexpr inline const std::string_view bodySpriteName{ "bodySprite" };
	constexpr inline const std::string_view visibleEnclosingRectName{ "visibleEnclosingRect" };
	constexpr inline const std::string_view bodyShapeName{ "bodyShape" };
	constexpr inline const std::string_view emissionParticlesName{ "emissionParticles" };
	constexpr inline const std::string_view flameSpriteName{ "flameSprite" };
	constexpr inline const std::string_view thrusterLabelName{ "levelLabel" };
	constexpr inline const std::string_view thrusterMonitorName{ "thrusterMonitor" };
	constexpr inline const std::string_view linearAccMarkerName{ "linearAccMarker" };
	constexpr inline const std::string_view linearAccWidgetName{ "linearAccWidget" };
	constexpr inline const std::string_view torqueAccWidgetName{ "torqueAccWidget" };
	constexpr inline const std::string_view landingStrutLegBaseSpriteName{ "landingStrutLegBaseSprite" };
	constexpr inline const std::string_view landingStrutBodyName{ "landingStrutBody" };
	constexpr inline const std::string_view grooveJointName{ "grooveJoint" };
	constexpr inline const std::string_view springJointName{ "springJoint" };

	template <typename... Args>
	concept NonEmpty = sizeof...(Args) > 0;

	template<class T, class... Classes>
	concept AllSame = (std::is_same_v<T, Classes> && ...);

	template <int a, int b>
	concept SameInts = a == b;

	template <class TDerived, class TBase, bool allowSame = false>
	concept ChildOf = std::is_base_of_v<TBase, TDerived> &&
		(allowSame || !std::is_same_v<TBase, TDerived>);

	template <typename TDerived, template<typename> typename TBase>
	struct is_derived_from_any
	{
		template<typename TParam>
		static constexpr std::true_type is_derived(const TBase<TParam>&) noexcept;
		static constexpr std::false_type is_derived(...) noexcept;
		using type = decltype(is_derived(std::declval<TDerived&>()));
	};

	template <class TDerived, template<typename> typename TBase>
	using is_derived_from_any_t = is_derived_from_any<TDerived, TBase>::type;

	template <class TDerived, template<typename> typename TBase>
	inline constexpr bool is_derived_from_any_v{ is_derived_from_any_t<TDerived, TBase>::value };

	template <class TDerived, template<typename> typename TBase>
	concept ChildOfAny = is_derived_from_any_v<TDerived, TBase>;

	template <class Callable, class FArg>
	constexpr void ApplyToEach(Callable&& foo, FArg&& fArg) noexcept(
		noexcept(std::invoke(std::forward<Callable>(foo), std::forward<FArg>(fArg))))
	{
		std::invoke(std::forward<Callable>(foo), std::forward<FArg>(fArg));
	}

	template <class Callable, class FArg, class... Args> requires NonEmpty<Args...>
	constexpr void ApplyToEach(Callable&& foo, FArg&& fArg, Args&&... args) noexcept(
		noexcept(ApplyToEach(std::forward<Callable>(foo), std::forward<FArg>(fArg))) &&
		noexcept(ApplyToEach(std::forward<Callable>(foo), std::forward<Args>(args)...)))
	{
		ApplyToEach(std::forward<Callable>(foo), std::forward<FArg>(fArg));
		ApplyToEach(std::forward<Callable>(foo), std::forward<Args>(args)...);
	}

	template <class ChildType, class ParentType, class OwnerType>
	ChildType* NewChild(ParentType& parent, OwnerType* owner)
	{
		ChildType* out = memnew(ChildType);
		parent.add_child(out);
		out->set_owner(owner);
		return out;
	}

	template <class ChildType, class ParentType, class OwnerType>
	ChildType* NewChildWithName(ParentType& parent, OwnerType* owner, const char* childName)
	{
		ChildType* out = memnew(ChildType);
		out->set_name(childName);
		parent.add_child(out);
		out->set_owner(owner);
		return out;
	}

	template <class ChildType, class ParentType, class OwnerType, class Callable>
	ChildType* RetreiveChild(ParentType& parent, OwnerType* owner, const char* childName, Callable&& foo)
		requires(std::is_void_v<decltype(std::invoke(
			std::forward<Callable>(foo), std::forward<ChildType*>(std::declval<ChildType*>())))>)
	{
		using namespace godot;
		ChildType* child = Object::cast_to<ChildType>(parent.find_child(childName, false));
		if (!child) {
			child = NewChildWithName<ChildType>(parent, owner, childName);

			std::invoke(std::forward<Callable>(foo), std::forward<ChildType*>(child));
		} 
		else {
			child->set_owner(owner);
		}
		return child;
	}

	template <class ChildType, class ParentType, class OwnerType>
	ChildType* RetreiveChild(ParentType& parent, OwnerType* owner, const char* childName)
	{
		using namespace godot;
		ChildType* child = Object::cast_to<ChildType>(parent.find_child(childName, false));
		if (!child) {
			child = NewChildWithName<ChildType>(parent, owner, childName);
		}
		else {
			child->set_owner(owner);
		}
		return child;
	}

	template <class ChildType, class ParentType, class OwnerType, class Callable>
	void RetreiveThisChild(ChildType*& child, ParentType& parent, OwnerType* owner, const char* childName, Callable&& foo) {
		child = RetreiveChild<ChildType>(parent, owner, childName, std::forward<Callable>(foo));
	}

	template <class ChildType, class ParentType, class OwnerType>
	void RetreiveThisChild(ChildType*& child, ParentType& parent, OwnerType* owner, const char* childName) {
		child = RetreiveChild<ChildType>(parent, owner, childName);
	}

	template <class ChildType, class ParentType, class OwnerType, class Callable>
	void RetreiveThisChild(ChildType*& child, ParentType& parent, OwnerType* owner, const std::string_view& childName, Callable&& foo) {
		child = RetreiveChild<ChildType>(parent, owner, childName.data(), std::forward<Callable>(foo));
	}

	template <class ChildType, class ParentType, class OwnerType>
	void RetreiveThisChild(ChildType*& child, ParentType& parent, OwnerType* owner, const std::string_view& childName) {
		child = RetreiveChild<ChildType>(parent, owner, childName.data());
	}

	template <class ParentType, class OwnerType, class ChildType>
	void RetreiveTheeseChildren(ParentType& parent, OwnerType* owner, ChildType*& child, const std::string_view& childName) {
		child = RetreiveChild<ChildType>(parent, owner, childName.data());
	}

	template <class ParentType, class OwnerType, class ChildType, class... Args> requires NonEmpty<Args...>
	void RetreiveTheeseChildren(ParentType& parent, OwnerType* owner, ChildType*& child, const std::string_view& childName, Args&&... args)
	{
		RetreiveTheeseChildren(parent, owner, child, childName);
		RetreiveTheeseChildren(parent, owner, std::forward<Args>(args)...);
	}

	[[nodiscard]] bool IsInRect(const godot::Vector2& p, const godot::Vector2& rLU, const godot::Vector2& rRD);
	[[nodiscard]] bool IsInRect(const godot::Vector2& relLoc, const godot::Vector2& bound);

	[[nodiscard]] godot::Vector2 SqrtVec(const godot::Vector2& vec);
	[[nodiscard]] godot::Vector2 SqrtVec(const godot::Vector2& vec, real_t length);
	[[nodiscard]] double RadiansReminder(double radAngle);
	[[nodiscard]] double RadiansFmod(double radAngle);
	[[nodiscard]] double Square(double in);
};

