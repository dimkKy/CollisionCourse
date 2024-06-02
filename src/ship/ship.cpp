// by Dmitry Kolontay

#include "ship.h"
#include "ship/thruster.h"
#include "ship/landing_strut.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/animated_sprite2d.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/rectangle_shape2d.hpp>
#include <godot_cpp/classes/collision_shape2d.hpp>
#include <godot_cpp/classes/collision_polygon2d.hpp>
#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/classes/sprite_frames.hpp>
#include <godot_cpp/classes/ref.hpp>
#include "utils.h"
#include <cassert>
#include <numeric>
#include <execution>
#include <godot_cpp/classes/input_event_key.hpp>


Ship::Ship()
{
	//add_child

	set_process_mode(PROCESS_MODE_DISABLED);
	set_physics_process(false);
	//set_physics_process
}

/*std::unordered_map<godot::StringName, std::pair<InputDirection, int>> Ship::GenerateActionMap()
{
	std::unordered_map<godot::StringName, std::pair<InputDirection, int>> out;
	out.rehash(4 * (maxThGroupCount + 1));

	static const std::string base{ "thrustInput" };
	for (int i{ 0 }; i < 4; ++i) {
		out.insert({ (base + std::to_string(i)).c_str(),
			{static_cast<InputDirection>(i), -1} });

		for (int g{ 0 }; g < maxThGroupCount; ++g) {
			out.insert({
				(base + std::to_string(i) + std::to_string(g)).c_str(),
				{ static_cast<InputDirection>(i), g}
				});
		}
	}

	return out;
}*/

//redo
void Ship::_bind_methods()
{
	using namespace godot;
	//ClassDB::bind_method(D_METHOD("add_thruster", "thruster_group"), &Ship::AddThruster);
	//ClassDB::bind_method(D_METHOD("dummy_getter"), &Ship::DummyGetter);
	//get_class_static()
	//ADD_PROPERTY(PropertyInfo(Variant::INT, "thruster_group"), "add_thruster", "dummy_getter");
	
}

void Ship::_enter_tree()
{
	DBG_PRINT("Ship::_enter_tree");
	RetreiveChildren(get_owner());
}

void Ship::_exit_tree()
{
	//Super::_exit_tree();

	DBG_PRINT("Ship::_exit_tree");
	//godot::StringName
	//godot::InputMap::get_singleton()->add_action("");
}

void Ship::_ready()
{
	//Super::_ready();
	DBG_PRINT("Ship::_ready");
	//DBG_PRINT(get_tree()->get_root());
	//DBG_PRINT(get_owner()->get_children());
	RetreiveChildren(get_owner());
	set_physics_process(true);
}

void Ship::_physics_process(double deltatime)
{
	//Super::_physics_process
	// TODO REWRITE
	//godot::PhysicsServer2D::get_singleton()->body_apply_force(get_rid(), p_force, p_position);
	/*for (int group{ 0 }; group < maxThGroupCount; ++group) {

		if (int input{ GetHorizontalInput(group) }) {
			auto rotator = [deltatime, input](Thruster* t)
				noexcept(noexcept(t->rotate(deltatime))) -> void
					{ t->rotate(deltatime * input); };

			std::for_each(std::execution::unseq,
				thGroups[group].begin(), thGroups[group].end(), rotator);
		}
		//a bit ugly
		if (int input{ GetVerticalInput(group) }) {
			auto powerLever = [deltatime, input](Thruster* t)
				noexcept(noexcept(t->ChangePowerLevel(deltatime))) -> void
					{ t->ChangePowerLevel(deltatime * input); };

			std::for_each(std::execution::unseq,
				thGroups[group].begin(), thGroups[group].end(), powerLever);
		}
	}*/
	//floats?
	auto draw{ CalcPowerDraw() };
	//TODO
	auto availablePower{ draw };
	//TODO
	if (draw - availablePower > Utils::kindaSmallFloat) {
		auto powerReducer = [mod{ availablePower / draw }](Thruster* t)
			noexcept(noexcept(t->SetPowerRelative(1.))) -> void
				{ t->SetPowerRelative(mod); };

		std::for_each(std::execution::unseq,
			thrusters.begin(), thrusters.end(), powerReducer);
	}

	auto vec{ CalcForceTorque()/* + CalcStrutsPush()*/ };

	auto* physServ{ godot::PhysicsServer2D::get_singleton() };
	
	physServ->body_apply_torque(get_rid(), vec.z);

	//physServ->body_get_state()

	godot::Vector2 linearVec{ vec.x, vec.y };
	linearVec = linearVec.rotated(get_global_rotation());

	physServ->body_apply_central_force(get_rid(), linearVec);

	auto* state{ physServ->body_get_direct_state(get_rid()) };

	//godot::PhysicsServer2D::get_singleton()->body_get_direct_state(get_rid())->get_total_angular_damp
	if (onLinearAccChanged) [[likely]] {
		auto dampForce{ state->get_total_linear_damp() * state->get_linear_velocity() };
		onLinearAccChanged(linearVec / get_mass() + state->get_total_gravity() - dampForce);

	}

	if (onTorqueAccChanged) [[likely]] {
		auto dampForce{ state->get_total_angular_damp() * state->get_angular_velocity() };
		onTorqueAccChanged(vec.z / get_mass() - dampForce);
	}

}

void Ship::RetreiveChildren(godot::Node* owner)&
{
	set_owner(owner);
	thrusters.clear();

	/*static auto clearer = [](std::vector<Thruster*>& t)
		noexcept(noexcept(t.clear())) -> void { t.clear(); };

	std::for_each(std::execution::unseq,
		thGroups.begin(), thGroups.end(), clearer);*/

	using namespace godot;

	{
	auto children{ find_children(
		Thruster::GetNamePattern(), Thruster::get_class_static(), false) };
	auto childrenSize{ children.size() };

	if (childrenSize) [[likely]] {
		thrusters.reserve(childrenSize);
		//no iterators ffs
		for (int i{ 0 }; i < childrenSize; ++i) {
			if (auto* th{ Object::cast_to<Thruster>(children[i]) }) {
				th->RetreiveChildren(get_owner());
				thrusters.push_back(th);
#ifndef NDEBUG
				//int thGroup{ th->GetThGroup() };
				//assert(thGroup >= 0 && thGroup < maxThGroupCount);
#endif // NDEBUG
				//thGroups[th->GetThGroup()].push_back(th);
			}
		}
	}
	}

	/*static auto shrinker = [](std::vector<Thruster*>& t)
		noexcept(noexcept(t.shrink_to_fit())) -> void { t.shrink_to_fit(); };

	std::for_each(std::execution::unseq,
		thGroups.begin(), thGroups.end(), shrinker);
	thrusters.shrink_to_fit();*/

	using namespace Utils;
	RetreiveTheeseChildren(*this, owner,
		bodySprite, bodySpriteName,
		bodyShape, bodyShapeName);

	RetreiveThisChild(visibleEnclosingRect, *this, owner, visibleEnclosingRectName,
		[](auto* bodyShapeT) { bodyShapeT->set_shape(memnew(RectangleShape2D)); }
	);
	visibleEnclosingRect->set_disabled(true);

	auto rectHalfSize{ GetVisibleEnclosingRect() * 0.5f };
	visibleEnclosingRectLLrel.x = -rectHalfSize.x;
	visibleEnclosingRectLLrel.y = -rectHalfSize.y;

	visibleEnclosingRectLUrel.x = -rectHalfSize.x;
	visibleEnclosingRectLUrel.y = rectHalfSize.y;
	/*landingStrut1 = Object::cast_to<LandingStrut>(find_child("landingStrut_0"));
	landingStrut2 = Object::cast_to<LandingStrut>(find_child("landingStrut_1"));
	landingStrut1->RetreiveChildrenFromShip(owner);
	landingStrut2->RetreiveChildrenFromShip(owner);*/

	//Utils::RetreiveThisChild(landingStrut, *this, owner, LandingStrut::get_class_static());
}

float Ship::CalcPowerDraw() const& noexcept
{
	static auto adder = [](float a, const Thruster* t)
		noexcept(noexcept(t->GetDraw()))
		{ return a + t->GetDraw(); };

	return std::reduce(std::execution::unseq,
		thrusters.begin(), thrusters.end(), 0.f, adder);
}

float Ship::CalcTorque() const&
{	
 	static auto adder = [](float a, const Thruster* t) {
		return a + t->get_position().cross(
			godot::Vector2::from_angle(t->get_rotation()) * t->GetThrust());
	};

	return std::reduce(std::execution::unseq,
		thrusters.begin(), thrusters.end(), 0.f, adder);
}

godot::Vector2 Ship::CalcForce() const&
{
	static auto adder = [](godot::Vector2 a, const Thruster* t) {
		return a + godot::Vector2::from_angle(t->get_rotation()) * t->GetThrust();
	};

	return std::reduce(std::execution::unseq,
		thrusters.begin(), thrusters.end(), godot::Vector2{}, adder);
}

godot::Vector2 Ship::GetVisibleEnclosingRect() const&
{
	return static_cast<godot::RectangleShape2D*>(visibleEnclosingRect->get_shape().ptr())->get_size();
}

godot::Vector2 Ship::GetVisibleEnclosingRectOffset() const&
{
	return visibleEnclosingRect->get_position();
}

godot::Vector2 Ship::GetVisibleEnclosingHalfSize() const&
{
	return GetVisibleEnclosingRect() * 0.5f;
}

real_t Ship::GetVisibleEnclosingRadius() const&
{
	return GetVisibleEnclosingHalfSize().length();
}

godot::Vector2 Ship::GetVisibleEnclosingHalfSize(real_t angl) const&
{
	auto rotatedLeftLow{ visibleEnclosingRectLLrel.rotated(angl) };
	auto rotatedLeftUp{ visibleEnclosingRectLUrel.rotated(angl) };
	return {std::max(std::abs(rotatedLeftLow.x), std::abs(rotatedLeftUp.x)), 
		std::max(std::abs(rotatedLeftLow.y), std::abs(rotatedLeftUp.y)) };
}

const std::vector<Thruster*>& Ship::GetThrusters() const&
{
	return thrusters;
}

godot::Vector3 Ship::CalcForceTorque() const&
{
	static auto adder = [](godot::Vector3 a, const Thruster* t) {
		//t->get_global_rotation()
		//auto vec{ godot::Vector2::from_angle(t->get_global_rotation()) * t->GetThrust() };
		auto vec{ godot::Vector2::from_angle(t->get_rotation()) * t->GetThrust() };
		return a + godot::Vector3{ vec.x, vec.y, t->get_position().cross(vec) };
	};

	return std::reduce(std::execution::unseq,
		thrusters.begin(), thrusters.end(), godot::Vector3{}, adder);
}

/*godot::Vector3 Ship::CalcStrutsPush() const&
{
	auto* physServ{ godot::PhysicsServer2D::get_singleton() };
	//godot::PhysicsDirectBodyState2D state;
	static auto adder = [physServ](godot::Vector3 a, const LandingStrut* t) {
		//t->get_global_rotation()
		auto* state{ physServ->body_get_direct_state(t->get_rid()) };
		if (state->get_contact_count()) {
			auto vec{ t->GetStrutAxis().rotated(t->get_rotation()) * -100000.f * t->CalcOffsetFromMax() };
			//DBG_PRINT(vec);
			return a + godot::Vector3{ vec.x, vec.y, t->get_position().cross(vec) };
		}
		else {
			return a;
		}
	};
	return adder(adder({}, landingStrut2), landingStrut1);
}

void Ship::AddThruster(size_t thGroup)
{
	if (thGroup < 0 || thGroup >= maxThGroupCount) [[unlikely]] {
		return;
	}
	//assert(thGroup >= 0 && thGroup < maxThGroupCount);
	if (!stateInfo.test(inputInfoBitsize)) {
		auto* owner{ get_owner() };
		//try?
		bodySprite = memnew(std::remove_pointer_t<decltype(bodySprite)>);
		bodyShape = memnew(std::remove_pointer_t<decltype(bodyShape)>);

		bodySprite->set_name(Utils::bodySpriteName.data());
		add_child(bodySprite);
		bodySprite->set_owner(owner);

		bodyShape->set_name(Utils::bodyShapeName.data());
		add_child(bodyShape);
		bodyShape->set_owner(owner);

		stateInfo.set(inputInfoBitsize);
	}

	auto* thruster{ memnew(Thruster) };
	thrusters.push_back(thruster);

	thruster->SetThrusterName(thGroup, thGroups[thGroup].size());
	thGroups[thGroup].push_back(thruster);

	add_child(thruster);
	thruster->RetreiveChildren(get_owner());

#ifndef NDEBUG
	print_tree_pretty();
#endif // NDEBUG
}*/

/*void Ship::ChangeInputState(InputDirection dir, bool isPressed)&
{
	//stateInfo.test(static_cast<size_t>(InDir::Up)) -
		//stateInfo.test(static_cast<size_t>(InDir::Down)
	stateInfo[static_cast<size_t>(dir)] = isPressed;
	//stateInfo.flip(static_cast<size_t>(dir));
}

void Ship::ChangeInputState(InputDirection dir, int thGroup, bool isPressed)&
{
	//I allow -1 here to be consistent in inputMap
	assert(thGroup >= -1 && thGroup < maxThGroupCount);
	//stateInfo.flip(4 * (thGroup + 1) + static_cast<size_t>(dir));
	stateInfo[4 * (static_cast<size_t>(thGroup + 1)) + static_cast<size_t>(dir)] = isPressed;
}

void Ship::RemoveThruster(size_t thGroup)&
{
	assert(thGroup >= 0 && thGroup < maxThGroupCount);
	//TODO
}

const Ship::thGroupsArrayType Ship::GetThGroups() const&
{
	return thGroups;
}*/

int Ship::DummyGetter() const noexcept
{
	return -1;
}

float Ship::GetSpriteRadius() const&
{
	if (bodySprite) {
		if (auto frames{ *bodySprite->get_sprite_frames() }) {
			auto anims{ frames->get_animation_names() };
			if (anims.size()) {
				return frames->get_frame_texture(anims[0], 0)->get_size().length();
			}
		}
	}
	return 0.f;
}
/*

int Ship::GetVerticalInput() const&
{
	return stateInfo.test(static_cast<size_t>(InDir::Up)) - 
		stateInfo.test(static_cast<size_t>(InDir::Down));
}

int Ship::GetVerticalInput(size_t thGroup) const&
{
	assert(thGroup >= 0 && thGroup < maxThGroupCount);

	size_t groupOffset{ 4 * (thGroup + 1) };

	switch (stateInfo.test(groupOffset + static_cast<size_t>(InDir::Up)) -
		stateInfo.test(groupOffset + static_cast<size_t>(InDir::Down)) +
		GetVerticalInput()) {
	case -2:
		return -1;
	case -1:
		return -1;
	case 1:
		return 1;
	case 2:
		return 1;
	default:
		[[fallthrough]];
	case 0:
		return 0;
	}
}

int Ship::GetHorizontalInput() const&
{
	return stateInfo.test(static_cast<size_t>(InDir::Right)) -
		stateInfo.test(static_cast<size_t>(InDir::Left));
}

int Ship::GetHorizontalInput(size_t thGroup) const&
{
	assert(thGroup >= 0 && thGroup < maxThGroupCount);

	size_t groupOffset{ 4 * (thGroup + 1) };

	switch (stateInfo.test(groupOffset + static_cast<size_t>(InDir::Right)) -
		stateInfo.test(groupOffset + static_cast<size_t>(InDir::Left)) +
		GetHorizontalInput()) {
	case -2:
		return -1;
	case -1:
		return -1;
	case 1:
		return 1;
	case 2:
		return 1;
	default:
		[[fallthrough]];
	case 0:
		return 0;
	}
}*/

void Ship::_process(double delta)
{
	//RigidBody2D::_process(delta);
	//power?
	/*using namespace godot;
	if (Input::get_singleton()->is_physical_key_pressed(Key::KEY_D)) {
		//RotateThrusters(delta);
		//DBG_PRINT("RotateThrusters");
	}*/
}

void Ship::RotateThrusters(double delta)&
{
	auto rotator = [delta](Thruster* th)
		noexcept(noexcept(th->rotate(delta))) -> void
			{ th->rotate(delta); };

	std::for_each(std::execution::unseq, 
		thrusters.begin(), thrusters.end(), rotator);

	if (onThrusterStateChanged) [[likely]] {
		onThrusterStateChanged(-1);
	}
}

void Ship::RotateThrusterNoNotify(double delta, size_t thNum)&
{
	assert(thNum < thrusters.size());
	thrusters[thNum]->rotate(delta);
}

void Ship::RotateThruster(double delta, size_t thNum)&
{
	RotateThruster(delta, thNum);
	if (onThrusterStateChanged) [[likely]] {
		onThrusterStateChanged(thNum);
	}
}

/*void Ship::RotateThrusters(double delta, size_t thGroup)&
{
	assert(thGroup >= 0 && thGroup < maxThGroupCount);
	auto rotator = [delta](Thruster* th) 
		noexcept(noexcept(th->rotate(delta))) -> void
			{ th->rotate(delta); };

	std::for_each(std::execution::unseq, 
		thGroups[thGroup].begin(), thGroups[thGroup].end(), rotator);

	if (onThrusterStateChanged) [[likely]] {
		onThrusterStateChanged(thGroup);
	}
}*/

void Ship::AddThrust(double delta)&
{
	auto adder = [delta](Thruster* th)
		noexcept(noexcept(th->ChangePowerLevel(delta))) -> void
			{ th->ChangePowerLevel(delta); };

	std::for_each(std::execution::unseq, 
		thrusters.begin(), thrusters.end(), adder);

	if (onThrusterStateChanged) [[likely]] {
		onThrusterStateChanged(-1);
	}
}

void Ship::AddThrustNoNotify(double delta, size_t thNum)&
{
	assert(thNum < thrusters.size());
	thrusters[thNum]->ChangePowerLevel(delta);
}

void Ship::AddThrust(double delta, size_t thNum)&
{
	AddThrustNoNotify(delta, thNum);
	if (onThrusterStateChanged) [[likely]] {
		onThrusterStateChanged(thNum);
	}
}

/*void Ship::AddThrust(double deltaLevel, size_t thGroup)&
{
	assert(thGroup >= 0 && thGroup < maxThGroupCount);
	auto adder = [deltaLevel](Thruster* th)
		noexcept(noexcept(th->ChangePowerLevel(deltaLevel))) -> void
		{ th->ChangePowerLevel(deltaLevel); };

	std::for_each(std::execution::unseq,
		thGroups[thGroup].begin(), thGroups[thGroup].end(), adder);

	if (onThrusterStateChanged) [[likely]] {
		onThrusterStateChanged(thGroup);
	}
}*/
