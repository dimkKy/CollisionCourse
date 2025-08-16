// by Dmitry Kolontay

#include "player_ship_controller.h"
#include "ship/ship.h"
#include "ship/thruster.h"
#include "gui/ship_gui.h"
#include <godot_cpp/classes/viewport.hpp>
//#include <godot_cpp/classes/input_event.hpp>
#include <godot_cpp/variant/transform2d.hpp>
#include <vector>
#include <cassert>
#include <algorithm>
#include "utils.h"
#include <godot_cpp/classes/input_event_key.hpp>
#include <godot_cpp/classes/canvas_layer.hpp>

void PlayerShipController::_bind_methods()
{

}

float PlayerShipController::CalcBaseScale(float shipAbsVel)
{
	assert(shipAbsVel >= 0.f);
	return std::clamp(shipAbsVel * velScaleCoef + baseScale, 
		scaleConstraint.first, scaleConstraint.second);
}

godot::Vector2 PlayerShipController::CalcSmoothed(const godot::Vector2& newVal, const godot::Vector2& oldVal)
{
	constexpr static float newWeight{ 0.35f };
	constexpr static float oldWeight{ 1.f - newWeight };

	//prevShipPos = newWeight * pos + oldWeight * prevShipPos;
	//prevShipPos += newWeight * (pos - prevShipPos);
	return newVal + newWeight * (newVal - oldVal);
}

float PlayerShipController::CalcSmoothed(float newVal, float oldVal)
{
	constexpr static float newWeight{ 0.35f };
	constexpr static float oldWeight{ 1.f - newWeight };

	//prevShipPos = newWeight * pos + oldWeight * prevShipPos;
	//prevShipPos += newWeight * (pos - prevShipPos);
	return oldVal * oldWeight + newWeight * newVal;
}

void PlayerShipController::_notification(int p_what)
{
	/*switch (p_what) {
		case NOTIFICATION_READY: {
			ship = Object::cast_to<Ship>(get_parent());
			break;
		}
		case NOTIFICATION_INTERNAL_PROCESS:
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			UpdateCamera();
			break;
		} 
		case NOTIFICATION_TRANSFORM_CHANGED: {
			if (!is_processing_internal() && !is_physics_processing_internal()) {
				//_update_scroll();
			}
			break;
		} 
		case NOTIFICATION_ENTER_TREE: {
			viewport = get_viewport();
			
			//canvas = get_canvas();


#ifdef TOOLS_ENABLED
			if (/*!is_part_of_edited_scene() &&* !viewport->get_camera_2d()) {
				Posess();
			}
#else
			if (!viewport->get_camera_2d()) {
				Posess();
		}
#endif
			
			//set process physics
			//_update_process_callback();
			//first = true;
			//_update_scroll();
			break;
		}
		case NOTIFICATION_EXIT_TREE: {
			/*if (is_current()) {
				clear_current();
			}
			viewport = nullptr;
			//callable_mp(this, &Camera2D::_reset_just_exited).call_deferred();
			break;
		} 
	}*/
}

void PlayerShipController::UpdateCamera(double deltatime)
{
	if (!is_inside_tree() || !viewport) {
		return;
	}

	viewport->set_canvas_transform(GetCameraTransform(deltatime));
}

godot::Transform2D PlayerShipController::GetCameraTransform(double deltatime)&
{
	assert(ship);

	//godot::Vector2 shipVel{ CalcSmoothedVel(ship->get_linear_velocity()) };
	godot::Vector2 shipVel{ ship->get_linear_velocity() };
	//prevShipVel = shipVel;

	{
		float newScale{ CalcBaseScale(ship->get_linear_velocity().length()) };
		if (std::abs(prevScale - newScale) > scaleDeadZone) {
			prevScale = CalcSmoothed(newScale, prevScale);
		}
	}
	/*if (std::abs(prevAbsVel - absVel) > velDeadZone) {
		prevScale = CalcBaseScale(absVel);
		prevAbsVel = absVel;
	}*/
	
	godot::Vector2 scaleVec{ prevScale, prevScale };
	godot::Vector2 scaledScreenSize{ GetCameraScreenSize() * prevScale };
	godot::Vector2 screenScaledHalfSize{ scaledScreenSize * 0.5f };

	//godot::Vector2 velOffset{ 0.f, 0.f };

	//godot::Vector2 desiredOffset{ 0.f, 0.f };

	//godot::Vector2 shipPos{ CalcSmoothed(ship->get_global_position()) };
	//godot::Vector2 shipPos{ CalcSmoothed(ship->get_global_position() + ship->GetVisibleEnclosingRectOffset()) };
	godot::Vector2 shipPos{ ship->get_global_position() + ship->GetVisibleEnclosingRectOffset() };
	//prevShipPos = shipPos;
	// deltatime?
	//godot::Vector2 shipNextPosWithVel{ shipPos + shipVel * deltatime };
	godot::Vector2 shipRelPos{ shipPos - prevCameraPos };
	//auto deadZoneOffset{ screenScaledHalfSize * cameraDeadZone };
	//float absVel{ shipVel.length() };

	prevEnclosingHalfRect = CalcSmoothed(ship->GetVisibleEnclosingHalfSize(ship->get_rotation()) * 1.25f, prevEnclosingHalfRect);
	godot::Vector2 limitingHalfSize{ screenScaledHalfSize - prevEnclosingHalfRect };

	if (auto velLength{ shipVel.length() }; velLength < Utils::kindaSmallFloat) {
		//standing still, shift to center slowly
		//DBG_PRINT("standing still, shift");
		if (auto shiftDist{ slowShiftSheed * deltatime };  shipRelPos.length() > shiftDist) {
			//shipRelPos.normalize();
			//prevCameraPos += shipRelPos * shiftDist;

			shipRelPos -= shipRelPos.normalized() * shiftDist;
		}
	}
	else {
		//DBG_PRINT("flying");
		if (velLength > velShiftThreshhold) {
			//shipRelPos -= shipVel * 0.01f/* * prevScale*/;
			//shipRelPos -= shipVel.normalized() * std::sqrt(velLength - velShiftThreshhold) * 0.5f;
			shipRelPos -= shipVel.normalized() * (velLength - velShiftThreshhold) * 0.015f;
		}
		//auto deadZoneOffset{ screenScaledHalfSize * cameraDeadZone };
		/*if (Utils::IsInRect(shipRelPos, deadZoneOffset)) {
			//do nothing
			prevCameraPos = CalcSmoothed(shipPos - shipRelPos, prevCameraPos);
		}
		else {*/
			//relative to limit
		/*prevEnclosingHalfRect = CalcSmoothed(ship->GetVisibleEnclosingHalfSize(ship->get_rotation()) * 1.25f, prevEnclosingHalfRect);
			godot::Vector2 limitingHalfSize{ screenScaledHalfSize - prevEnclosingHalfRect };*/
		//redo
			/*godot::Vector2 limitingHalfSize{ 
				screenScaledHalfSize.x - ship->GetVisibleEnclosingRadius(),
				screenScaledHalfSize.y - ship->GetVisibleEnclosingRadius(), };*/
			//shipRelPos = 

			//godot::Vector2 shipRelNextPos{ shipPos + shipVel/* * deltatime */};
			//godot::Vector2 shipNextRelPos{ shipNextPosWithVel - prevCameraPos };

			
			//shipNextPosRel = 
			//auto limitZoneOffset{ screenScaledHalfSize * cameraLimitZone };
			//ensure that is visible
			//shipRelPos.x = std::clamp(shipRelPos.x, -limitingHalfSize.x, limitingHalfSize.x);
			//shipRelPos.y = std::clamp(shipRelPos.y, -limitingHalfSize.y, limitingHalfSize.y);
			//prevCameraPos = prevShipPos - shipRelPos;
			//prevCameraPos = CalcSmoothed(shipPos - shipRelPos, prevCameraPos);
			//move camera to clamp in dead zone
			//godot::Vector2 relOffset{ screenScaledHalfSize, 0.f };
		//}
	}
	shipRelPos.x = std::clamp(shipRelPos.x, -limitingHalfSize.x, limitingHalfSize.x);
	shipRelPos.y = std::clamp(shipRelPos.y, -limitingHalfSize.y, limitingHalfSize.y);
	//prevCameraPos = prevShipPos - shipRelPos;
	prevCameraPos = CalcSmoothed(shipPos - shipRelPos, prevCameraPos);
	/*if (!Utils::IsInRect(shipRelPos, deadZoneOffset)) {
		//relative to limit

		godot::Vector2 shipNextPosWithVel{ shipPos + shipVel * deltatime };
		shipRelPos.x = std::clamp(shipRelPos.x, -deadZoneOffset.x, deadZoneOffset.x);
		shipRelPos.y = std::clamp(shipRelPos.y, -deadZoneOffset.y, deadZoneOffset.y);
		prevCameraPos = shipPos - shipRelPos;
			//move camera to clamp in dead zone
			//godot::Vector2 relOffset{ screenScaledHalfSize, 0.f };
	}
	else {
		if (absVel < Utils::kindaSmallFloat) {
			if (shipRelPos.length() > slowShiftSheed) {
				shipRelPos.normalize();
				prevCameraPos -= shipRelPos * slowShiftSheed;
			}			
		}
	}

	calc offset
	if (absVel > Utils::kindaSmallFloat) {
		//if shipPos in in dead zone - skip
		//godot::Vector2 shipPos{ CalcSmoothed(ship->get_global_position()) };
		godot::Vector2 shipPos{ CalcSmoothed(ship->get_global_position()) };
		auto deadZoneOffset{ screenScaledHalfSize * cameraDeadZone };
		if (!Utils::IsInRect(shipPos, prevCameraPos - deadZoneOffset, prevCameraPos + deadZoneOffset)) {
			 save relative offset
			//move camera 
			godot::Vector2 relOffset{ screenScaledHalfSize, 0.f };
		}
		else {
			//zoom only, skip
		}
		
		if (std::abs(desiredOffset.x) > cameraDeadZone.first) {
			prevCameraOffset.x = shipVel.x * velOffsetCoef;
		}
		if (std::abs(desiredOffset.y) > cameraDeadZone.second) {
			prevCameraOffset.y = shipVel.y * velOffsetCoef;
		}
	}
	else {
		//shift to 0.0 slowly
		godot::Vector2 desiredOffset{ 0.f, 0.f };
	}*/

	
	//TODO ADD SMOOTHING AND SHAKE OFFSET

	/*if (position_smoothing_enabled && !_is_editing_in_editor()) {
		real_t c = position_smoothing_speed * (process_callback == CAMERA2D_PROCESS_PHYSICS ? get_physics_process_delta_time() : get_process_delta_time());
		smoothed_camera_pos = ((camera_pos - smoothed_camera_pos) * c) + smoothed_camera_pos;
		ret_camera_pos = smoothed_camera_pos;
		//camera_pos=camera_pos*(1.0-position_smoothing_speed)+new_camera_pos*position_smoothing_speed;
	}
	else {
		ret_camera_pos = smoothed_camera_pos = camera_pos;
	}*/

	//screenRect.i

	godot::Transform2D xform;
	//TODO CONSTRUCTOR FROM 1 FLOAT
	xform.scale_basis(scaleVec);
	//prevCameraPos = 
	//xform.set_origin(prevCameraPos + prevCameraOffset * scaledScreenSize - screenScaledHalfSize);
	xform.set_origin(prevCameraPos - screenScaledHalfSize);
	//DBG_PRINT(prevCameraPos - screenScaledHalfSize);
	return xform.affine_inverse();
}

godot::Vector2 PlayerShipController::GetCameraScreenSize() const&
{
#ifdef TOOLS_ENABLED
	/*if (is_part_of_edited_scene()) {
		return Size2(GLOBAL_GET("display/window/size/viewport_width"), GLOBAL_GET("display/window/size/viewport_height"));
	}*/
#endif
	return get_viewport_rect().size;
}

ShipGUI* PlayerShipController::RetreiveShipGUI() const&
{
	//DBG_PRINT(get_owner());
	if (auto* canvas{ Object::cast_to<godot::CanvasLayer>(get_owner()->find_child("GUILayer", false)) }) {
		return Object::cast_to<ShipGUI>(canvas->find_child(ShipGUI::get_class_static(), false));
		//DBG_PRINT(canvas);
	}
	assert(false);
	return nullptr;
}

void PlayerShipController::OnThrusterGroupsChanged()
{
	size_t groupCount{ 0 };
	assert(thGroups.size());
	for (auto group : thGroups) {
		if (group > groupCount) {
			groupCount = group;
		}
	}
	//starts from 0
	groupCount += 1;
	inputStates.resize(groupCount + 1);
}

PlayerShipController::PlayerShipController()
{
	set_process_unhandled_key_input(false);
	//set_process_mode(PROCESS_MODE_DISABLED);
}

void PlayerShipController::Posess(Ship& newShip)&
{
	ShipController::Posess(newShip);
	//ship->set_process_unhandled_key_input(true);
	//update

	shipSizeRadius = ship->GetSpriteRadius();
	//DBG_PRINT(shipSizeRadius);
	//ship->set_process_mode(PROCESS_MODE_INHERIT);
	set_process_unhandled_key_input(true);
	//find ship gui
	shipGUI = RetreiveShipGUI();
	if (shipGUI) {
		shipGUI->SetShip(newShip);
	}
	//prevCameraPos = ship->get_global_position();
	UpdateCamera(0.);

	auto& thrusters{ ship->GetThrusters() };
	auto thrustersSize{ thrusters.size() };
	thGroups.resize(thrustersSize);
	for (size_t i{ 0 }; i < thrustersSize; ++i) {
		thGroups[i] = thrusters[i]->GetThGroup();
	}

	OnThrusterGroupsChanged();
}

void PlayerShipController::Unposess()&
{
	ShipController::Unposess();
	//ship->set_process_unhandled_key_input(false);
	//update
	if (ship) {
		shipGUI->OnUnposess(*ship);
	}
	//ship->set_process_mode(PROCESS_MODE_DISABLED);
	set_process_unhandled_key_input(false);
}

int PlayerShipController::GetVerticalInput() const&
{
	return inputStates[0].test(static_cast<size_t>(InDir::Up)) -
		inputStates[0].test(static_cast<size_t>(InDir::Down));
}

int PlayerShipController::GetVerticalInput(size_t thGroup) const&
{
	assert(thGroup >= 0 && thGroup <= static_cast<int>(inputStates.size()));


	switch (inputStates[thGroup + 1].test(static_cast<size_t>(InDir::Up)) -
		inputStates[thGroup + 1].test(static_cast<size_t>(InDir::Down)) +
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

int PlayerShipController::GetHorizontalInput() const&
{
	return inputStates[0].test(static_cast<size_t>(InDir::Right)) -
		inputStates[0].test(static_cast<size_t>(InDir::Left));
}

int PlayerShipController::GetHorizontalInput(size_t thGroup) const&
{
	assert(thGroup >= 0 && thGroup <= inputStates.size());

	switch (inputStates[thGroup + 1].test(static_cast<size_t>(InDir::Right)) -
		inputStates[thGroup + 1].test(static_cast<size_t>(InDir::Left)) +
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
}

void PlayerShipController::_enter_tree()
{
	ShipController::_enter_tree();
	DBG_PRINT("PlayerShipController::_enter_tree");
	viewport = get_viewport();
}

void PlayerShipController::_exit_tree()
{
	ShipController::_exit_tree();
	DBG_PRINT("PlayerShipController::_exit_tree");
	viewport = nullptr;
}

void PlayerShipController::_ready()
{
	ShipController::_ready();
	DBG_PRINT("PlayerShipController::_ready");
	
}

void PlayerShipController::_process(double deltatime)
{
	UpdateCamera(deltatime);
}

void PlayerShipController::_physics_process(double deltatime)
{
	if (bStopIssued) {
		double linWeight{ 0.9999999 };
		weights = vec2{ linWeight, 1. - linWeight };
		//recalc

		CalcSetDesiredLinearForce({ 0., 0. }, deltatime);
		CalcSetDesiredTorquByRotation(0., deltatime);
		//DBG_PRINT(desiredEngineForces);
		//desiredEngineForces = vec3{ 0., 0., 0. };
		assert(ship);
		auto& thrusters{ ship->GetThrusters() };
		size_t thrustersSize{ thrusters.size() };

		//alloca on stack - try
		std::vector<double> X{ relPosX };
		std::vector<double> Y{ relPosX };

		auto xDelta{ deltatime * GetThrusterRotationSpeed() };
		auto yDelta{ deltatime * GetThrusterPowerChangeSpeed() };

		for (size_t i{ 0 }; i < thrustersSize; ++i) {
			X[i] = thrusters[i]->get_rotation();
			Y[i] = thrusters[i]->GetThrust();

			constrX[i].first = X[i] - xDelta;
			constrX[i].second = X[i] + xDelta;


			thrusters[i]->GetThrustDelta(yDelta, constrY[i].first, constrY[i].second);
			

			//constrY[i].first 
			//constrY[i].second = /Y[i] +*/ thrusters[i]->GetMaxThrust();
		}

		SolveV2(X, Y);

		for (size_t i{ 0 }; i < thrustersSize; ++i) {
			thrusters[i]->set_rotation(X[i]);
			thrusters[i]->SetPowerFromThrust(Y[i]);
			//thrusters[i]->SetZeroPower();
		}
		//ship->apply_force(godot::Vector2(desiredEngineForces[0], desiredEngineForces[1]));
		//ship->apply_torque(desiredEngineForces[2]);
		return;
	}

	switch (controlMode) {
	case ShipControlMode::Direct:
	{
		auto& thrusters{ ship->GetThrusters() };
		for (int i{ 0 }; i < thrusters.size(); ++i) {
			if (int input{ GetHorizontalInput(thGroups[i]) }) {
				thrusters[i]->rotate(deltatime * input * GetThrusterRotationSpeed());
				//notifications?
			}

			

			if (int input{ GetVerticalInput(thGroups[i]) }) {
				thrusters[i]->ChangePowerLevel(deltatime * input * GetThrusterPowerChangeSpeed());
				//notifications?
			}
		}
		break;
	}
	case ShipControlMode::WASD:
		break;
	default:
		break;
	}
}

void PlayerShipController::_unhandled_key_input(const godot::Ref<godot::InputEvent>& event)
{
	//DBG_PRINT("PlayerShipController::_unhandled_key_input");
	if (event->is_echo()) {
		return;
	}

	if (auto* action{ Object::cast_to<godot::InputEventKey>(*event) }) {
		//action->get_keycode_with_modifiers
		if (auto mapped{ specialInputMap.find(action->get_keycode()) };
			mapped != specialInputMap.end()) {

			switch (mapped->second) {
			case SpecialInputs::IssueStop:
			{
				bStopIssued = true;
				get_viewport()->set_input_as_handled();
				return;
			}
			default:
				break;
			}
		}
	}


	switch (controlMode) {
	case ShipControlMode::Direct:
	{
		if (auto * action{ Object::cast_to<godot::InputEventKey>(*event) }) {
			//action->get_keycode_with_modifiers
			if (auto mapped{ inputMap.find(action->get_keycode()) };
				mapped != inputMap.end()) {

				//DBG_PRINT(event.ptr());
				ChangeInputState(mapped->second.first, mapped->second.second, action->is_pressed());
				//godot::Viewport
				get_viewport()->set_input_as_handled();
				bStopIssued = false;
			}
		}
		break;
	}
	case ShipControlMode::WASD:

		break;
	default:
		break;
	}

	//change input info
	
	//not keys
}

void PlayerShipController::ChangeInputState(InputDirection dir, bool isPressed)&
{
	//inputStates.test(static_cast<size_t>(InDir::Up)) -
		//inputStates.test(static_cast<size_t>(InDir::Down)

	inputStates[0][static_cast<size_t>(dir)] = isPressed;
	//inputStates.flip(static_cast<size_t>(dir));
}

void PlayerShipController::ChangeInputState(InputDirection dir, int thGroup, bool isPressed)&
{
	//I allow -1 here to be consistent in inputMap
	assert(thGroup >= -1 && thGroup <= static_cast<int>(inputStates.size()));
	//inputStates.flip(4 * (thGroup + 1) + static_cast<size_t>(dir));
	inputStates[static_cast<size_t>(thGroup + 1)][static_cast<size_t>(dir)] = isPressed;
}


/*void PlayerShipController::RotateThrusters(double delta)&
{
	assert(ship);
	ship->RotateThrusters(delta);
}

void PlayerShipController::RotateThrusters(double delta, size_t thGroup)&
{
	assert(ship);
	for (size_t i{ 0 }; i < thGroups.size(); ++i) {
		if (thGroups[i] == thGroup) {
			ship->RotateThruster(delta, i);
		}
	}

	/*auto rotator = [delta, thGroup](Thruster* th)
		noexcept(noexcept(th->rotate(delta))) -> void
		{ th->rotate(delta); };*
}

void PlayerShipController::AddThrust(double delta)&
{
	assert(ship);
	ship->AddThrust(delta);
}

void PlayerShipController::AddThrust(double delta, size_t thGroup)&
{
	assert(ship);
	for (size_t i{ 0 }; i < thGroups.size(); ++i) {
		if (thGroups[i] == thGroup) {
			//ship->AddThrust(delta, i);
		}
	}
}*/
