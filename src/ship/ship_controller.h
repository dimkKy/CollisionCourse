// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include "ship/input_directions.h"

#include <array>
#include <vector>
#include <tuple>
#include <thread>

class Ship;
class ShipGUI;

namespace godot {
	template <class T>
	class Ref;
	class Viewport;
	struct Vector2;
	struct Vector3;
}


class ShipController : public godot::Node2D
{
	//GDCLASS(ShipController, godot::Node2D)
public:
	ShipController();
	ShipController(const ShipController& other) = delete;
	ShipController(ShipController&& other) = delete;
	ShipController& operator=(const ShipController& other) = delete;
	ShipController& operator=(ShipController&& other) = delete;
	virtual ~ShipController() override;

	virtual void Posess(Ship& newShip)&;
	virtual void Unposess()&;

	double GetThrusterRotationSpeed() const;

	//virtual void _enter_tree() override;
	virtual void _exit_tree() override;
	virtual void _ready() override;
	virtual void _process(double deltatime) override;
	//virtual void _unhandled_key_input(const godot::Ref<godot::InputEvent>& event) override;

	
	using vec2 = std::pair<double, double>;
	using vec3 = std::array<double, 3>;

	void SetDesiredLinearForce(const godot::Vector2& desiredSpeed);
	void SetDesiredLinearForce(const godot::Vector2& desiredSpeed, double physDeltatime);

	//x - rotation, y - thrust
	double Solve(const vec2& weights, const vec3& desiredEngineForces,
		std::vector<double>& X, std::vector<double>& Y);

	//x - rotation, y - thrust
	static double SubSolve(const vec2& weights, const vec3& desiredEngineForces,
		std::vector<double>& X, std::vector<double>& Y,
		const std::vector<double>& relX, const std::vector<double>& relY,
		const std::vector<vec2>& constrX, const std::vector<vec2>& constrY);
	//x - rotation, y - thrust
	static double SubSolveNoRotationConstraint(const vec2& weights, const vec3& desiredEngineForces,
		std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, 
		const std::vector<double>& relY, const std::vector<vec2>& constrY);
	//x - rotation, y - thrust
	static double SubSolveWithFixedRotation(const vec2& weights, const vec3& desiredEngineForces,
		const std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX,
		const std::vector<double>& relY, const std::vector<vec2>& constrY);

	static double SubSolvePrepareDataForOut(const vec2& weights, const std::vector<double>& X, const std::vector<double>& Y, 
		const std::vector<double>& relX, const std::vector<double>& relY, const vec3& desiredEngineForces, 
		std::vector<double>& ySinX, std::vector<double>& yCosX, vec3& sumMinDesired);

	//x - rotation, y - thrust
	static double SubPreSolveSingle(const vec2& weights, const vec3& desiredEngineForces,
		double& X, double Y, double relX, double relY);

	static void WorkerSolveStatic(std::stop_token stoken, double& outVal, std::atomic_flag& flag,
		const vec2& weights, const vec3& desiredEngineForces, std::vector<double>& X,
		std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY,
		const std::vector<vec2>& constrX, const std::vector<vec2>& constrY);
	
	void WorkerSolve(std::stop_token stoken, size_t workerNum);

	//static double RotationDerivative(const vec2& weights, const vec3& desiredEngineForces,
		//double y, double relX, double relY, double sinX, double cosX);

	static double RotationDerivative(const vec2& weights, const vec3& sumsMindes,
		double ycosX, double ysinX, double relX, double relY);

	static double ThrustDerivative(const vec2& weights, const vec3& sumsMindes,
		double sinX, double cosX, double relX, double relY);
protected:
	Ship* ship{ nullptr };

	//void 
	//bool bWorkersInited{ false };

	vec2 weights{};
	vec3 desiredEngineForces{};

	static constexpr size_t workerCount{ 7 };
	std::array<std::jthread, workerCount> workers;
	std::array<std::atomic_flag, workerCount> workerFlags;
	std::array<double, workerCount> workerOuts{};
	std::array<std::vector<double>, workerCount> workerXs;
	std::array<std::vector<double>, workerCount> workerYs;


	//std::jthread allRWorker;
	//std::jthread allLWorker;
	//std::jthread XLYRWorker;
	//std::jthread XRYLWorker;

	//need to ensure sync with solver
	std::vector<double> relPosX;
	std::vector<double> relPosY;

	std::vector<vec2> constrX;
	std::vector<vec2> constrY;

	bool bStopIssued{ false };
	bool bWorkersInited{ false };
};

