// by Dmitry Kolontay

#pragma once

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include "ship/input_directions.h"

#include <array>
#include <vector>
#include <tuple>
#include <thread>
#include <numbers>

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
	double GetThrusterPowerChangeSpeed() const;

	//virtual void _enter_tree() override;
	virtual void _exit_tree() override;
	virtual void _ready() override;
	virtual void _process(double deltatime) override;
	//virtual void _unhandled_key_input(const godot::Ref<godot::InputEvent>& event) override;

	
	using vec2 = std::pair<double, double>;
	using vec3 = std::array<double, 3>;

	void CalcSetDesiredLinearForce(const godot::Vector2& desiredSpeed);
	void CalcSetDesiredLinearForce(const godot::Vector2& desiredSpeed, double physDeltatime);

	void CalcSetDesiredTorquByRotation(real_t desiredRotation);
	void CalcSetDesiredTorquByRotation(real_t desiredRotation, double physDeltatime);

protected:
	//x - rotation, y - thrust
	double Solve(std::vector<double>& X, std::vector<double>& Y)&;
	double SolveV2(std::vector<double>& X, std::vector<double>& Y)&;

	//x - rotation, y - thrust
	static double SubSolve(const vec2& weights, const vec3& desiredForces,
		std::vector<double>& X, std::vector<double>& Y,
		const std::vector<double>& relX, const std::vector<double>& relY,
		const std::vector<vec2>& constrX, const std::vector<vec2>& constrY);
	//x - rotation, y - thrust
	static double SubSolveNoRotationConstraint(const vec2& weights, const vec3& desiredForces,
		std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, 
		const std::vector<double>& relY, const std::vector<vec2>& constrY);
	//x - rotation, y - thrust
	static double SubSolveWithFixedRotation(const vec2& weights, const vec3& desiredForces,
		const std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX,
		const std::vector<double>& relY, const std::vector<vec2>& constrY);

	/*static double SubSolveWithFixedRotationSingle(const vec2& weights, const vec3& desiredForces,
		double& X, double& Y, double relX,
		double relY, const vec2& constrY);*/


	static double SubSolvePrepareDataForOut(const vec2& weights, const std::vector<double>& X, const std::vector<double>& Y, 
		const std::vector<double>& relX, const std::vector<double>& relY, const vec3& desiredForces, 
		std::vector<double>& ySinX, std::vector<double>& yCosX, vec3& sumMinDesired);

	static double SubSolvePrepareDataForOut(const vec2& weights, const vec3& sumMinDesired);

	static void CalcSumsMinusDesired(vec3& out, const vec3& desiredForces,
		const std::vector<double>& ySinX, const std::vector<double>& yCosX,
		const std::vector<double>& relX, const std::vector<double>& relY);

	static void CalcSumsMinusDesired(vec3& out, const vec3& desiredForces,
		double ySinX, double yCosX, double relX, double relY);

	static void PreUpdateSumsMinusDesired(vec3& out, double yCosX, double ySinX, double relX, double relY);
	static void PostUpdateSumsMinusDesired(vec3& out, double yCosX, double ySinX, double relX, double relY);

	//curY will store squares
	static double CalcDifferenceNorm(std::vector<double>& curY, const std::vector<double>& nextY);
	//curX and curY will store squares
	static double CalcDifferenceNorm(std::vector<double>& curX, const std::vector<double>& nextX, 
		std::vector<double>& curY, const std::vector<double>& nextY);

	//x - rotation, y - thrust
	static void SubPreSolveSingle(const vec2& weights, const vec3& desiredForces,
		std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY, 
		const std::vector<vec2>& constrY);

	static void SubPreFindSingleX(const vec2& weights, const vec3& desiredForces,
		std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY,
		const std::vector<vec2>& constrY);

	static double SubPreFindSingleX(const vec2& weights, const vec3& desiredForces,
		double& X, double Y, double relX, double relY, double maxTh);

	static double SubPreFindSingleY(const vec2& weights, const vec3& desiredForces,
		double X, double& Y, double relX, double relY, const vec2& constrY);

	static void WorkerSolveStatic(std::stop_token stoken, double& outVal, std::atomic_flag& flag,
		const vec2& weights, const vec3& desiredForces, std::vector<double>& X,
		std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY,
		const std::vector<vec2>& constrX, const std::vector<vec2>& constrY);

	static void WorkerSolveStaticV2(std::stop_token stoken, double& outVal, std::atomic_flag& flag,
		const vec2& weights, const vec3& desiredForces, std::vector<double>& X,
		std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY,
		const std::vector<vec2>& constrX, const std::vector<vec2>& constrY, const std::vector<float>& maxTh);

	void SignalWorkers()&;
	void WaitForWorkers() const&;
	
	void WorkerSolve(std::stop_token stoken, size_t workerNum);

	//static double RotationDerivative(const vec2& weights, const vec3& desiredEngineForces,
		//double y, double relX, double relY, double sinX, double cosX);

	static double RotationDerivative(const vec2& weights, const vec3& sumsMindes,
		double ycosX, double ysinX, double relX, double relY);

	static double RotationDerivativeSingle(const vec2& weights, const vec3& desiredForces,
		double cosX, double sinX, double relX, double relY, double constrY);

	static double ThrustDerivative(const vec2& weights, const vec3& sumsMindes,
		double sinX, double cosX, double relX, double relY);

	Ship* ship{ nullptr };

	//void 
	//bool bWorkersInited{ false };

	vec2 weights{};
	vec3 desiredEngineForces{};

	static constexpr double targetInitXStepSize{ 0.05 };
	//relative to max power
	static constexpr double targetInitYStepSize{ 0.01 };
	static constexpr double momentumParam{ 0. };
	static constexpr double derMaxValue{ 100. };
	static_assert(targetInitXStepSize > 0. && derMaxValue > 0.);
	
	static constexpr size_t workerCount{ 7 };
	static constexpr vec2 perWorkerOfsset{ 
		2 * std::numbers::pi / (workerCount + 1),
		1. / (workerCount + 1) };

	static constexpr vec2 searchStepSize{ 0.015, 0.01 };
	static constexpr std::pair<size_t, size_t> searchInterations{
		static_cast<size_t>(perWorkerOfsset.first / searchStepSize.first) + 1,
		static_cast<size_t>(1. / (workerCount + 1) / searchStepSize.second) + 1
	};

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
	std::vector<float> maxThrusts;

	float maxLinearSummedThrust{ 0.f };
	double maxAngularSummedThrust{ 0. };

	bool bStopIssued{ false };
	bool bWorkersInited{ false };
	//size_t
};

