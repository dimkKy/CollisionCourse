// by Dmitry Kolontay

#include "ship_controller.h"
#include "ship/ship.h"
#include "ship/thruster.h"
#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include "utils.h"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <execution>

/*double ShipController::RotationDerivative(const vec2& weights, const vec3& desMinSums, 
    double y, double relX, double relY, double sinX, double cosX)
{
    return 2. * y * (weights.first * (desMinSums[0] * sinX - desMinSums[1] * cosX) -
        weights.second * desMinSums[2] * (relX * cosX + relY * sinX));
}*/

double ShipController::RotationDerivative(const vec2& weights, const vec3& sumsMinDes,
    double ycosX, double ysinX, double relX, double relY)
{
    return 2. * (weights.first * (sumsMinDes[1] * ycosX - sumsMinDes[0] * ysinX) +
        weights.second * sumsMinDes[2] * (relX * ycosX + relY * ysinX));
}

double ShipController::RotationDerivativeSingle(const vec2& weights, const vec3& desiredForces, double cosX, double sinX, double relX, double relY, double constrY)
{
    return /*2. * */ (weights.first * ((desiredForces[0] - constrY * cosX) * sinX - (desiredForces[1] - constrY * sinX) * cosX) +
        weights.second * (constrY * (relX * sinX - relY * cosX) - desiredForces[2]) * (relX * cosX + relY * sinX));
}

/*double ShipController::ThrustDerivative(const vec2& weights, const vec3& desMinSums,
    double relX, double relY, double sinX, double cosX)
{
    return 2. * (weights.second * desMinSums[2] * (relY * cosX - relX * sinX) -
        weights.first * (desMinSums[0] * cosX + desMinSums[1] * sinX) );
}*/

double ShipController::ThrustDerivative(const vec2& weights, const vec3& sumMinDesired,
    double sinX, double cosX, double relX, double relY)
{
    return 2. * (weights.first * (sumMinDesired[0] * cosX + sumMinDesired[1] * sinX) +
        weights.second * sumMinDesired[2] * (relX * sinX - relY * cosX));
}

/*double ShipController::ThrustDerivativeSingle(const vec2& weights, const vec3& sumMinDesired,
    double sinX, double cosX, double relX, double relY)
{
    return 2. * (weights.first * (sumMinDesired[0] * cosX + sumMinDesired[1] * sinX) +
        weights.second * sumMinDesired[2] * (relX * sinX - relY * cosX));
}*/

//implement zip iterator?

void ShipController::CalcSetDesiredLinearForce(const godot::Vector2& desiredSpeed)
{
    CalcSetDesiredLinearForce(desiredSpeed, get_physics_process_delta_time());
}

void ShipController::CalcSetDesiredLinearForce(const godot::Vector2& desiredSpeed, double physDeltatime)
{
    assert(ship);
    //ship->GetExternalLinearForce()
    godot::Vector2 curSpeed{ ship->get_linear_velocity() };
    godot::Vector2 desiredAcceleration{ (desiredSpeed - curSpeed) / physDeltatime };
        
    double shipMass{ ship->get_mass() };
    godot::Vector2 desiredTotalForce{ desiredAcceleration * shipMass };

    //auto external{ ship->GetExternalLinearForce() };
    auto* shipState{ ship->GetPhysicsState() };

    godot::Vector2 dampForce{ - shipState->get_total_linear_damp() * desiredSpeed * shipMass };
    //godot::Vector2 dampForce{ - shipState->get_total_linear_damp() * curSpeed * shipMass };
    //double dampTorque{ - shipState->get_total_angular_damp() * shipState->get_angular_velocity() * shipMass };

    //auto speedRatio{ std::sqrt(desiredSpeed.length_squared() / curSpeed.length_squared()) };
    //dampForce *= speedRatio;
    //total = desired + gravity + damp
    // desired = total - gravity - damp
    godot::Vector2 desiredEngineForce = (desiredTotalForce - dampForce)
        .rotated(ship->get_global_rotation());
    desiredEngineForce -= shipState->get_total_gravity() * 0.01;

    auto desLengthRatio{ desiredEngineForce.length() / maxLinearSummedThrust };
    if (desLengthRatio > 1.) {
        desiredEngineForce /= desLengthRatio;
    }

    //DBG_PRINT(desiredEngineForce);
    //DBG_PRINT(-ship->get_global_rotation());
    //add damping 
    //desiredEngineForce *= 0.9;
    desiredEngineForces[0] = desiredEngineForce.x;
    desiredEngineForces[1] = desiredEngineForce.y;
    //desiredEngineForces[2] = desiredEngineForce.y;
}

void ShipController::CalcSetDesiredTorquByRotation(real_t desiredRotation)
{
    CalcSetDesiredTorquByRotation(desiredRotation, get_physics_process_delta_time());
}

void ShipController::CalcSetDesiredTorquByRotation(real_t desiredRotation, double physDeltatime)
{
    assert(ship);
    //ship->GetExternalLinearForce()
    auto curRotation{ ship->get_rotation() };
    auto desiredAngularSpeed{ (desiredRotation - curRotation) / physDeltatime };
    auto desiredAcceleration{ desiredAngularSpeed / physDeltatime };

    double shipMass{ ship->get_mass() };
    auto desiredTotalTorque{ desiredAcceleration * shipMass };

    auto* shipState{ ship->GetPhysicsState() };

    auto dampTorque{ -shipState->get_total_angular_damp() * desiredAngularSpeed * shipMass };

    desiredEngineForces[2] = std::clamp(desiredTotalTorque - dampTorque, -maxAngularSummedThrust, maxAngularSummedThrust);
}

double ShipController::Solve(std::vector<double>& X, std::vector<double>& Y)&
{
    //REDO
    //worker 0 - max thrust, same rotation
    //worker 1 - max thrust, left rotation
    //worker 2 - max thrust, right rotation
    //worker 3 - same thrust, left rotation
    //worker 4 - same thrust, right rotation

    //for(size_t i{})
    const size_t thrustersCount{ X.size() };
    assert(thrustersCount == Y.size());
    assert(thrustersCount == relPosX.size());
    assert(thrustersCount == relPosY.size());
    assert(thrustersCount == constrX.size());
    assert(thrustersCount == constrY.size());
    
    for (size_t w{ 0 }; w < workerCount; ++w) {
        assert(thrustersCount == workerXs[w].size());
        assert(thrustersCount == workerYs[w].size());
        double workerXOffset{ perWorkerOfsset.first * (w + 1) };
        for (size_t i{ 0 }; i < thrustersCount; ++i) {
            workerXs[w][i] = Utils::RadiansReminder(X[i] + workerXOffset);
        }
    }
    //data for individual search prepared
    //try to find the best rotation for each thruster independently as starting point
    SignalWorkers();
    SubPreFindSingleX(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrY);
    WaitForWorkers();

    /*vec3 desiredForcesTemp{ desiredEngineForces };

    for (size_t i{ 0 }; i < thrustersCount; ++i) {
        SignalWorkers();
        SubPreFindSingleX(weights, desiredEngineForces, X[i], Y[i], relPosX[i], relPosY[i], constrY[i]);
        WaitForWorkers();

        for (size_t w{ 0 }; w < workerCount; ++w) {
            if (workerYs[w][i] < Y[i]) {
                Y[i] = workerYs[w][i];
                X[i] = workerXs[w][i];
                desiredEngineForces = workerOutsV[w];
            }
        }
        SubSolveWithFixedRotationSingle(weights, desiredEngineForces, X[i], Y[i], relPosX[i], relPosY[i], constrY[i]);
        double ySinX{ Y[i] * std::sin(X[i]) };
        double yCosX{ Y[i] * std::cos(X[i]) };
        desiredEngineForces[0] -= yCosX;
        desiredEngineForces[1] -= ySinX;
        desiredEngineForces[2] -= (relPosX[i] * ySinX + relPosY[i] * yCosX);
    }

    desiredEngineForces = desiredForcesTemp;*/
    /*for (size_t w{ 0 }; w < workerCount; ++w) {
        assert(thrustersCount == workerXs[w].size());
        assert(thrustersCount == workerYs[w].size());

        for (size_t i{ 0 }; i < thrustersCount; ++i) {
            if (workerYs[w][i] < Y[i]) {
                Y[i] = workerYs[w][i];
                X[i] = workerXs[w][i];
            }
        }
    }*/
    //Sleep(0)
    //now X holds the best (supposely) rotations
    //initiate global search using X as base starting point
    //redo?
    /*for (size_t i{ 0 }; i < thrustersCount; ++i) {
        Y[i] = (X[i] - (constrX[i].first + constrX[i].second) * 0.5) / workerCount;

        for (size_t w{ 0 }; w < workerCount; ++w) {
            workerXs[w][i] = X[i] - Y[i] * (w + 1);
        }
    }*/
    for (size_t i{ 0 }; i < thrustersCount; ++i) {
        //redo clamp
        //X[i] = std::clamp(X[i], constrX[i].first, constrX[i].second);
        double offsetY{ (constrY[i].second - constrY[i].first) / (workerCount + 3) };
        Y[i] = offsetY;
        for (size_t w{ 0 }; w < workerCount; ++w) {
            assert(thrustersCount == workerXs[w].size());
            assert(thrustersCount == workerYs[w].size());
            workerXs[w][i] = X[i];
            workerYs[w][i] = offsetY * (w + 2);
            
        }
    }
    SignalWorkers();
    double out{ SubSolveWithFixedRotation(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrY) };
    WaitForWorkers();

    /*for (size_t w{ 0 }; w < workerCount; ++w) {
        assert(thrustersCount == workerXs[w].size());
        assert(thrustersCount == workerYs[w].size());

        for (size_t i{ 0 }; i < thrustersCount; ++i) {
            workerXs[w][i] = X[i];
            workerYs[w]
            if (workerYs[w][i] < Y[i]) {
                Y[i] = workerYs[w][i];
                X[i] = workerXs[w][i];
            }
        }
    }*
    SignalWorkers();
    for (size_t i{ 0 }; i < thrustersCount; ++i) {
        Y[i] = constrY[i].second * 0.5;
    }
    double out{ SubSolveNoRotationConstraint(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrY) };
    WaitForWorkers();*/
    std::vector<double>* bestY{ &Y };

    for (size_t w{ 0 }; w < workerCount; ++w) {
        if (workerOuts[w] < out) {
            out = workerOuts[w];
            bestY = &workerYs[w];
        }
    }
    if (bestY != &Y) {
        Y.swap(*bestY);
    }
    //SAVE to comparison next tick?
    //global minimum approximated, shift to it
    /*for (size_t i{ 0 }; i < thrustersCount; ++i) {
        X[i] = std::clamp(X[i], constrX[i].first, constrX[i].second);
    }*/

    //SubSolveWithFixedRotation(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrY);

    /*//resize arrays
    for (size_t i{ 0 }; i < X.size(); ++i) {
        workerYs[0][i] = constrY[i].second;
        workerXs[0][i] = X[i];

        workerYs[1][i] = constrY[i].second;
        workerXs[1][i] = constrX[i].first;

        workerYs[2][i] = constrY[i].second;
        workerXs[2][i] = constrX[i].second;

        workerYs[3][i] = Y[i];
        workerXs[3][i] = constrX[i].first;

        workerYs[4][i] = Y[i];
        workerXs[4][i] = constrX[i].second;

        static_assert(workerCount - 1 == 4);
    }
    //data prepared
    for (auto& flag : workerFlags) {
        flag.test_and_set();
        flag.notify_one();
    }

    double out{ SubSolve(weights, desiredForces, X, Y, relPosX, relPosY, constrX, constrY) };
    double out{ SubSolveNoRotationConstraint(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrX) };

    for (auto& flag : workerFlags) {
        flag.wait(true);
    }

    for (size_t i{ 0 }; i < workerCount; ++i) {
        if (workerOuts[i] < out) {
            out = workerOuts[i];
            X = workerXs[i];
            Y = workerYs[i];
        }
    }*/
    return out;
}

double ShipController::SolveV2(std::vector<double>& X, std::vector<double>& Y)&
{
    //REDO
    const size_t thrustersCount{ X.size() };
    assert(thrustersCount == Y.size());
    assert(thrustersCount == relPosX.size());
    assert(thrustersCount == relPosY.size());
    assert(thrustersCount == constrX.size());
    assert(thrustersCount == constrY.size());

    for (size_t w{ 0 }; w < workerCount; ++w) {
        assert(thrustersCount == workerXs[w].size());
        assert(thrustersCount == workerYs[w].size());
        double workerXOffset{ perWorkerOfsset.first * (w + 1) };
        double workerYOffset{ perWorkerOfsset.second * (w + 1) };


        for (size_t i{ 0 }; i < thrustersCount; ++i) {
            workerXs[w][i] = Utils::RadiansReminder(X[i] + workerXOffset);
            workerYs[w][i] = Y[i] + workerYOffset;
        }
    }
    //data for individual search prepared
    //try to find the best rotation for each thruster independently as starting point

    vec3 originalDesiredForces{ desiredEngineForces };

    for (size_t i{ 0 }; i < thrustersCount; ++i) {
        SignalWorkers();
        double out{ SubPreFindSingleX(weights, desiredEngineForces, X[i], Y[i], relPosX[i], relPosY[i], maxThrusts[i]) };
        WaitForWorkers();

        for (size_t w{ 0 }; w < workerCount; ++w) {
            if (workerOuts[w] < out) {
                out = workerOuts[w];
                X[i] = workerXs[w][i];
            }
        }
        for (size_t w{ 0 }; w < workerCount; ++w) {
            workerXs[w][i] = X[i];
        }

        SignalWorkers();
        out = SubPreFindSingleY(weights, desiredEngineForces, X[i], Y[i], relPosX[i], relPosY[i], constrY[i]);
        WaitForWorkers();
        for (size_t w{ 0 }; w < workerCount; ++w) {
            if (workerOuts[w] < out) {
                out = workerOuts[w];
                Y[i] = workerYs[w][i];
            }
        }

        double ySinX{ Y[i] * std::sin(X[i]) };
        double yCosX{ Y[i] * std::cos(X[i]) };
        desiredEngineForces[0] -= yCosX;
        desiredEngineForces[1] -= ySinX;
        desiredEngineForces[2] -= (relPosX[i] * ySinX + relPosY[i] * yCosX);
    }

    desiredEngineForces = originalDesiredForces;


    for (size_t i{ 0 }; i < thrustersCount; ++i) {
        //redo clamp
        //X[i] = std::clamp(X[i], constrX[i].first, constrX[i].second);
        double offsetY{ (constrY[i].second - constrY[i].first) / (workerCount + 3) };
        Y[i] = offsetY;
        for (size_t w{ 0 }; w < workerCount; ++w) {
            assert(thrustersCount == workerXs[w].size());
            assert(thrustersCount == workerYs[w].size());
            workerXs[w][i] = X[i];
            workerYs[w][i] = offsetY * (w + 2);

        }
    }
    SignalWorkers();
    double out{ SubSolveWithFixedRotation(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrY) };
    WaitForWorkers();

    std::vector<double>* bestY{ &Y };

    for (size_t w{ 0 }; w < workerCount; ++w) {
        if (workerOuts[w] < out) {
            out = workerOuts[w];
            bestY = &workerYs[w];
        }
    }
    if (bestY != &Y) {
        Y.swap(*bestY);
    }
    //SAVE to comparison next tick?
    //global minimum approximated, shift to it
    return out;
}

double ShipController::SubSolve(const vec2& weights, const vec3& desiredForces,
    std::vector<double>& X, std::vector<double>& Y,
    const std::vector<double>& relX, const std::vector<double>& relY,
    const std::vector<vec2>& constrX, const std::vector<vec2>& constrY)
{
    //TODO NOT READY
    assert(false);
    std::vector<double> bufX{ X };
    std::vector<double> bufY{ Y };
    std::vector<double>& curX{ X };
    std::vector<double>& curY{ Y };
    std::vector<double>& nextX{ bufX };
    std::vector<double>& nextY{ bufY };

    std::vector<double> sinX{ X };
    std::vector<double> cosX{ Y };

    std::vector<double> ycosX{ Y };
    std::vector<double> ysinX{ Y };

    vec3 sumMinDesired;

    const size_t n{ X.size() };

    assert(n == Y.size());
    assert(n == relX.size());
    assert(n == relY.size());
    assert(n == constrX.size());
    assert(n == constrY.size());

    //eps?
    //TODO
    double stepSize{ 1. };

    static auto sqrSummator = [](double a, double b) {
        return (a - b) * (a - b);
    };

    do { 
        for (size_t i{ 0 }; i < n; ++i) {
            std::transform(std::execution::par_unseq,
                curX.cbegin(), curX.cend(), sinX.begin(),
                static_cast<double(*)(double)>(std::sin));
            std::transform(std::execution::par_unseq,
                curX.cbegin(), curX.cend(), cosX.begin(),
                static_cast<double(*)(double)>(std::cos));

            std::transform(std::execution::par_unseq,
                sinX.cbegin(), sinX.cend(), curY.cbegin(),
                ysinX.begin(), std::multiplies());
            std::transform(std::execution::par_unseq,
                cosX.cbegin(), cosX.cend(), curY.cbegin(),
                ycosX.begin(), std::multiplies());

            CalcSumsMinusDesired(sumMinDesired, desiredForces, ysinX, ycosX, relX, relY);

            double projXi{ curX[i] - stepSize * RotationDerivative(weights,
                sumMinDesired, ycosX[i], ysinX[i], relX[i], relY[i]) };
            double projYi{ curY[i] - stepSize * ThrustDerivative(weights,
                sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]) };

            nextX[i] = std::clamp(projXi, constrX[i].first, constrX[i].second);
            nextY[i] = std::clamp(projYi, constrY[i].first, constrY[i].second);

        }
        //difference, cur is free

        std::transform(std::execution::par_unseq,
            nextX.cbegin(), nextX.cend(), curX.cbegin(),
            curX.begin(), sqrSummator);
        std::transform(std::execution::par_unseq,
            nextY.cbegin(), nextY.cend(), curY.cbegin(),
            curY.begin(), sqrSummator);

        if (std::reduce(std::execution::par_unseq, curX.cbegin(), curX.cend(), 0.) +
            std::reduce(std::execution::par_unseq, curY.cbegin(), curY.cend(), 0.) < Utils::kindaSmallDouble * 10.) {
            break;
        }

        stepSize *= 0.95;
        std::swap(curX, nextX);
        std::swap(curY, nextY);

    } while (true);

    if (nextX != X) {
        X.swap(nextX);
    }
    if (nextY != Y) {
        Y.swap(nextY);
    }

    return SubSolvePrepareDataForOut(weights, X, Y, relX, relY, desiredForces, ysinX, ycosX, sumMinDesired);
}

double ShipController::SubSolveNoRotationConstraint(const vec2& weights, const vec3& desiredForces,
    std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, 
    const std::vector<double>& relY, const std::vector<vec2>& constrY)
{
    //TODO NOT READY
    assert(false);
    std::vector<double> bufX{ X };
    std::vector<double> bufY{ Y };
    std::vector<double>& curX{ X };
    std::vector<double>& curY{ Y };
    std::vector<double>& nextX{ bufX };
    std::vector<double>& nextY{ bufY };

    std::vector<double> sinX{ X };
    std::vector<double> cosX{ Y };

    std::vector<double> ycosX{ Y };
    std::vector<double> ysinX{ Y };

    vec3 sumMinDesired;

    const size_t n{ X.size() };

    assert(n == Y.size());
    assert(n == relX.size());
    assert(n == relY.size());
    assert(n == constrY.size());

    //eps?
    //TODO
    double stepSize{ 5. };

    static auto sqrSummator = [](double a, double b) {
        return (a - b) * (a - b);
    };

    do {
        for (size_t i{ 0 }; i < n; ++i) {
            std::transform(std::execution::par_unseq,
                curX.cbegin(), curX.cend(), sinX.begin(),
                static_cast<double(*)(double)>(std::sin));
            std::transform(std::execution::par_unseq,
                curX.cbegin(), curX.cend(), cosX.begin(),
                static_cast<double(*)(double)>(std::cos));

            std::transform(std::execution::par_unseq,
                sinX.cbegin(), sinX.cend(), curY.cbegin(),
                ysinX.begin(), std::multiplies());
            std::transform(std::execution::par_unseq,
                cosX.cbegin(), cosX.cend(), curY.cbegin(),
                ycosX.begin(), std::multiplies());

            
            CalcSumsMinusDesired(sumMinDesired, desiredForces, ysinX, ycosX, relX, relY);

            auto projXi{ curX[i] - stepSize * RotationDerivative(weights,
                sumMinDesired, ycosX[i], ysinX[i], relX[i], relY[i]) };
            auto projYi{ curY[i] - stepSize * ThrustDerivative(weights,
                sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]) };


            nextX[i] = Utils::RadiansReminder(projXi);
            nextY[i] = std::clamp(projYi, constrY[i].first, constrY[i].second);

        }

        //difference, cur is free

        std::transform(std::execution::par_unseq,
            nextX.cbegin(), nextX.cend(), curX.cbegin(),
            curX.begin(), sqrSummator);
        std::transform(std::execution::par_unseq,
            nextY.cbegin(), nextY.cend(), curY.cbegin(),
            curY.begin(), sqrSummator);

        if (std::reduce(std::execution::par_unseq, curX.cbegin(), curX.cend(), 0.) +
            std::reduce(std::execution::par_unseq, curY.cbegin(), curY.cend(), 0.) < Utils::kindaSmallDouble * 10.) {
            break;
        }

        stepSize *= 0.95;
        std::swap(curX, nextX);
        std::swap(curY, nextY);

    } while (true);

    if (nextX != X) {
        X.swap(nextX);
    }
    if (nextY != Y) {
        Y.swap(nextY);
    }

    return SubSolvePrepareDataForOut(weights, X, Y, relX, relY, desiredForces, ysinX, ycosX, sumMinDesired);
}

double ShipController::SubSolveWithFixedRotation(const vec2& weights, const vec3& desiredForces,
    const std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, 
    const std::vector<double>& relY, const std::vector<vec2>& constrY)
{
    const size_t n{ X.size() };
    assert(n == Y.size());
    assert(n == relX.size());
    assert(n == relY.size());
    assert(n == constrY.size());

    std::vector<double> bufY{ Y };
    std::vector<double>* curY{ &Y };
    std::vector<double>* nextY{ &bufY };

    std::vector<double> sinX{ X };
    std::vector<double> cosX{ X };

    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), sinX.begin(),
        static_cast<double(*)(double)>(std::sin));
    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), cosX.begin(),
        static_cast<double(*)(double)>(std::cos));

    std::vector<double> ySinX{ sinX };
    std::vector<double> yCosX{ cosX };
   
    vec3 sumMinDesired;
    //eps?
    //TODO
    double stepSizeMod{ targetInitYStepSize };
    //just unseq?
    std::transform(std::execution::par_unseq,
        sinX.cbegin(), sinX.cend(), curY->cbegin(),
        ySinX.begin(), std::multiplies());
    std::transform(std::execution::par_unseq,
        cosX.cbegin(), cosX.cend(), curY->cbegin(),
        yCosX.begin(), std::multiplies());

    CalcSumsMinusDesired(sumMinDesired, desiredForces, ySinX, yCosX, relX, relY);

    do {
        for (size_t i{ 0 }; i < n; ++i) {

            double der{ std::clamp(ThrustDerivative(weights, sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]),
               -derMaxValue, derMaxValue) };

            double projYi{ curY->at(i) - stepSizeMod * (der / derMaxValue) * constrY[i].second * der +
            0.9 * (curY->at(i) - nextY->at(i)) };

            nextY->at(i) = std::clamp(projYi, constrY[i].first, constrY[i].second);

            PreUpdateSumsMinusDesired(sumMinDesired, yCosX[i], ySinX[i], relX[i], relY[i]);

            ySinX[i] = sinX[i] * nextY->at(i);
            yCosX[i] = cosX[i] * nextY->at(i);

            PostUpdateSumsMinusDesired(sumMinDesired, yCosX[i], ySinX[i], relX[i], relY[i]);
        }

        if (CalcDifferenceNorm(*curY, *nextY) < Utils::kindaSmallDouble * n) {
            break;
        }

        stepSizeMod *= 0.97;
        std::swap(curY, nextY);
    } while (true);

    if (nextY != &Y) {
        Y.swap(*nextY);
    }

    return SubSolvePrepareDataForOut(weights, sumMinDesired);
}

/*double ShipController::SubSolveWithFixedRotationSingle(const vec2& weights, const vec3& desiredForces, double& X, double& Y, double relX, double relY, const vec2& constrY)
{

    double bufY{ Y };
    std::vector<double>* curY{ &Y };
    std::vector<double>* nextY{ &bufY };

    double sinX{ std::sin(X) };
    double cosX{ std::cos(X) };

    double ySinX{ Y * sinX };
    double yCosX{ Y * cosX };

    vec3 sumMinDesired;
    //eps?
    //TODO
    double stepSizeMod{ targetInitYStepSize };
    //just unseq?

    CalcSumsMinusDesired(sumMinDesired, desiredForces, ySinX, yCosX, relX, relY);

    do {
        double der{ std::clamp(ThrustDerivative(weights, sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]),
                -derMaxValue, derMaxValue) };

        double projYi{ curY->at(i) - stepSizeMod * (der / derMaxValue) * constrY[i].second * der +
        0.9 * (curY->at(i) - nextY->at(i)) };

        nextY->at(i) = std::clamp(projYi, constrY[i].first, constrY[i].second);

        PreUpdateSumsMinusDesired(sumMinDesired, yCosX[i], ySinX[i], relX[i], relY[i]);

        ySinX[i] = sinX[i] * nextY->at(i);
        yCosX[i] = cosX[i] * nextY->at(i);

        PostUpdateSumsMinusDesired(sumMinDesired, yCosX[i], ySinX[i], relX[i], relY[i]);

        if (std::abs(*curY - *nextY) < Utils::kindaSmallDouble) {
            break;
        }

        stepSizeMod *= 0.97;
        std::swap(curY, nextY);
    } while (true);

    if (nextY != &Y) {
        Y.swap(*nextY);
    }

    return SubSolvePrepareDataForOut(weights, sumMinDesired);*
    return 0.;
}*/

double ShipController::SubSolvePrepareDataForOut(const vec2& weights, const std::vector<double>& X, const std::vector<double>& Y,
    const std::vector<double>& relX, const std::vector<double>& relY, const vec3& desiredForces,
    std::vector<double>& ySinX, std::vector<double>& yCosX, vec3& sumMinDesired)
{
    const size_t n{ X.size() };
    assert(n == Y.size());
    assert(n == relX.size());
    assert(n == relY.size());
    assert(n == ySinX.size());
    assert(n == yCosX.size());

    static auto ySinner = [](double x, double y) {
        return y * std::sin(x);
    };

    static auto yCosiner = [](double x, double y) {
        return y * std::cos(x);
    };

    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), Y.cbegin(),
        ySinX.begin(), ySinner);

    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), Y.cbegin(),
        yCosX.begin(), yCosiner);

    CalcSumsMinusDesired(sumMinDesired, desiredForces, ySinX, yCosX, relX, relY);

    return weights.first * (std::abs(sumMinDesired[0]) + std::abs(sumMinDesired[1])) +
        weights.second * std::abs(sumMinDesired[2]);
}

double ShipController::SubSolvePrepareDataForOut(const vec2& weights, const vec3& sumMinDesired)
{
    return weights.first * (std::abs(sumMinDesired[0]) + std::abs(sumMinDesired[1])) +
        weights.second * std::abs(sumMinDesired[2]);
}

void ShipController::CalcSumsMinusDesired(vec3& out, const vec3& desiredForces, 
    const std::vector<double>& ySinX, const std::vector<double>& yCosX, 
    const std::vector<double>& relX, const std::vector<double>& relY)
{
    const size_t n{ ySinX.size() };
    assert(n == yCosX.size());
    assert(n == relX.size());
    assert(n == relY.size());

    //reconsider minus
    out[0] = std::reduce(std::execution::par_unseq,
        yCosX.cbegin(), yCosX.cend(), -desiredForces[0]);

    out[1] = std::reduce(std::execution::par_unseq,
        ySinX.cbegin(), ySinX.cend(), -desiredForces[1]);

    out[2] = -desiredForces[2];

    for (size_t i{ 0 }; i < n; ++i) {
        out[2] += (ySinX[i] * relX[i] - yCosX[i] * relY[i]);
    }
}

void ShipController::CalcSumsMinusDesired(vec3& out, const vec3& desiredForces, double ySinX, double yCosX, double relX, double relY)
{
    out[0] = yCosX - desiredForces[0];
    out[1] = ySinX - desiredForces[0];
    out[2] = ySinX * relX - yCosX * relY - desiredForces[2];
}

void ShipController::PreUpdateSumsMinusDesired(vec3& out, double yCosX, double ySinX, double relX, double relY)
{
    out[0] -= yCosX;
    out[1] -= ySinX;
    out[2] -= (ySinX * relX - yCosX * relY);
}

void ShipController::PostUpdateSumsMinusDesired(vec3& out, double yCosX, double ySinX, double relX, double relY)
{
    out[0] += yCosX;
    out[1] += ySinX;
    out[2] += (ySinX * relX - yCosX * relY);
}

double ShipController::CalcDifferenceNorm(std::vector<double>& curY, const std::vector<double>& nextY)
{
    static auto sqrSummator = [](double a, double b) {
        return Utils::Square(a - b);
    };

    std::transform(std::execution::par_unseq,
        nextY.cbegin(), nextY.cend(), curY.cbegin(),
        curY.begin(), sqrSummator);

    return std::reduce(std::execution::par_unseq, curY.cbegin(), curY.cend(), 0.);
}

double ShipController::CalcDifferenceNorm(std::vector<double>& curX, const std::vector<double>& nextX, 
    std::vector<double>& curY, const std::vector<double>& nextY)
{
    static auto sqrSummator = [](double a, double b) {
        return Utils::Square(a - b);
    };

    std::transform(std::execution::par_unseq,
        nextX.cbegin(), nextX.cend(), curX.cbegin(),
        curX.begin(), sqrSummator);

    std::transform(std::execution::par_unseq,
        nextY.cbegin(), nextY.cend(), curY.cbegin(),
        curY.begin(), sqrSummator);

    return std::reduce(std::execution::par_unseq, curX.cbegin(), curX.cend(), 0.) +
        std::reduce(std::execution::par_unseq, curY.cbegin(), curY.cend(), 0.);
}


void ShipController::SubPreSolveSingle(const vec2& weights, const vec3& desiredForces, 
    std::vector<double>& X, std::vector<double>& Y,
    const std::vector<double>& relX, const std::vector<double>& relY, const std::vector<vec2>& constrY)
{
    //doest work use SubPreFindSingleX
    assert(false);

    const size_t n{ X.size() };
    //double resAmplifier{ 100. };
    assert(n == relX.size());
    assert(n == relY.size()); 

    for (size_t i{ 0 }; i < n; ++i) {
        double bufX;
        double* curX{ &X[i] };
        double* nextX{ &bufX };
        //double NAG{ *curX };

        /*double der{ RotationDerivativeSingle(weights, desiredForces,
               std::cos(NAG), std::sin(NAG), relX[i], relY[i], constrY[i].second)};
        der = std::clamp(der, -derMaxValue, derMaxValue);*/

        double stepSize{ targetInitXStepSize / derMaxValue };
        assert(stepSize > 0. && stepSize <= (targetInitXStepSize + Utils::kindaSmallDouble));
        do {
            /*double der{ RotationDerivativeSingle(weights, desiredForces,
                y * std::cos(curX), y * std::sin(curX), relX[i], relY[i]) };*/
            /*nextX = Utils::RadiansReminder(curX - stepSize * 
                RotationDerivativeSingle(weights, desiredForces,
                y * std::cos(curX), y * std::sin(curX), relX[i], relY[i]));*/

            double der{ RotationDerivativeSingle(weights, desiredForces,
               std::cos(*curX), std::sin(*curX), relX[i], relY[i], constrY[i].second) };
            der = std::clamp(der, -derMaxValue, derMaxValue);

            *nextX = *curX - stepSize * der;
            //*nextX = NAG - stepSize * der;
            if (std::abs(*nextX - *curX) < Utils::kindaSmallDouble) {
                X[i] = *nextX;
                break;
            }
            //NAG = *nextX + momentumParam * (*nextX - *curX);

            std::swap(curX, nextX);

            /*der = std::clamp(
                RotationDerivativeSingle(weights, desiredForces, std::cos(NAG), std::sin(NAG), relX[i], relY[i], constrY[i].second),
                -derMaxValue, derMaxValue);*/

            stepSize *= 0.97;
            //
            
        } while (true);

        vec3 desiredMinYX;
        double sinX = std::sin(X[i]);
        double cosX = std::cos(X[i]);

        desiredMinYX[0] = desiredForces[0] - constrY[i].second * cosX;
        desiredMinYX[1] = desiredForces[1] - constrY[i].second * sinX;
        desiredMinYX[2] = desiredForces[2] - constrY[i].second * (sinX * relX[i] + cosX * relY[i]);
        Y[i] = SubSolvePrepareDataForOut(weights, desiredMinYX);
    }
}

void ShipController::SubPreFindSingleX(const vec2& weights, const vec3& desiredForces, 
    std::vector<double>& X, std::vector<double>& Y, 
    const std::vector<double>& relX, const std::vector<double>& relY, const std::vector<vec2>& constrY)
{
    const size_t n{ X.size() };
    //double resAmplifier{ 100. };
    assert(n == relX.size());
    assert(n == relY.size());

    /*static auto ySinner = [](double x, double y) {
        return y * std::sin(x);
        };

    static auto yCosiner = [](double x, double y) {
        return y * std::cos(x);
        };

    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), Y.cbegin(),
        ySinX.begin(), ySinner);

    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), Y.cbegin(),
        yCosX.begin(), yCosiner);*/

    //init values

    static auto targetF = [&weights, &desiredForces](double yCosX, double ySinX, double relX, double relY) {
        return weights.first * (std::abs(desiredForces[0] - yCosX) + std::abs(desiredForces[1] - ySinX)) +
            weights.second * std::abs(desiredForces[2] - relX * ySinX + relY * yCosX);
    };

    double ySinX;
    double yCosX;
    //redo to transform?
    double y;
    double bestX;
    double curX;

    for (size_t i{ 0 }; i < n; ++i) {
        ySinX = constrY[i].second * std::sin(X[i]);
        yCosX = constrY[i].second * std::cos(X[i]);
        Y[i] = targetF(yCosX, ySinX, relX[i], relY[i]);

        bestX = X[i];

        for (size_t it{ 1 }; it < searchInterations.first; ++it) {
            curX = X[i] + it * searchStepSize.first;

            ySinX = constrY[i].second * std::sin(curX);
            yCosX = constrY[i].second * std::cos(curX);

            y = targetF(yCosX, ySinX, relX[i], relY[i]);

            if (y < Y[i]) {
                Y[i] = y;
                bestX = curX;
            }
        }
        X[i] = bestX;
    }
}

double ShipController::SubPreFindSingleX(const vec2& weights, const vec3& desiredForces, double& X, double Y, double relX, double relY, double maxTh)
{
    static auto targetF = [&weights, &desiredForces](double yCosX, double ySinX, double relX, double relY) {
        return weights.first * (std::abs(desiredForces[0] - yCosX) + std::abs(desiredForces[1] - ySinX)) +
            weights.second * std::abs(desiredForces[2] - relX * ySinX + relY * yCosX);
        };

    double ySinX{ maxTh * std::sin(X) };
    double yCosX{ maxTh * std::cos(X) };

    Y = targetF(yCosX, ySinX, relX, relY);

    double bestX{ X };

    double curX, y;
    
    for (size_t it{ 1 }; it < searchInterations.first; ++it) {
        curX = X + it * searchInterations.first;

        ySinX = maxTh * std::sin(curX);
        yCosX = maxTh * std::cos(curX);

        y = targetF(yCosX, ySinX, relX, relY);

        if (y < Y) {
            Y = y;
            bestX = curX;
        }
    }
    X = bestX;
    return Y;
}

double ShipController::SubPreFindSingleY(const vec2& weights, const vec3& desiredForces,
    double X, double& Y, double relX, double relY, const vec2& constrY)
{
    static auto targetF = [&weights, &desiredForces](double yCosX, double ySinX, double relX, double relY) {
        return weights.first * (std::abs(desiredForces[0] - yCosX) + std::abs(desiredForces[1] - ySinX)) +
            weights.second * std::abs(desiredForces[2] - relX * ySinX + relY * yCosX);
        };

    const double sinX{ std::sin(X) };
    const double cosX{ std::cos(X) };

    double ySinX{ Y * std::sin(X) };
    double yCosX{ Y * std::cos(X) };

    X = targetF(yCosX, ySinX, relX, relY);

    double curY, x;
    auto interval{ constrY.second - constrY.first };

    for (size_t it{ 1 }; it < searchInterations.second; ++it) {
        curY = Y + it * searchStepSize.second * interval;

        ySinX = curY * sinX;
        yCosX = curY * cosX;

        x = targetF(yCosX, ySinX, relX, relY);

        if (x < X) {
            X = x;
            Y = curY;
        }
    }
    return X;
}

void ShipController::WorkerSolveStatic(std::stop_token stoken, double& outVal, std::atomic_flag& flag,
     const vec2& weights, const vec3& desiredForces, std::vector<double>& X,
     std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY, 
     const std::vector<vec2>& constrX, const std::vector<vec2>& constrY)
{
    //can it never end?
    
    const size_t n{ X.size() };

    assert(n == Y.size());
    assert(n == relX.size());
    assert(n == relY.size());
    assert(n == constrX.size());
    assert(n == constrY.size());

    while (true) {
        flag.wait(false);
        if (stoken.stop_requested()) {
            return;
        }
        /*outV = desiredForces;
        SubPreFindSingleX(weights, outV, X[0], Y[0], relX[0], relY[0], constrY[0]);
        flag.clear();
        flag.notify_one();

        for (size_t i{ 1 }; i < n; ++i) {
            flag.wait(false);
            outV = desiredForces;
            SubPreFindSingleX(weights, outV, X[i], Y[i], relX[i], relY[i], constrY[i]);
            flag.clear();
            flag.notify_one();
        }*/
         
        SubPreFindSingleX(weights, desiredForces, X, Y, relX, relY, constrY);

        flag.clear();
        flag.notify_one();
        flag.wait(false);

        /*for (size_t i{ 0 }; i < n; ++i) {
            Y[i] = constrY[i].second * 0.5;
        }
        outVal = SubSolveNoRotationConstraint(weights, desiredForces, X, Y, relX, relY, constrY);*/
        outVal = SubSolveWithFixedRotation(weights, desiredForces, X, Y, relX, relY, constrY);

        flag.clear();
        flag.notify_one();
        /*flag.wait(false);

        outVal = SubSolveWithFixedRotation(weights, desiredForces, X, Y, relX, relY, constrY);
        flag.clear();
        flag.notify_one();*/
    }
}

void ShipController::WorkerSolveStaticV2(std::stop_token stoken, double& outVal, std::atomic_flag& flag, 
    const vec2& weights, const vec3& desiredForces, std::vector<double>& X, 
    std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY, 
    const std::vector<vec2>& constrX, const std::vector<vec2>& constrY, const std::vector<float>& maxTh)
{
    const size_t n{ X.size() };

    assert(n == Y.size());
    assert(n == relX.size());
    assert(n == relY.size());
    assert(n == constrX.size());
    assert(n == constrY.size());

    while (true) {
        flag.wait(false);
        if (stoken.stop_requested()) {
            return;
        }

        /*outVal = SubPreFindSingleX(weights, desiredForces, X[0], Y[0], relX[0], relY[0], constrY[0]);

        flag.clear();
        flag.notify_one();
        flag.wait(false);

        outVal = SubPreFindSingleY(weights, desiredForces, X[0], Y[0], relX[0], relY[0], constrY[0]);

        flag.clear();
        flag.notify_one();
        flag.wait(false);*/


        for (size_t i{ 0 }; i < n; ++i) {
         
            outVal = SubPreFindSingleX(weights, desiredForces, X[i], Y[i], relX[i], relY[i], maxTh[i]);
            flag.clear();
            flag.notify_one();
            flag.wait(false);

            outVal = SubPreFindSingleY(weights, desiredForces, X[i], Y[i], relX[i], relY[i], constrY[i]);
            flag.clear();
            flag.notify_one();
            flag.wait(false);
        }

        outVal = SubSolveWithFixedRotation(weights, desiredForces, X, Y, relX, relY, constrY);

        flag.clear();
        flag.notify_one();
    }
}

void ShipController::SignalWorkers()&
{
    for (auto& flag : workerFlags) {
        flag.test_and_set();
        flag.notify_one();
    }
}

void ShipController::WaitForWorkers() const&
{
    for (auto& flag : workerFlags) {
        flag.wait(true);
    }
}

 void ShipController::WorkerSolve(std::stop_token stoken, size_t i)
 {
     //WorkerSolveStatic(stoken, workerOuts[i], workerFlags[i],
         //weights, desiredEngineForces, workerXs[i], workerYs[i], relPosX, relPosY, constrX, constrY);

     WorkerSolveStaticV2(stoken, workerOuts[i], workerFlags[i],
         weights, desiredEngineForces, workerXs[i], workerYs[i], relPosX, relPosY, constrX, constrY, maxThrusts);
 }

/*void ShipController::SubSolve(const vec2& weights, const vec3& desiredEngineForces,
    std::vector<vec2>& XY,
    const std::vector<double>& relX, const std::vector<double>& relY,
    const std::vector<vec2>& constrX, const std::vector<vec2>& constrY)
{
    //X.size() = thrustersCount
    //Y.size() = thrustersCount
    //constrX.size() = thrustersCount
    //constrY.size() = thrustersCount
    //relPosX.size() = thrustersCount
    //relPosY.size() = thrustersCount
    //START SEVERAL THREADS?
    std::vector<vec2> buf{ XY };
    std::vector<vec2>& cur{ XY };
    std::vector<vec2>& next{ buf };

    std::vector<double> sinXcosX{ X };
    std::vector<double> cosX{ Y };

    std::vector<double> ycosX{ Y };
    std::vector<double> ysinX{ Y };

    vec3 sumMinDesired;

    const size_t thrustersCount{ X.size() };

    assert(thrustersCount == Y.size() == relX.size() == relY.size() == constrX.size() == constrY.size());

    //eps?
    while (true) {
        for (size_t i{ 0 }; i < thrustersCount; ++i) {
            //TODO
            double stepSize;
            std::transform(std::execution::par_unseq,
                curX.cbegin(), curX.cend(), sinX.begin(),
                static_cast<double(*)(double)>(std::sin));
            std::transform(std::execution::par_unseq,
                curX.cbegin(), curX.cend(), cosX.begin(),
                static_cast<double(*)(double)>(std::cos));

            std::transform(std::execution::par_unseq,
                sinX.cbegin(), sinX.cend(), curY.cbegin(),
                ysinX.begin(), std::multiplies());
            std::transform(std::execution::par_unseq,
                cosX.cbegin(), cosX.cend(), curY.cbegin(),
                ycosX.begin(), std::multiplies());

            //reconsider minus
            sumMinDesired[0] = std::reduce(std::execution::par_unseq,
                ycosX.cbegin(), ycosX.cend(), -desiredEngineForces[0]);

            sumMinDesired[1] = std::reduce(std::execution::par_unseq,
                ysinX.cbegin(), ysinX.cend(), -desiredEngineForces[1]);

            sumMinDesired[2] = -desiredEngineForces[2];

            for (size_t i{ 0 }; i < thrustersCount; ++i) {
                sumMinDesired[2] += (ysinX[i] * relX[i] - ycosX[i] * relY[i]);
            }

            auto projXi{ curX[i] - stepSize * RotationDerivative(weights,
                sumMinDesired, ycosX[i], ysinX[i], relX[i], relY[i]) };
            auto projYi{ curY[i] - stepSize * ThrustDerivative(weights,
                sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]) };

            nextX[i] = projXi;
            if (projXi > constrX[i].second) {
                nextX[i] = constrX[i].second;
            }
            else {
                if (projXi < constrX[i].first) {
                    nextX[i] = constrX[i].first;
                }
            }

            nextY[i] = projYi;
            if (projYi > constrY[i].second) {
                nextY[i] = constrY[i].second;
            }
            else {
                if (projYi < constrY[i].first) {
                    nextY[i] = constrY[i].first;
                }
            }


        }
        std::swap(curX, nextX);
        std::swap(curY, nextY);
    }

}*/



ShipController::ShipController()
{
    set_process_mode(PROCESS_MODE_DISABLED);
}

ShipController::~ShipController()
{
    for (auto& worker : workers) {
        worker.request_stop();
    }

    for (auto& flag : workerFlags) {
        flag.test_and_set();
        flag.notify_one();
    }
}

void ShipController::Posess(Ship& newShip)&
{
    //ship->set_process_unhandled_key_input(true);
    //update
    ship = &newShip;

    auto& thrusters{ ship->GetThrusters() };
    auto thrustersSize{ thrusters.size() };
    relPosX.resize(thrustersSize);
    relPosY.resize(thrustersSize);
    constrX.resize(thrustersSize);
    constrY.resize(thrustersSize);
    maxThrusts.resize(thrustersSize);

    for (auto& ar : workerXs) {
        ar.resize(thrustersSize);
    }
    for (auto& ar : workerYs) {
        ar.resize(thrustersSize);
    }
    maxLinearSummedThrust = 0.;
    maxAngularSummedThrust = 0.;

    for (size_t i{ 0 }; i < thrustersSize; ++i) {
        auto pos{ thrusters[i]->get_position() };
        auto maxThrust{ thrusters[i]->GetMaxThrust() };
        relPosX[i] = pos.x;
        relPosY[i] = pos.y;
        constrY[i].first = 0.;
        maxThrusts[i] = maxThrust;
        maxLinearSummedThrust += maxThrust;
        //pos.r
        maxAngularSummedThrust += pos.cross(godot::Vector2{ maxThrust, 0. }.rotated(
            pos.angle() + std::numbers::pi_v<real_t> * 0.5 ));
    }
    assert(maxLinearSummedThrust >= 0.);
    assert(maxAngularSummedThrust >= 0.);

    if (!bWorkersInited) {
        for (size_t i{ 0 }; i < workers.size(); ++i) {
            workers[i] = std::jthread( std::bind_front(&ShipController::WorkerSolve, this), i);
            /*workers[i] = std::jthread( &ShipController::WorkerSolveStatic, std::ref(workerOuts[i]), std::ref(workerFlags[i]), std::cref(weights),
                std::cref(desiredEngineForces), std::ref(workerXs[i]), std::ref(workerYs[i]), std::cref(relPosX), std::cref(relPosY), std::cref(constrX), std::cref(constrY));*/

            //workers[i].request_stop();
        }
        bWorkersInited = true;

    }

    //shipSizeRadius = ship->GetSpriteRadius();
    //DBG_PRINT(shipSizeRadius);
    set_process_mode(PROCESS_MODE_INHERIT);
    //ship->set_process_mode(PROCESS_MODE_INHERIT);
    //set_process_unhandled_key_input(true);
    //prevCameraPos = ship->get_global_position();
    //UpdateCamera(0.);

}

void ShipController::Unposess()&
{
    ship = nullptr;
    set_process_mode(PROCESS_MODE_DISABLED);
}

double ShipController::GetThrusterRotationSpeed() const
{
    return 1.0;
}

double ShipController::GetThrusterPowerChangeSpeed() const
{
    return 1.0;
}

void ShipController::_exit_tree()
{
    Unposess();
}

void ShipController::_ready()
{
    Unposess();
    ship = Object::cast_to<Ship>(get_parent());
    if (ship) {
        Posess(*ship);
    }
}

void ShipController::_process(double deltatime)
{
    /*static vec2 weights{ 0.8, 0.2 };
    static vec3 desiredEngineForces{ 0., 0., 0. };
    assert(ship);
    auto& thrusters{ ship->GetThrusters() };
    size_t thrustersSize{ thrusters.size() };

    //alloca on stack - try
    std::vector<double> X{ relPosX };
    std::vector<double> Y{ relPosX };

    for (size_t i{ 0 }; i < thrustersSize; ++i) {
        auto pos{ thrusters[i]->get_position() };
        X[i] = thrusters[i]->GetThrust();
        Y[i] = thrusters[i]->GetThrust();

        constrX[i].first = thrusters[i]->get_rotation();
        constrX[i].second = constrX[i].first + deltatime * GetThrusterRotationSpeed();
        constrX[i].first -= deltatime * GetThrusterRotationSpeed();

        constrY[i].second = thrusters[i]->GetMaxThrust();
    }

    Solve(weights, desiredEngineForces, X, Y);

    for (size_t i{ 0 }; i < thrustersSize; ++i) {
        thrusters[i]->set_rotation(X[i]);
        //thrusters[i]->SetPowerRelative();
    }*/
}
