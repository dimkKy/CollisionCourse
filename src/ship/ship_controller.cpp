// by Dmitry Kolontay

#include "ship_controller.h"
#include "ship/ship.h"
#include "ship/thruster.h"
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

double ShipController::RotationDerivative(const vec2& weights, const vec3& sumsMindes,
    double ycosX, double ysinX, double relX, double relY)
{
    return 2. * (weights.first * (sumsMindes[1] * ycosX - sumsMindes[0] * ysinX) +
        weights.second * sumsMindes[2] * (relX * ycosX + relY * ysinX));
}

/*double ShipController::ThrustDerivative(const vec2& weights, const vec3& desMinSums,
    double relX, double relY, double sinX, double cosX)
{
    return 2. * (weights.second * desMinSums[2] * (relY * cosX - relX * sinX) -
        weights.first * (desMinSums[0] * cosX + desMinSums[1] * sinX) );
}*/

double ShipController::ThrustDerivative(const vec2& weights, const vec3& sumsMindes,
    double sinX, double cosX, double relX, double relY)
{
    return 2. * (weights.first * (sumsMindes[0] * cosX + sumsMindes[1] * sinX) +
        weights.second * sumsMindes[2] * (relX * sinX - relY * cosX));
}

//implement zip iterator?

void ShipController::SetDesiredLinearForce(const godot::Vector2& desiredSpeed)
{
    SetDesiredLinearForce(desiredSpeed, get_physics_process_delta_time());
}

void ShipController::SetDesiredLinearForce(const godot::Vector2& desiredSpeed, double physDeltatime)
{
    assert(ship);
    //ship->GetExternalLinearForce()

    auto desiredAcceleration{ (desiredSpeed - ship->get_linear_velocity()) / physDeltatime };
    auto desiredTotalForce{ desiredAcceleration * ship->get_mass() };
    auto external{ ship->GetExternalLinearForce() };
    auto desiredEngineForce = desiredTotalForce - ship->GetExternalLinearForce();
    //add damping 
    //desiredEngineForce *= 0.9;
    desiredEngineForces[0] = desiredEngineForce.x;
    desiredEngineForces[1] = desiredEngineForce.y;
}

double ShipController::Solve(const vec2& weights, const vec3& desiredEngineForces,
    std::vector<double>& X, std::vector<double>& Y)
{
    //REDO
    //worker 0 - max thrust, same rotation
    //worker 1 - max thrust, left rotation
    //worker 2 - max thrust, right rotation
    //worker 3 - same thrust, left rotation
    //worker 4 - same thrust, right rotation

    //resize arrays
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

    double out{ SubSolve(weights, desiredEngineForces, X, Y, relPosX, relPosY, constrX, constrY) };
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
    }
    return out;
}

double ShipController::SubSolve(const vec2& weights, const vec3& desiredEngineForces,
    std::vector<double>& X, std::vector<double>& Y,
    const std::vector<double>& relX, const std::vector<double>& relY,
    const std::vector<vec2>& constrX, const std::vector<vec2>& constrY)
{
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

            //reconsider minus
            sumMinDesired[0] = std::reduce(std::execution::par_unseq,
                ycosX.cbegin(), ycosX.cend(), -desiredEngineForces[0]);

            sumMinDesired[1] = std::reduce(std::execution::par_unseq,
                ysinX.cbegin(), ysinX.cend(), -desiredEngineForces[1]);

            sumMinDesired[2] = -desiredEngineForces[2];

            for (size_t i{ 0 }; i < n; ++i) {
                sumMinDesired[2] += (ysinX[i] * relX[i] - ycosX[i] * relY[i]);
            }

            double projXi{ curX[i] - stepSize * RotationDerivative(weights,
                sumMinDesired, ycosX[i], ysinX[i], relX[i], relY[i]) };
            double projYi{ curY[i] - stepSize * ThrustDerivative(weights,
                sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]) };

            nextX[i] = std::clamp(projXi, constrX[i].first, constrX[i].second);
            nextY[i] = std::clamp(projYi, constrY[i].first, constrY[i].second);

        }
        std::swap(curX, nextX);
        std::swap(curY, nextY);

        //difference, next is free

        std::transform(std::execution::par_unseq,
            nextX.cbegin(), nextX.cend(), curX.cbegin(),
            nextX.begin(), sqrSummator);
        std::transform(std::execution::par_unseq,
            nextY.cbegin(), nextY.cend(), curY.cbegin(),
            nextY.begin(), sqrSummator);

        if (std::reduce(std::execution::par_unseq, nextX.cbegin(), nextX.cend(), 0.) +
            std::reduce(std::execution::par_unseq, nextY.cbegin(), nextY.cend(), 0.) < Utils::kindaSmallDouble * 10.) {
            break;
        }
        stepSize *= 0.95;

    } while (true);

    if (curX != X) {
        X.swap(curX);
    }
    if (curY != Y) {
        Y.swap(curY);
    }

    static auto ySinner = [](double x, double y) {
        return y * std::sin(x);
    };

    static auto yCosiner = [](double x, double y) {
        return y * std::cos(x);
    };

    return SubSolvePrepareDataForOut(weights, X, Y, relX, relY, desiredEngineForces, ysinX, ycosX, sumMinDesired);
}

double ShipController::SubSolveNoRotationConstraint(const vec2& weights, const vec3& desiredEngineForces, 
    std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, 
    const std::vector<double>& relY, const std::vector<vec2>& constrY)
{
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

            //reconsider minus
            sumMinDesired[0] = std::reduce(std::execution::par_unseq,
                ycosX.cbegin(), ycosX.cend(), -desiredEngineForces[0]);

            sumMinDesired[1] = std::reduce(std::execution::par_unseq,
                ysinX.cbegin(), ysinX.cend(), -desiredEngineForces[1]);

            sumMinDesired[2] = -desiredEngineForces[2];

            for (size_t i{ 0 }; i < n; ++i) {
                sumMinDesired[2] += (ysinX[i] * relX[i] - ycosX[i] * relY[i]);
            }

            auto projXi{ curX[i] - stepSize * RotationDerivative(weights,
                sumMinDesired, ycosX[i], ysinX[i], relX[i], relY[i]) };
            auto projYi{ curY[i] - stepSize * ThrustDerivative(weights,
                sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]) };

            nextX[i] = projXi;
            nextY[i] = std::clamp(projYi, constrY[i].first, constrY[i].second);

        }
        std::swap(curX, nextX);
        std::swap(curY, nextY);

        //difference, next is free

        std::transform(std::execution::par_unseq,
            nextX.cbegin(), nextX.cend(), curX.cbegin(),
            nextX.begin(), sqrSummator);
        std::transform(std::execution::par_unseq,
            nextY.cbegin(), nextY.cend(), curY.cbegin(),
            nextY.begin(), sqrSummator);

        if (std::reduce(std::execution::par_unseq, nextX.cbegin(), nextX.cend(), 0.) +
            std::reduce(std::execution::par_unseq, nextY.cbegin(), nextY.cend(), 0.) < Utils::kindaSmallDouble * 10.) {
            break;
        }
        stepSize *= 0.95;

    } while (true);

    if (curX != X) {
        X.swap(curX);
    }
    if (curY != Y) {
        Y.swap(curY);
    }

    SubSolvePrepareDataForOut(weights, X, Y, relX, relY, desiredEngineForces, ysinX, ycosX, sumMinDesired);
}

double ShipController::SubSolveWithFixedRotation(const vec2& weights, const vec3& desiredEngineForces, 
    const std::vector<double>& X, std::vector<double>& Y, const std::vector<double>& relX, 
    const std::vector<double>& relY, const std::vector<vec2>& constrY)
{
    std::vector<double> bufY{ Y };
    std::vector<double>& curY{ Y };
    std::vector<double>& nextY{ bufY };

    std::vector<double> sinX{ X };
    std::vector<double> cosX{ Y };

    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), sinX.begin(),
        static_cast<double(*)(double)>(std::sin));
    std::transform(std::execution::par_unseq,
        X.cbegin(), X.cend(), cosX.begin(),
        static_cast<double(*)(double)>(std::cos));

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
    double stepSize{ 1. };

    static auto sqrSummator = [](double a, double b) {
        return (a - b) * (a - b);
    };

    do {
        for (size_t i{ 0 }; i < n; ++i) {
            //just unseq
            std::transform(std::execution::par_unseq,
                sinX.cbegin(), sinX.cend(), curY.cbegin(),
                ysinX.begin(), std::multiplies());
            std::transform(std::execution::par_unseq,
                cosX.cbegin(), cosX.cend(), curY.cbegin(),
                ycosX.begin(), std::multiplies());

            sumMinDesired[0] = std::reduce(std::execution::par_unseq,
                ycosX.cbegin(), ycosX.cend(), -desiredEngineForces[0]);

            sumMinDesired[1] = std::reduce(std::execution::par_unseq,
                ysinX.cbegin(), ysinX.cend(), -desiredEngineForces[1]);

            sumMinDesired[2] = -desiredEngineForces[2];

            for (size_t i{ 0 }; i < n; ++i) {
                sumMinDesired[2] += (ysinX[i] * relX[i] - ycosX[i] * relY[i]);
            }

            auto projYi{ curY[i] - stepSize * ThrustDerivative(weights,
                sumMinDesired, sinX[i], cosX[i], relX[i], relY[i]) };

            nextY[i] = std::clamp(projYi, constrY[i].first, constrY[i].second);

        }
        std::swap(curY, nextY);

        std::transform(std::execution::par_unseq,
            nextY.cbegin(), nextY.cend(), curY.cbegin(),
            nextY.begin(), sqrSummator);

        if (std::reduce(std::execution::par_unseq, nextY.cbegin(), nextY.cend(), 0.) < Utils::kindaSmallDouble * 10.) {
            break;
        }
        stepSize *= 0.95;

    } while (true);

    if (curY != Y) {
        Y.swap(curY);
    }

    return SubSolvePrepareDataForOut(weights, X, Y, relX, relY, desiredEngineForces, ysinX, ycosX, sumMinDesired);
}

double ShipController::SubSolvePrepareDataForOut(const vec2& weights, const std::vector<double>& X, const std::vector<double>& Y,
    const std::vector<double>& relX, const std::vector<double>& relY, const vec3& desiredEngineForces, 
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

    sumMinDesired[0] = std::reduce(std::execution::par_unseq,
        yCosX.cbegin(), yCosX.cend(), -desiredEngineForces[0]);

    sumMinDesired[1] = std::reduce(std::execution::par_unseq,
        ySinX.cbegin(), ySinX.cend(), -desiredEngineForces[1]);

    sumMinDesired[2] = -desiredEngineForces[2];

    for (size_t i{ 0 }; i < n; ++i) {
        sumMinDesired[2] += (ySinX[i] * relX[i] - yCosX[i] * relY[i]);
    }

    return weights.first * (sumMinDesired[0] * sumMinDesired[0] + sumMinDesired[1] * sumMinDesired[1]) +
        weights.second * sumMinDesired[2] * sumMinDesired[2];
}

double ShipController::SubPreSolveSingle(const vec2& weights, const vec3& desiredEngineForces, double& X, double Y, double relX, double relY)
{
    return 0.0;
}

void ShipController::WorkerSolveStatic(std::stop_token stoken, double& outVal, std::atomic_flag& flag,
     const vec2& weights, const vec3& desiredEngineForces, std::vector<double>& X, 
     std::vector<double>& Y, const std::vector<double>& relX, const std::vector<double>& relY, 
     const std::vector<vec2>& constrX, const std::vector<vec2>& constrY)
{
     //can it never end?
     while (true) {
         flag.wait(false);
         if (stoken.stop_requested()) {
             return;
         }
         //outVal = SubSolve(weights, desiredEngineForces, X, Y, relX, relY, constrX, constrY);
         outVal = SubSolveNoRotationConstraint(weights, desiredEngineForces, X, Y, relX, relY, constrX);
         flag.clear();
         flag.notify_one();

         flag.wait(false);
         outVal = SubSolveWithFixedRotation(weights, desiredEngineForces, X, Y, relX, relY, constrY);
         flag.clear();
         flag.notify_one();
     }
}

 void ShipController::WorkerSolve(std::stop_token stoken, size_t i)
 {
     WorkerSolveStatic(stoken, workerOuts[i], workerFlags[i],
         weights, desiredEngineForces, workerXs[i], workerYs[i], relPosX, relPosY, constrX, constrY);
 }

/*void ShipController::SubSolve(const vec2& weights, const vec3& desiredEngineForces,
    std::vector<vec2>& XY,
    const std::vector<double>& relX, const std::vector<double>& relY,
    const std::vector<vec2>& constrX, const std::vector<vec2>& constrY)
{
    //X.size() = n
    //Y.size() = n
    //constrX.size() = n
    //constrY.size() = n
    //relPosX.size() = n
    //relPosY.size() = n
    //START SEVERAL THREADS?
    std::vector<vec2> buf{ XY };
    std::vector<vec2>& cur{ XY };
    std::vector<vec2>& next{ buf };

    std::vector<double> sinXcosX{ X };
    std::vector<double> cosX{ Y };

    std::vector<double> ycosX{ Y };
    std::vector<double> ysinX{ Y };

    vec3 sumMinDesired;

    const size_t n{ X.size() };

    assert(n == Y.size() == relX.size() == relY.size() == constrX.size() == constrY.size());

    //eps?
    while (true) {
        for (size_t i{ 0 }; i < n; ++i) {
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

            for (size_t i{ 0 }; i < n; ++i) {
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

    for (auto& ar : workerXs) {
        ar.resize(thrustersSize);
    }
    for (auto& ar : workerYs) {
        ar.resize(thrustersSize);
    }

    for (size_t i{ 0 }; i < thrustersSize; ++i) {
        auto pos{ thrusters[i]->get_position() };
        relPosX[i] = pos.x;
        relPosY[i] = pos.y;
        constrY[i].first = 0.;
    }


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
