// by Dmitry Kolontay

#include "ship_controller.h"
#include "ship/ship.h"
#include "ship/thruster.h"
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

void ShipController::Solve(const vec2& weights, const vec3& desired, 
    std::vector<double>& X, std::vector<double>& Y,
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
                ycosX.cbegin(), ycosX.cend(), -desired[0]);

            sumMinDesired[1] = std::reduce(std::execution::par_unseq,
                ysinX.cbegin(), ysinX.cend(), -desired[1]);

            sumMinDesired[2] = -desired[2];

            for (size_t i{ 0 }; i < n; ++i) {
                sumMinDesired[2] += (ysinX[i] * relX[i] - ycosX[i] * relY[i]);
            }

            auto projXi{ curX[i] - stepSize * RotationDerivative(weights,
                sumMinDesired, ycosX[i], ysinX[i], relX[i], relY[i])};
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
    
}



ShipController::ShipController()
{
    set_process_mode(PROCESS_MODE_DISABLED);
}

void ShipController::Posess(Ship& newShip)&
{
    //ship->set_process_unhandled_key_input(true);
    //update
    ship = &newShip;

    auto& thrusters{ ship->GetThrusters() };
    auto thrustersSize{ thrusters.size() };
    relPos.resize(thrustersSize * 2);

    for (size_t i{ 0 }; i < thrusters.size(); ++i) {
        auto pos{ thrusters[i]->get_position() };
        relPos[2 * i] = pos.x;
        relPos[2 * i + 1] = pos.y;
    }
    relPos.shrink_to_fit();

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

}
