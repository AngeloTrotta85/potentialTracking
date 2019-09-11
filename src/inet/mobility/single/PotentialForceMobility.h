//
// Author: Emin Ilker Cetinbas (niw3_at_yahoo_d0t_com)
// Copyright (C) 2005 Emin Ilker Cetinbas
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#ifndef __INET_POTENTIALFORCEMOBILITY_H
#define __INET_POTENTIALFORCEMOBILITY_H

#include "inet/common/INETDefs.h"
#include "inet/mobility/base/MovingMobilityBase.h"

namespace inet {

/**
 * @brief Linear movement model. See NED file for more info.
 *
 * @ingroup mobility
 * @author Emin Ilker Cetinbas
 */
class INET_API PotentialForceMobility : public MovingMobilityBase
{
  protected:
    double speed;
    Coord activeForce;
    double mu;

    double actualEnergy;
    double maxEnergy;
    double dischargeWatt;
    double rechargeWatt;

    bool chargingUAV;
    PotentialForceMobility *buddy;


  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters.*/
    virtual void initialize(int stage) override;

    /** @brief Move the host*/
    virtual void move() override;

    /** @brief Update the actual velocity based on the force*/
    virtual void updateVelocity(double elapsedTime);

    /** @brief Update the actual residual energy*/
    virtual void updateEnergy(double elapsedTime);

  public:
    virtual double getMaxSpeed() const override { return speed; }
    PotentialForceMobility();

    void stop(void);
    void setPosition(Coord newPos);

    const Coord& getActiveForce() const { return activeForce; }
    void setActiveForce(const Coord& activeForce) { this->activeForce = activeForce; }

    double getActualEnergy() const { return actualEnergy;   }
    void setActualEnergy(double actualEnergy) { this->actualEnergy = actualEnergy; }

    void setMaxEnergy(double maxEnergy) { this->maxEnergy = maxEnergy; }
    double getMaxEnergy() const { return maxEnergy;   }

    void setRechargeWatt(double rechargeWatt) { this->rechargeWatt = rechargeWatt; }
    void setDischargeWatt(double dischargeWatt) { this->dischargeWatt = dischargeWatt; }

    bool isChargingUav() const { return chargingUAV; }
    void setChargingUav(bool chargingUav) { chargingUAV = chargingUav; }

    PotentialForceMobility* getBuddy() { return buddy; }
    void setBuddy(PotentialForceMobility* buddy) { this->buddy = buddy; }

};

} // namespace inet

#endif // ifndef __INET_POTENTIALFORCEMOBILITY_H

