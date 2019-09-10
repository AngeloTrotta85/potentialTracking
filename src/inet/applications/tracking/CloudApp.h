//
// Copyright (C) 2000 Institut fuer Telematik, Universitaet Karlsruhe
// Copyright (C) 2004,2011 Andras Varga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#ifndef __INET_CLOUDAPP_H
#define __INET_CLOUDAPP_H

#include <vector>

#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"

#include "inet/applications/base/ApplicationBase.h"

#include "../../mobility/single/PotentialForceMobility.h"

namespace inet {

/**
 * UDP application. See NED for more info.
 */
class INET_API CloudApp : public ApplicationBase
{
  protected:
    enum SelfMsgKinds { START = 1, SEND, STOP };

    // parameters
    simtime_t startTime;
    simtime_t stopTime;
    simtime_t forceUpdateTime;

    //unsigned int areaMinX;
    //unsigned int areaMaxX;
    //unsigned int areaMinY;
    //unsigned int areaMaxY;

    double wp;
    double kp;
    double dp;

    double wu;
    double ku;
    double du;

    double kr;
    double dr;

    double wc;
    double kc;
    double dc;

    double wk;
    double kk;
    double dk;

    double epsilon;

    double force_exponent;
    double deattraction_impact;

    double uavMaxEnergyJ;
    double chargingW;
    double dischargingW;

    // state
    cMessage *selfMsg = nullptr;
    cMessage *selfMsg_run = nullptr;

    // internal variables
    //std::vector< std::vector< double > > uavForcesMatrix;
    //std::vector< std::vector< double > > carForcesMatrix;
    std::vector< PotentialForceMobility * > uavMobilityModules;
    std::vector< PotentialForceMobility * > carMobilityModules;
    std::vector< IMobility * > pedonsMobilityModules;


  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    virtual void processStart();
    virtual void processStop();

    virtual void rechargeSchedule();
    virtual void updateUAVForces();
    virtual void updateMobileChargerForces();

    virtual Coord calculateAttractiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps);
    virtual Coord calculateRepulsiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps);
    virtual double calculateAttractiveForceReduction(Coord pedonPos, unsigned int uav_id, double impact, double coeffForce,
            double dRefForce, double epsForce);

    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

  public:
    CloudApp() {}
    ~CloudApp();

  public:
    static double algebraicsum(double a, double b) {
        return (a + b - (a * b));
    }
};

} // namespace inet

#endif // ifndef __INET_CLOUDAPP_H

