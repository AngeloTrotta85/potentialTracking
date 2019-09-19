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
#include <list>

#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"

#include "inet/applications/base/ApplicationBase.h"

#include "../../mobility/single/PotentialForceMobility.h"
#include "../../mobility/single/PedestrianMobility.h"

namespace inet {

/**
 * UDP application. See NED for more info.
 */
class INET_API CloudApp : public ApplicationBase
{
public:
    typedef enum chargingType_t {
        MOBILE_CAR,
        FIXED_STATION,
        NO_CHARGER
    } chargingType_t;

    typedef enum chargingScheduling_t {
        STIMULUS_SCHEDULING,
        GREEDY_SCHEDULING
    } chargingScheduling_t;

    typedef enum crowdFollow_t {
        CROWD_NO,
        CROWD_KNOWN,
        CROWD_CLUSTER
    } crowdFollow_t;

  protected:
    enum SelfMsgKinds { START = 1, SEND, STOP };

    // parameters
    simtime_t startTime;
    simtime_t stopTime;
    simtime_t forceUpdateTime;

    unsigned int areaMinX;
    unsigned int areaMaxX;
    unsigned int areaMinY;
    unsigned int areaMaxY;

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

    double wg;
    double kg;
    double dg;

    double epsilon;

    double force_exponent;
    double deattraction_impact;

    double uavMaxEnergyJ;
    double uavInitEnergyJ;
    double chargingW;
    double dischargingW;

    simtime_t nextRechargeTime;
    double rechargeTimeOffset;

    double coverageDistance;

    bool keepFullMap;
    double forceMapOffset;
    double forceDrawCoefficient;
    std::string fileData;

    double statisticOffset;

    std::string chargingType;
    chargingType_t cType;

    std::string chargingScheduling;
    chargingScheduling_t cScheduling;

    std::string crowdFollow;
    crowdFollow_t crowd;

    int numClusterCrowd;

    // state
    cMessage *selfMsg = nullptr;
    cMessage *selfMsg_run = nullptr;
    cMessage *selfMsg_stat = nullptr;

    // internal variables
    std::vector< std::vector< std::vector< Coord > > > uavForcesMatrix;
    std::vector< std::vector< std::vector< Coord > > > carForcesMatrix;

    std::vector< PotentialForceMobility * > uavMobilityModules;
    std::vector< PotentialForceMobility * > carMobilityModules;
    std::vector< IMobility * > pedonsMobilityModules;
    std::vector< bool > pedonsKnowledge;
    //std::vector< std::vector< double > > coverageMap;
    //std::vector< std::pair<Coord, double> >coveragePointsVect;
    //std::vector< Coord >coveragePointsVect;
    std::vector< std::pair <Coord, simtime_t> > lastRandom;


    std::vector< std::vector< double > > pedIsCovered;
    std::vector< std::vector< double > > pedCoverage;

    //stats
    double actMean_probRecharge;
    double nMean_probRecharge;
    double actMean_probFly;
    double nMean_probFly;


  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    virtual void processStart();
    virtual void processStop();

    void updatePedestrianKnowledge(void);
    void makeStat(void);

    virtual void rechargeSchedule();
    Coord calculateUAVForce(int u, Coord pos);
    virtual void updateUAVForces();
    Coord calculateMobileChargerForce(int c, Coord pos);
    virtual void updateMobileChargerForces();
    virtual void updatePedestrianForces();
    virtual void checkLifetime();

    virtual Coord calculateAttractiveForcePedestrianGroup(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps);
    virtual Coord calculateAttractiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps);
    virtual Coord calculateRepulsiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps);
    virtual double calculateAttractiveForceReduction(Coord pedonPos, unsigned int uav_id, double impact, double coeffForce,
            double dRefForce, double epsForce);

    virtual void calculateKmeans(std::vector< Coord > &cHeads);

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

    static bool energySort(const std::tuple<PotentialForceMobility *, double> &first, const std::tuple<PotentialForceMobility *, double> &second) {
        return (std::get<1>(first) < std::get<1>(second));
    }

    static bool distanceSort(const std::tuple<PotentialForceMobility *, Coord, Coord > &first, const std::tuple<PotentialForceMobility *, Coord, Coord > &second) {
        return (std::get<1>(first).distance(std::get<2>(first)) < std::get<1>(second).distance(std::get<2>(second)));
    }
};

} // namespace inet

#endif // ifndef __INET_CLOUDAPP_H

