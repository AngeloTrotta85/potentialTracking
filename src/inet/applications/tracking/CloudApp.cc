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

#include <iostream>
#include <fstream>

#include "CloudApp.h"

#include "inet/common/ModuleAccess.h"
#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"

namespace inet {

Define_Module(CloudApp);

CloudApp::~CloudApp() {
    cancelAndDelete(selfMsg);
    cancelAndDelete(selfMsg_run);
}

void CloudApp::initialize(int stage) {
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {

        startTime = par("startTime");
        stopTime = par("stopTime");

        wp = par("wp");
        kp = par("kp");
        dp = par("dp");

        wu = par("wu");
        ku = par("ku");
        du = par("du");

        kr = par("kr");
        dr = par("dr");

        wc = par("wc");
        kc = par("kc");
        dc = par("dc");

        wk = par("wk");
        kk = par("kk");
        dk = par("dk");

        wg = par("wg");
        kg = par("kg");
        dg = par("dg");

        force_exponent = par("force_exponent");
        deattraction_impact = par("deattraction_impact");
        uavMaxEnergyJ = par("uavMaxEnergyJ");
        uavInitEnergyJ = par("uavInitEnergyJ");
        chargingW = par("chargingW");
        dischargingW = par("dischargingW");
        epsilon = par("epsilon");
        rechargeTimeOffset = par("rechargeTimeOffset");
        coverageDistance = par("coverageDistance");

        keepFullMap = par("keepFullMap").boolValue();
        forceMapOffset = par("forceMapOffset");
        forceDrawCoefficient = par("forceDrawCoefficient");
        fileData = std::string(par("fileData").stringValue());

        areaMinX = floor(par("areaMinX").doubleValue());
        areaMaxX = floor(par("areaMaxX").doubleValue());
        areaMinY = floor(par("areaMinY").doubleValue());
        areaMaxY = floor(par("areaMaxY").doubleValue());

        forceUpdateTime = par("forceUpdateTime");
        statisticOffset = par("statisticOffset");

        chargingType = std::string(par("chargingType").stringValue());
        if (chargingType.compare("mobileCar") == 0) {
            cType = MOBILE_CAR;
        } else if (chargingType.compare("fixedStation") == 0) {
            cType = FIXED_STATION;
        } else if (chargingType.compare("noCharger") == 0) {
            cType = NO_CHARGER;
        } else {
            throw cRuntimeError("Invalid chargingType parameter");
        }

        chargingScheduling = std::string(par("chargingScheduling").stringValue());
        if (chargingScheduling.compare("stimrespSched") == 0) {
            cScheduling = STIMULUS_SCHEDULING;
        } else if (chargingScheduling.compare("greedySched") == 0) {
            cScheduling = GREEDY_SCHEDULING;
        } else {
            throw cRuntimeError("Invalid chargingScheduling parameter");
        }

        numClusterCrowd = par("numClusterCrowd");
        crowdFollow = std::string(par("crowdFollow").stringValue());
        if (crowdFollow.compare("noCrowd") == 0) {
            crowd = CROWD_NO;
        } else if (crowdFollow.compare("knownCrowd") == 0) {
            crowd = CROWD_KNOWN;
        } else if (crowdFollow.compare("clusterCrowd") == 0) {
            crowd = CROWD_CLUSTER;
        } else {
            throw cRuntimeError("Invalid crowdFollow parameter");
        }


        if (stopTime >= SIMTIME_ZERO && stopTime < startTime)
            throw cRuntimeError("Invalid startTime/stopTime parameters");
        selfMsg = new cMessage("sendTimer");
        selfMsg_run = new cMessage("forceTimer");

        selfMsg_stat = new cMessage("statTimer");
        scheduleAt(simTime() + statisticOffset, selfMsg_stat);

        actMean_probRecharge = 0;
        nMean_probRecharge = 0;
        actMean_probFly = 0;
        nMean_probFly = 0;

        WATCH(actMean_probRecharge);
    }
    else if (stage == INITSTAGE_LAST) {
        int numUAV = this->getParentModule()->getParentModule()->getSubmodule("uav", 0)->getVectorSize();
        int numPedestrian = this->getParentModule()->getParentModule()->getSubmodule("pedestrian", 0)->getVectorSize();
        int numCar = this->getParentModule()->getParentModule()->getSubmodule("mobcharger", 0)->getVectorSize();

        uavMobilityModules.resize(numUAV);
        lastRandom.resize(numUAV, std::make_pair(Coord::ZERO, simTime()));
        pedonsMobilityModules.resize(numPedestrian);
        pedonsKnowledge.resize(numPedestrian, false);
        carMobilityModules.resize(numCar);

        for (int u = 0; u < numUAV; u++) {
            PotentialForceMobility *uavMob = dynamic_cast<PotentialForceMobility *> (this->getParentModule()->getParentModule()->getSubmodule("uav", u)->getSubmodule("mobility"));
            uavMob->setMaxEnergy( uavMaxEnergyJ );
            uavMob->setActualEnergy( uavInitEnergyJ );
            uavMob->setChargingUav( false );
            uavMob->setDischargeWatt( dischargingW );
            uavMob->setRechargeWatt( chargingW );
            uavMobilityModules[u] = uavMob;
        }
        for (int p = 0; p < numPedestrian; p++) {
            IMobility *pedMob = dynamic_cast<IMobility *> (this->getParentModule()->getParentModule()->getSubmodule("pedestrian", p)->getSubmodule("mobility"));
            pedonsMobilityModules[p] = pedMob;
        }
        for (int c = 0; c < numCar; c++) {
            PotentialForceMobility *carMob = dynamic_cast<PotentialForceMobility *> (this->getParentModule()->getParentModule()->getSubmodule("mobcharger", c)->getSubmodule("mobility"));
            carMobilityModules[c] = carMob;
        }

        /*coverageMap.resize(areaMaxX);
        for (auto& vm : coverageMap) {
            vm.resize(areaMaxY, 0);
        }*/

        pedIsCovered.resize(numPedestrian);
        pedCoverage.resize(numPedestrian);


        //generate the 2 forces maps
        if (keepFullMap) {
            uavForcesMatrix.resize(numUAV);
            for (auto& u : uavForcesMatrix) {
                u.resize(floor(((double) areaMaxX) / forceMapOffset));
                for (auto& r : u) {
                    r.resize(floor(((double) areaMaxY) / forceMapOffset), Coord::ZERO);
                }
            }
            //uavForcesMatrix.resize(floor(areaMaxX / forceMapOffset));
            //for (auto& r : uavForcesMatrix) {
            //    r.resize(floor(areaMaxY / forceMapOffset), 0);
            //}

            //carForcesMatrix.resize(floor(areaMaxX / forceMapOffset));
            //for (auto& r : carForcesMatrix) {
            //    r.resize(floor(areaMaxY / forceMapOffset), 0);
            //}
            carForcesMatrix.resize(numCar);
            for (auto& c : carForcesMatrix) {
                c.resize(floor(((double) areaMaxX) / forceMapOffset));
                for (auto& r : c) {
                    r.resize(floor(((double) areaMaxY) / forceMapOffset), Coord::ZERO);
                }
            }
        }
    }
}

void CloudApp::finish() {

    recordScalar("lifetime", simTime());

    double sumCovered = 0;
    double sumCoverage = 0;
    double countCovered = 0;
    double countCoverage = 0;

    for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
        for (auto& pc : pedIsCovered[p]){
            sumCovered += pc;
            countCovered += 1.0;
        }
        for (auto& pc : pedCoverage[p]){
            sumCoverage += pc;
            countCoverage += 1.0;
        }
    }

    if ((countCoverage > 0) && (countCovered > 0)) {
        recordScalar("pedestrianNumCoverage", sumCoverage / countCoverage);
        recordScalar("pedestrianIsCovered", sumCovered / countCovered);
    }

    recordScalar("recharge probability", actMean_probRecharge);
    recordScalar("fly probability", actMean_probFly);

    if (keepFullMap) {
        updatePedestrianKnowledge();

        for (unsigned int u = 0; u < uavForcesMatrix.size(); u++) {
            for (unsigned int x = 0; x < uavForcesMatrix[u].size(); x++) {
                for (unsigned int y = 0; y < uavForcesMatrix[u][x].size(); y++) {
                    uavForcesMatrix[u][x][y] = calculateUAVForce(u, Coord(x*forceMapOffset, y*forceMapOffset)) * forceDrawCoefficient;
                }
            }
        }
        for (unsigned int c = 0; c < carForcesMatrix.size(); c++) {
            for (unsigned int x = 0; x < carForcesMatrix[c].size(); x++) {
                for (unsigned int y = 0; y < carForcesMatrix[c][x].size(); y++) {
                    carForcesMatrix[c][x][y] = calculateMobileChargerForce(c, Coord(x*forceMapOffset, y*forceMapOffset)) * forceDrawCoefficient;
                }
            }
        }

        char bufftxt[32];
        char buffname[256];

        for (unsigned int u = 0; u < uavForcesMatrix.size(); u++) {
            snprintf(bufftxt, sizeof(bufftxt), "u%d", u);
            snprintf(buffname, sizeof(buffname), fileData.c_str(), bufftxt);

            //std::ofstream ofs (buffname, std::ofstream::out | std::ofstream::app);
            std::ofstream ofs (buffname, std::ofstream::out);
            if (ofs.is_open()) {
                for (unsigned int x = 0; x < uavForcesMatrix[u].size(); x++) {
                    for (unsigned int y = 0; y < uavForcesMatrix[u][x].size(); y++) {
                        ofs << x << "\t" << y << "\t" << uavForcesMatrix[u][x][y].x << "\t" << uavForcesMatrix[u][x][y].y;
                        ofs << endl;
                    }
                }

                ofs.close();
            }
        }

        for (unsigned int c = 0; c < carForcesMatrix.size(); c++) {
            snprintf(bufftxt, sizeof(bufftxt), "c%d", c);
            snprintf(buffname, sizeof(buffname), fileData.c_str(), bufftxt);

            //std::ofstream ofs (buffname, std::ofstream::out | std::ofstream::app);
            std::ofstream ofs (buffname, std::ofstream::out);
            if (ofs.is_open()) {
                for (unsigned int x = 0; x < carForcesMatrix[c].size(); x++) {
                    for (unsigned int y = 0; y < carForcesMatrix[c][x].size(); y++) {
                        ofs << x << "\t" << y << "\t" << carForcesMatrix[c][x][y].x << "\t" << carForcesMatrix[c][x][y].y;
                        ofs << endl;
                    }
                }
                ofs.close();
            }
        }
    }

    ApplicationBase::finish();
}

void CloudApp::processStart() {
    if (stopTime >= SIMTIME_ZERO) {
        selfMsg->setKind(STOP);
        scheduleAt(stopTime, selfMsg);
    }

    nextRechargeTime = simTime() + rechargeTimeOffset;

    scheduleAt(simTime() + forceUpdateTime, selfMsg_run);
}


void CloudApp::processStop() {
    cancelEvent(selfMsg_run);
}

void CloudApp::handleMessageWhenUp(cMessage *msg) {

    //std::cerr << "handleMessageWhenUp: " << msg->getFullName() << endl << std::flush;

    if (msg->isSelfMessage()) {

        if (msg == selfMsg) {
            switch (selfMsg->getKind()) {
            case START:
                processStart();
                break;

            case STOP:
                processStop();
                break;

            default:
                throw cRuntimeError("Invalid kind %d in self message", (int)selfMsg->getKind());
            }
        }
        else if (msg == selfMsg_run) {
            //std::cerr << "handleMessageWhenUp: recognized " << msg->getFullName() << endl << std::flush;

            updatePedestrianKnowledge();

            if (cType != NO_CHARGER) {
                rechargeSchedule();
            }

            updateUAVForces();

            if (cType != NO_CHARGER) {
                updateMobileChargerForces();
            }

            //updatePedestrianForces();

            checkLifetime();

            //std::cerr << "handleMessageWhenUp: OK updated forces" << endl << std::flush;

            scheduleAt(simTime() + forceUpdateTime, selfMsg_run);
        }
        else if (msg == selfMsg_stat) {
            makeStat();
            scheduleAt(simTime() + statisticOffset, selfMsg_stat);
        }
        else {
            throw cRuntimeError("Invalid self message", msg->getFullPath());
        }
    }
}

void CloudApp::makeStat(void) {

    for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
        double isCovered = 0;
        double numCoverage = 0;
        Coord pPos = pedonsMobilityModules[p]->getCurrentPosition();

        for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
            if (pPos.distance(uavMobilityModules[u]->getCurrentPosition()) <= coverageDistance) {
                isCovered = 1.0;
                numCoverage += 1.0;
            }
        }

        pedIsCovered[p].push_back(isCovered);
        if (isCovered > 0) {
            pedCoverage[p].push_back(numCoverage);
        }
    }
}

void CloudApp::refreshDisplay() const {
    ApplicationBase::refreshDisplay();

    //char buf[100];
    //sprintf(buf, "rcvd: %d pks\nsent: %d pks", numReceived, numSent);
    //getDisplayString().setTagArg("t", 0, buf);
}

void CloudApp::handleStartOperation(LifecycleOperation *operation) {
    simtime_t start = std::max(startTime, simTime());
    if ((stopTime < SIMTIME_ZERO) || (start < stopTime) || (start == stopTime && startTime == stopTime)) {
        selfMsg->setKind(START);
        scheduleAt(start, selfMsg);
    }
}

void CloudApp::handleStopOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    cancelEvent(selfMsg_run);
    delayActiveOperationFinish(par("stopOperationTimeout"));
}

void CloudApp::handleCrashOperation(LifecycleOperation *operation) {
    cancelEvent(selfMsg);
    cancelEvent(selfMsg_run);
}

void CloudApp::updatePedestrianKnowledge(void) {
    for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
        IMobility *pmob = pedonsMobilityModules[p];
        bool covered = false;

        for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
            if(pmob->getCurrentPosition().distance(uavMobilityModules[u]->getCurrentPosition()) <= coverageDistance) {
                covered = true;
                break;
            }
        }

        pedonsKnowledge[p] = covered;
    }
}

Coord CloudApp::calculateAttractiveForcePedestrianGroup(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps) {

    Coord ris = target - me;
    while (ris == Coord::ZERO) {
        ris = Coord(dblrand() - 0.5, dblrand() - 0.5);
    }
    ris.normalize();

    if (dRef < 0) {
        ris = ris * maxVal * (1.0 - exp(-(coeff * pow(me.distance(target), force_exponent))));
    }
    else {
        ris = ris * maxVal * (1.0 - exp(log(eps) * pow(me.distance(target) / dRef, force_exponent)));
    }

    return ris;
}

Coord CloudApp::calculateAttractiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps) {

    Coord ris = target - me;
    while (ris == Coord::ZERO) {
        ris = Coord(dblrand() - 0.5, dblrand() - 0.5);
    }
    ris.normalize();

    if (dRef < 0) {
        ris = ris * maxVal * exp(-(coeff * pow(me.distance(target), force_exponent)));
    }
    else {
        ris = ris * maxVal * exp(log(eps) * pow(me.distance(target) / dRef, force_exponent));
    }

    return ris;
}

double CloudApp::calculateAttractiveForceReduction(Coord pedonPos, unsigned int uav_id, double impact, double coeffForce, double dRefForce, double epsForce) {
    double reducingFactor = 1.0;

    for (unsigned int j = 0; j < uavMobilityModules.size(); j++) {
        if (uav_id != j) {
            double dist_jUAV_Ped = uavMobilityModules[j]->getCurrentPosition().distance(pedonPos);
            double val;

            if (dRefForce < 0) {
                val = exp(-(coeffForce * pow(dist_jUAV_Ped, force_exponent)));
            }
            else {
                val = exp(log(epsForce) * pow(dist_jUAV_Ped / dRefForce, force_exponent));
            }

            double factor = 1.0 - (impact * val);
            reducingFactor *= factor;
        }
    }
    return reducingFactor;
}

Coord CloudApp::calculateRepulsiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps) {

    Coord ris = me - target;
    while (ris == Coord::ZERO) {
        ris = Coord(dblrand() - 0.5, dblrand() - 0.5);
    }
    ris.normalize();

    if (dRef < 0) {
        ris = ris * maxVal * exp(-(coeff * pow(me.distance(target), force_exponent)));
    }
    else {
        ris = ris * maxVal * exp(log(eps) * pow(me.distance(target) / dRef, force_exponent));
    }

    return ris;
}

void CloudApp::rechargeSchedule() {
    //EV << "Scheduling the recharges!!!" << endl;

    // free UAV and charging stations if UAV is fully charged
    for (auto& puav : uavMobilityModules) {
        if ((puav->isChargingUav()) && (puav->getActualEnergy() >= puav->getMaxEnergy())) {

            PotentialForceMobility *pcar = puav->getBuddy();

            pcar->setChargingUav(false);
            pcar->setBuddy(nullptr);

            puav->setChargingUav(false);
            puav->setBuddy(nullptr);
        }
    }

    if (simTime() >= nextRechargeTime) {
        std::list< std::tuple<PotentialForceMobility *, double> > flyingUAVs;
        double minEnergy = std::numeric_limits<double>::max();
        double maxEnergy = 0;

        for (auto& puav : uavMobilityModules) {
            if (!puav->isChargingUav()) {
                // energy
                if (puav->getActualEnergy() < minEnergy) minEnergy = puav->getActualEnergy();
                if (puav->getActualEnergy() > maxEnergy) maxEnergy = puav->getActualEnergy();
            }
        }

        // check UAV to get off from the charging station
        for (auto& puav : uavMobilityModules) {
            if (puav->isChargingUav()) {
                double prob = 0;

                //AOB probability
                //double energyRatio = puav->getActualEnergy() / puav->getMaxEnergy();
                //double worseUAV = 0;
                //for (auto& puavcheck : uavMobilityModules) {
                //    if ( (!puavcheck->isChargingUav()) && (puavcheck->getActualEnergy() < puav->getActualEnergy()) ) {
                //        ++worseUAV;
                //    }
                //}
                //double prob = pow((energyRatio), 1.0 / (worseUAV + 1.0) );      // vers.1
                //double prob = 1.0 - pow((1.0 - energyRatio), 1.0 / (worseUAV + 1.0) );      // vers.2

                if (cScheduling == STIMULUS_SCHEDULING) {   // Stimulus-Reply probability

                    if (maxEnergy == minEnergy) {
                        prob = puav->getActualEnergy() / puav->getMaxEnergy();
                    }
                    else {
                        double stimulus = (puav->getActualEnergy() - minEnergy) / (maxEnergy - minEnergy);
                        //double threshold = (maxEnergy - puav->getActualEnergy()) / (maxEnergy - minEnergy);
                        double threshold = 1.0 - (puav->getActualEnergy() / puav->getMaxEnergy());
                        prob = pow(stimulus, 2.0) / (pow(stimulus, 2.0) + pow(threshold, 2.0));
                    }
                } else if (cScheduling == GREEDY_SCHEDULING) {   // Greedy probability
                    prob = puav->getActualEnergy() / puav->getMaxEnergy();
                }

                if (nMean_probFly == 0) {
                    actMean_probFly = prob;
                }
                else {
                    actMean_probFly = actMean_probFly + ((prob - actMean_probFly) / (nMean_probFly + 1.0));
                }
                ++nMean_probFly;

                if (dblrand() < prob) {
                    PotentialForceMobility *pcar = puav->getBuddy();

                    pcar->setChargingUav(false);
                    pcar->setBuddy(nullptr);

                    puav->setChargingUav(false);
                    puav->setBuddy(nullptr);
                }
            }
        }

        minEnergy = std::numeric_limits<double>::max();
        maxEnergy = 0;
        for (auto& puav : uavMobilityModules) {
            if (!puav->isChargingUav()) {
                // energy
                if (puav->getActualEnergy() < minEnergy) minEnergy = puav->getActualEnergy();
                if (puav->getActualEnergy() > maxEnergy) maxEnergy = puav->getActualEnergy();
            }
        }
        double nCoveredPed = 0.0;
        for (auto& pped : pedonsMobilityModules) {
            Coord pPos = pped->getCurrentPosition();
            for (auto& puav : uavMobilityModules) {
                if ((!puav->isChargingUav()) && (puav->getCurrentPosition().distance(pPos) <= coverageDistance)) {
                    nCoveredPed += 1.0;
                    break;
                }
            }
        }

        // get and order (energy based) all the flying UAVs
        for (auto& puav : uavMobilityModules) {
            if (!puav->isChargingUav()) {
                flyingUAVs.push_back(std::make_tuple(puav, puav->getActualEnergy()));
            }
        }
        flyingUAVs.sort(CloudApp::energySort);

        //for each UAV check to recharge
        for (auto& t_uav : flyingUAVs) {
            PotentialForceMobility *puav = std::get<0>(t_uav);
            Coord uavPos = puav->getCurrentPosition();

            double energyRatio = puav->getActualEnergy() / puav->getMaxEnergy();
            double coveredPed = 0.0;

            for (auto& pped : pedonsMobilityModules) {
                if (pped->getCurrentPosition().distance(uavPos) <= coverageDistance) {
                    coveredPed += 1.0;
                }
            }

            // get and order (UAV-distance based) all the free charging stations
            std::list< std::tuple<PotentialForceMobility *, Coord, Coord > > freeChargers;
            for (auto& pcar : carMobilityModules) {
                if (!pcar->isChargingUav()) {
                    freeChargers.push_back(std::make_tuple(pcar, pcar->getCurrentPosition(), puav->getCurrentPosition()));
                }
            }
            freeChargers.sort(CloudApp::distanceSort);

            //check if to charge
            for (auto& t_car : freeChargers) {
                double prob = 0;
                PotentialForceMobility *pcar = std::get<0>(t_car);

                //AOB probability
                //double dist = puav->getCurrentPosition().distance(pcar->getCurrentPosition());
                //double prob = pow((1.0 - energyRatio),((dist/100.0) + 1.0));


                if (cScheduling == STIMULUS_SCHEDULING) {   // Stimulus-Reply probability
                    //double stimulus = (maxEnergy - puav->getActualEnergy()) / (maxEnergy - minEnergy);
                    double stimulus = 1.0 - energyRatio;
                    //double threshold = coveredPed / nCoveredPed;
                    if ((nCoveredPed > 0) && ((maxEnergy > minEnergy))){
                        double threshold = algebraicsum(coveredPed / nCoveredPed, (puav->getActualEnergy() - minEnergy) / (maxEnergy - minEnergy)) ;
                        prob = pow(stimulus, 2.0) / (pow(stimulus, 2.0) + pow(threshold, 2.0));
                    }
                    else {
                        prob = stimulus;
                    }

                    /*EV << "UAV " << pcar->getCurrentPosition() <<
                            ". Stimulus = " << stimulus <<
                            "; Threshold = " << threshold <<
                            "; PROB = " << prob << endl;*/
                }
                else if (cScheduling == GREEDY_SCHEDULING) {   // Greedy probability
                    prob = 1.0 - energyRatio;
                }

                if (nMean_probRecharge == 0) {
                    actMean_probRecharge = prob;
                }
                else {
                    actMean_probRecharge = actMean_probRecharge + ((prob - actMean_probRecharge) / (nMean_probRecharge + 1.0));
                }
                ++nMean_probRecharge;

                if (dblrand() < prob) {

                    puav->setChargingUav(true);
                    puav->setBuddy(pcar);

                    pcar->setChargingUav(true);
                    pcar->setBuddy(puav);

                }
                break;
            }
        }

        nextRechargeTime = simTime() + rechargeTimeOffset;
    }
}

Coord CloudApp::calculateUAVForce(int u, Coord pos) {
    Coord uavForce = Coord::ZERO;

    // update the force on the flying UAVs only
    if (!uavMobilityModules[u]->isChargingUav()) {

        // add attractive force from the pedestrians
        for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
            if (pedonsKnowledge[p]) {
                Coord actPedForce = calculateAttractiveForce(pos, pedonsMobilityModules[p]->getCurrentPosition(), wp, kp, dp, epsilon);
                actPedForce += calculateAttractiveForce(pos, pedonsMobilityModules[p]->getCurrentPosition(), 0.5, kr, dr, epsilon);

                double reducingFactor = calculateAttractiveForceReduction(pedonsMobilityModules[p]->getCurrentPosition(), u, deattraction_impact, kr, dr, epsilon);

                uavForce += actPedForce * reducingFactor;
            }
        }

        //add repulsive forces from the other flying UAVs
        for (unsigned int j = 0; j < uavMobilityModules.size(); j++) {
            if ((u != j) && (!uavMobilityModules[j]->isChargingUav())) {
                uavForce += calculateRepulsiveForce(pos, uavMobilityModules[j]->getCurrentPosition(), wu, ku, du, epsilon);
            }
        }

        if (crowd == CROWD_NO) {
            // if no pedestrian/uav close, make coverage
            if(uavForce.length() < 0.1) {
                if ((simTime() - lastRandom[u].second) > 60) {
                    lastRandom[u].first = Coord(dblrand() - 0.5, dblrand() - 0.5) * 5.0;
                    lastRandom[u].second = simTime();
                }
                uavForce += lastRandom[u].first;
            }
        }
        else if (crowd == CROWD_KNOWN) {
            // follow pedestrian[0]
            uavForce += calculateAttractiveForce(pos, pedonsMobilityModules[0]->getCurrentPosition(), wg, kg, dg, epsilon);
        }
        else if (crowd == CROWD_CLUSTER) {
            // make numClusterCrowd clusters and follow the closest
            std::vector< Coord > cHead (numClusterCrowd, Coord(dblrand() * ((double)(areaMaxX)), dblrand() * ((double)(areaMaxY))) );
            calculateKmeans(cHead);

            uavForce += calculateAttractiveForce(pos, cHead[intrand(cHead.size())], wg, kg, dg, epsilon);
        }

    }
    else {
        uavForce = uavMobilityModules[u]->getBuddy()->getCurrentPosition() - pos;
    }

    return uavForce;
}

void CloudApp::calculateKmeans(std::vector< Coord > &cHeads) {
    std::vector< std::list<IMobility *> > pedClusters;
    pedClusters.resize(cHeads.size());
    int numLoops = 30;

    do {
        // update pedestrians clusters
        for (auto& pl : pedClusters) {
            pl.clear();
        }
        for(unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
            if (pedonsKnowledge[p]) {
                Coord pPos = pedonsMobilityModules[p]->getCurrentPosition();
                double bestDist = std::numeric_limits<double>::max();
                int idx = -1;

                for(unsigned int c = 0; c < cHeads.size(); c++) {
                    if ((idx < 0) || (pPos.distance(cHeads[c]) < bestDist) ) {
                        idx = c;
                        bestDist = pPos.distance(cHeads[c]);
                    }
                }

                pedClusters[idx].push_back(pedonsMobilityModules[p]);
            }
        }

        // update cluster heads
        for (unsigned int c = 0; c < cHeads.size(); c++) {
            if (pedClusters[c].size() > 0) {
                cHeads[c] = Coord::ZERO;
                for (auto& ph : pedClusters[c]) {
                    cHeads[c] += ph->getCurrentPosition();
                }
                cHeads[c] /= (double) pedClusters[c].size();
            }
        }

        --numLoops;
    } while (numLoops > 0);
}

void CloudApp::updateUAVForces() {
    //EV << "Updating UAV Forces!!!" << endl;
    bool coverageMapUpdated = false;

    for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {

        uavMobilityModules[u]->setActiveForce(calculateUAVForce(u, uavMobilityModules[u]->getCurrentPosition()));
    }
}

Coord CloudApp::calculateMobileChargerForce(int c, Coord pos) {
    Coord carForce = Coord::ZERO;

    if (!carMobilityModules[c]->isChargingUav()) {

        // add attractive force from the flying UAVs
        for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
            if (!uavMobilityModules[u]->isChargingUav()) {
                Coord actUAVForce = calculateAttractiveForce(pos, uavMobilityModules[u]->getCurrentPosition(), wc, kc, dc, epsilon);
                double reducingFactor = 1.0 - (uavMobilityModules[u]->getActualEnergy() / uavMobilityModules[u]->getMaxEnergy());

                carForce += actUAVForce * reducingFactor;
            }
        }

        //add repulsive forces from the pedestrians
        for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
            carForce += calculateRepulsiveForce(pos, pedonsMobilityModules[p]->getCurrentPosition(), wk, kk, dk, epsilon);
        }

    }

    return carForce;
}

void CloudApp::updateMobileChargerForces() {
    //EV << "Updating Mobile Charger Forces!!!" << endl;

    if (cType == MOBILE_CAR) {
        for (unsigned int c = 0; c < carMobilityModules.size(); c++) {
            carMobilityModules[c]->setActiveForce(calculateMobileChargerForce(c, carMobilityModules[c]->getCurrentPosition()));
        }
    }
    else if (cType == FIXED_STATION) {
        for (unsigned int c = 0; c < carMobilityModules.size(); c++) {
            Coord centerPoint = Coord((areaMinX + areaMaxX) / 2.0, (areaMinY + areaMaxY) / 2.0);
            carMobilityModules[c]->setActiveForce(centerPoint - carMobilityModules[c]->getCurrentPosition());
        }
    }
}

void CloudApp::updatePedestrianForces() {
    /*for (unsigned int p = 1; p < pedonsMobilityModules.size(); p++) {
        cModule *cmPed = dynamic_cast<cModule *>(pedonsMobilityModules[p]);

        if (cmPed->hasSubmodules()) {
            PotentialForceMobility *pFmob = dynamic_cast<PotentialForceMobility *>(cmPed->getSubmodule("element", 1));
            Coord pedForce = Coord::ZERO;

            pedForce += calculateAttractiveForcePedestrianGroup(pFmob->getCurrentPosition(),
                    pedonsMobilityModules[0]->getCurrentPosition(), 5, 0.01, 20, 0.1);

            //for (unsigned int p1 = 1; p1 < pedonsMobilityModules.size(); p1++) {
            //    if (p != p1) {
            //        PotentialForceMobility *pFmob1 = dynamic_cast<PotentialForceMobility *>((dynamic_cast<cModule *>(pedonsMobilityModules[p1]))->getSubmodule("element", 1));
            //        pedForce += calculateRepulsiveForce(pFmob->getCurrentPosition(), pFmob1->getCurrentPosition(), wu, ku, du, epsilon);
            //    }
            //}

            pFmob->setActiveForce(pedForce);
        }
        else {
            break;
        }
    }*/

    /*for (unsigned int p = 1; p < pedonsMobilityModules.size(); p++) {
        PedestrianMobility *pFmob = dynamic_cast<PedestrianMobility *>(pedonsMobilityModules[p]);
        Coord leadVel = pedonsMobilityModules[0]->getCurrentVelocity();

        if ((pFmob) && (dblrand() < 0.05)) {
        //if (pFmob) {
            Coord newVel = leadVel;

            newVel += calculateAttractiveForcePedestrianGroup(pedonsMobilityModules[p]->getCurrentPosition(),
                    pedonsMobilityModules[0]->getCurrentPosition(), 0.2, 0.01, 20, 0.2);

            newVel += Coord(dblrand(), dblrand());

            pFmob->setVelocity(newVel);
        }
    }*/

    /*for (unsigned int p = 1; p < pedonsMobilityModules.size(); p++) {
        VirtualSpringMobility *pFmob = dynamic_cast<VirtualSpringMobility *>(pedonsMobilityModules[p]);

        if (pFmob) {
            Coord myPos = pFmob->getCurrentPosition();

            // clear everything
            pFmob->clearVirtualSprings();

            for (unsigned int p1 = 0; p1 < pedonsMobilityModules.size(); p1++) {
                if (p != p1) {
                    Coord neighPos = pedonsMobilityModules[p1]->getCurrentPosition();
                    double dist = neighPos.distance(myPos);
                    double l0 = 10;

                    if (dist < 2000.0) {

                        double springDispl = l0 - dist;

                        Coord uVec = neighPos - myPos;
                        uVec.normalize();

                        //EV << "Setting force with displacement: " << springDispl << " (distance: " << distance << ")" << endl;

                        pFmob->addVirtualSpring(uVec, 0.1, l0, springDispl);
                    }
                }
            }
        }
    }*/
}


void CloudApp::checkLifetime() {
    for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
        if (uavMobilityModules[u]->getActualEnergy() <= 0) {
            endSimulation();
        }
    }
}

} // namespace inet

