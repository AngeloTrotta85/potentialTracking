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

        force_exponent = par("force_exponent");
        deattraction_impact = par("deattraction_impact");
        uavMaxEnergyJ = par("uavMaxEnergyJ");
        chargingW = par("chargingW");
        dischargingW = par("dischargingW");
        epsilon = par("epsilon");
        rechargeTimeOffset = par("rechargeTimeOffset");

        //areaMinX = floor(par("areaMinX").doubleValue());
        //areaMaxX = floor(par("areaMaxX").doubleValue());
        //areaMinY = floor(par("areaMinY").doubleValue());
        //areaMaxY = floor(par("areaMaxY").doubleValue());

        forceUpdateTime = par("forceUpdateTime");

        if (stopTime >= SIMTIME_ZERO && stopTime < startTime)
            throw cRuntimeError("Invalid startTime/stopTime parameters");
        selfMsg = new cMessage("sendTimer");
        selfMsg_run = new cMessage("forceTimer");

        //generate the 2 forces maps
        /*uavForcesMatrix.resize(areaMaxX);
        for (auto& r : uavForcesMatrix) {
            r.resize(areaMaxY, 0);
        }
        carForcesMatrix.resize(areaMaxX);
        for (auto& r : carForcesMatrix) {
            r.resize(areaMaxY, 0);
        }*/
    }
    else if (stage == INITSTAGE_LAST) {
        int numUAV = this->getParentModule()->getParentModule()->getSubmodule("uav", 0)->getVectorSize();
        int numPedestrian = this->getParentModule()->getParentModule()->getSubmodule("pedestrian", 0)->getVectorSize();
        int numCar = this->getParentModule()->getParentModule()->getSubmodule("mobcharger", 0)->getVectorSize();

        uavMobilityModules.resize(numUAV);
        pedonsMobilityModules.resize(numPedestrian);
        carMobilityModules.resize(numCar);

        for (int u = 0; u < numUAV; u++) {
            PotentialForceMobility *uavMob = dynamic_cast<PotentialForceMobility *> (this->getParentModule()->getParentModule()->getSubmodule("uav", u)->getSubmodule("mobility"));
            uavMob->setActualEnergy( uavMaxEnergyJ );
            uavMob->setMaxEnergy( uavMaxEnergyJ );
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

        double uavMaxEnergyJ;
            double chargingW;
            double dischargingW;
    }
}

void CloudApp::finish() {

    recordScalar("lifetime", simTime());

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
            rechargeSchedule();
            updateUAVForces();
            updateMobileChargerForces();
            checkLifetime();

            //std::cerr << "handleMessageWhenUp: OK updated forces" << endl << std::flush;

            scheduleAt(simTime() + forceUpdateTime, selfMsg_run);
        }
        else {
            throw cRuntimeError("Invalid self message", msg->getFullPath());
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

Coord CloudApp::calculateAttractiveForce(Coord me, Coord target, double maxVal, double coeff, double dRef, double eps) {

    Coord ris = target - me;
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

        // check UAV to get off from the charging station
        for (auto& puav : uavMobilityModules) {
            if (puav->isChargingUav()) {
                double energyRatio = puav->getActualEnergy() / puav->getMaxEnergy();
                double worseUAV = 0;

                for (auto& puavcheck : uavMobilityModules) {
                    if ( (!puavcheck->isChargingUav()) && (puavcheck->getActualEnergy() < puav->getActualEnergy()) ) {
                        ++worseUAV;
                    }
                }

                //AOB probability
                //double prob = pow((energyRatio), 1.0 / (worseUAV + 1.0) );      // vers.1
                double prob = 1.0 - pow((1.0 - energyRatio), 1.0 / (worseUAV + 1.0) );      // vers.2

                if (dblrand() < prob) {
                    PotentialForceMobility *pcar = puav->getBuddy();

                    pcar->setChargingUav(false);
                    pcar->setBuddy(nullptr);

                    puav->setChargingUav(false);
                    puav->setBuddy(nullptr);
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

            double energyRatio = puav->getActualEnergy() / puav->getMaxEnergy();

            // get and order (uAV-distance based) all the free charging stations
            std::list< std::tuple<PotentialForceMobility *, Coord, Coord > > freeChargers;
            for (auto& pcar : carMobilityModules) {
                if (!pcar->isChargingUav()) {
                    freeChargers.push_back(std::make_tuple(pcar, pcar->getCurrentPosition(), puav->getCurrentPosition()));
                }
            }
            freeChargers.sort(CloudApp::distanceSort);

            //check if to charge
            for (auto& t_car : freeChargers) {
                PotentialForceMobility *pcar = std::get<0>(t_car);
                double dist = puav->getCurrentPosition().distance(pcar->getCurrentPosition());

                //AOB probability
                double prob = pow((1.0 - energyRatio),((dist/100.0) + 1.0));

                if (dblrand() < prob) {

                    puav->setChargingUav(true);
                    puav->setBuddy(pcar);

                    pcar->setChargingUav(true);
                    pcar->setBuddy(puav);

                    break;
                }
            }
        }

        nextRechargeTime = simTime() + rechargeTimeOffset;
    }
}

void CloudApp::updateUAVForces() {
    //EV << "Updating UAV Forces!!!" << endl;

    for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
        Coord uavForce = Coord::ZERO;

        // update the force on the flying UAVs only
        if (!uavMobilityModules[u]->isChargingUav()) {

            // add attractive force from the pedestrians
            for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
                Coord actPedForce = calculateAttractiveForce(uavMobilityModules[u]->getCurrentPosition(), pedonsMobilityModules[p]->getCurrentPosition(), wp, kp, dp, epsilon);
                double reducingFactor = calculateAttractiveForceReduction(pedonsMobilityModules[p]->getCurrentPosition(), u, deattraction_impact, kr, dr, epsilon);

                uavForce += actPedForce * reducingFactor;
            }

            //add repulsive forces from the other flying UAVs
            for (unsigned int j = 0; j < uavMobilityModules.size(); j++) {
                if ((u != j) && (!uavMobilityModules[j]->isChargingUav())) {
                    uavForce += calculateRepulsiveForce(uavMobilityModules[u]->getCurrentPosition(), uavMobilityModules[j]->getCurrentPosition(), wu, ku, du, epsilon);
                }
            }

        }
        else {
            uavForce = uavMobilityModules[u]->getBuddy()->getCurrentPosition() - uavMobilityModules[u]->getCurrentPosition();
        }

        uavMobilityModules[u]->setActiveForce(uavForce);
    }
}

void CloudApp::updateMobileChargerForces() {
    //EV << "Updating Mobile Charger Forces!!!" << endl;

    for (unsigned int c = 0; c < carMobilityModules.size(); c++) {
        Coord carForce = Coord::ZERO;

        if (!carMobilityModules[c]->isChargingUav()) {

            // add attractive force from the flying UAVs
            for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
                if (!uavMobilityModules[u]->isChargingUav()) {
                    Coord actUAVForce = calculateAttractiveForce(carMobilityModules[c]->getCurrentPosition(), uavMobilityModules[u]->getCurrentPosition(), wc, kc, dc, epsilon);
                    double reducingFactor = 1.0 - (uavMobilityModules[u]->getActualEnergy() / uavMobilityModules[u]->getMaxEnergy());

                    carForce += actUAVForce * reducingFactor;
                }
            }

            //add repulsive forces from the pedestrians
            for (unsigned int p = 0; p < pedonsMobilityModules.size(); p++) {
                carForce += calculateRepulsiveForce(carMobilityModules[c]->getCurrentPosition(), pedonsMobilityModules[p]->getCurrentPosition(), wk, kk, dk, epsilon);
            }

        }

        carMobilityModules[c]->setActiveForce(carForce);
    }
}

void CloudApp::checkLifetime() {
    for (unsigned int u = 0; u < uavMobilityModules.size(); u++) {
        if (uavMobilityModules[u]->getActualEnergy() <= 0) {
            endSimulation();
        }
    }
}

} // namespace inet

