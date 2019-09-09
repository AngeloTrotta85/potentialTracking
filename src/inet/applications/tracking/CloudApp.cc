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

        forceUpdateTime = par("forceUpdateTime");

        if (stopTime >= SIMTIME_ZERO && stopTime < startTime)
            throw cRuntimeError("Invalid startTime/stopTime parameters");
        selfMsg = new cMessage("sendTimer");
        selfMsg_run = new cMessage("forceTimer");
    }
}

void CloudApp::finish() {
    //recordScalar("packets sent", numSent);
    //recordScalar("packets received", numReceived);
    ApplicationBase::finish();
}

void CloudApp::processStart() {
    if (stopTime >= SIMTIME_ZERO) {
        selfMsg->setKind(STOP);
        scheduleAt(stopTime, selfMsg);
    }

    scheduleAt(simTime(), selfMsg_run);
}


void CloudApp::processStop() {
    cancelEvent(selfMsg_run);
}

void CloudApp::handleMessageWhenUp(cMessage *msg) {
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
            updateUAVForces();
            updateMobileChargerForces();
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


void CloudApp::updateUAVForces() {
    EV << "Updating UAV Forces!!!" << endl;
}

void CloudApp::updateMobileChargerForces() {
    EV << "Updating Mobile Charger Forces!!!" << endl;
}



} // namespace inet

