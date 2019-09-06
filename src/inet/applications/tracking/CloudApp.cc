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

        if (stopTime >= SIMTIME_ZERO && stopTime < startTime)
            throw cRuntimeError("Invalid startTime/stopTime parameters");
        selfMsg = new cMessage("sendTimer");
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


}


void CloudApp::processStop() {
    cancelEvent(selfMsg_run);
}

void CloudApp::handleMessageWhenUp(cMessage *msg) {
    if (msg->isSelfMessage()) {
        ASSERT(msg == selfMsg);
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

} // namespace inet

