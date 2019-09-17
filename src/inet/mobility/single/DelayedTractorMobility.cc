//
// Copyright (C) 2007 Peterpaul Klein Haneveld
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
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#include "inet/common/INETMath.h"
#include "DelayedTractorMobility.h"

namespace inet {

Define_Module(DelayedTractorMobility);

DelayedTractorMobility::DelayedTractorMobility()
{
    delay = 0;
}

void DelayedTractorMobility::initialize(int stage)
{
    TractorMobility::initialize(stage);

    EV_TRACE << "initializing DelayedTractorMobility stage " << stage << endl;
    if (stage == INITSTAGE_LOCAL) {
        delay = par("delay");
    }
}

} // namespace inet

