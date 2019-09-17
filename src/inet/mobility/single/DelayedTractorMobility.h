/* -*- mode:c++ -*- ********************************************************/
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

#ifndef __INET_DELAYEDTRACTORMOBILITY_H
#define __INET_DELAYEDTRACTORMOBILITY_H

#include "inet/common/INETDefs.h"
#include "inet/mobility/single/TractorMobility.h"

namespace inet {

/**
 * @brief Tractor movement model. See NED file for more info.
 *
 * NOTE: Does not yet support 3-dimensional movement.
 * @ingroup mobility
 * @author Peterpaul Klein Haneveld
 */
class INET_API DelayedTractorMobility : public TractorMobility
{
  protected:
    double delay;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int) override;


  public:
    DelayedTractorMobility();
    virtual double getMaxSpeed() const override { return speed; }
};

} // namespace inet

#endif // ifndef __INET_DELAYEDTRACTORMOBILITY_H

