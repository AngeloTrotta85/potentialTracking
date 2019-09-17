//
// Copyright (C) 2004 Andras Varga
// Copyright (C) 2000 Institut fuer Telematik, Universitaet Karlsruhe
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

package inet.applications.tracking;

import inet.applications.contract.IApp;

//
// Sends UDP packets to the given IP address at the given interval.
// Compatible with both ~Ipv4 and ~Ipv6.
//
// The sending interval can be a constant or a random value (e.g. exponential(1)).
// If the destAddresses parameter contains more than one address, one
// of them is randomly chosen for each packet. An address may be given in the
// dotted decimal notation, or with the module name. (The L3AddressResolver
// class is used to resolve the address.) To disable the model, set
// destAddresses to "".
//
// Received packets are discarded.
//
// The peer can be a ~UdpSink, another ~UdpBasicApp (it handles received packets
// like ~UdpSink), or a ~UdpEchoApp. When used with ~UdpEchoApp, the rcvdPkLifetime
// statistic will contain the round-trip times.
//
simple CloudApp like IApp
{
    parameters:        
        double startTime @unit(s) = default(0s); // application start time 
        double stopTime @unit(s) = default(-1s);  // time of finishing sending, -1s means forever
        
        double forceUpdateTime @unit(s) = default(0.5s); // force update interval 
        
        double epsilon = default(0.1); // epsilon in the slope calculation
        
        
        double wp = default(1);	// force uav-pedestrian max val
        double kp = default(0.001);	// force uav-pedestrian coefficient
        double dp = default(-1); // force uav-pedestrian distance to epsilon
        
        double wu = default(2);	// force uav-UAVs max val
        double ku = default(0.01);	// force uav-UAVs coefficient
        double du = default(-1); // force uav-UAVs distance to epsilon
                
        double kr = default(0.001);	// force recording coefficient
        double dr = default(-1); // force recording distance to epsilon
        
        
        double wc = default(2);	// force cars-uav max val
        double kc = default(0.01);	// force cars-uav coefficient
        double dc = default(-1); // force cars-uav distance to epsilon
        
        double wk = default(2);	// force cars-pedestrians max val
        double kk = default(0.01);	// force cars-pedestrians coefficient
        double dk = default(-1); // force cars-pedestrian distance to epsilon
        
        double wg = default(3);	// force UAV-crowd max val
        double kg = default(0.01);	// force UAV-crowd coefficient
        double dg = default(-1); // force UAV-crowd distance to epsilon
        
        double coverageDistance @unit(m) = default(20 m); 
        
        double force_exponent = default(4);	// force exponent
        
        double deattraction_impact = default(1.0); // deattraction factor when a pedestrian is being covered
        
        double uavMaxEnergyJ = default(130000); // maximum UAV energy
        double uavInitEnergyJ = default(uavMaxEnergyJ); // initial UAV energy
        double chargingW = default(15); // charging power
        double dischargingW = default(85); //  discharging power
        
        double rechargeTimeOffset @unit(s) = default(30 s);
        
        double statisticOffset @unit(s) = default(10 s);
        
        double areaMinX @unit(m) = default(0 m); // min x position of the constraint area
        double areaMinY @unit(m) = default(0 m); // min y position of the constraint area
        double areaMaxX @unit(m) = default(0 m); // max x position of the constraint area
        double areaMaxY @unit(m) = default(0 m); // max y position of the constraint area
        
        bool keepFullMap = default(false);
        double forceMapOffset @unit(m) = default(10 m);
        double forceDrawCoefficient = default(1);
        string fileData = default("forces_%s.dat");
        
        string chargingType @enum("mobileCar","fixedStation","noCharger") = default("mobileCar");
        string chargingScheduling @enum("stimrespSched","greedySched") = default("stimrespSched");
        
        string crowdFollow @enum("noCrowd","knownCrowd","clusterCrowd") = default("noCrowd");
        int numClusterCrowd = default(1);
        
        @display("i=block/app");
        @lifecycleSupport;        
    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}

