//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package potentialtracking.simulations.potential_tracking;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.contract.IIntegratedVisualizer;

import inet.node.nodes.*;

//
// TODO auto-generated type
//
network TrackingScenario
{    
    parameters:
        int numUAVs = default(1);
        int numChargingCars = default(1);
        int numPedestrians = default(1);
        
        bool hasVisualizer = default(false);
        
    submodules:
        visualizer: <default("IntegratedCanvasVisualizer")> like IIntegratedVisualizer if hasVisualizer {
            parameters:
                @display("p=100,50");
        }    
        configurator: Ipv4NetworkConfigurator {
            parameters:
                @display("p=100,150");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            parameters:
                @display("p=100,250");
        }  
        
        cloud: Cloud {
            parameters:
                @display("p=100,350");
        }    
        
        pedestrian[numPedestrians]: PedestrianHost {
            parameters:
                @display("p=300,250");
        } 
        
        mobcharger[numChargingCars]: MobileChargerHost {
            parameters:
                @display("p=300,150");
        }   
        
        uav[numUAVs]: UAVHost {
            parameters:
                @display("p=300,50");
        }   
    
}
