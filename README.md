# contraspect

Contraspect Drone Location and Navigation System
Budapest University of Technology and Economics
Department of Telecommunications and Media Informatics
Author: Áron Hajdu-Moharos
Consultant: Dr. Gábor Fehér

# EXECUTE BEFORE USE:
roscd contraspect
mv ./contraspect* ..

# FINETUNING:
Adjust values in <contraspect/include/contraspect/contraspect.h>

/* Jegyzokonyvbe */
BASE DELAYS
Max allowed base delay in sec: 0.5*res/speedofsound
res being the desired trilateration resolution in m
0.5 because we send it twice: once to Loc_sim, then again to Drone
Our desired resolution: 0.1 m -> Max allowed base delay: 0.5*0.1/343 = 0.000146 s
Check base delay of Dn-clk compared Bcns: set DCS-B keep at 0 -> run all -> stop DCS-B -> check
Dn-clk 1ms/s ahead of beacons
Current CLK shift 1ms/s -> Adjusting Dn clk:
from this min CLK-sync freq to attain max delay: 0.001/0.000146s = 6.85Hz
Reduce CLK shift to lower min CLK-sync freq to 10-100Hz - no need, even 10Hz already low enough.
At CLK_sync_freq=Dn_calc_freq=10Hz, minres=0.1m, avg base delay: stays at a value less than 0.000125 for first 1224 triang recvd, then climbs steadily higher to 0.0002, then drops back down. (base delay:= signal delay of Triang to Dn clk in non-demo mode - direct Triang channel from Bcns to Dn - with clks synchronized via DCS-B)
Let us increase clk_sync_freq. At CLK_sync_freq=50Hz, Dn_calc_freq=10Hz, minres=0.1m, avg base delay: no significant difference, avg base delay between 0.0002 and 0.0003 sec. This is twice our desired maxdelay!
So there is a base delay at the order of magnitude of 100us. We want a base delay which is always less than 150us.

Measuring avg delay.
Avg Adjust after 1000 adjusts (measured by DCS-B in non-demo mode)
Clk_sync_freq[Hz],avg_adjust[sec],stdev_adjust[sec],adjust_count:
0.01		 ,1.359201	 ,0.521269	   ,10
0.01		 ,1.455777	 ,0.435538	   ,20
0.01		 ,1.394343	 ,0.284707	   ,50
0.01		 ,2.133556	 ,1.086274	   ,100
0.01		 ,2.215303	 ,1.073248	   ,250

Conclusion: changing clk_sync_freq may not actually work as intended?

DRONE POS CALC
In demo mode at Drone: [BcnDelays(received-data),DronePos(calculated-data)]T*[mean,variance]
Explain trilateration theory, implementation, and "fatal error"

What works:
Dcs_A, Dcs_B, Dcs_C, Bcn_init_pos, Triang, Triang_demo, Loc_sim_calc, Loc_sim-Triang-fwd, Dn_status_map, Dn_calc

What doesn't work:
Triang added delay
Beacon and Drone CLKs sync together

* Check dn-bcns clk-sync quality from dcs-b adjustment data as a function of clk_sync-freq /* to jegyzokonyv */
* Connect dronePosCalc,Dn_status_map frequency, change to use average delay over past x triang_recvd. Have calc,Status_map frequency and x both be function of speed/accuracy setting variable -> Heisenberg uncertainty principle!! /* to jegyzokonyv */
* Then see how we can correct the base delay.
* Try shifting pubrate and subrate.
* Check Demo Dn_calc -> DCS-B -> DCS-C
* Lessening ros master load by ceasing all high frequency publishing except Triang.
* Changing speedofsound.
* Check accuracy of Loc_sim added delays
* At 10Hz, Compare Loc_sim added Triang delay and Dn measured Triang delay
* dronePosCalc: compare map, w arrays size to N. no more, no less.
* If Dn_calc AND Loc_sim delay -> Dn delay fully reliable: dronePosCalc assign map[0] values
* Make Dn_node execute dronePosCalc, Loc_sim execute delaysCalc at 10Hz -> Dn_status_map at 20Hz
* Loc_sim alters delays to Dcs_C but all uniformly. Fix.
* Now if need be cls for Topic_spy_*_calc
* First complete demo mode
* Dn_node non Demo mode move from Dn_ctrl
* Rewrite Loc_sim Dn_node and Bcn_node info.txt(x2): omit init order
* Rewrite all nodes to reflect services, new variables, etc
* Create overview diagram with full explanation of function
* Instructions: at least 3 NON-COPLANAR (and non-colinear) homing beacons
* Edit in graph: Loc_sim no sub Bcn_init_pos

SZD
UWB AdA/AoD módszer
Vik Vizsgaszabályzat záróvizsgára bocsájtás feltételei, szakdolgozat követelmények
VIK HK

git commit -m "contraspect-20th-commit. Tested-and-working. Init-demo-added-start-Loc_sim. Added-diagnostic-tools-Dn_node-base-delay-DCS-B-Adjust-Loc_sim-delay-error. Dn_status_map-Changed-msg-to-srv. DCS-A-changed-publish-speed. DCS-B-and-Dn-base_delay-added-standard-deviation-measure-diagnostic. Cleaned-up-period-constants. dronePosCalc-change-now-calculate-position-from-avg-of-last-X-delays."

# Timeline
May 2023 - Project begun.

# DEMO step-by-step
1. Running the system in normal (not demo) mode
- To demonstrate node and topic architecture via rqt graph
1.1 roscore
1.2 rosrun contraspect Dcs_node A
1.3 rosrun rviz rviz
D.4 rosrun contraspect_demo Loc_sim
1.4 rosrun contraspect Dn_node	     1.0 1.0 1.0
1.5 rosrun contraspect Beacon_node 1 0.0 0.0 0.0 
    rosrun contraspect Beacon_node 2 0.0 2.0 0.0 
    rosrun contraspect Beacon_node 3 2.0 2.0 0.0 
    rosrun contraspect Beacon_node 4 2.0 0.0 0.0
1.6 rosrun rqt_graph   rqt_graph
- Show all the nodes working properly and interacting
- Explain the function of each node and topic
1.7 rosrun contraspect Contraspect_info Dn_node
    rosrun contraspect Contraspect_info Beacon_node
    rosrun contraspect Contraspect_info Dcs_node
    close last 3
- Explain how the drone would calculate its position from the beacon delays
1.8 rosrun contraspect Dcs_node B
1.9 rosrun contraspect_demo Topic_spy Dn_calc
- Explain the issue with the time delay and how to fix
2. Running the system in demo mode
- To demonstrate full and proper functioning of the system
2.1 shut down Dn_node and Beacon_nodes
2.2 rosrun contraspect 	    Dn_node	  1.0 1.0 1.0 demo
2.3 rosrun contraspect_demo Loc_sim
2.3 rosrun contraspect 	    Beacon_node 1 0.0 0.0 0.0 
    rosrun contraspect 	    Beacon_node 2 0.0 2.0 0.0 
    rosrun contraspect 	    Beacon_node 3 2.0 2.0 0.0 
    rosrun contraspect 	    Beacon_node 4 2.0 0.0 0.0
2.4 rosrun contraspect	    Dcs_node B
- Show RQT Graph and explain
- Show all the nodes working properly and interacting
- Explain the function of each node and topic
2.5 rosrun contraspect_demo Contraspect_demo_info Loc_sim
    rosrun contraspect_demo Contraspect_demo_info Topic_spy
    close last 2
2.6 rosrun contraspect_demo Topic_spy Loc_sim_calc

# PACKAGES
contraspect
contraspect_demo
contraspect_msgs

# NODES
Node			Package
Dn_node			contraspect
DCS_node		contraspect
Beacon_node		contraspect
Contraspect_info	contraspect
Topic_spy		contraspect_demo
Loc_sim			contraspect_demo
Contraspect_demo_info	contraspect_demo

# MESSAGES
Topic		    Msg_type		Msg_contents				Package
Dn_calc		    String		string data				std_msgs
Loc_sim_calc	    String		string data				std_msgs
CLK		    CLK.msg		float32 clk uint8 bid	  		contraspect_msgs
Triang		    Triang.msg		float32 timestamp uint8 bid		contraspect_msgs
Triang_demo	    Triang.msg		float32 timestamp uint8 bid		contraspect_msgs

SERVICE TYPES
Dn_status_map	Status_map.srv	||					     contraspect_msgs
				float32 sx  float32 sy  float32 sz
		    		float32 dx  float32 dy  float32 dz
		    		float32 b1x float32 b1y float32 b1z
		    		float32 b2x float32 b2y float32 b2z
		    		float32 b3x float32 b3y float32 b3z
		    		float32 b4x float32 b4y float32 b4z
Bcn_init_pos	Bcn_pos.srv	float32 x y z uint8 bid || uint8 bid	     contraspect_msgs
Clk_sync     	Clk_sync.srv  	float64 clk || float32 adjust	     	     contraspect_msgs
Dn_ctrl	     	Dn_ctrl.srv	float64 x  float64 y  float64 z ||	     contraspect_msgs

srv_Clk_sync:
 request: uint8 all_syncd		client-DCS-B		
 response: n/a	 			server-Beacon-Dn	
srv_Dn_ctrl:
 request: Point move			client-DCS-C		
 response: n/a	 			server-Loc_sim-Dn	
srv_Bcn_init_pos:
 request: float32 x, y, z, uint8 bid	client-Beacon(-Dn)
 response: n/a	      	    		server-Dn(-Beacon)		
srv_Status_map:
 request: n/a				client-DCS-A-Loc_sim
 response: float32 [s,d,b1,b2,b3]*xyz	server-Dn