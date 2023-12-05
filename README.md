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

- Export DCS-B adjust data, matlab visualize. Vary CLK_SYNC_FREQ, optimize. Check true CLK_sync_freqas function of clk_sync_freq variable /* to jegyzokonyv */
- Delays_vector_size a function of new .h "pos_calc_precision" var and period_dn_pos_calc. period_dn_pos_calc in turn a function of new .h "pos_calc_speed" var. performance (base delay) stats as function of precision and speed, matlab, find optimal /* to jegyzokonyv */
- Further ideas to improve base delay: shift pubrate,subrate, find and kill high freq publishing, instead of base delay target Loc_sim sent vs Dn_node recvd delay discrepancy, finally if no other option, decrease speedofsound
- Demo test: Dn_calc -> DCS-B -> DCS-C
- Fix accuracy of Loc_sim added delays
- If Dn_calc AND Loc_sim->Dn delay fully reliable: dronePosCalc assign map[0] values
- Loc_sim alters delays to Dcs_C but all uniformly. Fix.
- Big Dn_calc issue: calculated position doesn't really correspond to distances!! Eg. distances 8,9,8,8, position z value: 100?? Do control reverse calculation to test Dn_calc validity. If not valid fix error.
- To readme: how install curses.h, Eigen/Dense packages
- After demo mode complete: Dn_node non Demo mode move from Dn_ctrl
- Edit in graph: Loc_sim no sub Bcn_init_pos
- OPTNL: Rewrite all txt files
- OPTNL: Create overview diagram with full explanation

SZD
UWB AdA/AoD módszer
Vik Vizsgaszabályzat záróvizsgára bocsájtás feltételei, szakdolgozat követelmények
VIK HK

git commit -m "contraspect-21st-commit. Decreased-Speedofsound-to-34.3-m/s. Loc-sim-calc-more-frequently. Beacon-delays-stored-as-average-of-last-n-measured-delay-values. Tested-and-working. Accurately-calculates-Dn-position-but-some-error-and-constantly-increasing-dist-from-beacons. Need-fix."

# Timeline
May 2023 - Project begin
Jul 2023 - Concept complete
Aug 2023 - Code scaffold
Sep 2023 - Running

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
