# contraspect

Contraspect Drone Location and Navigation System
Budapest University of Technology and Economics
Department of Telecommunications and Media Informatics
Author: Áron Hajdu-Moharos
Consultant: Dr. Gábor Fehér

Upload all to Github now.
Show me the added status decreasing in Loc_sim
Show me the delays stored in Dn
Show me Dn_calc, Loc_sim_calc
Now if need be cls for Topic_spy_*_calc
First complete demo mode
Dn and Bcn clks discrepancy. Dn clk slower than Bcns clk
Dn_node non Demo mode move from Dn_ctrl
Rewrite Loc_sim Dn_node and Bcn_node info.txt(x2): omit init order
Rewrite all nodes to reflect services, new variables, etc
Create overview diagram with full explanation of function

Edit in graph: Loc_sim no sub Bcn_init_pos
Non-fatal error: Dcs-A at node shutdown: "segmentation fault (core dumped)"

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
Dn_status_map	    Status_map.msg	float32 sx  float32 sy  float32 sz	contraspect_msgs
		    			float32 dx  float32 dy  float32 dz
		    			float32 b1x float32 b1y float32 b1z
		    			float32 b2x float32 b2y float32 b2z
		    			float32 b3x float32 b3y float32 b3z
		    			float32 b4x float32 b4y float32 b4z
SERVICE TYPES
Bcn_init_pos	Bcn_pos.msg	float32 x, y, z, uint8 bid || uint8 bid	     contraspect_msgs
Clk_sync     	Clk_sync.msg  	uint8 all_syncd || uint8 bid		     contraspect_msgs
Dn_ctrl	     	Dn_ctrl.sr	float64 x  float64 y  float64 z ||	     contraspect_msgs

srv_Clk_sync:
 request: uint8 all_syncd		client-DCS-B		
 response: n/a	 			server-Beacon-Dn	
srv_Dn_ctrl:
 request: Point move			client-DCS-C		
 response: n/a	 			server-Loc_sim-Dn	
srv_Bcn_init_pos:
 request: float32 x, y, z, uint8 bid	client-Beacon(-Dn)
 response: n/a	      	    		server-Dn(-Beacon)		
