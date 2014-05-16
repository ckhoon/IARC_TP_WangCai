/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    hal.uartB->begin(115200,256,32); // set tx space to 32 since it's is used as a condition used in AP_HAL_AVR/uartdrivers.cpp to avoid the controller from dropping OF_msgs due to wrong decode
    raw_flow_read.ground_distance = 0.30; 
    last_of_comp_y = 0.0;
    last_of_comp_x = 0.0;
    last_of_alt = 0.30;
    
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    uint32_t of_dt = 0;
    mavlink_channel_t of_ch;
    mavlink_message_t of_msg;
    mavlink_status_t of_stat;
    uint16_t nbytes, c;
    mavlink_comm_1_port = hal.uartB;
    of_ch = MAVLINK_COMM_1;
    nbytes = comm_get_available(of_ch);
    static float of_dz, of_tot_x = 0, of_tot_y = 0;
    static int spikeCount=0;
    
    for (uint16_t i=0; i<nbytes; i++){
      c = comm_receive_ch(of_ch);  
      if ((mavlink_parse_char(MAVLINK_COMM_1, c, &of_msg, &of_stat))){
        switch(of_msg.msgid){
          case MAVLINK_MSG_ID_OPTICAL_FLOW:
              mavlink_msg_optical_flow_decode(&of_msg, &raw_flow_read);
              raw_of_last_update = millis();
              of_dt = raw_of_last_update-last_raw_of_update;
              //hal.console->printf(("\n of_dt %lu"), raw_of_last_update);
              //averaging filtering for flow
              raw_of_y_cm = (((raw_flow_read.flow_comp_m_x + last_of_comp_x)/2)*of_dt)/10;
              raw_of_x_cm = (((raw_flow_read.flow_comp_m_y + last_of_comp_y)/2)*of_dt)/10;
              of_time_interval++;
              //memory for previous value
              last_raw_of_update = raw_of_last_update;
              last_of_comp_y = raw_flow_read.flow_comp_m_y;
              last_of_comp_x = raw_flow_read.flow_comp_m_x;
              

              //Lalith version start
              /*
              // filtering for altitude
              of_dz = last_of_alt - raw_flow_read.ground_distance;
              if((((of_dz*100)>0)&&((of_dz*100)<30.0))||(((of_dz*100)<0)&&((of_dz*-100)<30.0))||(of_dz==0)){
              avg_of_alt = (raw_flow_read.ground_distance + last_of_alt)/2;
              //of_alt_climbrate = (of_dz*100)*(of_dt*0.001f);
              of_alt_last_update = millis();
              //hal.console->printf((":: alt_dt %lu"), of_alt_last_update);
              last_of_alt = raw_flow_read.ground_distance;
              }
              //Lalith version end
              */

              // yunfa version start

              of_dz = (last_of_alt - raw_flow_read.ground_distance)*100.0;
              if(of_dz<150.0 && of_dz>-150.0){
            	  avg_of_alt = (raw_flow_read.ground_distance + last_of_alt)/2;
            	  last_of_alt = raw_flow_read.ground_distance;
            	  //spikeCount=0;
                  of_alt_last_update = millis();
              }
              //else if(++spikeCount>20){
            	  //spikeCount=0;
            	  //avg_of_alt=last_of_alt=raw_flow_read.ground_distance;
                //  of_alt_last_update = millis();
              //}

              // yunfa version end
              
              i=256;
              break;
          default:
              //hal.console->printf(("\nmsg_id: %d"), of_msg.msgid);
              break;
        }
      }
    }
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
#define USER_TAKEOFF 0
#define USER_HOVER 1
#define USER_MOVE1 2
#define USER_MOVE2 3
#define USER_HOVER2 4
#define USER_HOVER3 5
#define USER_LAND 6
#define USER_RADIO_LOW	1000
#define USER_RADIO_HIGH	2000

    // put your 10Hz code here
	static uint16_t state = USER_TAKEOFF;
	static uint16_t nHoverCounter = 0;
	static uint16_t nMoveCounter = 0;
	static uint16_t nMoveDistance = 0;
	static int16_t nOfTargetX = 0;

	if (g.rc_6.radio_in < USER_RADIO_LOW) {
    	//hal.console->printf(("\n radio 6 LOW-> %d"), g.rc_6.radio_in);
		state = USER_TAKEOFF;
    	nHoverCounter = 0;
    	nMoveCounter = 0;
    	nMoveDistance = 0;
    	//nOfTargetX = 0;
    	b_ofSetpt = false;
		forceTakeOff = false;
    }else if (g.rc_6.radio_in > USER_RADIO_HIGH) {
    	//hal.console->printf(("\n radio 6 HIGH-> %d"), g.rc_6.radio_in);
    	switch(state){
    		case USER_TAKEOFF :
					if (current_loc.alt < 100) {
						forceTakeOff = true;
						controller_desired_alt += 1;
						if (controller_desired_alt < 60)
							controller_desired_alt = 60;
					}
					else {
						state = USER_HOVER;
						forceTakeOff = false;
    					nHoverCounter = 0;
					}
    				break;
    		case USER_HOVER :
    				if (nHoverCounter++ > 50){
    					nHoverCounter = 0;
						state = USER_MOVE1;
    					nMoveCounter = 0;
    				}
    				break;
    		case USER_MOVE1 :
    				if (nMoveCounter == 0){
    					of_xSetPt_cm = of_xTravelled_cm;
    					b_ofSetpt = true;
    		            g.pid_optflow_roll.reset_I();
    					nMoveCounter++;
    				}
    				else if (nMoveCounter < 100){
    					nMoveCounter++;
    					of_xSetPt_cm-=1;
    				}
    				else{
    					of_xSetPt_cm = of_xTravelled_cm;
    					nMoveCounter = 0;
    					b_ofSetpt = false;
						state = USER_HOVER2;
	   					nHoverCounter = 0;
    				}
     				break;
    		case USER_HOVER2 :
    				if (nHoverCounter++ > 100){
    					nHoverCounter = 0;
						state = USER_MOVE2;
			            g.pid_optflow_roll.reset_I();
	   					nMoveCounter = 0;
    				}
    				break;
    		case USER_MOVE2 :
					if (nMoveCounter == 0){
						of_xSetPt_cm = of_xTravelled_cm;
						b_ofSetpt = true;
						nMoveCounter++;
					}
					else if (nMoveCounter < 100){
						nMoveCounter++;
						of_xSetPt_cm+=1;
					}
					else{
						of_xSetPt_cm = of_xTravelled_cm;
						nMoveCounter = 0;
						b_ofSetpt = false;
						state = USER_HOVER3;
						nHoverCounter = 0;
					}
					break;
    		case USER_HOVER3 :
    				if (nHoverCounter++ > 100){
    					nHoverCounter = 0;
						state = USER_LAND;
    				}
    				break;
    		case USER_LAND :
					if (current_loc.alt > 30) {
						controller_desired_alt -= 1;
					}
					if (controller_desired_alt <= 30) {
						set_land_complete(true);
			            set_throttle_out(0, false);
			            throttle_accel_deactivate();
					}
    				break;
    	}
    }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
