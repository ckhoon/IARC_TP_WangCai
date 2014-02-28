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
    // put your 10Hz code here
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
	static bool bTakeoff = true;
	static uint16_t nHoverCounter = 0;
    // put your 1Hz code here
    if (g.rc_6.radio_in < 1000) {
    	//hal.console->printf(("\n radio 6 LOW-> %d"), g.rc_6.radio_in);
    	bTakeoff = true;
    	nHoverCounter = 0;
    }else if (g.rc_6.radio_in > 2000) {
    	//hal.console->printf(("\n radio 6 HIGH-> %d"), g.rc_6.radio_in);
    	if (bTakeoff) {
    		if (current_loc.alt < 100) {
    			controller_desired_alt += 5;
    			if (controller_desired_alt < 60)
    				controller_desired_alt = 60;
    		}
    		else
    			bTakeoff = false;
    	}else{
    		if (nHoverCounter++ > 20){
    			if (current_loc.alt > 20)
    				controller_desired_alt -= 5;
    		}
    	}
    }
}
#endif
