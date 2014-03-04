// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if (HIL_MODE != HIL_MODE_ATTITUDE) && (CONFIG_CAS == ENABLED)

static ModeFilterInt16_Size3 sonar_mode_filter_cas(1);
static AP_RangeFinder_MaxsonarXL *sonar_cas;
static bool gCAS_active=false;

void
setup_CAS(){
	sonar_cas = new AP_RangeFinder_MaxsonarXL(hal.analogin->channel(1),&sonar_mode_filter_cas);
}
void 
init_CAS(){
    //  init CAS
    sonar_cas->calculate_scaler(AP_RANGEFINDER_MAXSONARXLL, 5.0f);
}

int16_t
read_CAS(){
    return sonar_cas->read();
}
// calculate modified roll/pitch depending upon optical flow calculated position
void
get_of_cas_roll(){
	static int16_t cas_reading;
	cas_reading=read_CAS();
	if(cas_reading<50){
		tot_x_cm+=(cas_reading-50)*0.5;
		gCAS_active=true;
	}
	else{
		gCAS_active=false;
	}
}
void
get_of_cas_pitch(){
	static int16_t cas_reading;
	cas_reading=read_CAS();
	if(cas_reading<50){
		tot_y_cm+=(cas_reading-50)*0.5;
		gCAS_active=true;
	}
	else{
		gCAS_active=false;
	}
}

#endif
