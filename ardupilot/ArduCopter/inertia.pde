/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
    static uint8_t log_counter_inav = 0;
    
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    if( motors.armed() && (g.log_bitmask & MASK_LOG_INAV) ) {
        log_counter_inav++;
        if( log_counter_inav >= 10 ) {
            log_counter_inav = 0;
            Log_Write_INAV();
        }
    }
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
static void read_inertial_altitude()
{
    // with inertial nav we can update the altitude and climb rate at initially 50hzs, but now at 100hz
    static uint32_t ins_of_dt;
    ins_of_dt = millis()-of_alt_last_update;    
    //if((ins_of_dt)<30){                      //check that there is a of_update at atleast with in 30ms.
    current_loc.alt = (avg_of_alt*100);//inertial_nav.get_altitude();
    //}
    //else {                                  //if there is no of_update at atleast with in 30ms use baro value
    //current_loc.alt = read_barometer();
    //}
    climb_rate = inertial_nav.get_velocity_z();
    //hal.console->printf(("\nclimbrate %d"), climb_rate);
}
