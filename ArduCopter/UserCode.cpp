#include "Copter.h"
#include "GCS_Mavlink.h"

/*
    case MAVLINK_MSG_ID_MULTIUAV_AUTOMATA_STATES:
    {
        mavlink_multiuav_automata_states_t packet;
        mavlink_multiuav_automata_states_decode(msg, &packet);
        break;
    }

    case MAVLINK_MSG_ID_MULTIUAV_TASKEND:
    {
        mavlink_multiuav_taskend_t packet;
        mavlink_multiuav_taskend_decode(msg, &packet);
        break;
    }
 */


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "REDDY - 10 HZ");
    /*
     *
	mavlink_msg_multiuav_automata_states_send(
					chan,
					1,1,1);
     */
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
