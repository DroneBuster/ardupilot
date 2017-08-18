#include "Plane.h"


/* 
   call parachute library update
*/
void Plane::parachute_check()
{
#if PARACHUTE == ENABLED
    if(parachute.alt_min() > relative_ground_altitude(false) && parachute_fs_state.parachute_fs_en)
    {
         parachute_release();
    }
    parachute.update();
#endif
}

#if PARACHUTE == ENABLED

/*
  parachute_release - trigger the release of the parachute
*/
void Plane::parachute_release()
{
    if (parachute.release_in_progress()) {
        return;
    }
    // send message to gcs and dataflash
    if (parachute.released()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released again");
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");
    }

    // release parachute
    parachute.release();
}

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
bool Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return false;
    }

    // if we get this far release parachute
    parachute_release();

    return true;
}

#endif
