/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include "D:/px4soft/px4/Firmware/src/drivers/sonar_group_service/sonar_group_topic.h"

#include <uORB/uORB.h>                      //
#include <uORB/topics/sensor_combined.h>    //Defination of sensor_combined_s
#include <uORB/topics/vehicle_attitude.h>   //Defination of vehicle_attitude_s

//#include <uORB/topics/distance_around.h>    //by yly:Defination of distance measured by Sonar groups


__EXPORT int px4_simple_main(int argc, char *argv[]);

int px4_simple_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    //ORB_DECLARE(distance_around);
    /* subscribe to sensor_combined topic */
    int sonardata_sub_fd = orb_subscribe(ORB_ID(SonarGroupDistance));
    orb_set_interval(sonardata_sub_fd, 1000);

    /* advertise attitude topic */
    struct SonarGroupDistance_s sonardata;

    memset(&sonardata, 0, sizeof(sonardata));

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = sonardata_sub_fd,   .events = POLLIN },
        //      { .fd = distancearound_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
                                 * { .fd = other_sub_fd,   .events = POLLIN },
                                 */
    };

    int error_counter = 0;

    for (int i = 0; i < 5; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("[px4_simple_app] Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("[px4_simple_app] ERROR return value from poll(): %d"
                        , poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct SonarGroupDistance_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(SonarGroupDistance), sonardata_sub_fd, &raw);

                printf("--------------------------------\n");

                for (int j=1;j<=5;j++)
                {

                    printf("[SONAR_GROUP]Range(%d)=%d(cm)\n",j,raw.sonar_group_distance[j-1]);
                    printf("[SONAR_GROUP]status(%d)=%d\n",j,raw.sonar_group_status[j-1]);


                }
                printf("--------------------------------\n");



            }

        }


    }

    PX4_INFO("exiting");

    return 0;
}

