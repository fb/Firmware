
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hello_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "hello_example.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>

#include <poll.h> // POLLIN
#include <matrix/math.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>

px4::AppState HelloExample::appState;

int HelloExample::main()
{
	appState.setRunning(true);

    struct vehicle_attitude_s attitude;

	/* subscribe to parameter changes */
	int sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(sub_fd, 200);

    struct pollfd fds[] = {
        { .fd = sub_fd, .events = POLLIN }
    };

	while (!appState.exitRequested()) {
		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/* poll error, ignore */

		} else if (ret == 0) {
			/* no return value, ignore */
			// warnx("no sensor data");

		} else {
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude), sub_fd, &attitude);

                matrix::Eulerf euler = matrix::Quatf(attitude.q);

				// write out on accel 0, but collect for all other sensors as they have updates
				printf("%llu,%1.1f,%1.1f,%1.1f\n", attitude.timestamp, (double)euler.phi(), (double)euler.psi(), (double)euler.theta());
			}
		}
	}

	return 0;
}
