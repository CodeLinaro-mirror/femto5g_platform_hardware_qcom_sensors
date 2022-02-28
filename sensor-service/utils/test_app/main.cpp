/*
Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
 
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
 
    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sensor_util.h>
#include <stdint.h>

static const char *options = "a:g:b:fn:d:s:le:o:Nhv?";

static const struct option long_options[] = {
		{"yaw",	             required_argument, 0,  'a' },
		{"pitch",            required_argument, 0,  'b' },
		{"roll",             required_argument, 0,  'c' },
		{"x",                required_argument, 0,  'd' },
	        {"y", 	             required_argument, 0,  'e' },
		{"z",                required_argument, 0,  'f' },
		{"towingthreshold",  required_argument, 0,  'g' },
		{"towingtime",       required_argument, 0,  'z' },
		{"crashthreshold",   required_argument, 0,  'i' },
		{"crashtime",        required_argument, 0,  'j' },
		{"ignition",         required_argument, 0,  'k' },
		{"help",              no_argument, 	0,  '?' },
		{0,                  0,                 0,   0  }
};

static void help(char *argv)
{
	printf("To Update sensor position\n");
	printf("\tsensor_util_lib_testapp --x [-65--+65] --y [-65--+65] --z [-65--+65]\n");

	printf("To Update Sensor Euler angles\n");
	printf("\tsensor_util_lib_testapp --yaw [0--3600] --pitch [0--3600] --roll [0--3600]\n");

	printf("To Update towing threshold and time\n");
	printf("\tsensor_util_lib_testapp --towingthreshold [10--1000] --towingtime [2--89000]\n");

	printf("To Update crash threshold and time\n");
	printf("\tsensor_util_lib_testapp --crashthreshold [100--2000] --crashtime [2--89000]\n");

	printf("To Update ignition state\n");
	printf("\tsensor_util_lib_testapp --ignition [0--1]\n");

	exit(0);
}

int main(int argc, char **argv)
{
	int ret;
	int c;
	int digit_optind = 0;
	int rm_value = 0;
	int sp_value = 0;
	int towing = 0;
	int crash = 0;
	int ignition = 0;
	int i;
	int err = 0;
	char *yawd = NULL, *pitchd = NULL, *rolld = NULL;
	char *xd = NULL, *yd = NULL, *zd = NULL;
	char *ttd = NULL, *ttimed = NULL, *ccd = NULL, *ctimed = NULL;
	char *ignd = NULL;
	uint16_t yaw = 0, pitch = 0, roll = 0; 
	uint16_t towing_threshold = 0, crash_threshold = 0;
	uint32_t towing_time = 0, crash_time = 0;
	int16_t x = 0, y = 0, z = 0;
	uint32_t ign_state;

	while (1) {
		int this_option_optind = optind ? optind : 1;
		int option_index = 0;

		c = getopt_long(argc, argv, options,
				long_options, &option_index);
		if (c == -1){
			break;
		}

		switch (c) {
		case 'a':
			yawd = optarg;
			yaw = atoi(yawd);
			rm_value = 1;
			break;
		case 'b':
			pitchd = optarg;
			pitch = atoi(pitchd);
			rm_value = 1;
			break;
		case 'c':
			rolld = optarg;
			roll = atoi(rolld);
			rm_value = 1;
			break;
		case 'd':
			xd = optarg;
			x = atoi(xd);
			sp_value = 1;
			break;
		case 'e':
			yd = optarg;
			y = atoi(yd);
			sp_value = 1;
			break;
		case 'f':
			zd = optarg;
			z = atoi(zd);
			sp_value = 1;
			break;
		case 'g':
			ttd = optarg;
			towing_threshold = atoi(ttd);
			towing = 1;
			break;
		case 'z':
			ttimed = optarg;
			towing_time = atoi(ttimed);
			towing = 1;
			break;
		case 'i':
			ccd = optarg;
			crash_threshold = atoi(ccd);
			crash = 1;
			break;
		case 'j':
			ctimed = optarg;
			crash_time = atoi(ctimed);
			crash = 1;
			break;
		case 'k':
			ignd = optarg;
			ign_state = atoi(ignd);
			ignition = 1;
			break;
		default:
			help(argv[0]);
		}
	}

	if (rm_value)
		update_sensor_rotation_matrix(yaw,pitch,roll);

	if (sp_value)
		update_sensor_placement(x,y,z);

	if (towing)
		update_sensor_towing_jack_parameters(towing_threshold, towing_time);

	if (crash)
		update_sensor_crash_detection_parameters(crash_threshold, crash_time);

	if (ignition)
		update_ignition_state(ign_state);

	return 0;
}
