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
		{"yaw",	      required_argument, 0,  'y' },
		{"pitch",     required_argument, 0,  'p' },
		{"roll",      required_argument, 0,  'o' },
		{"x",         required_argument, 0,  'a' },
	        {"y", 	      required_argument, 0,  'b' },
		{"z",         required_argument, 0,  'c' },
		{"help",      no_argument,       0,  '?' },
		{0,           0,                 0,   0  }
};

static void help(char *argv)
{
        int index = 0;

        printf("usage: %s [OPTIONS]\n\n", argv);
        printf("OPTIONS:\n");

	printf("\t--%s:\t\tUpdate yaw to calculate rotational matrix. Max <= 3600\n",
               long_options[index++].name);
	printf("\t--%s:\tUpdate pitch to calculate rotational matrix. Max <= 3600\n",
               long_options[index++].name);
	printf("\t--%s:\t\tUpdate roll to calculate rotational matrix. Max <= 3600\n",
               long_options[index++].name);
	printf("\t--%s:\t\tUpdate position for x-axis\n",
               long_options[index++].name);
	printf("\t--%s:\t\tUpdate position for y-axis\n",
               long_options[index++].name);
	printf("\t--%s:\t\tUpdate position for z-axis\n",
               long_options[index++].name);
	printf("\t--%s:\t\tThis help\n", long_options[index++].name);

	exit(0);
}

int main(int argc, char **argv)
{
	int ret;
	int c;
	int digit_optind = 0;
	int rm_value = 0;
	int sp_value = 0;
	int i;
	int err = 0;
	char *yawd = NULL, *pitchd = NULL, *rolld = NULL;
	char *xd = NULL, *yd = NULL, *zd = NULL;
	uint16_t yaw = 0, pitch = 0, roll = 0;
	int16_t x = 0, y = 0, z = 0;

	while (1) {
		int this_option_optind = optind ? optind : 1;
		int option_index = 0;

		c = getopt_long(argc, argv, options,
				long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'y':
			yawd = optarg;
			yaw = atoi(yawd);
				rm_value = 1;
			break;
		case 'p':
			pitchd = optarg;
			pitch = atoi(pitchd);
				rm_value = 1;
			break;
		case 'o':
			rolld = optarg;
			roll = atoi(rolld);
				rm_value = 1;
			break;
		case 'a':
			xd = optarg;
			x = atoi(xd);
			sp_value = 1;
			break;
		case 'b':
			yd = optarg;
			y = atoi(yd);
			sp_value = 1;
			break;
		case 'c':
			zd = optarg;
			z = atoi(zd);
			sp_value = 1;
			break;
		default:
			help(argv[0]);
		}
	}

	if (rm_value)
		update_sensor_rotation_matrix(yaw,pitch,roll);

	if (sp_value)
		update_sensor_placement(x,y,z);

	return 0;
}
