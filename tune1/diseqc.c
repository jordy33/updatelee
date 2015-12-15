/* diseqc -- simple diseqc commands for the Linux DVB S2 API
 *
 * UDL (updatelee@gmail.com)
 * Derived from work by:
 * 	Igor M. Liplianin (liplianin@me.by)
 * 	Alex Betis <alex.betis@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "diseqc.h"

struct dvb_diseqc_master_cmd committed_switch_cmds[] = {
	{ { 0xE0, 0x10, 0x38, 0xF0, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x38, 0xF4, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x38, 0xF8, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x38, 0xFC, 0x00, 0x00 }, 4 }
};

struct dvb_diseqc_master_cmd uncommitted_switch_cmds[] = {
	{ { 0xE0, 0x10, 0x39, 0xF0, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF1, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF2, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF3, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF4, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF5, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF6, 0x00, 0x00 }, 4 },
	{ { 0xE0, 0x10, 0x39, 0xF7, 0x00, 0x00 }, 4 }
};

struct dvb_diseqc_master_cmd dir_cmd[] =
{
	{ { 0xe0, 0x31, 0x68, 0xFF, 0x00, 0x00 }, 4 }, // Drive Motor East 1 step
	{ { 0xe0, 0x31, 0x69, 0xFF, 0x00, 0x00 }, 4 }  // Drive Motor West 1 step
};

double radian( double number )
{
	return number*M_PI/180;
}

double degree( double number )
{
	return number*180/M_PI;
}

int last_digit(double var)
{
	var -= floor(var);
	int i = 0;
	double test;
	for (test = 0; test <= var; test += 0.0625 )
		i++;
	return --i;
}

void motor_usals(int frontend_fd, double site_lat, double site_long, double sat_long)
{
	if (ioctl(frontend_fd, FE_SET_TONE, SEC_TONE_OFF) == -1)
		printf("FE_SET_TONE ERROR! \n");
	usleep(20000);

	if (ioctl(frontend_fd, FE_SET_VOLTAGE, SEC_VOLTAGE_18) == -1)
		printf("FE_SET_VOLTAGE ERROR! \n");
	usleep(20000);

	double r_eq = 6378.14;		// Earth radius
	double r_sat = 42164.57;	// Distance from earth centre to satellite

	site_lat  = radian(site_lat);
	site_long = radian(site_long);
	sat_long  = radian(sat_long);

	// x = [0], y = [1], z = [2]
	double dishVector[3] = { r_eq * cos(site_lat), 0, r_eq * sin(site_lat) };
	double satVector[3] = { r_sat * cos(site_long - sat_long), r_sat * sin(site_long - sat_long), 0 };
	double satPointing[3] = { satVector[0] - dishVector[0], satVector[1] - dishVector[1], satVector[2] - dishVector[2] } ;

	double motor_angle = degree( atan( satPointing[1]/satPointing[0] ) );
	int RotorCmd = (int)floor(fabs(motor_angle)) * 0x10 + last_digit(fabs(motor_angle));

	if (motor_angle < 0) // the east else west
		RotorCmd |= 0xE000;
	else
		RotorCmd |= 0xD000;
	int angle_1 = (RotorCmd & 0xFF00) / 0x100;
	int angle_2 = RotorCmd & 0xFF;
	printf("Long: %.2f, Lat: %.2f, Orbital Pos: %.2f, RotorCmd: %02x %02x, motor_angle: %.2f \n", degree(site_long), degree(site_lat), degree(sat_long), angle_1, angle_2, motor_angle);

	struct dvb_diseqc_master_cmd usals_cmd = { { 0xe0, 0x31, 0x6e, angle_1, angle_2, 0x00 }, 5 };

	int i;
	for (i = 0; i < 5; i++) {
		diseqc_send_msg(frontend_fd, usals_cmd);
		sleep(6);
	}
}

void motor_dir(int frontend_fd, int dir)
{
	diseqc_send_msg(frontend_fd, dir_cmd[dir]);
	usleep(20000);
}

void diseqc_send_msg(int frontend_fd, struct dvb_diseqc_master_cmd cmd)
{
	printf("DiSEqC: %02x %02x %02x %02x %02x %02x length: %d\n",
		cmd.msg[0], cmd.msg[1],
		cmd.msg[2], cmd.msg[3],
		cmd.msg[4], cmd.msg[5], cmd.msg_len);

	if (ioctl(frontend_fd, FE_DISEQC_SEND_MASTER_CMD, &cmd) == -1)
		printf("FE_DISEQC_SEND_MASTER_CMD ERROR! \n");
	usleep(20000);
}

void setup_switch (int frontend_fd, fe_sec_voltage_t voltage, fe_sec_tone_mode_t tone, int committed, int uncommitted)
{
	if (tone)
		printf("22khz OFF\n");
	else
		printf("22khz ON\n");

	if (ioctl(frontend_fd, FE_SET_TONE, SEC_TONE_OFF) == -1)
		printf("FE_SET_TONE ERROR! \n");
	usleep(20000);

	if (ioctl(frontend_fd, FE_SET_VOLTAGE, voltage) == -1)
		printf("FE_SET_VOLTAGE ERROR! \n");
	usleep(20000);

	if (uncommitted)
		diseqc_send_msg(frontend_fd, uncommitted_switch_cmds[uncommitted-1]);

	if (committed)
		diseqc_send_msg(frontend_fd, committed_switch_cmds[committed-1]);

	if (ioctl(frontend_fd, FE_SET_TONE, tone) == -1)
		printf("FE_SET_TONE ERROR! \n");
	usleep(20000);
}
