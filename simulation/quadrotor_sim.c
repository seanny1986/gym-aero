#include <stdio.h>
#include "quadrotor.h"

// Functions to neaten stuff up a bit
typedef ExtY_quadrotor_T sim_data_t;
sim_data_t * const data = &quadrotor_Y;

typedef ExtU_quadrotor_T cmd_inputs;
cmd_inputs * const cmds = &quadrotor_U;

void sim_step(double rpm_cmd1, double rpm_cmd2, double rpm_cmd3, double rpm_cmd4, int n)
{
    int i;
    for (i=0; i<n; i++) {
        quadrotor_U.rpm_cmd[0] = rpm_cmd1;
        quadrotor_U.rpm_cmd[1] = rpm_cmd2;
        quadrotor_U.rpm_cmd[2] = rpm_cmd3;
        quadrotor_U.rpm_cmd[3] = rpm_cmd4;
        quadrotor_step();
        //printf("%.4f %.4f %.4f\n", data->xyz[0], data->xyz[1], data->xyz[2]);
    }
}

void set_rpm(double rpm_cmd1, double rpm_cmd2, double rpm_cmd3, double rpm_cmd4)
{
    quadrotor_U.rpm_cmd[0] = rpm_cmd1;
    quadrotor_U.rpm_cmd[1] = rpm_cmd2;
    quadrotor_U.rpm_cmd[2] = rpm_cmd3;
    quadrotor_U.rpm_cmd[3] = rpm_cmd4;
}

void sim_init(void)
{
    quadrotor_initialize();
}

void set_init_state(double pos[3], double zeta[3], double omega[3], double vel[3], double rpm[4])
{
    int i;
    for (i=0; i<3; i++)
    {
        simparam_init_pos[i] = pos[i];
        simparam_init_euler[i] = zeta[i];
        simparam_init_omega[i] = omega[i];
        simparam_init_vel[i] = vel[i];
        simparam_init_rpm[i] = rpm[i];
    }
    simparam_init_rpm[3] = rpm[3];
    
}

void sim_term(void)
{
    quadrotor_terminate();
}

// getters for state data
double get_x()
{
    return data->xyz[0];
}

double get_y()
{
    return data->xyz[1];
}

double get_z()
{
    return data->xyz[2];
}

double get_phi()
{
    return data->zeta[0];
}

double get_theta()
{
    return data->zeta[1];
}

double get_psi()
{
    return data->zeta[2];
}

double get_u()
{
    return data->uvw[0];
}

double get_v()
{
    return data->uvw[1];
}

double get_w()
{
    return data->uvw[2];
}

double get_p()
{
    return data->pqr[0];
}

double get_q()
{
    return data->pqr[1];
}

double get_r()
{
    return data->pqr[2];
}


// getters for sim parameters
float get_torque_coeff()
{
    return simparam_kq;
}

float get_thrust_coeff()
{
    return simparam_kt;
}

float get_mass()
{
    return simparam_m;
}

float get_rad()
{
    return simparam_r;
}

float get_gravity()
{
    return simparam_g;
}

//float get_time_step()
//{
//    return Timing.simTimeStep;
//}

// getters for rpm
float get_rpm_0()
{
    return data->rpm[0];
}

float get_rpm_1()
{
    return data->rpm[1];
}

float get_rpm_2()
{
    return data->rpm[2];
}

float get_rpm_3()
{
    return data->rpm[3];
}

// Use the simulator like this:
int test(int argc, char **argv)
{
    int i;
    
    // Get pointer to output values struct. Below is the list of members of this struct.
//   pqr[3];
//   zeta[3];
//   xyz[3];
//   uvw[3];
//   pqr_dot[3];
//   uvw_dot[3];
//   uvw_earth[3];
//   q[4];
//   rpm[4];
    sim_data_t * const data = &quadrotor_Y;
    
    // Override "tuneable parameters" if necessary. This is a complete list.
    // Defaults can be found in quadrotor.c
//     simparam_I[9];
//     simparam_g;
//     simparam_init_euler[3];
//     simparam_init_omega[3];
//     simparam_init_pos[3];
//     simparam_init_vel[3];
//     simparam_kq;
//     simparam_kt;
//     simparam_m;
//     simparam_tau;
//     simparam_r;
//     simparam_init_rpm[4];

    // Run the simulation
    //sim_init();
    
    //sim_step(0, 0, 40, 40, 100);
    //printf("%.4f %.4f %.4f\n", data->zeta[0], data->zeta[1], data->zeta[2]);
    //sim_term();
    
    return 0;
}
