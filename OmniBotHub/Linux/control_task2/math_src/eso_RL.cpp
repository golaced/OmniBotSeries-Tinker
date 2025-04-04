#include "eso_RL.h"
#define LIMIT_ESO(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

ESO_RL joint_eso[4][3];

void ESO_Init(ESO_RL *eso_in, float dof_pos, float dof_vel)
{
	eso_in->dof_pos_obsr = dof_pos;
	eso_in->dof_vel_obsr = dof_vel;
	eso_in->dof_dist_obsr = 0;
	eso_in->l1 = 3 * eso_in->wn;
	eso_in->l2 = 3 * pow(eso_in->wn, 2);
	eso_in->l3 = pow(eso_in->wn, 3);	
}

void ESO_update(ESO_RL *eso_in, float dof_pos, float torque, float dt)
{
	eso_in->dt = dt;

	eso_in->dof_pos_obsr_error = dof_pos - eso_in->dof_pos_obsr;

	eso_in->dof_pos_obsrd = eso_in->dof_vel_obsr + eso_in->l1 * eso_in->dof_pos_obsr_error;
	eso_in->dof_vel_obsrd = -eso_in->alpha_v * eso_in->dof_vel_obsr + eso_in->ctrl_gain * torque + eso_in->dof_dist_obsr + eso_in->l2 * eso_in->dof_pos_obsr_error;
	eso_in->dof_dist_obsrd = eso_in->l3 * eso_in->dof_pos_obsr_error;

	eso_in->dof_pos_obsr += eso_in->dof_pos_obsrd * eso_in->dt;
	eso_in->dof_vel_obsr += eso_in->dof_vel_obsrd * eso_in->dt;
	eso_in->dof_dist_obsr += eso_in->dof_dist_obsrd * eso_in->dt;
}

void compute_torque_comp(ESO_RL *eso_in)
{
	eso_in->torque_comp = -eso_in->beta_ratio * eso_in->dof_dist_obsr / eso_in->ctrl_gain;
	eso_in->torque_comp = LIMIT_ESO(eso_in->torque_comp, -eso_in->torque_comp_limit, eso_in->torque_comp_limit);
}
