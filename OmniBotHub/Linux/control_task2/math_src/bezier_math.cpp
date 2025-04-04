/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "Interpolation.h"
#include "FootSwingTrajectory.h"

_TRAJ_SWING_BS traj_swing[4];
/*!
 * Compute foot swing trajectory with a bezier curve 用贝塞尔曲线计算足部摆动轨迹
 * @param phase : How far along we are in the swing (0 to 1) 在整个摆动过程百分比(从0到1)
 * @param swingTime : How long the swing should take (seconds) 摆动需要多久
 */
void setInitialPosition(_TRAJ_SWING_BS *input, Vect3  p0) {
	input->_p0 = p0;
}

/*!
 * Set the desired final position of the foot 设置脚的终点位置
 * @param pf : the final foot posiiton
 */
void setFinalPosition(_TRAJ_SWING_BS *input, Vect3  pf) {
	input->_pf = pf;
}

/*!
 * Set the maximum height of the swing 最大高度
 * @param h : the maximum height of the swing, achieved halfway through the swing 在一半路程到达最高
 */
void setHeight(_TRAJ_SWING_BS *input, float h) {
	input->_height = h;
}

//void computeSwingTrajectoryBezier(float phase, float swingTime);

/*!
 * Get the foot position at the current point along the swing//获得轨迹坐标
 * @return : the foot position
 */
Vect3  getPosition(_TRAJ_SWING_BS *input ) {
	return input->_p;
}

/*!
 * Get the foot velocity at the current point along the swing//获得此时轨迹导数
 * @return : the foot velocity
 */
Vect3  getVelocity(_TRAJ_SWING_BS *input ) {
	return input->_v;
}

/*!
 * Get the foot acceleration at the current point along the swing 得到脚在当前点上的加速
 * @return : the foot acceleration
 */
Vect3  getAcceleration(_TRAJ_SWING_BS *input ) {
	return input->_a;
}

void FootSwingTrajectory_init(_TRAJ_SWING_BS *input) {
	Vect3 zero;
	zero.x = zero.y = zero.z = 0;
	input->_p0 = zero;// .setZero();//初始点
	input->_pf = zero;//.setZero();//终点
	input->_p = zero;//.setZero();//轨迹点
	input->_v = zero;//.setZero();//轨迹速度
	input->_a = zero;//.setZero();//轨迹加速度
	input->_height = 0;//轨迹高度
}

void computeSwingTrajectoryBezier(_TRAJ_SWING_BS *input, float phase, float swingTime) {

	input->_p.x = cubicBezier(input->_p0.x, input->_pf.x, phase);
	input->_v.x = cubicBezierFirstDerivative(input->_p0.x, input->_pf.x, phase) / swingTime;
	input->_a.x = cubicBezierSecondDerivative(input->_p0.x, input->_pf.x, phase) / (swingTime * swingTime);

	input->_p.y = cubicBezier(input->_p0.y, input->_pf.y, phase);
	input->_v.y = cubicBezierFirstDerivative(input->_p0.y, input->_pf.y, phase) / swingTime;
	input->_a.y = cubicBezierSecondDerivative(input->_p0.y, input->_pf.y, phase) / (swingTime * swingTime);
  float zp, zv, za;

  if(phase < float(0.5)) {
    zp = cubicBezier(input->_p0.z, input->_p0.z + input->_height, phase * 2);
    zv = cubicBezierFirstDerivative(input->_p0.z, input->_p0.z + input->_height, phase * 2) * 2 / swingTime;
    za = cubicBezierSecondDerivative(input->_p0.z, input->_p0.z + input->_height, phase * 2) * 4 / (swingTime * swingTime);
  } else {
    zp = cubicBezier(input->_p0.z + input->_height, input->_pf.z, phase * 2 - 1);
    zv = cubicBezierFirstDerivative(input->_p0.z + input->_height, input->_pf.z, phase * 2 - 1) * 2 / swingTime;
    za = cubicBezierSecondDerivative(input->_p0.z + input->_height, input->_pf.z, phase * 2 - 1) * 4 / (swingTime * swingTime);
  }

  input->_p.z = zp;
  input->_v.z = zv;
  input->_a.z = za;
}

