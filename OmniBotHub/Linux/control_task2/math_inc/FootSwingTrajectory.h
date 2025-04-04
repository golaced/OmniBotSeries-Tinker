/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories. 实用工具，以产生脚摆动轨迹。
 *
 * Currently uses Bezier curves like Cheetah 3 does 目前使用贝兹曲线，像猎豹3号
 */

#ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#include "gait_math.h"
//#include "cppTypes.h"
typedef struct {
	Vect3 _p0, _pf, _p, _v, _a;
	float _height;
}_TRAJ_SWING_BS;

extern _TRAJ_SWING_BS traj_swing[4];

void setInitialPosition(_TRAJ_SWING_BS *input, Vect3  p0);
void setFinalPosition(_TRAJ_SWING_BS *input, Vect3  pf);
void setHeight(_TRAJ_SWING_BS *input, float h);
Vect3  getPosition(_TRAJ_SWING_BS *input);
Vect3  getVelocity(_TRAJ_SWING_BS *input);
Vect3  getAcceleration(_TRAJ_SWING_BS *input);
void FootSwingTrajectory_init(_TRAJ_SWING_BS *input);
void computeSwingTrajectoryBezier(_TRAJ_SWING_BS *input, float phase, float swingTime);
//
///*!
// * A foot swing trajectory for a single foot单脚的摆动轨迹
// */
//template<typename T>
//class FootSwingTrajectory {
//public:
//
//  /*!
//   * Construct a new foot swing trajectory with everything set to zero 建立一个新的脚摆动轨迹，一切设置为零
//   */
//  FootSwingTrajectory() {
//    _p0.setZero();//初始点
//    _pf.setZero();//终点
//    _p.setZero();//轨迹点
//    _v.setZero();//轨迹速度
//    _a.setZero();//轨迹加速度
//    _height = 0;//轨迹高度
//  }
//
//  /*!
//   * Set the starting location of the foot 设置脚的起始位置
//   * @param p0 : the initial foot position
//   */
//  void setInitialPosition(Vec3<T> p0) {
//    _p0 = p0;
//  }
//
//  /*!
//   * Set the desired final position of the foot 设置脚的终点位置
//   * @param pf : the final foot posiiton
//   */
//  void setFinalPosition(Vec3<T> pf) {
//    _pf = pf;
//  }
//
//  /*!
//   * Set the maximum height of the swing 最大高度
//   * @param h : the maximum height of the swing, achieved halfway through the swing 在一半路程到达最高
//   */
//  void setHeight(T h) {
//    _height = h;
//  }
//
//  void computeSwingTrajectoryBezier(T phase, T swingTime);
//
//  /*!
//   * Get the foot position at the current point along the swing//获得轨迹坐标
//   * @return : the foot position
//   */
//  Vec3<T> getPosition() {
//    return _p;
//  }
//
//  /*!
//   * Get the foot velocity at the current point along the swing//获得此时轨迹导数
//   * @return : the foot velocity
//   */
//  Vec3<T> getVelocity() {
//    return _v;
//  }
//
//  /*!
//   * Get the foot acceleration at the current point along the swing 得到脚在当前点上的加速
//   * @return : the foot acceleration
//   */
//  Vec3<T> getAcceleration() {
//    return _a;
//  }
//
//private:
//  Vec3<T> _p0, _pf, _p, _v, _a;
//  T _height;
//};


#endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
