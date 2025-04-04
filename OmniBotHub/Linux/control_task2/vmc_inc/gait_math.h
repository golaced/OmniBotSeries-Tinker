#ifndef __GAIT_MATH_H__
#define __GAIT_MATH_H__
#include "base_struct.h"
#include "math.h"

#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

//-----------------------------------common math-----------------------
float sindw(float in);
float cosdw(float in);
float To_180_degreesw(float x);
float To_360_degreesw(float x);
float limitw(float x,float min,float max);
float deadw(float x,float zoom);
float my_abs(float f);
float fast_atan2(float y, float x);
float my_atan(float x, float y);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float my_deathzoom_rc(float x,float zoom);
float limit_mine(float x,float zoom);
float limit_mine2(float x,float min,float max);
float To_180_degrees(float x);
float my_pow_2_curve(float in,float a,float max);
float fast_sqrt(float number);
float cosd(double in);
float tand(double in);
float sqrtd(double in);
float sind(float in);
float dead(float x,float zoom);
void invet22(const float A[4], float *dA, float inA[4]);
void invet33(const float A[9], float *dA, float inA[9]);
void cal_ero_so3(float exp_att[3],float *err_so3);
//----------------------------------Kin Dof math-------------------------------------
void espd_to_neg_dq(int id,float dt);
void force_to_tao(int id,float dt);
void force_to_tao_kin1(Vect3 tar_force_h,int id, float dt);
void force_to_tao_kin12(Vect3 tar_force_h,int id, float dt);
void force_to_tao_kin2(Vect3 tar_force_h,int id, float dt);
void force_to_tao_input(int id,Vect3 force_h, float dt);
void inv_KI(int id,Vect3 epos_h,float *s0,float *s1,float *s2);
char estimate_end_state_new(VMC *in, float dt);
char inv_end_state_new(VMC *vmc,float x,float y,float z,float *sita1,float *sita2,float *sita3);
char cal_invjacobi(VMC *in);
char cal_jacobi_new(VMC *in,float dt);

//----------------------------------RT Matrix math---------------------
void converV_n_to_bw(Vect3 vn,Vect3* vb);
void converV_n_to_bw_noroll(Vect3 vn,Vect3* vb);
void converV_b_to_legw(int id,Vect3 vb,Vect3* vl);
void converV_b_to_leg_ow(char leg,float xb,float yb,float zb,float *xl,float *yl,float *zl);
void converV_leg_to_bw(int id,Vect3 vl,Vect3* vb);
void converV_b_to_nw_noroll(Vect3 vb,Vect3* vn);
void converV_b_to_nw(Vect3 vb,Vect3* vn);
void force_n_to_bw(Vect3 fn,Vect3* fb);
void converV_b_to_n_RTw(float RT[3][3], float yaw, float xb,float yb,float zb,float *xn,float *yn,float *zn);
void converV_n_to_b_w_yaWww(float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb);
void mat_trans(float src[3][3],float dis[3][3]);
void converV_n_to_b(float xn,float yn,float zn,float *xb,float *yb,float *zb);
void converV_n_to_b_noroll(float xn,float yn,float zn,float *xb,float *yb,float *zb);
void converV_n_to_b_RT(float RT[3][3],float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb);
void converV_n_to_b_w_yaw(float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb);
void converV_b_to_leg(char leg,float xb,float yb,float zb,float *xl,float *yl,float *zl);
void converV_leg_to_b(char leg,float xl,float yl,float zl,float *xb,float *yb,float *zb);
void converV_b_to_n(float xb,float yb,float zb,float *xn,float *yn,float *zn);
void converV_b_to_n_RT(float RT[3][3], float yaw, float xb,float yb,float zb,float *xn,float *yn,float *zn);
void force_n_to_b(VMC *in);
void force_n_to_bw_noroll(Vect3 fn,Vect3* fb);
void converV_n_to_g(Vect3 vn, Vect3* vb);
void converV_g_to_n(Vect3 vn, Vect3* vb);
void matrx33_mult_vect2(float matrix[2][2], Vect3 src, Vect3* dir);
void vect3_2_cross(Vect3 w, float cross[3][3]);
void matrx33_mult_vect3(float matrix[3][3], Vect3 src, Vect3* dir);
void matrx33_mult_matrx33(float matrix1[3][3], float matrix2[3][3], float matrixo[3][3]);
void rotate_vect3_with_yaw(END_POS point, END_POS *point_n, float yaw);
float converV_n_to_b_w_yaw_temp(float att_rt_use[3], float xn, float yn, float zn, float *xb, float *yb, float *zb);
//------------------------------------Filter math-----------------------------------------------------------
#define F_PI 3.1415926
#define LPF_COF_05Hz  1.0f/(2*F_PI*0.5)
#define LPF_COF_1t5Hz  1.0f/(2*F_PI*3)
#define LPF_COF_5t10Hz  1.0f/(2*F_PI*7)
#define LPF_COF_10t15Hz  1.0f/(2*F_PI*12)
#define LPF_COF_15t20Hz  1.0f/(2*F_PI*17)
#define LPF_COF_20t25Hz  1.0f/(2*F_PI*22)
#define LPF_COF_25t30Hz  1.0f/(2*F_PI*27)
#define LPF_COF_30t50Hz  1.0f/(2*F_PI*40)
#define LPF_COF_50t70Hz  1.0f/(2*F_PI*60)
#define LPF_COF_70t100Hz  1.0f/(2*F_PI*80)
#define LPF_COF_100tHz  1.0f/(2*F_PI*100)
#define MED_WIDTH_NUM 100
#define MED_FIL_ITEM  100

/* 2 Dimension */
typedef struct {
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r;        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman2_state;  
extern void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);

void DigitalLPF(float in, float* out, float cutoff_freq, float dt);
void DigitalLPF_Double(float in, double* out, float cutoff_freq, float dt);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);

extern float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
extern float med_filter_out[MED_FIL_ITEM];
extern u8 med_fil_cnt[MED_FIL_ITEM];
float Moving_Median(u8 item,u8 width_num,float in);

void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
float kalman2_filter(kalman2_state *state, float z_measure);
void DigitalLPFw(float in, float* out, float cutoff_freq, float dt);

typedef struct
{ //control parameter
	float h0;
	float v1,v2,r0;
}ESO_X;
void OLDX_SMOOTH_IN_ESOX(ESO_X *eso_in,float in);

#define ESO_AngularRate_his_length 4

typedef struct
{
	float beta1;
	float beta2;
	
	float T;
	float invT;
	float z_inertia;
	float z1;
	float z2;
	float u;
	float his_z1[ ESO_AngularRate_his_length ];

	float h;

	char err_sign;
	float err_continues_time;
	
	float b;
}ESO_AngularRate;

extern ESO_AngularRate leg_td2[4][3];

typedef struct
{
	unsigned char tracking_mode;
	
	float x1;
	float x2;
	float x3;
	float x4;
	
	float P1;
	float P2;
	float P3;
	float P4;
	
	float r2p , r2n , r3p , r3n , r4p , r4n;
}_TD4;

extern ESO_AngularRate leg_td2[4][3];
extern _TD4 leg_td4[4][3];
extern _TD4 odom_td[3],att_cmd_mpc[3];
extern _TD4 bldc_td4[4][3];

void TD4_reset( _TD4* filter );
void TD4_init( _TD4* filter , float P1 , float P2 , float P3 , float P4 );
float TD4_track4( _TD4* filter , const float expect , const float h );
float TD4_track3( _TD4* filter , const float expect , const float h );
void init_ESO_AngularRate( ESO_AngularRate* eso , float T , float b , float beta1 , float beta2 );
void ESO_AngularRate_update_u( ESO_AngularRate* eso , float u );
float ESO_AngularRate_run( ESO_AngularRate* eso , const float v , const float h );
float sign(float x);
void OLDX_SMOOTH_IN_ESOX(ESO_X *eso_in,float in);

//---------------------------------------------------Traj math-----------------------------------------
typedef struct
{
 float pt[3];
 float vt[3];
 float at[3];
 float ps[3];
 float vs[3];
 float as[3];
 float pe[3];
 float ve[3];
 float ae[3];
 float param[10];
 float Time,time_now,Dis;
 float cost,cost_all;
 float traj_pre_d;
 char defined[3];
	
}_TRA;

extern _TRA traj[10];

void GenerateTrajectory(float  p0,float v0,float a0,float pf,float vf,float af,float Tf,char defined[3],float*a,float*b,float*g,float *cost);
void get_trajecotry(float p0,float v0,float a0,float a,float b,float g,float t,float *pos,float *spd,float *acc,float *jerk);
void plan_tra(_TRA *tra);
void get_tra(_TRA *tra,float t);
int swing_jerk_planner_2d_3point(VMC* in,float lift_spd,float td_spd,float mid_spd_w,float T_sw);
int get_swing_jerk_2d_3point(VMC* in, float time_now);
//----------------------------------------多项式----------------------------------------
void cal_curve_from_pos_new(VMC *in, END_POS t_pos, float sw_z, float desire_time);
END_POS cal_pos_tar_from_curve(VMC *in,float desire_time, float dt);

int swing_jerk_planner_3point(VMC* in, float lift_spd, float td_spd, float sw_z, float T_sw);
int get_swing_jerk_3point(VMC* in, float time_now);
int swing_jerk_planner_5point(VMC* in, float lift_spd, float td_spd, float sw_z, float T_sw);
int get_swing_jerk_5point(VMC* in, float time_now);

int foot_trap_detector(VMC* in, char mode, float dt);
int upstair_detector(VMC* in, char mode, float dt);
//----------------------------------------MIT Traj
int swing_Bezier_planner(VMC* in, float lift_spd, float td_spd,float sw_z, float T_sw);
int get_swing_Bezier(VMC* in, float t_sw, float time_now);
float cal_BezierP3(float s, float a, float b, float c, float d);
float cal_Impluse_Fz(float alfa, float s_st, float s_peak);
float cal_Impluse_gs(float s_st);
float cal_Impluse_tauP(float alfa, float s_st, char sel);
float cal_Impluse_sw_z(float dh, float s_st, float s_peak);
//----------------------------------------ZMP math

void line_function_from_two_point_n(float x1, float y1, float x2, float y2, float *a, float *b, float *c);
void line_function_from_two_point(float x1, float y1, float x2, float y2, float *k, float *b);
void line_function_from_arrow_n(float x, float y, float yaw, float *a, float *b, float *c);
void line_function_from_arrow(float x, float y, float yaw, float *k, float *b);

void line_function90_from_arrow(float x, float y, float yaw, float *k, float *b);

u8 cross_point_of_lines(float k1, float b1, float k2, float b2, float *x, float *y);

u8 cross_point_of_lines_n(float a1, float b1, float c1, float a2, float b2, float c2, float *x, float *y);

void swap(float *a, float *b);

float cal_area_trig(END_POS p1, END_POS p2, END_POS p3);

//一个点在三角形内部
u8 inTrig(END_POS point, END_POS p1, END_POS p2, END_POS p3);

//一个点在四边形内部
//			  y
//	d----------b          /\
// 	     |                |
//			 O                L
//			 |                |
//	a----------c   x			\/
u8 segmentsIntr(END_POS b, END_POS c, END_POS d, END_POS a, float *x, float *y);
//一个点在四边形内部
u8 inTrig2(END_POS point, END_POS p1, END_POS p2, END_POS p3, END_POS p4);

//找到离xy最近的点
void find_closet_point(u8*min_id, float x, float y, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, u8 num);

float cal_bear(float x2, float  y2, float  x1, float  y1);
END_POS cal_quad_cog(END_POS a, END_POS b, END_POS c, END_POS d);

END_POS cal_tri_cog(END_POS a, END_POS b, END_POS c);

END_POS cal_quad_cog1(END_POS a, END_POS b, END_POS c, END_POS d);

END_POS cal_tri_cog1(END_POS a, END_POS b, END_POS c);
char point_is_right_line_with_arrow(END_POS point, float yaw_line, float k, float b);

char point_is_front_line_with_arrow(END_POS point, float yaw_90, float k, float b);

float cal_dis_point_line(END_POS point, float k, float b);

char check_leg_out_space(char leg, float chech_band, float *band_dis);
float cal_yaw_xy(float x, float y);

float cal_stable_area(void);
//------------------------------------------------------------------
//点在直线上
u8 check_point_on_line(END_POS point, float k, float b, float err);
//点在点矢量方向前
u8 check_point_front_arrow(END_POS point, END_POS vector_p, float yaw);

u8 check_point_front_arrow_n(END_POS point, END_POS vector_p, float yaw);
//判断两点在线同一侧
u8 check_points_same_side(float x1, float y1, float x2, float y2, float k, float b);
//点矢量垂线与直线交点
u8 check_cross_arrow90_line(float cx, float cy, float yaw, float k, float b, float *x, float *y);
//计算两点距离
float cal_dis_of_points(float x1, float y1, float x2, float y2);
//判断一个点在椭圆内部
u8 in_circle(END_POS center, END_POS point, float d_short, float d_long);
//点到直线距离
float dis_point_to_line(END_POS point, float k, float b);
////点矢量与四边形交点
u8 check_point_to_tangle(float x, float y, float yaw, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4
    , float *jiao1_x, float *jiao1_y, float *jiao2_x, float *jiao2_y);
////点矢量与四边形交点
u8 check_point_to_tangle_n(float x, float y, float yaw, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4
    , float *jiao1_x, float *jiao1_y, float *jiao2_x, float *jiao2_y);
void cal_jiao_of_tuo_and_line(float cx, float cy, float d_short, float d_long, float yaw, float *jiao1_x, float *jiao1_y, float *jiao2_x, float *jiao2_y);
//计算当前稳态余量
float cal_steady_s3(END_POS point, END_POS p1, END_POS p2, END_POS p3);
//计算当前稳态余量4leg
float cal_steady_s4(END_POS point, END_POS p1, END_POS p2, END_POS p3, END_POS p4);
void cal_tri_scale_point(END_POS p1, END_POS p2, END_POS p3, END_POS* p11, END_POS* p22, END_POS* p33, float scale);//计算三角形缩放顶点


//---------------------------------Momentum of contact point contoller
typedef struct
{
    float t;
    float T_c;
    float l_c;
    float mess;
    float Ix;
    float Hs;
    float L_mesure;
    float L_t;
    float L_t_kf;
    float L_est;
    float L_tar;
    float desired_W;
    float g;
    float kp_est;
    float sup_p;
    float p_r;
    float v_r;
    char support_leg;//= support_leg
    END_POS contact_pos;
    END_POS left_foot_pos;//= [0.0, 0.0, 0.0]
    END_POS right_foot_pos;// = [0.0, 0.0, 0.0]
    END_POS COM_pos;//= [0.0, 0.0, 0.0]
    END_POS swing_pos;//= [0.0, 0.0, 0.0]
}_Mom_LIP;
extern _Mom_LIP mom_lip[2];

void momentum_control_init(_Mom_LIP *lip, char leg_support, float d_W, float Ts);
void momentum_planner(_Mom_LIP *lip, float dt);
void momentum_estimator(_Mom_LIP *lip, float cog_H, float dt);
void lateral_controller(_Mom_LIP *lip, float dt);

void momentum_planner2(_Mom_LIP *lip, float dt);
void momentum_estimator2(_Mom_LIP *lip, float cog_H, float dt);
void lateral_controller2(_Mom_LIP *lip, float dt);

void momentum_control_init3(_Mom_LIP *lip, char leg_support, float Ts);
void momentum_planner3(_Mom_LIP *lip, float cog_H, float desired_W, float dt);
void momentum_estimator3(_Mom_LIP *lip, float cog_H, float dt);
void lateral_controller3(_Mom_LIP *lip);
#endif

