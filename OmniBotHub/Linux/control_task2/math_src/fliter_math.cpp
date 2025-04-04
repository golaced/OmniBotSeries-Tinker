#include "include.h"
#include "gait_math.h"

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];
u8 med_fil_cnt[MED_FIL_ITEM];
ESO_AngularRate leg_td2[4][3];
_TD4 leg_td4[4][3];
_TD4 odom_td[3],att_cmd_mpc[3];

void DigitalLPF(float in, float* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
		float lpf_cof;
		int frep=cutoff_freq;
		
		if(cutoff_freq<1)
            lpf_cof=1/(2*F_PI*cutoff_freq);
		else if(cutoff_freq>=1&&cutoff_freq<5)
			lpf_cof=LPF_COF_1t5Hz;
		else if(cutoff_freq>=5&&cutoff_freq<10)
			lpf_cof=LPF_COF_5t10Hz;	
		else if(cutoff_freq>=10&&cutoff_freq<15)
			lpf_cof=LPF_COF_10t15Hz;
		else if(cutoff_freq>=15&&cutoff_freq<20)
			lpf_cof=LPF_COF_15t20Hz;
		else if(cutoff_freq>=20&&cutoff_freq<25)
			lpf_cof=LPF_COF_20t25Hz;
		else if(cutoff_freq>=25&&cutoff_freq<30)
			lpf_cof=LPF_COF_25t30Hz;
		else if(cutoff_freq>=30&&cutoff_freq<50)
			lpf_cof=LPF_COF_30t50Hz;
		else if(cutoff_freq>=50&&cutoff_freq<70)
			lpf_cof=LPF_COF_50t70Hz;
		else if(cutoff_freq>=70&&cutoff_freq<100)
			lpf_cof=LPF_COF_70t100Hz;		
		else
            lpf_cof=1.0f/(2*F_PI*cutoff_freq);
		float rc = lpf_cof;
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}

void DigitalLPF_Double(float in, double* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
		float lpf_cof;
		int frep=cutoff_freq;
		
		if(cutoff_freq<1)
			lpf_cof=LPF_COF_05Hz;
		else if(cutoff_freq>=1&&cutoff_freq<5)
			lpf_cof=LPF_COF_1t5Hz;
		else if(cutoff_freq>=5&&cutoff_freq<10)
			lpf_cof=LPF_COF_5t10Hz;	
		else if(cutoff_freq>=10&&cutoff_freq<15)
			lpf_cof=LPF_COF_10t15Hz;
		else if(cutoff_freq>=15&&cutoff_freq<20)
			lpf_cof=LPF_COF_15t20Hz;
		else if(cutoff_freq>=20&&cutoff_freq<25)
			lpf_cof=LPF_COF_20t25Hz;
		else if(cutoff_freq>=25&&cutoff_freq<30)
			lpf_cof=LPF_COF_25t30Hz;
		else if(cutoff_freq>=30&&cutoff_freq<50)
			lpf_cof=LPF_COF_30t50Hz;
		else if(cutoff_freq>=50&&cutoff_freq<70)
			lpf_cof=LPF_COF_50t70Hz;
		else if(cutoff_freq>=70&&cutoff_freq<100)
            lpf_cof=LPF_COF_70t100Hz;
		else
            lpf_cof=1.0f/(2*F_PI*cutoff_freq);
		float rc = lpf_cof;
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}

void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
}

float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(width_num==0)
		return in;
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}
		return ( tmp[(u16)width_num/2] );
	}
}

/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}

/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}

void DigitalLPFw(float in, float* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
    float rc = 1.0f/(2*3.1415926*cutoff_freq);
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}


void TD4_reset( _TD4* filter )
{
	filter->x1 = filter->x2 = filter->x3 = filter->x4 = 0;
}

void TD4_init( _TD4* filter , float P1 , float P2 , float P3 , float P4 )
{
	filter->P1 = P1;
	filter->P2 = P2;
	filter->P3 = P3;
	filter->P4 = P4;
	filter->r2p = filter->r2n = filter->r3p = filter->r3n = filter->r4p = filter->r4n = 1e12;
	TD4_reset( filter );
}

float TD4_track4( _TD4* filter , const float expect , const float h )
{
	filter->tracking_mode = 4;
	
	float e1 = expect - filter->x1;
	float e1_1 = -filter->x2;
	float e1_2 = -filter->x3;
	float e1_3 = -filter->x4;
	float T2 = filter->P1 * e1;
	float P1 = 0;
	if( T2 > filter->r2p )
		T2 = filter->r2p;
	else if( T2 < -filter->r2n )
		T2 = -filter->r2n;
	else
		P1 = filter->P1;
	float T2_1 = P1 * e1_1;
	float T2_2 = P1 * e1_2;
	float T2_3 = P1 * e1_3;
	
	float e2 = T2 - filter->x2;
	float e2_1 = T2_1-filter->x3;
	float e2_2 = T2_2-filter->x4;
	float T3 = filter->P2 * e2;
	float P2 = 0;
	if( T3 > filter->r3p )
		T3 = filter->r3p;
	else if( T3 < -filter->r3n )
		T3 = -filter->r3n;
	else
		P2 = filter->P2;
	T3 += T2_1;
	float T3_1 = P2 * e2_1 + T2_2;
	float T3_2 = P2 * e2_2 + T2_3;
	
	float e3 = T3 - filter->x3;
	float e3_1 = T3_1-filter->x4;
	float T4 = filter->P3 * e3;
	float P3 = 0;
	if( T4 > filter->r4p )
		T4 = filter->r4p;
	else if( T4 < -filter->r4n )
		T4 = -filter->r4n;
	else
		P3 = filter->P3;
	T4 += T3_1;
	float T4_1 = P3 * e3_1 + T3_2;
	
	float e4 = T4 - filter->x4;
	float T5 = filter->P4 * e4 + T4_1;
	
	filter->x1 += h*filter->x2;
	filter->x2 += h*filter->x3;
	filter->x3 += h*filter->x4;
	filter->x4 += h*T5;
	
	return filter->x1;
}

float TD4_track3( _TD4* filter , const float expect , const float h )
{
	filter->tracking_mode = 3;
	
	float e2 = expect - filter->x2;
	float e2_1 = -filter->x3;
	float e2_2 = -filter->x4;
	float T3 = filter->P2 * e2;
	float P2 = 0;
	if( T3 > filter->r3p )
		T3 = filter->r3p;
	else if( T3 < -filter->r3n )
		T3 = -filter->r3n;
	else
		P2 = filter->P2;
	float T3_1 = P2 * e2_1;
	float T3_2 = P2 * e2_2;
	
	float e3 = T3 - filter->x3;
	float e3_1 = T3_1-filter->x4;
	float T4 = filter->P3 * e3;
	float P3 = 0;
	if( T4 > filter->r4p )
		T4 = filter->r4p;
	else if( T4 < -filter->r4n )
		T4 = -filter->r4n;
	else
		P3 = filter->P3;
	T4 += T3_1;
	float T4_1 = P3 * e3_1 + T3_2;
	
	float e4 = T4 - filter->x4;
	float T5 = filter->P4 * e4 + T4_1;
	
	filter->x2 += h*filter->x3;
	filter->x3 += h*filter->x4;
	filter->x4 += h*T5;
	
	return filter->x2;
}

void init_ESO_AngularRate( ESO_AngularRate* eso , float T , float b , float beta1 , float beta2 )
{
	eso->beta1 = beta1;
	eso->beta2 = beta2;
	
	eso->z1 = eso->z2 = eso->z_inertia = 0;
	eso->err_continues_time = 0;	
	
	eso->T = T;	eso->invT = 1.0f / T;
	eso->b = b;
	for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
		eso->his_z1[i] = 0;
}

void ESO_AngularRate_update_u( ESO_AngularRate* eso , float u )
{
	eso->u = u;
	eso->z_inertia += eso->h * eso->invT * ( eso->b*eso->u - eso->z_inertia );
	eso->z1 += eso->h * ( eso->z_inertia + eso->z2 );
}

float ESO_AngularRate_run( ESO_AngularRate* eso , const float v , const float h )
{
	float err = v - eso->his_z1[0];
	
	if( (err > 0) ^ eso->err_sign )
	{
		eso->err_continues_time = 0;
		eso->err_sign = err > 0;
	}
	else
		eso->err_continues_time += h;
	
	float max_beta1_scale = 0.9f / eso->beta1;
	float err_continues_time3 = eso->err_continues_time*eso->err_continues_time*eso->err_continues_time;
	float beta1_scale = 1 + 500 * err_continues_time3;	
	float beta2_scale = 1 + 50 * err_continues_time3;		
	if( beta1_scale > 15 )
		beta1_scale = 15;
	if( beta2_scale > 15 )
		beta2_scale = 15;
	if( beta1_scale > max_beta1_scale )
		beta1_scale = max_beta1_scale;
	
	//DT?y
	float z1_correction = beta1_scale*eso->beta1*err;
	float z2_correction = beta2_scale*eso->beta2*err;
	float filter_dt = h;
	for( unsigned char k = 0 ; k < ESO_AngularRate_his_length - 1 ; ++k )
	{
		eso->his_z1[ k ] = eso->his_z1[ k + 1 ] + z1_correction + filter_dt*z2_correction;
		filter_dt += h;
	}
	eso->z2 += z2_correction;
	eso->z1 += z1_correction + filter_dt*z2_correction;
	eso->his_z1[ ESO_AngularRate_his_length - 1 ] = eso->z1;
	
	eso->h = h;
	return eso->z2;
}

float sign(float x)
{
  if(x>=0)
      return(1);
  if(x<0)
      return(-1);
	else
		  return 0;
}

static float fst2(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0,a1,a2;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	float sy,sa;
	
	d=w*h*h;
	a0=h*x2;
	td_y=x1+a0;
	a1=my_sqrt(d*(d+8*ABS(td_y)));
	a2=a0+sign(td_y)*(a1-d)/2;
	sy=(sign(td_y+d)-sign(td_y-d))/2;
	a=(a0+td_y-a2)*sy+a2;
	sa=(sign(a+d)-sign(a-d))/2;
	fhan=-w*(a/d-sign(a))*sa-w*sign(a);
	return(fhan);
}

static float fst(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	
	d=w*h;
	d0=h*d;
	td_y=x1+h*x2;
	a0=my_sqrt(d*d+8*w*ABS(td_y));
	
	if(ABS(td_y)>d0)
		a=x2+0.5*(a0-d)*sign(td_y);
	else
		a=x2+td_y/h;
		
	if (ABS(a)>d)
		fhan=-w*sign(a);
	else
		fhan=-w*a/d;
	return(fhan);
}

void OLDX_SMOOTH_IN_ESOX(ESO_X *eso_in,float in)
{
eso_in->v1+=eso_in->h0*eso_in->v2;                        //td_x1=v1;
eso_in->v2+=eso_in->h0*fst(eso_in->v1-in,eso_in->v2,eso_in->r0,eso_in->h0);           //td_x2=v2;
}
