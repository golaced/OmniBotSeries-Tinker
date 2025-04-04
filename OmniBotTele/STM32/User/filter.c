#include "sys.h"
#include "filter.h"
#include "mymath.h"
/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x
n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

// #define WIDTH_NUM 101
// #define FIL_ITEM  10

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1)
{
	LPF_1_(gain_hz,dT,(in - f1->out),f1->a); //低通后的变化量

	f1->b = my_pow(in - f1->out);

	f1->e_nr = LIMIT(safe_div(my_pow(f1->a),((f1->b) + my_pow(f1->a)),0),0,1); //变化量的有效率
	
	LPF_1_(base_hz *f1->e_nr,dT,in,f1->out); //低通跟踪
}

 void Moving_Average1(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
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

#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  30

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	if(width_num==0)
	return in;
	else if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
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


void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
// 	 out->x = h_tmp_x *in->x - ref->x *in->z;
// 	 out->y = ref->z *in->y - ref->y *in->z;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}



///////////////////////////////////////////////////////////////////////////////

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////

float ACC_HIGHPASS_TAU        = 4.0;
float ACC_LOWPASS_TAU        = 0.05;
float ACC_LOWPASS_SAMPLE_TIME =0.02f;
float ACC_LOWPASS_A        ;
float ACC_LOWPASS_GX1      ;
float ACC_LOWPASS_GX2      ;
float ACC_LOWPASS_GX3      ;

float BARO_LOWPASS_TAU        = 0.05f;
float BARO_LOWPASS_SAMPLE_TIME =0.02f;
float BARO_LOWPASS_A        ;
float BARO_LOWPASS_GX1      ;
float BARO_LOWPASS_GX2      ;
float BARO_LOWPASS_GX3      ;

float FLOW_LOWPASS_TAU        = 0.005f;
float FLOW_LOWPASS_SAMPLE_TIME =0.02f;
float FLOW_LOWPASS_A        ;
float FLOW_LOWPASS_GX1      ;
float FLOW_LOWPASS_GX2      ;
float FLOW_LOWPASS_GX3      ;
firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T)
{ 
	ACC_LOWPASS_SAMPLE_TIME= T;
	ACC_LOWPASS_A       =    (2.0f * ACC_LOWPASS_TAU / ACC_LOWPASS_SAMPLE_TIME );
	ACC_LOWPASS_GX1    =     (1.0f / (1.0f + ACC_LOWPASS_A));
	ACC_LOWPASS_GX2    =     ACC_LOWPASS_GX1;
	ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
	
//	ACC_LOWPASS_A       =    (2.0f * ACC_HIGHPASS_TAU / ACC_LOWPASS_SAMPLE_TIME);
//  ACC_LOWPASS_GX1     =    ( ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
//  ACC_LOWPASS_GX2     =   (-ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
//  ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
  firstOrderFilters[ACC_LOWPASS_X].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_X].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_X].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Y].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Y].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Y].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Z].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Z].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Z].gx3 = ACC_LOWPASS_GX3;
	
	BARO_LOWPASS_SAMPLE_TIME= T;
	BARO_LOWPASS_A       =    (2.0f * BARO_LOWPASS_TAU / BARO_LOWPASS_SAMPLE_TIME );
	BARO_LOWPASS_GX1    =     (1.0f / (1.0f + BARO_LOWPASS_A));
	BARO_LOWPASS_GX2    =     BARO_LOWPASS_GX1;
	BARO_LOWPASS_GX3     =    ((1.0f - BARO_LOWPASS_A) / (1.0f + BARO_LOWPASS_A));
	firstOrderFilters[BARO_LOWPASS].gx1 = BARO_LOWPASS_GX1;
	firstOrderFilters[BARO_LOWPASS].gx2 = BARO_LOWPASS_GX2;
	firstOrderFilters[BARO_LOWPASS].gx3 = BARO_LOWPASS_GX3;
	
	FLOW_LOWPASS_SAMPLE_TIME= T;
	FLOW_LOWPASS_A       =    (2.0f * FLOW_LOWPASS_TAU / FLOW_LOWPASS_SAMPLE_TIME );
	FLOW_LOWPASS_GX1    =     (1.0f / (1.0f + FLOW_LOWPASS_A));
	FLOW_LOWPASS_GX2    =     FLOW_LOWPASS_GX1;
	FLOW_LOWPASS_GX3     =    ((1.0f - FLOW_LOWPASS_A) / (1.0f + FLOW_LOWPASS_A));
	firstOrderFilters[FLOW_LOWPASS_X].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_X].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_X].gx3 = FLOW_LOWPASS_GX3;
	firstOrderFilters[FLOW_LOWPASS_Y].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_Y].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_Y].gx3 = FLOW_LOWPASS_GX3;
}


float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T)
{   static u8 init; 
    float output;
    if(!init){init=1;initFirstOrderFilter(T);}
		initFirstOrderFilter(T);
    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}
