#include "include.h"
#include "eso.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif
//------------------------------数学库
float sindw(float in){
    return sinf(in/57.3);
}

float cosdw(float in){
    return cosf(in/57.3);
}

float To_180_degreesw(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

float To_360_degreesw(float x)
{
	if(x<0)
		return 360+x;
    else if (x>=360)
        return (int)(x*1000)%360/1000.;
	else
		return x;
}

float limitw(float x,float min,float max)
{
  if(x>max)return max;
	if(x<min)return min;
	return x;
}

float deadw(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

float fast_atan_table[257] = 
{
	0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
	1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
	3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
	4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
	6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
	7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
	9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
	1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
	1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
	1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
	1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
	1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
	1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
	2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
	2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
	2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
	2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
	2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
	2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
	2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
	3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
	3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
	3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
	3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
	3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
	3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
	3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
	4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
	4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
	4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
	4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
	4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
	4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
	4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
	4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
	5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
	5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
	5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
	5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
	5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
	5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
	5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
	5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
	5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
	6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
	6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
	6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
	6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
	6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
	6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
	6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
	6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
	6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
	6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
	7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
	7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
	7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
	7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
	7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
	7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
	7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
	7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
	7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
	7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
	7.853983e-01
};

float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}

	return -f;
}

float fast_atan2(float y, float x) 
{
	float x_abs, y_abs, z;
	float alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if ((y == 0.0f) && (x == 0.0f))
		angle = 0.0f;
	else 
	{
		/* normalize to +/- 45 degree range */
		y_abs = my_abs(y);
		x_abs = my_abs(x);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			z = y_abs / x_abs;
		else
			z = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (z < TAN_MAP_RES)
			base_angle = z;
		else 
		{
			/* find index and interpolation value */
			alpha = z * (float) TAN_MAP_SIZE - .5f;
			index = (int) alpha;
			alpha -= (float) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (x >= 0.0f) 
			{           /* -45 -> 45 */
				if (y >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (y >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (y >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (x >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (x >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}
#ifdef ZERO_TO_TWOPI
	if (angle < 0)
		return (angle + TWOPI);
	else
		return (angle);
#else
	return (angle);
#endif
}

float my_atan(float x, float y)
{
	return fast_atan2(y, x);
}

float my_pow(float a)
{
	return a*a;
}

float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

#define ONE_PI   (3.14159265)
#define TWO_PI   (2.0 * 3.14159265)
#define ANGLE_UNIT (TWO_PI/10.0)

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	s8 flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

float my_cos(double rad)
{
	s8 flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin(rad)*flag;
}

float my_deathzoom(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}

  return (t);
}

float my_deathzoom_rc(float x,float zoom)
{
	float t;
	if(x>1500)
			t=LIMIT(my_deathzoom_2(x,zoom)-zoom,1500,2000);
		else
			t=LIMIT(my_deathzoom_2(x,zoom)+zoom,1000,1500);
  return (t);
}

float limit_mine(float x,float zoom)
{
    float t=x;
	
	if( x< -zoom)
	{
		t = -zoom;
	}
	else if( x>zoom)
	{
		t = zoom;
	}

  return (t);
}


float limit_mine2(float x,float min,float max)
{
    float t=x;
	
	if( x<min)
	{
		t = min;
	}
	else if( x>max)
	{
		t = max;
	}

  return (t);
}

float To_180_degrees(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

float my_pow_2_curve(float in,float a,float max)
{
	if( a > 1 || a < 0 )
	{
		return 0;
	}
	return( (1.0f - a) + a *ABS(in / max) * in );

}

float fast_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

float cosd(double in)
{
#if !RUN_WEBOTS
return arm_cos_f32(in*0.0173);
#else
return cosf(in*DEG_TO_RAD);
#endif
}

float tand(double in)
{
return tanf(in*0.0173);
}

float sqrtd(double in)
{
#if !RUN_WEBOTS
float32_t out; 
arm_sqrt_f32(in,&out);
#else
float out=0;
out=sqrtf(in);
#endif
return out;
}

float sind(float in)
{
#if !RUN_WEBOTS
return arm_sin_f32(in*0.0173);
#else
return sinf(in*0.0173);
#endif
}

float dead(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

//------------------------------------矩阵Matrix-----------------------
static void inv22(const float x[4], float y[4])
{
  float r;
  float t;
  if ((float)fabs(x[1]) > (float)fabs(x[0])) {
    r = x[0] / x[1];
    t = 1.0F / (r * x[3] - x[2]);
    y[0] = x[3] / x[1] * t;
    y[1] = -t;
    y[2] = -x[2] / x[1] * t;
    y[3] = r * t;
  } else {
    r = x[1] / x[0];
    t = 1.0F / (x[3] - r * x[2]);
    y[0] = x[3] / x[0] * t;
    y[1] = -r * t;
    y[2] = -x[2] / x[0] * t;
    y[3] = t;
  }
}

void invet22(const float A[4], float *dA, float inA[4])
{
  int ix;
  float x[4];
  signed char ipiv[2];
  int iy;
  char isodd;
  int k;
  float temp;
  float b_A[4];
  for (ix = 0; ix < 4; ix++) {
    x[ix] = A[ix];
  }

  for (ix = 0; ix < 2; ix++) {
    ipiv[ix] = (signed char)(1 + ix);
  }

  ix = 0;
  if ((float)fabs(A[1]) > (float)fabs(A[0])) {
    ix = 1;
  }

  if (A[ix] != 0.0F) {
    if (ix != 0) {
      ipiv[0] = 2;
      ix = 0;
      iy = 1;
      for (k = 0; k < 2; k++) {
        temp = x[ix];
        x[ix] = x[iy];
        x[iy] = temp;
        ix += 2;
        iy += 2;
      }
    }

    x[1] /= x[0];
  }

  if (x[2] != 0.0F) {
    x[3] += x[1] * -x[2];
  }

  *dA = x[0] * x[3];
  isodd = 0;
  if (ipiv[0] > 1) {
    isodd = 1;
  }

  if (isodd) {
    *dA = -*dA;
  }

  if (*dA == 0.0F) {
    for (ix = 0; ix < 2; ix++) {
      for (iy = 0; iy < 2; iy++) {
        b_A[ix + (iy << 1)] = 0.0F;
        for (k = 0; k < 2; k++) {
          b_A[ix + (iy << 1)] += A[k + (ix << 1)] * A[k + (iy << 1)];
        }
      }
    }

    inv22(b_A, x);
    for (ix = 0; ix < 2; ix++) {
      for (iy = 0; iy < 2; iy++) {
        inA[ix + (iy << 1)] = 0.0F;
        for (k = 0; k < 2; k++) {
          inA[ix + (iy << 1)] += x[ix + (k << 1)] * A[iy + (k << 1)];
        }
      }
    }
  } else {
    inv22(A, inA);
  }
}

static void inv33(const float x[9], float y[9])
{
  int p1;
  float b_x[9];
  int p2;
  int p3;
  float absx11;
  float absx21;
  float absx31;
  int itmp;
  for (p1 = 0; p1 < 9; p1++) {
    b_x[p1] = x[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = (float)fabs(x[0]);
  absx21 = (float)fabs(x[1]);
  absx31 = (float)fabs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      b_x[3] = x[5];
      b_x[5] = x[3];
      b_x[6] = x[8];
      b_x[8] = x[6];
    }
  }

  absx11 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx11 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= absx11 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if ((float)fabs(b_x[5]) > (float)fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= absx11 * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0F - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0F - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0F / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

void invet33(const float A[9], float *dA, float inA[9])
{
  int i0;
  float x[9];
  int j;
  signed char ipiv[3];
  int c;
  char isodd;
  int jA;
  int k;
  int ix;
  float smax;
  float s;
  float b_A[9];
  int iy;
  int ijA;
  for (i0 = 0; i0 < 9; i0++) {
    x[i0] = A[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    ipiv[i0] = (signed char)(1 + i0);
  }

  for (j = 0; j < 2; j++) {
    c = j << 2;
    jA = 0;
    ix = c;
    smax = (float)fabs(x[c]);
    for (k = 2; k <= 3 - j; k++) {
      ix++;
      s = (float)fabs(x[ix]);
      if (s > smax) {
        jA = k - 1;
        smax = s;
      }
    }

    if (x[c + jA] != 0.0F) {
      if (jA != 0) {
        ipiv[j] = (signed char)((j + jA) + 1);
        ix = j;
        iy = j + jA;
        for (k = 0; k < 3; k++) {
          smax = x[ix];
          x[ix] = x[iy];
          x[iy] = smax;
          ix += 3;
          iy += 3;
        }
      }

      i0 = (c - j) + 3;
      for (iy = c + 1; iy < i0; iy++) {
        x[iy] /= x[c];
      }
    }

    jA = c;
    iy = c + 3;
    for (k = 1; k <= 2 - j; k++) {
      smax = x[iy];
      if (x[iy] != 0.0F) {
        ix = c + 1;
        i0 = (jA - j) + 6;
        for (ijA = 4 + jA; ijA < i0; ijA++) {
          x[ijA] += x[ix] * -smax;
          ix++;
        }
      }

      iy += 3;
      jA += 3;
    }
  }

  *dA = x[0];
  isodd = 0;
  for (k = 0; k < 2; k++) {
    *dA *= x[(k + 3 * (k + 1)) + 1];
    if (ipiv[k] > 1 + k) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    *dA = -*dA;
  }

  if (*dA == 0.0F) {
    for (i0 = 0; i0 < 3; i0++) {
      for (iy = 0; iy < 3; iy++) {
        b_A[i0 + 3 * iy] = 0.0F;
        for (jA = 0; jA < 3; jA++) {
          b_A[i0 + 3 * iy] += A[jA + 3 * i0] * A[jA + 3 * iy];
        }
      }
    }

    inv33(b_A, x);
    for (i0 = 0; i0 < 3; i0++) {
      for (iy = 0; iy < 3; iy++) {
        inA[i0 + 3 * iy] = 0.0F;
        for (jA = 0; jA < 3; jA++) {
          inA[i0 + 3 * iy] += x[i0 + 3 * jA] * A[iy + 3 * jA];
        }
      }
    }
  } else {
    inv33(A, inA);
  }
}

//----------------------------------M px4-
void q_to_dcm(float data[4],float dcm[3][3]){
  float aSq = data[0] * data[0];
    float bSq = data[1] * data[1];
    float cSq = data[2] * data[2];
    float dSq = data[3] * data[3];
    dcm[0][0] = aSq + bSq - cSq - dSq;
    dcm[0][1] = 2.0f * (data[1] * data[2] - data[0] * data[3]);
    dcm[0][2] = 2.0f * (data[0] * data[2] + data[1] * data[3]);
    dcm[1][0] = 2.0f * (data[1] * data[2] + data[0] * data[3]);
    dcm[1][1] = aSq - bSq + cSq - dSq;
    dcm[1][2] = 2.0f * (data[2] * data[3] - data[0] * data[1]);
    dcm[2][0] = 2.0f * (data[1] * data[3] - data[0] * data[2]);
    dcm[2][1] = 2.0f * (data[0] * data[1] + data[2] * data[3]);
    dcm[2][2] = aSq - bSq - cSq + dSq;
}

void dcm_to_euler(float euler[3],float data[3][3] )  {
    euler[1] = asinf(-data[2][0]);

    if (fabs(euler[1] - 1.57079632679489661923) < 1.0e-3f) {
        euler[0] = 0.0f;
        euler[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler[0];

    } else if (fabs(euler[1] + 1.57079632679489661923) < 1.0e-3f) {
        euler[0] = 0.0f;
        euler[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler[0];

    } else {
        euler[0] = atan2f(data[2][1], data[2][2]);
        euler[2] = atan2f(data[1][0], data[0][0]);
    }

}

void M33_X( float* m1, float* m2,float* m3)
{int m=3;int n=1;int p=1;
     for(int i=0;i<3;++i)                 //叉乘运算
    {
        for(int j=0;j<3;++j)
        {
                for(int k=0;k<3;++k)
                {
                    (*(m3+i*p+j))+=(*(m1+i*n+k))*(*(m2+k*p+j));
             }
        }
    }
}

 void M33_Pluse( float in1[3][3], float in2[3][3],float out[3][3])
{u8 i,j,s;

out[0][0]=in1[0][0]*in2[0][0]+in1[0][1]*in2[1][0]+in1[0][2]*in2[2][0];
out[0][1]=in1[0][0]*in2[0][1]+in1[0][1]*in2[1][1]+in1[0][2]*in2[2][2];
out[0][2]=in1[0][0]*in2[0][2]+in1[0][1]*in2[1][2]+in1[0][2]*in2[2][2];

out[1][0]=in1[1][0]*in2[0][0]+in1[1][1]*in2[1][0]+in1[1][2]*in2[2][0];
out[1][1]=in1[1][0]*in2[0][1]+in1[1][1]*in2[1][1]+in1[1][2]*in2[2][2];
out[1][2]=in1[1][0]*in2[0][2]+in1[1][1]*in2[1][2]+in1[1][2]*in2[2][2];

out[2][0]=in1[2][0]*in2[0][0]+in1[2][1]*in2[1][0]+in1[2][2]*in2[2][0];
out[2][1]=in1[2][0]*in2[0][1]+in1[2][1]*in2[1][1]+in1[2][2]*in2[2][2];
out[2][2]=in1[2][0]*in2[0][2]+in1[2][1]*in2[1][2]+in1[2][2]*in2[2][2];
}

void M33_Pluse_2( float in1[3][3], float in2,float out[3][3])
{u8 i,j,s;
   for(i=0;i<3;i++)
 {
  for(j=0;j<3;j++)
  {

    out[i][j]=in1[i][j]*in2;

  }
    }
}

 void M33_add( float in1[3][3], float in2[3][3],float out[3][3])
{u8 i,j,s;
   for(i=0;i<3;i++)
 {
  for(j=0;j<3;j++)
  {

    out[i][j]=in1[i][j]+in2[i][j];

  }
    }
}

void M33_t(float in[3][3],float out[3][3]) //求姿态矩阵的转置
{
  int i,j;

  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
 out[i][j]=in[j][i];
}


void cal_ero_so3(float exp_att[3],float *err_so3){
u8 i,j;
float e_R[3];
float exp_angle[3]={0,0,0},ero_angle_px4[4];
float angle_now[3]={0,0,0};
float Q_control[2][4]={0};
float R_control_set[3][3],R_control_ero[3][3];
float R_control_now[3][3],R_control_now_t[3][3];
float yaw_dcm;

exp_angle[1]=exp_att[PITr];
exp_angle[0]=exp_att[ROLr];
exp_angle[2]=exp_att[YAWr];

float sp = sin(exp_angle[0]*0.01745);
float cp = cos(exp_angle[0]*0.01745);
float sr = sin(exp_angle[1]*0.01745);
float cr = cos(exp_angle[1]*0.01745);
float sy = sin(exp_angle[2]*0.01745);
float cy = cos(exp_angle[2]*0.01745);

R_control_set[0][0] = cp * cy;
R_control_set[0][1] = (sr * sp * cy) - (cr * sy);
R_control_set[0][2] = (cr * sp * cy) + (sr * sy);
R_control_set[1][0] = cp * sy;
R_control_set[1][1] = (sr * sp * sy) + (cr * cy);
R_control_set[1][2] = (cr * sp * sy) - (sr * cy);
R_control_set[2][0] = -sp;
R_control_set[2][1] = sr * cp;
R_control_set[2][2] = cr * cp;

//q_to_dcm(q_nav,R_control_now);
for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
        R_control_now[i][j]=vmc_all.Rn_b[i][j];
/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
float R_z[3]={R_control_now[0][2],R_control_now[1][2],R_control_now[2][2]};
float R_sp_z[3]={R_control_set[0][2],R_control_set[1][2],R_control_set[2][2]};
///* axis and sin(angle) of desired rotation */
float R_x_z[3];
M33_t(R_control_now,R_control_now_t);
R_x_z[0]=R_z[1]*R_sp_z[2]-R_z[2]*R_sp_z[1];
R_x_z[1]=R_z[2]*R_sp_z[0]-R_z[0]*R_sp_z[2];
R_x_z[2]=R_z[0]*R_sp_z[1]-R_z[1]*R_sp_z[0];

e_R[0]=R_control_now_t[0][0]*R_x_z[0]+R_control_now_t[0][1]*R_x_z[1]+R_control_now_t[0][2]*R_x_z[2];
e_R[1]=R_control_now_t[1][0]*R_x_z[0]+R_control_now_t[1][1]*R_x_z[1]+R_control_now_t[1][2]*R_x_z[2];
e_R[2]=R_control_now_t[2][0]*R_x_z[0]+R_control_now_t[2][1]*R_x_z[1]+R_control_now_t[2][2]*R_x_z[2];
/* calculate weight for yaw control */
//float yaw_w = R_sp(2, 2) * R_sp(2, 2);
float yaw_w =R_control_set[2][2] * R_control_set[2][2];
/* calculate angle error */
//float e_R_z_sin = e_R.length();
float e_R_z_sin=sqrtf(e_R[0]*e_R[0]+e_R[1]*e_R[1]+e_R[2]*e_R[2]);
//float e_R_z_cos = R_z * R_sp_z;
float e_R_z_cos=R_z[0]*R_sp_z[0]+R_z[1]*R_sp_z[1]+R_z[2]*R_sp_z[2];
/* calculate rotation matrix after roll/pitch only rotation */
//math::Matrix<3, 3> R_rp;
float R_rp[3][3];
float e_R_cp[3][3]={0};

if (e_R_z_sin> 0.0f) {
    /*get axis-angle representation */
    //float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
    float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
    float e_R_z_axis[3] ;
    //Vector <3> e_R_z_axis = e_R / e_R_z_sin;
    e_R_z_axis[0]=e_R[0]/e_R_z_sin;
    e_R_z_axis[1]=e_R[1]/e_R_z_sin;
    e_R_z_axis[2]=e_R[2]/e_R_z_sin;
    //e_R = e_R_z_axis * e_R_z_angle;
    e_R[0] =e_R_z_axis[0] * e_R_z_angle;
    e_R[1] =e_R_z_axis[1] * e_R_z_angle;
    e_R[2] =e_R_z_axis[2] * e_R_z_angle;
    /*cross product matrix for e_R_axis */

    e_R_cp[0][1] = -e_R_z_axis[2];
    e_R_cp[0][2] = e_R_z_axis[1];
    e_R_cp[1][0] = e_R_z_axis[2];
    e_R_cp[1][2] = -e_R_z_axis[0];
    e_R_cp[2][0] = -e_R_z_axis[1];
    e_R_cp[2][1] = e_R_z_axis[0];
    /*rotation matrix for roll/pitch only rotation */
    //R_control_ero= R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos)
    float temp1[3][3]={0};
    M33_Pluse(e_R_cp,e_R_cp,temp1);
    float temp2[3][3]={0};
    M33_Pluse_2 (temp1,1.0f - e_R_z_cos,temp2);
    float temp3[3][3]={0};
    M33_Pluse_2 (e_R_cp,e_R_z_sin,temp3);
    float temp4[3][3]={0};
    M33_add (temp2,temp3,temp4);
    float temp5[3][3]={0};
    float _I[3][3]={1,0,0,0,1,0,0,0,1};
    M33_add (temp4,_I,temp5);
    M33_Pluse (R_control_now,temp5,R_rp);
 } else {
    /*zero roll/pitch rotation */ // R_rp = R;
    for(i=0;i<3;i++)for(j=0;j<3;j++)R_rp[i][j]=R_control_now[i][j];
 }

    /* R_rp and R_sp has the same Z axis, calculate yaw error
math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w; */
    float  R_sp_x[3];
    R_sp_x[0]=R_control_set[0][0];
    R_sp_x[1]=R_control_set[1][0];
    R_sp_x[2]=R_control_set[2][0];
    float  R_rp_x[3];
    R_rp_x[0]=R_rp[0][0];
    R_rp_x[1]=R_rp[1][0];
    R_rp_x[2]=R_rp[2][0];

    float R_rp_x_x_R_sp_x[3];
    M33_t(R_control_now,R_control_now_t);
    R_rp_x_x_R_sp_x[0]=R_rp_x[1]*R_sp_x[2]-R_rp_x[2]*R_sp_x[1];
    R_rp_x_x_R_sp_x[1]=R_rp_x[2]*R_sp_x[0]-R_rp_x[0]*R_sp_x[2];
    R_rp_x_x_R_sp_x[2]=R_rp_x[0]*R_sp_x[1]-R_rp_x[1]*R_sp_x[0];
    e_R[2] = atan2f(R_rp_x_x_R_sp_x[0] * R_sp_z[0]+R_rp_x_x_R_sp_x[1] * R_sp_z[1]+R_rp_x_x_R_sp_x[2] * R_sp_z[2],
    R_rp_x[0] * R_sp_x[0]+R_rp_x[1] * R_sp_x[1]+R_rp_x[2] * R_sp_x[2]) * yaw_w;
//    if (e_R_z_cos < 0.0f) {
//        /* for large thrust vector rotations use another rotation method:
//         * calculate angle and axis for R -> R_sp rotation directly */
//        math::Quaternion q;
//        q.from_dcm(R.transposed() * R_sp);
//        math::Vector<3> e_R_d = q.imag();
//        e_R_d.normalize();
//        e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));
//        /* use fusion of Z axis based rotation and direct rotation */
//        float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
//        e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
//    }

    *(err_so3+0)=ero_angle_px4[0]=e_R[0]*57.3;//x
    *(err_so3+1)=ero_angle_px4[1]=-e_R[1]*57.3;//y
    *(err_so3+2)=ero_angle_px4[2]=e_R[2]*57.3;//z
}

