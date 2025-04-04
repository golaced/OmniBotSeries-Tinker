/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常用方法
// #include"algorithms.h"
=====================================================*/
#pragma once
#include<cmath>
using namespace std;

static const double Pi =3.14159265359;
static const double Pi2=6.28318530718;
static const double P2i=1.57079632679;

#define For2 for(int i(0);i!=2;++i)
#define For3 for(int i(0);i!=3;++i)
#define For4 for(int i(0);i!=4;++i)
#define For6 for(int i(0);i!=6;++i)
#define For(x) for(int i(0);i!=x;++i)

namespace Alg{
//==符号函数======================
	inline int sign(const int &a){return a<0?-1:1;}
	inline int sign(const float &a){return a<0?-1:1;}
	inline int sign(const double &a){return a<0?-1:1;}
//==双曲正割，钟形=================
	inline double sech(const float &x){return 2/(exp(x)+exp(-x));}
	inline double sech(const double &x){return 2/(exp(x)+exp(-x));}
//==高斯函数，钟形，高阶,更收缩=======
	inline double gause(const float &x){return exp(-x*x);}
	inline double gause(const double &x){return exp(-x*x);}
//==线性插值映射到0~1====================
	inline double zero2one(const float low,const float up,const float x);
	inline double zero2one(const double low,const double up,const double x);
//==上下限区间，触发过边界return 1====================
	template<typename T1,typename T2,typename T3>
	bool clip(T1&x, const T2&a,const T3&b);
	template<typename T1,typename T2>
	bool clip(T1&x, T2 lim);
//==递增函数=============当cmd=tgt时return 1
	inline bool cmd2out1step(const double &cmd,double &tgt,double step){
		if(step<0){step=-step;}
		if(tgt-cmd<=-step){tgt+=step;return 0;}
		else if(tgt-cmd>=step){tgt-=step;return 0;}
		else{tgt=cmd;return 1;}
	}
//==阀值=============
	inline void thresh(double &x,double threshold){
		if(threshold<0){threshold=-threshold;}
		if(x>threshold){x-=threshold;}
		else if(x<-threshold){x+=threshold;}
		else{x=0;}
	}
	inline double threshed(double x,double threshold){
		thresh(x,threshold);
		return x;
	}
//==双斜率折线============
	inline void grind(double &x,const double &threshold,const double &k,const double &k2=1){
		if(x>threshold){x=k2*(x-threshold)+k*threshold;}
		else if(x<-threshold){x=k2*(x+threshold)-k*threshold;}
		else{x*=k;}
	}
	inline double grinded(double x,const double &threshold,const double &k,const double &k2=1){
		grind(x,threshold,k,k2);
		return x;
	}
//==一阶滤波 class============
class filterOneClass{
public:
	filterOneClass();
	filterOneClass(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1);
	double init(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1);
	double reset(double cutF,double ki=0,double y0=0);
	double setBase(double y0=0);
	void setCutF(double cutF,double ki=0);
	const double &filt(const double &x);
private:
	double dt;
	double y,errSum;
	double k,ks,maxStep;
};
//==二阶低通滤波 class============
class filterTwoClass{
public:
	filterTwoClass();
	filterTwoClass(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1,double damp2=1.414,double maxAcc=-1);
	double init(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1,double damp2=1.414,double maxAcc=-1);
	double reset(double cutF,double ki=0,double y0=0);
	double setBase(double y0=0);
	void setCutF(double cutF,double ki=0,double damp2=1.14);
	void setDamp2(double damp2);
	void setMax(double maxDerivate=-1,double maxAcc=-1);
	const double &filt(const double &x);
	double getV();
private:
	double dt;
	double y,yd,ydd;
	double errSum,ks,maxV,maxAcc;
	double w,dampW2,ww;//w=2*Pi*截止频率
};
}//namespace