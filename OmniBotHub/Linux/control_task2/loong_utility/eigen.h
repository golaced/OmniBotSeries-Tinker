/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

简化Eigen类名，以及常用函数
// #include"eigen.h"
=====================================================*/
#pragma once
#include<Eigen/Dense>
#include"algorithms.h"

#define vec2d Eigen::Vector2d
#define vec3d Eigen::Vector3d
#define vec6d Eigen::Matrix<double,6,1>
#define vecXd Eigen::VectorXd
#define vecNd(r) Eigen::Matrix<double,r,1>

#define vec2f Eigen::Vector2f
#define vec3f Eigen::Vector3f
#define vec6f Eigen::Matrix<float,6,1>
#define vecXf Eigen::VectorXf
#define vecNf(r) Eigen::Matrix<float,r,1>

#define mat2d Eigen::Matrix2d
#define mat3d Eigen::Matrix3d
#define matXd Eigen::MatrixXd
#define matNd(r,c) Eigen::Matrix<double,r,c>

#define mat2f Eigen::Matrix2f
#define mat3f Eigen::Matrix3f
#define matXf Eigen::MatrixXf
#define matNf(r,c) Eigen::Matrix<float,r,c>

#define vec3(T) Eigen::Vector3<T>
#define mat3(T) Eigen::Matrix3<T>

#define quat4d Eigen::Quaterniond
#define quat4f Eigen::Quaternionf

#define aAxid  Eigen::AngleAxisd
#define aAxiXd(rol) Eigen::AngleAxisd(rol,Eigen::Vector3d::UnitX())
#define aAxiYd(pit) Eigen::AngleAxisd(pit,Eigen::Vector3d::UnitY())
#define aAxiZd(yaw) Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())

#define aAxif  Eigen::AngleAxisf
#define aAxiXf(rol) Eigen::AngleAxisf(rol,Eigen::Vector3f::UnitX())
#define aAxiYf(pit) Eigen::AngleAxisf(pit,Eigen::Vector3f::UnitY())
#define aAxiZf(yaw) Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ())

namespace Ei{
//==上下限区间==========
	inline void clip(vec3d &x,const vec3d &a, const vec3d &b){
		double*xx=x.data();
		const double*aa=a.data();
		const double*bb=b.data();
		For3{Alg::clip(xx[i],aa[i],bb[i]);}
	}
	inline void clip(vec3d &x,double lim){
		if(lim<0){lim=-lim;}
		For3{
			if(x[i]>lim){x[i]=lim;}
			else if(x[i]<-lim){x[i]=-lim;}
		}
	}
	inline void clip(vec3d &x,double a, double b){
		if(a<b){
			For3{
				if(x[i]>b){x[i]=b;}
				else if(x[i]<a){x[i]=a;}
			}
		}else{
			For3{
				if(x[i]>a){x[i]=a;}
				else if(x[i]<b){x[i]=b;}
			}
		}
	}
//==阀值=============
	inline void thresh(vec3d &x,double threshold){
		if(threshold<0){threshold=-threshold;}
		For3{
			if(x[i]>threshold){x[i]-=threshold;}
			else if(x[i]<-threshold){x[i]+=threshold;}
			else{x[i]=0;}
		}
	}
	inline vec3d threshed(vec3d &x,double threshold){
		if(threshold<0){threshold=-threshold;}
		For3{
			if(x[i]>threshold){x[i]-=threshold;}
			else if(x[i]<-threshold){x[i]+=threshold;}
			else{x[i]=0;}
		}
		return x;
	}
//==递增函数=============当cmd=tgt时return 1
	inline bool cmd2out1step(const vec3d &cmd,vec3d &tgt,const double &step){
		const double*c=cmd.data();
		double*t=tgt.data();
		bool isDone(1);
		for(int i=0;i<3;++i){
			isDone &=Alg::cmd2out1step(c[i],t[i],step);
		}
		return isDone;
	}
	inline bool cmd2out1step(const mat3d &R1,mat3d &R,double step){
		aAxid tmp(R1*R.transpose());
		double ang=tmp.angle();
		if(step<0){step=-step;}
		if(ang>step){ang=step;}
		if(ang<-step){ang=-step;}
		else{R=R1;return 1;}
		R=aAxid(ang,tmp.axis())*R;
		return 0;
	}
//==反对称矩阵===============
	inline mat3d skew(const vec3d &v){
		mat3d M;
		M<<.0,-v[2],v[1], v[2],.0,-v[0], -v[1],v[0],.0;
		return M;
	};
	inline void skew(const vec3d &v,mat3d &M){
		M<<.0,-v[2],v[1], v[2],.0,-v[0], -v[1],v[0],.0;
	};
//==四元数转rpy==============
	template<typename T1, typename T2>
	inline void wxyz2rpy(T1(&q)[4],T2(&rpy)[3]){
		rpy[0]=atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
		rpy[1]=asin(2*(q[0]*q[2]-q[3]*q[1]));
		rpy[2]=atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
	}
//==rpy转R==============
// eigen自带需要轴角乘3次，重写
	template<typename T>
	inline mat3(T) rpy2R(const T &rol,const T &pit,const T &yaw){
		mat3(T) R;
		T cx=cos(rol),cy=cos(pit),cz=cos(yaw);
		T sx=sin(rol),sy=sin(pit),sz=sin(yaw);
		R<< cz*cy,-sz*cx+cz*sy*sx, sz*sx+cz*sy*cx,
			sz*cy, cz*cx+sz*sy*sx,-cz*sx+sz*sy*cx,
			  -sy,          cy*sx,          cy*cx;
		return R;
	}
	inline mat3d rpy2R(const vec3d &rpy){
		return rpy2R(rpy[0],rpy[1],rpy[2]);
	}
	inline mat3f rpy2Rf(const vec3f &rpy){//double、float重载无法适配segment
		return rpy2R(rpy[0],rpy[1],rpy[2]);
	}
	template<typename T>
	inline mat3(T) rpy2R(T (&rpy)[3]){
		return rpy2R(rpy[0],rpy[1],rpy[2]);
	}
	template<typename T>
	inline mat3(T) Rx(const T &rol){
		mat3(T) R;
		T c=cos(rol),s=sin(rol);
		R<< 1, 0, 0,
			0, c,-s,
			0, s, c;
		return R;
	}
	template<typename T>
	inline mat3(T) Ry(const T &pit){
		mat3(T) R;
		T c=cos(pit),s=sin(pit);
		R<<  c, 0, s,
			 0, 1, 0,
			-s, 0, c;
		return R;
	}
	template<typename T>
	inline mat3(T) Rz(const T &yaw){
		mat3(T) R;
		T c=cos(yaw),s=sin(yaw);
		R<< c,-s, 0,
			s, c, 0,
			0, 0, 1;
		return R;
	}
//==2维==============
	inline mat2d Rz2d(const double &yaw){
		mat2d R;
		double c=cos(yaw),s=sin(yaw);
		R<<c,-s,
			s,c;
		return R;
	}
//==R转rpy==============
// eigen自带欧拉角范围不对，重写
	inline void R2rpy(const mat3d &R,double rpy[3]){
		rpy[0]=atan2(R(2,1),R(2,2));
		rpy[1]=asin(-R(2,0));
		rpy[2]=atan2(R(1,0),R(0,0));
	}
	inline vec3d R2rpy(const mat3d &R){
		vec3d rpy;
		rpy[0]=atan2(R(2,1),R(2,2));
		rpy[1]=asin(-R(2,0));
		rpy[2]=atan2(R(1,0),R(0,0));
		return rpy;
	}
//==伪逆===============
//（伪逆独立在pInv.h）

//==一阶滤波============
class filterOneClass{
public:
	filterOneClass(){};
	filterOneClass(double cutF){
		k=cutF*dt;
		Alg::clip(k,0,1);
		ks=k/50;
	};
	void init(double dt,double cutF,double ki=0,vec3d y0=vec3d::Zero()){
		this->dt=dt;
		y=y0;
		setCutF(cutF,ki);
	};
	//默认50
	void setCutF(double cutF,double ki=0){
		k=cutF*dt;
		Alg::clip(k,0,1);
		Alg::clip(ki,0,1);
		ks=ki*k/50;
	};
	const vec3d & filt(const vec3d &x){
		y+=k*(x-y)-ks*ySum;
		ySum+=y-x;
		return y;
	};
private:
	vec3d y=vec3d::Zero(), ySum=vec3d::Zero();
	double dt=0.001, k=50*dt, ks=k/50;
};
}//namespace
