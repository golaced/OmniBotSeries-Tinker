/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

测试环境
fout定义，集成更易用的print系列
=====================================================*/
#pragma once
#include<iostream>
#include"eigen.h"
#include"ini.h"


//==eigen 打印================
template<typename Derived>
void printEi(const Eigen::MatrixBase<Derived> &m,bool enter=0);
template<typename Derived>
extern void printEi(int idx,const Eigen::MatrixBase<Derived> &m,bool enter=0);
template<typename Derived>
extern void printEi(const string &tag,const Eigen::MatrixBase<Derived> &m,bool enter=0);
template<typename Derived>
extern void printEiWid(const Eigen::MatrixBase<Derived> &m,bool enter=0);
//==eigen log==========================
template<typename Derived>
extern void fprintEi(const Eigen::MatrixBase<Derived> &m,bool enter=0);
template<typename Derived>
extern void fprintEiWid(const Eigen::MatrixBase<Derived> &m,bool enter=0);
#include"iopack.hxx"
//==变参数打印================
template<typename T>
void print(const T& a){
	cout<<a<<endl;
};
// void print(const double& a){
// 	cout<<a<<endl;
// };
template<typename T1,typename... T2>
void print(const T1& a,const T2&...b){
	cout<<a<<"\t";
	print(b...);
};
// template<typename... T2>
// void print(const double& a,const T2&...b){
// 	if(abs(a)<1e-4){cout<<"0\t";}
// 	else{cout<<a<<"\t";}
// 	print(b...);
// };
//--连行无tab--
template<typename T>
void printL(const T& a){
	cout<<a<<endl;
};
template<typename T1,typename... T2>
void printL(const T1& a,const T2&...b){
	cout<<a;
	printL(b...);
};
//--数组
template<typename T,int n>
void print(const T(&a)[n]){
	for(int i=0;i<n-1;i++){
		if(abs(a[i])>1e-4){cout<<a[i]<<", ";}
		else{cout<<"0, ";}
	}
	if(abs(a[n-1])>1e-4){cout<<a[n-1]<<endl;}
	else{cout<<"0\n";}
}
inline void print(const char*c){//字符打印单独写，避免自动数组化模板推导
	cout<<c<<endl;
}
//==变参数log===========================
template<typename T>
void fprint(const T& a){
	fout<<a<<"\t;"<<endl;
}
template<typename T1,typename... T2>
void fprint(const T1& a,const T2&...b){
	fout<<a<<"\t";
	fprint(b...);
}
// template<typename... T2>
// void fprint(const double& a,const T2&...b){
// 	if(abs(a)<1e-4){fout<<"0\t";}
// 	else{fout<<a<<"\t";}
// 	fprint(b...);
// }
