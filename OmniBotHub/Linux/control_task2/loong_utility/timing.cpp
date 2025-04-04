/*=========== ***doc description @ yyp*** ===========
c风格隔离封装
=====================================================*/
#include"timing.h"
#include<chrono>
#include<iostream>

using namespace std;
using namespace chrono;


static const int Num=20;


namespace Timing{

class timingClass::impClass{
public:
	double rcd[Num],ave;
	int idx;
	system_clock::time_point tag;
};
	timingClass::timingClass():imp(*new impClass()){}
	timingClass::~timingClass(){delete &imp;}
	
	void timingClass::tic(){
		imp.tag=high_resolution_clock::now();
	}
	double timingClass::toc(){
		auto dt=high_resolution_clock::now()-imp.tag;
		return dt.count()/1e6;
	}
	void timingClass::toc20(){
		auto dt=high_resolution_clock::now() -imp.tag;
		imp.rcd[imp.idx]=dt.count()/1e6;
		imp.ave+=imp.rcd[imp.idx]/Num;
		imp.idx++; imp.idx%=Num;
		imp.ave-=imp.rcd[imp.idx]/Num;
		if(imp.idx==0){cout<<"计算平均耗时(ms)="<<imp.ave<<endl;}
	}
// ===========================
timingClass tim;
void tic(){tim.tic();}
double toc(){return tim.toc();}
void toc20(){tim.toc20();}

};