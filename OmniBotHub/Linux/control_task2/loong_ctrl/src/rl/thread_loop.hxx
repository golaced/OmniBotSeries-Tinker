/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

腿位置线性预测，忽略pit、rol对惯量影响
=====================================================*/
#include"thread_loop.h"
#include<thread>
#include<chrono>
#include<atomic>
#include<sstream>
#include"iopack.h"

// #define DefLoopTiming

namespace Thr{

template<class T>
class threadLoopClass<T>::impClass{
public:
	void loop();
	bool (T::*runOnce)();
	T*t;
	string name;

	int cpuId;
	atomic<bool> isStarted{0};
};

	template<class T>
	void threadLoopClass<T>::impClass::loop(){
		cpu_set_t mask;
		CPU_ZERO(&mask);
		CPU_SET(cpuId,&mask);
		stringstream ss;
		if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask)<0){
			ss<<name<<"绑定cpu失败";;
			throw runtime_error(ss.str());
		}else{
			CPU_ZERO(&mask);
			if(pthread_getaffinity_np(pthread_self(),sizeof(mask),&mask)<0){
				ss<<name<<"获取绑定cpu失败";
				throw runtime_error(ss.str());
			}else{
				int cpuNum=8;
				For(cpuNum){
					if(CPU_ISSET(i,&mask)){printL(name,"绑定到cpu-",i);}
				}
			}
		}
		auto tag0=chrono::high_resolution_clock::now();
		while(isStarted){
			#ifdef DefLoopTiming
			auto tag1=chrono::high_resolution_clock::now();
			#endif
			(t->*runOnce)();
			//==平均耗时计算（20个点）==
			#ifdef DefLoopTiming
			static double rcd0[20]{},average0=0,rcd1[20]{},average1=0;
			static int idx=0;
			auto now=chrono::high_resolution_clock::now();
			auto dddt0=chrono::duration_cast<chrono::nanoseconds>(now-tag0);
			auto dddt1=chrono::duration_cast<chrono::nanoseconds>(now-tag1);
			rcd0[idx]=dddt0.count()/1e6;
			rcd1[idx]=dddt1.count()/1e6;
			average0+=rcd0[idx]/20;
			average1+=rcd1[idx]/20;
			idx++; idx%=20;
			average0-=rcd0[idx]/20;
			average1-=rcd1[idx]/20;
			if(idx==0){cout<<"循环平均耗时(ms)="<<average0<<"，计算平均耗时(ms)="<<average1<<endl;}
			tag0=now;
			#endif
		}
	}
//==================================================
	template<class T>
	threadLoopClass<T>& threadLoopClass<T>::instance(){
		static threadLoopClass<T> singleton;
		return singleton;
	}
	template<class T>
	threadLoopClass<T>::threadLoopClass():imp(*new impClass()){}
	template<class T>
	threadLoopClass<T>::~threadLoopClass(){
		imp.isStarted=0;
	}

	template<class T>
	bool threadLoopClass<T>::start(const string &name, int cpuId, bool(T::*fun)(), T&t){
		if(!imp.isStarted){
			imp.cpuId=cpuId;
			imp.runOnce=fun;
			imp.t=&t;
			imp.name=name;
			thread thd(&threadLoopClass<T>::impClass::loop,&(this->imp));
			imp.isStarted=1;
			thd.detach();
			return 1;
		}
		return 0;
	}
}//namespace

