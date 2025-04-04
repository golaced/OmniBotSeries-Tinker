/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#pragma once
#include"eigen.h"

namespace Thr{

template<class T>
class threadLoopClass{
public:
	static threadLoopClass& instance();
	~threadLoopClass();
	bool start(const string &name, int cpuId, bool(T::*fun)(), T&t);
private:
	threadLoopClass();
	threadLoopClass(const threadLoopClass&)=delete;
	threadLoopClass& operator=(const threadLoopClass&)=delete;
	
	class impClass;
	impClass &imp;
};
}//namespace


#include"thread_loop.hxx"