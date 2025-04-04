/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#pragma once
#include"eigen.h"

namespace Tvm{
class tvmClass{
public:
	tvmClass();
	~tvmClass();
	bool init(const string&soFile, int numIn, int numOut);
	void run();
	vecXf in,out;
private:
	class impClass;
	impClass &imp;
};
}//namespace
