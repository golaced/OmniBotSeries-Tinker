/*=========== ***doc description @ yyp*** ===========
计时
=====================================================*/
#pragma once

namespace Timing{
class timingClass{
public:
	timingClass();
	~timingClass();
	void tic();
	double toc();
	void toc20();
private:
	class impClass;
	impClass &imp;
};
// ==========================
void tic();
double toc();
void toc20();

}//namespace
