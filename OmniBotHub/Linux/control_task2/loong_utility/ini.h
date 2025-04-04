/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

读取解析配置文件*.ini
字符匹配非常耗力，因此不要在实时循环使用，可以在进实时循环前先找变量赋值
iniClass为通用类，可用于打开任意ini文件
ini000Class子类，默认打开000.ini，通过静态单例特性全局可用

#include"ini.h"
=====================================================*/
#pragma once
#include<string>
#include<fstream>

using namespace std;

namespace Ini{
//==通用基类，用于打开任意ini文件===========
//字符匹配非常耗力，因此不要在实时循环使用
class iniClass{
public:
	iniClass();
	iniClass(const string &fileName);//等价于无参构造+open
	bool open(const string &fileName,bool append=0);//append=1用于打开文件并在原有基础上键值对
	double operator[](const string &key);//不要在实时循环使用
	const string &getStr(const string &key);//不要在实时循环使用
	template<typename T,int n>
	void getArray(const string &key,T (&value)[n]){getArray(key,value,n);};//不要在实时循环使用
	template<typename T>
	void getArray(const string &key,T *value,int n);//不要在实时循环使用
protected:
	class impClass;
	impClass&imp;
};
//==继承子类单例，默认打开000.ini，使用define ini简化代码============
//字符匹配非常耗力，因此不要在实时循环使用
class ini000Class:public iniClass{
public:
	static ini000Class& instance();
	ofstream &getFout();
private:
	ini000Class();
	ini000Class(const ini000Class&)=delete;
	ini000Class & operator=(const ini000Class&)=delete;
	bool open(const string &fileName,bool append=0)=delete;
	class imp2Class;
	imp2Class &imp2;
};

}//namespace

//尝试了多种方式，这是在保留通用性的基础上、能够跨域编译实现全局初始化的最小影响方法
#define ini Ini::ini000Class::instance()
extern ofstream &fout;
