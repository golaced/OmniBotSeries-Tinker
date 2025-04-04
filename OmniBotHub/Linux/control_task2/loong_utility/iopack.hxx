/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

测试环境
集成更易用的print系列
=====================================================*/
#include<sstream>
#include<iomanip>
#include"iopack.h"

namespace Ak12ERc7a{
template<typename Derived>
void eiPrint(ostream &out,const Eigen::MatrixBase<Derived> &m,bool enter){
	auto r=m.rows(),c=m.cols();
	//找最大宽度
	size_t width{0};
	For(r){
		for(int j=0;j!=c;++j){
			stringstream ss;
			if(abs(m(i,j))>1e-4){
				ss<<m(i,j);
				if(ss.str().length()>width){
					width=ss.str().length();
				}
			}
		}
	}
	//列向量行打印
	if(c==1){
		For(r){
			if(abs(m(i,0))<1e-4){out.width(width);out<<0;}
			else{out.width(width);out<<m(i,0);}
			if(i<r-1){out<<" ";}
		}
		out<<";\n";
	}else{
		For(r){
			for(int j=0;j!=c;++j){	
				if(abs(m(i,j))<1e-4){out.width(width);out<<0;}
				else{out.width(width);out<<m(i,j);}
				if(j<c-1){out<<" ";}
			}
			out<<";\n";
		}
	}
	if(enter){out<<endl;}
}
// =====================================
template<typename Derived>
void eiPrintWid(ostream &out,const Eigen::MatrixBase<Derived> &m,bool enter){
	auto r=m.rows(),c=m.cols();
	//找最大宽度
	size_t width{9};
	//列向量行打印
	if(c==1){
		For(r){
			if(abs(m(i,0))<1e-4){out.width(width);out<<0;}
			else{out.width(width);out<<fixed<<m(i,0);}
			if(i<r-1){out<<" ";}
		}
		out<<";\n";
	}else{
		For(r){
			for(int j=0;j!=c;++j){
				if(abs(m(i,j))<1e-4){out.width(width);out<<0;}
				// else{out.width(width);out<<fixed<<setprecision(3)<<m(i,j);}
				else{out.width(width);out<<fixed<<m(i,j);}
				if(j<c-1){out<<" ";}
			}
			out<<";\n";
		}
	}
	if(enter){out<<endl;}
}
}//namespace

// =================================
template<typename Derived>
void printEi(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrint(cout,m,enter);
}
template<typename Derived>
void printEi(int idx,const Eigen::MatrixBase<Derived> &m,bool enter){
	cout<<"【"<<idx<<"】：";
	Ak12ERc7a::eiPrint(cout,m,enter);
}
template<typename Derived>
void printEi(const string &tag,const Eigen::MatrixBase<Derived> &m,bool enter){
	cout<<"【"<<tag<<"】："<<endl;
	Ak12ERc7a::eiPrint(cout,m,enter);
}
template<typename Derived>
void fprintEi(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrint(fout,m,enter);
}
// ==============================
template<typename Derived>
void printEiWid(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrintWid(cout,m,enter);
}
template<typename Derived>
void fprintEiWid(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrintWid(fout,m,enter);
}