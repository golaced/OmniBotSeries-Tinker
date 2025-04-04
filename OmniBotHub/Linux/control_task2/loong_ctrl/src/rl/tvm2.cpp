/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

与tvm.cpp类似，也是用于与 TVM 深度学习框架交互进行推理运算。
区别在于，此代码处理了两个输入（numIn1 和 numIn2）而不是一个输入。这使得它适用于处理需要多个输入的模型（比如多输入神经网络）。
它定义了一个名为 tvm2Class 的类，通过其内部实现类 impClass 来管理模型的加载、输入输出数据的传递和推理执行

=====================*/

#include"tvm2.h"

#include <dlpack/dlpack.h>
#include <tvm/runtime/module.h>
#include <tvm/runtime/packed_func.h>
#include <tvm/runtime/registry.h>

#include"iopack.h"

static const int dim=1;

namespace Tvm{
class tvm2Class::impClass{
public:
	impClass(tvm2Class*omp);
	bool init(const string&soFile, int numIn1, int numIn2, int numOut);
	void step();
	tvm2Class&omp;

	tvm::runtime::Module gmod;
	tvm::runtime::PackedFunc setInput;
	tvm::runtime::PackedFunc getOutput;
	tvm::runtime::PackedFunc run;
	DLTensor *x1,*x2,*y;
	// tvm::runtime::NDArray x,y;
	int numIn1,numIn2,numOut;
	int memIn1,memIn2,memOut;
};
	tvm2Class::impClass::impClass(tvm2Class*omp):omp(*omp){
		
	}
	bool tvm2Class::impClass::init(const string&soFile, int numIn1, int numIn2, int numOut){
		tvm::runtime::Module lib=tvm::runtime::Module::LoadFromFile(soFile);
		DLDevice dev{kDLCPU, 0};
		gmod=lib.GetFunction("default")(dev);
		setInput=gmod.GetFunction("set_input");
		getOutput=gmod.GetFunction("get_output");
		// getOutput=gmod.GetFunction("get_output_info");
		run=gmod.GetFunction("run");
		this->numIn1=numIn1;
		this->numIn2=numIn2;
		this->numOut=numOut;
		memIn1 =numIn1 *sizeof(float);
		memIn2 =numIn2 *sizeof(float);
		memOut=numOut*sizeof(float);
		omp.in1.setZero(numIn1);
		omp.in2.setZero(numIn2);
		omp.out.setZero(numOut);
		
		int64_t shapeIn1[dim]{numIn1},shapeIn2[dim]{numIn2},
				shapeOut[2]{1,numOut};

		TVMArrayAlloc(shapeIn1, dim, kDLFloat, 32, 1, kDLCPU, 0, &x1);
		TVMArrayAlloc(shapeIn2, dim, kDLFloat, 32, 1, kDLCPU, 0, &x2);
		TVMArrayAlloc(shapeOut, 2, kDLFloat, 32, 1, kDLCPU, 0, &y);

		// x=tvm::runtime::NDArray::Empty({numIn},  DLDataType{kDLFloat, 32, 1}, dev);
		// y=tvm::runtime::NDArray::Empty({numOut}, DLDataType{kDLFloat, 32, 1}, dev);
		return 1;
	}
	void tvm2Class::impClass::step(){
		TVMArrayCopyFromBytes(x1,omp.in1.data(),memIn1);
		TVMArrayCopyFromBytes(x2,omp.in2.data(),memIn2);
		// For(numIn){
		// 	static_cast<float*>(x->data)[i]=omp.in[i];
		// }
		setInput("input0", x1);
		setInput("input1", x2);
		// print("q");
		run();
		// print("w");
		getOutput(0,y);
		// long long *a;
		// getOutput(*a);
		// print(a);
		// print("e");
		TVMArrayCopyToBytes(y,omp.out.data(),memOut);
		// print("r");
		// For(numOut){
		// 	omp.out[i]=static_cast<float*>(y->data)[i];
		// }
	}
// =======================================
	tvm2Class::tvm2Class():imp(*new impClass(this)){}
	bool tvm2Class::init(const string&soFile, int numIn1, int numIn2, int numOut){
		return imp.init(soFile,numIn1,numIn2,numOut);
	}
	void tvm2Class::run(){
		imp.step();
	}
	tvm2Class::~tvm2Class(){
		if(imp.x1){TVMArrayFree(imp.x1);}
		if(imp.x2){TVMArrayFree(imp.x2);}
		if(imp.y){TVMArrayFree(imp.y);}
		delete &imp;
	}
}//namespace
