/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"tvm.h"

#include <dlpack/dlpack.h>
#include <tvm/runtime/module.h>
#include <tvm/runtime/packed_func.h>
#include <tvm/runtime/registry.h>

#include"iopack.h"

static const int dim=1;

namespace Tvm{
// impClass是tvmClass的内部实现类，负责与TVM库交互，加载共享库（.so 文件）并管理输入输出数据
class tvmClass::impClass{
public:
	impClass(tvmClass*omp);//指向tvmClass实例的引用，用于访问外部的输入输出数据
	bool init(const string&soFile, int numIn, int numOut);
	void step();
	tvmClass&omp;

	tvm::runtime::Module gmod;//表示加载的TVM模块（Module）
	tvm::runtime::PackedFunc setInput;//设置输入数据的函数
	tvm::runtime::PackedFunc getOutput;//获取输出数据的函数
	tvm::runtime::PackedFunc run;//执行模型推理的函数
	DLTensor *x,*y;
	// tvm::runtime::NDArray x,y;
	int numIn,numOut;//输入和输出的数量
	int memIn,memOut;//输入和输出数据的内存大小
};
	// impClass 的构造函数接受一个 tvmClass* 类型的指针，并将其存储在成员变量 omp 中，供类的其他方法使用
	tvmClass::impClass::impClass(tvmClass*omp):omp(*omp){
		
	}
	//init方法加载指定的.so文件，调用tvm::runtime::Module::LoadFromFile加载TVM模块（即经过编译的深度学习模型）
	bool tvmClass::impClass::init(const string&soFile, int numIn, int numOut){
		tvm::runtime::Module lib=tvm::runtime::Module::LoadFromFile(soFile);
		DLDevice dev{kDLCPU, 0};
		gmod=lib.GetFunction("default")(dev);//从模块中获取入口函数并将其应用到 CPU 设备上
		setInput=gmod.GetFunction("set_input");
		getOutput=gmod.GetFunction("get_output");
		run=gmod.GetFunction("run");
		this->numIn=numIn;
		this->numOut=numOut;
		memIn =numIn *sizeof(float);
		memOut=numOut*sizeof(float);
		omp.in.setZero(numIn);//初始化输入数据
		omp.out.setZero(numOut);//初始化输出数据
		
		int64_t shapeIn[dim]{numIn},
				shapeOut[dim]{numOut};

		TVMArrayAlloc(shapeIn, dim, kDLFloat, 32, 1, kDLCPU, 0, &x);//为输入数据分配内存，x为输入张量的指针
		TVMArrayAlloc(shapeOut,dim, kDLFloat, 32, 1, kDLCPU, 0, &y);//为输出数据分配内存，y为输出张量的指针

		// x=tvm::runtime::NDArray::Empty({numIn},  DLDataType{kDLFloat, 32, 1}, dev);
		// y=tvm::runtime::NDArray::Empty({numOut}, DLDataType{kDLFloat, 32, 1}, dev);
		return 1;
	}
	//step方法执行推理，将输入数据复制到输入张量中，调用run函数执行推理，并将输出数据复制到输出张量中
	void tvmClass::impClass::step(){
		TVMArrayCopyFromBytes(x,omp.in.data(),memIn);
		// For(numIn){
		// 	static_cast<float*>(x->data)[i]=omp.in[i];
		// }
		//调用setInput函数将输入数据传递给模型。"input0"是输入数据的名称，x是输入张量。
		setInput("input0", x);
		run();
		getOutput(0,y);
		TVMArrayCopyToBytes(y,omp.out.data(),memOut);//将输出数据从TVM张量y复制到 omp.out
		// For(numOut){
		// 	omp.out[i]=static_cast<float*>(y->data)[i];
		// }
	}
// =======================================
	//tvmClass 的构造函数创建了一个 impClass 的实例，并将其与 tvmClass 关联
	tvmClass::tvmClass():imp(*new impClass(this)){}
	//调用 impClass::init来初始化TVM模块，加载.so文件并配置输入输出
	bool tvmClass::init(const string&soFile, int numIn, int numOut){
		return imp.init(soFile,numIn,numOut);
	}
	// run方法：调用 impClass::step 来执行模型的推理步骤
	void tvmClass::run(){
		imp.step();
	}
	// 析构函数：释放 x 和 y 张量的内存，并销毁 imp 实例
	tvmClass::~tvmClass(){
		if(imp.x){TVMArrayFree(imp.x);}
		if(imp.y){TVMArrayFree(imp.y);}
		delete &imp;
	}
}//namespace
