/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常量定义，首字母大写
#include"nabo_nabo_config.h"

// 腰1 +头2 +臂4*2  +腿6*2 =23
// 0   1    3       11
=====================================================*/
#pragma once
#include<array>
//涉及数组大小，必须编译前给定
static const int NLegDof       =5;
static const int NMot          =NLegDof*2;
static const int MemMotD       =NMot*8;
static const int MemMotF       =NMot*4;
static const int NBaseDof      =6;
static const int NGenDof       =NBaseDof +NMot;
//其他常量
static const double HipY       =0.05;
static const double LenThigh   =0.14;
static const double LenShank   =0.14;
static const double BodyH      =0.35;
static const double MaxMotToq  =100;
static const double MaxFz      =500;

const int QdIdHip[2]{11,6};
const int QdIdKnee[2]{14,9};



namespace Nabo{
//==传入组====
struct cmdStruct{
	int key;
	double vx=0,vy=0,wz=0,zOff=0;
	//常用以上这几个，为保持接口一致性，特殊用途数据推荐在appStruct字节流中添加
};
struct sensorStruct{
	int cnt;
	double supP[3]{},supV[3]{};
	double rpy[3]{},gyr[3]{},acc[3]{};
	double j[NMot]{},w[NMot]{},t[NMot]{};
	int tmp[NMot]{};//温度
};
struct appStruct{
	// data不是每个plan都会用，自定义数据流，直接透传
	int dataLen=0;
	char data[1024]{};
};
//==传出组=====
struct ctrlStruct{
	bool enFlag=0;
	double j[NMot],w[NMot],t[NMot];
};
struct paramStruct{
	std::array<float,NMot> kp,kd;
};
struct infoStruct{
	char planName[32];
	// bodyP于世界系，其余均为操作系F（操作系F始终以机身为原点，始终以当前朝向为x，z竖直向上）
	float bodyP[3],bodyV2F[3];
	// data不是每个plan都会用，自定义数据流，直接透传
	int dataLen;
	char data[1024];
};
//==最终封装=========================================
struct inputStruct{
	cmdStruct cmd;
	sensorStruct sens;
	appStruct app;
};
struct outputStruct{
	ctrlStruct ctrl;
	paramStruct param;
	infoStruct info;
};

}//namespace