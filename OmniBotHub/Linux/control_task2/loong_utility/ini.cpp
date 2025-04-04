/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

<map>查找不要用value=map[key]，这样会插入不存在的key
=====================================================*/
#include"ini.h"
#include<map>
#include<vector>
#include<iostream>
#include<sstream>

namespace Ini{
	//按token分割
	inline void splitString(string &str, vector<string> &substrings, char token){
		string tmp;
		stringstream ss(str);
		while(getline(ss, tmp, token)){
			substrings.push_back(tmp);
		}
	};
	//去除空格、tab
	inline void wipeWhite(string &str){
		size_t id=0;
		while((id=str.find(' ',id)) != string::npos){
			str.erase(id,1);
		}
		id=0;
		while((id=str.find('\t',id)) != string::npos){
			str.erase(id,1);
		}
	};
//==========================
class iniClass::impClass{
public:
	impClass(){};
	impClass(const string &fileName);
	map<string,string>mp;
	string file;

	bool open(const string &fileName,bool append);
	double operator[](const string &key);
	const string &getStr(const string &key);

	template<typename T,int n>
	void getArray(const string &key,T (&value)[n]){getArray(key,value,n);};
	template<typename T>
	void getArray(const string &key,T *value,int n);
};
	iniClass::impClass::impClass(const string &fileName){
		open(fileName,0);
	}
	bool iniClass::impClass::open(const string &fileName,bool append){
		ifstream f;
		f.open(fileName);
		if(!f.good()){
			stringstream ss;
			ss<<"=== 未找到"<<fileName<<" !===";
			throw runtime_error(ss.str());
			return 0;
		}
		file=fileName;
		if(!append){
			mp.clear();
		}
		string line;
		string preKey;
		while(getline(f,line)){
			size_t id=0;
			id=line.find(";");//分号是ini的注释标志
			if(id!=string::npos){
				line=line.substr(0,id);
			}
			wipeWhite(line);
			if(line.size()==0){
				continue;
			}
			id=line.find("=");
			if(id==string::npos){//无“=”号，视为上一行的换行
				if(preKey.size()>0){
					auto &value=mp[preKey];
					if(value[value.size()-1]!=',' && line[0]!=','){
						stringstream ss;
						ss<<"ini：跨行数组拼接，未发现逗号分割符！file="<<fileName<<"key="<<preKey;
						cout<<ss.str()<<endl;
						throw runtime_error(ss.str());
					}else{
						value+=line;
					}
				}
			}else{//有“=”号，新键值对
				string key,value;
				key.assign(line,0,id);
				value.assign(line,id+1,line.size()-id);
				mp.insert(pair<string,string>(key,value));
				preKey=key;
			}
		}
		// auto it=mp.begin();
			
		// 	for(int i=0;i<mp.size();i++){
		// 		cout<<it->first<<endl;
		// 		it++;
		// 	}
		return 1;
	}
	double iniClass::impClass::operator[](const string &key){
		auto it=mp.find(key);
		if(it!=mp.end()){
			try{
				return stod(it->second);
			}catch(...){
				stringstream ss;
				ss<<"\" ini:[\""<<key<<"\"] \" 值类型不匹配，file="<<file;
				cout<<ss.str()<<endl;
				throw runtime_error(ss.str());
			}
		}else{
			stringstream ss;
			ss<<"\" ini:[\""<<key<<"\"] \" 找不到key，file="<<file;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
	};
	const string &iniClass::impClass::getStr(const string &key){
		auto it=mp.find(key);
		if(it!=mp.end()){
			return it->second;
		}else{
			stringstream ss;
			ss<<"\" ini:[\""<<key<<"\"] \" 找不到key，file="<<file;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
	};
	template<typename T>
	void iniClass::impClass::getArray(const string &key,T *value,int n){
		auto it=mp.find(key);
		if(it!=mp.end()){
			std::vector<std::string> vs;
			splitString(it->second,vs,',');
			if(vs.size()<n){
				stringstream ss;
				ss<<"\"ini:getArray\" 数目不匹配，key="<<key<<"，file="<<file;
				cout<<ss.str()<<endl;
				throw runtime_error(ss.str());
			}
			try{
				for(int i=0;i<n;i++){
					value[i]=(T)stod(vs[i]);
				}
			}catch(...){
				stringstream ss;
				ss<<"\"ini:getArray\" 值类型不匹配，key="<<key<<"，file="<<file;
				cout<<ss.str()<<endl;
				throw runtime_error(ss.str());
			}
		}else{
			// cout<<"111  "<<key<<endl;
			// cout<<mp.size()<<endl;
			// auto it=mp.begin();
			
			// for(int i=0;i<mp.size();i++){
			// 	cout<<it->first<<endl;
			// 	it++;
			// 	perror("cc");
			// }
			// while(it!=mp.end()){
			// 	cout<<it->first;
			// 	it++;
			// }

			stringstream ss;
			ss<<"\"ini:getArray\" 找不到key="<<key<<"，file="<<file;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
	}
	//==========
	iniClass::iniClass():imp(*new impClass()){}
	iniClass::iniClass(const string &fileName):imp(*new impClass(fileName)){}
	bool iniClass::open(const string &fileName,bool append){return imp.open(fileName,append);}
	double iniClass::operator[](const string &key){
		return imp[key];
	};
	const string &iniClass::getStr(const string &key){
		return imp.getStr(key);
	};
	template<typename T>
	void iniClass::getArray(const string &key,T *value,int n){
		imp.getArray(key,value,n);
	}
	template void iniClass::getArray(const string &key,int *value,int n);
	template void iniClass::getArray(const string &key,float *value,int n);
	template void iniClass::getArray(const string &key,double *value,int n);

//=================================================
class ini000Class::imp2Class{
public:
	imp2Class();
	string file;
	ofstream fout2;//放在这里原因：在impClass构造函数中匹配“000.ini”目录
};
	ini000Class::imp2Class::imp2Class(){
		ifstream f;
		bool found=0;
	#ifdef _WIN32
		if(!found){
			file="../../config/000.ini";
			f.open(file);
			if(f.good()){
				fout2.open("../../zzz.txt", ios::trunc);//放最外面
				found=1;
			}
		}
		if(!found){
			file="../../000.ini";
			f.open(file);
			if(f.good()){
				fout2.open("../../zzz.txt", ios::trunc);//放最外面
				found=1;
			}
		}
	#endif
		if(!found){
			file="../config/000.ini";
			f.open(file);
			if(f.good()){
				fout2.open("../zzz.txt", ios::trunc);//放最外面
				found=1;
			}
		}
		if(!found){
			file="../000.ini";
			f.open(file);
			if(f.good()){
				fout2.open("../zzz.txt", ios::trunc);//放最外面
				found=1;
			}
		}
		if(!found){
			file="000.ini";
			f.open(file);
			if(f.good()){
				fout2.open("zzz.txt", ios::trunc);//放最外面
				found=1;
			}
		}
	}
	//=========================
	ini000Class::ini000Class():imp2(*new imp2Class()){
		imp.open(imp2.file,0);
	}
	ini000Class& ini000Class::instance(){
		static ini000Class singtn;
		return singtn;
	}
	ofstream &ini000Class::getFout(){
		return imp2.fout2;
	}
}//namespace

ofstream &fout=Ini::ini000Class::instance().getFout();
