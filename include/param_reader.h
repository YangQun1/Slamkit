/*
 * 参数读取类
 * 参考高翔的"一起做RGB-D SLAM"和csdn博客"https://blog.csdn.net/david_xtd/article/details/9320549"实现
 * Author: YangQun
 * Date: 2018/5/10
 * Last modified: 2018/5/10
 */

#ifndef PARAM_READER_H
#define PARAM_READER_H


#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <sstream>

using namespace std;

// 参数读取类
class ParameterReader
{
public:
	ParameterReader( string filename);

	template <class Type> Type getParam( const string& key );

private:
	map<string, string> data;
};

// 模板函数的定义,g++编译器不支持模板函数分离编译(头文件中声明,源文件中定义)
template <class Type>  
Type ParameterReader::getParam( const string& key )
{
	// 从map中查找key对应的value
	map<string, string>::iterator iter = data.find(key);
	if (iter == data.end()) {
		cerr<<"Parameter name "<<key<<" not found!"<<endl;
		throw "NOT_FOUND";
	}
	
	// 将string类型的value转成模板类型
	istringstream iss(iter->second);
	Type num;
	iss >> num;
	
	return num;
}

// 下面两个函数必须声明为inline,不然会出现multiple defination的错误
template<>  
inline bool ParameterReader::getParam<bool>( const string& key )  
{  
	// 从map中查找key对应的value
	map<string, string>::iterator iter = data.find(key);
	if (iter == data.end()) {
		cerr<<"Parameter name "<<key<<" not found!"<<endl;
		throw "NOT_FOUND";
	}
	
	string sup = iter->second; 
	bool b = true; 
	if( sup==std::string("FALSE") || sup==std::string("F") ||  
		sup==std::string("NO") || sup==std::string("N") ||  
		sup==std::string("0") || sup==std::string("NONE") ||
		sup==std::string("false")|| sup==std::string("Flase")) 
	{	
		b = false;
	}

	return b;  
}

template<>  
inline string ParameterReader::getParam<string>( const string& key )  
{  
	// 从map中查找key对应的value
	map<string, string>::iterator iter = data.find(key);
	if (iter == data.end()) {
		cerr<<"Parameter name "<<key<<" not found!"<<endl;
		throw "NOT_FOUND";
	}
	
	return iter->second;
}

#endif

