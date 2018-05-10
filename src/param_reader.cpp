/*
 */

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <sstream>

#include "param_reader.h"

using namespace std;

ParameterReader::ParameterReader( string filename )
{
	ifstream fin( filename.c_str() );
	if (!fin) {
		cerr<<"parameter file does not exist."<<endl;
		return;
	}
	while( ! fin.eof() ) {
		string str;
		getline( fin, str );
		if (str[0] == '#')
		{
		// 以‘＃’开头的是注释
		continue;
		}

		int pos = str.find("=");
		if (pos == -1)
		continue;
		string key = str.substr( 0, pos );
		string value = str.substr( pos+1, str.length() );
		data[key] = value;

		if ( !fin.good() )
		break;
	}
}
