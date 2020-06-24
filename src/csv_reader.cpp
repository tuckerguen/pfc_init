#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "csv_reader.h"
 
using namespace std;
 
/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
vector<vector<string> > CSVReader::getData()
{
	ifstream file(fileName);
    if (file.fail()){
        cout << "couldn't open file" << endl;
    }
	vector<vector<string> > dataList;
 
	string line = "";
	// Iterate through each line and split the content using delimeter
	while (getline(file, line))
	{
        cout << line << endl;
		vector<string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
	}
	// Close the File
	file.close();
 
	return dataList;
}