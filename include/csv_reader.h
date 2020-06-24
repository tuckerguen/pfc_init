#ifndef CSV_READER_H
#define CSV_READER_H

#include <string>
#include <vector>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace std;

/*
 * A class to read data from a csv file.
 */
class CSVReader
{
	string fileName;
	string delimeter;
 
public:
	CSVReader(string filename, string delm = ",") :
			fileName(filename), delimeter(delm)
	{ }
 
	// Function to fetch data from a CSV File
	vector<vector<string> > getData();
};

#endif