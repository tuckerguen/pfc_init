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
		vector<string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
	}
	// Close the File
	file.close();
 
	return dataList;
}