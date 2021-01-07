// module to read a calibration file stored line-by-line in CSV file format

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
typedef vector <double> record_t;
typedef vector <record_t> data_t;

// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.

istream& operator>>(istream& ins, record_t& record) {
	// make sure that the returned record contains only the stuff we read now
	record.clear();

	// read the entire line into a string (a CSV record is terminated by a newline)
	string line;
	getline(ins, line);

	// now we'll use a stringstream to separate the fields out of the line
	stringstream ss(line);
	string field;
	while (getline(ss, field, ',')) {
		// for each field we wish to convert it to a double
		// (since we require that the CSV contains nothing but floating-point values)
		stringstream fs(field);
		double f = 0.0; // (default value is 0.0)
		fs >> f;

		// add the newly-converted field to the end of the record
		record.push_back(f);
	}

	// Now we have read a single line, converted into a list of fields, converted the fields
	// from strings to doubles, and stored the results in the argument record, so
	// we just return the argument stream as required for this kind of input overload function.
	return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator>>(istream& ins, data_t& data) {
	// make sure that the returned data only contains the CSV data we read here
	data.clear();

	// For every record we can read from the file, append it to our resulting data
	record_t record;
	while (ins >> record) {
		data.push_back(record);
	}

	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;
}

//return of "true" means all is well
bool read_calibration_file(std::string fname, std::vector<Eigen::Vector2d> &xy_pixels_vec, std::vector<Eigen::Vector2d> &xy_targets_vec, std::vector<Eigen::Vector3d> &xyz_sled_vec ) {
	
		//open the trajectory file:
	ifstream infile(fname.c_str());
	if (!infile){ //The file couldn't be opened.
		cerr << "Error: file could not be opened; giving up" << endl;
		return false;
	}
	

	// Here is the data we want.
	data_t data;

	// Here is the file containing the data. Read it into data.
	infile >> data;

	// Complain if something went wrong.
	if (!infile.eof()) {
		cout << "error reading file!\n";
		return false;
	}

	infile.close();

	// Otherwise, list some basic information about the file.
	cout << "CSV file contains " << data.size() << " records.\n";

	unsigned min_record_size = data[0].size();
	unsigned max_record_size = 0;
	for (unsigned n = 0; n < data.size(); n++) {
		if (max_record_size < data[ n ].size())
			max_record_size = data[ n ].size();
		if (min_record_size > data[ n ].size())
			min_record_size = data[ n ].size();
	}
	if (max_record_size > 7) {
		cout<<"bad file"<<endl;
		cout << "The largest record has " << max_record_size << " fields.\n";
		return false;
	}
	if (min_record_size < 7) {
		cout<<"bad file"<<endl;
		cout << "The smallest record has " << min_record_size << " fields.\n";
		return false;
	}

	
	//the following args are passed in. Make sure they are empty
	xy_pixels_vec.clear();
	xy_targets_vec.clear();
	xyz_sled_vec.clear();
        Eigen::Vector2d xy_pixels,xy_targets;
        Eigen::Vector3d xyz_sled_vals;
    
	
	int nlines = data.size();
	for (int n = 0; n < nlines; n++) {
            xy_pixels(0) = data[n][0];
            xy_pixels(1) = data[n][1];
            xy_targets(0) = data[n][2];
            xy_targets(1) = data[n][3];
            xyz_sled_vals(0) = data[n][4];
            xyz_sled_vals(1) = data[n][5];
            xyz_sled_vals(2) = data[n][6];
        
            xy_pixels_vec.push_back(xy_pixels);
            xy_targets_vec.push_back(xy_targets);
            xyz_sled_vec.push_back(xyz_sled_vals);
	}
	return true;
}
