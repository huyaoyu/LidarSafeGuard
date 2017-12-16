
#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <boost/tokenizer.hpp>

#include <Eigen/Dense>

#include <LidarMask.hpp>
#include <LidarSafeGuard.hpp>

#ifndef M_PI
#define MY_PI (3.141592653589793238462643383279502884197169399375105820974944592307816406286)
#else
#define MY_PI M_PI
#endif

#define PRINT_ARRAY(array, len) \
	std::cout << #array << " = [" << std::endl;\
	for ( int iArray = 0; iArray < len; ++iArray )\
	{\
		std::cout << iArray << ", " << array[iArray] << std::endl;\
	}\
	std::cout << "]" << std::endl;\

Eigen::MatrixXd* read_csv_into_Eigen_matrix(std::string& fn)
{
	using namespace std;
	using namespace boost;
	using Eigen::MatrixXd;

	// Create the input stream.
	ifstream in(fn.c_str());

	if ( !in.is_open() )
	{
		cout << "Failed to open file " << fn << endl;
		return LSG_NULL;
	}

	typedef tokenizer< escaped_list_separator<char> > Tokenizer;

	vector< vector<double>* > vecOuter;
	double temp = 0.0;
	string line;
	stringstream ss;
	size_t firstColumnSize = 0;

	while ( getline(in, line) )
	{
		Tokenizer tok(line, escaped_list_separator<char>('\\', ',', '\"'));
		Tokenizer::iterator itr;

		// Create a new vector.
		vector<double>* row = new vector<double>;

		for ( itr = tok.begin(); itr != tok.end(); ++itr )
		{
			ss.clear();
			ss.str(*itr);
			ss >> temp;

			row->push_back(temp);
		}

		if ( row->size() < 1 )
		{
			delete row;
		}
		else
		{
			if ( 0 == firstColumnSize )
			{
				firstColumnSize = row->size();
			}
			else
			{
				if ( row->size() != firstColumnSize )
				{
					// This should be an error.
					// Just warn the user.
					cout << "WARNING: Wrong column size! firstColumnSize = "
						 << firstColumnSize << ". The current row ("
						 << vecOuter.size() <<", 0 based) has "
						 << row->size() << " columns." << endl;
				}
			}

			vecOuter.push_back(row);
		}
	}

	// Create the Eigen matrix.
	MatrixXd* matrix = new MatrixXd(vecOuter.size(), firstColumnSize);

	// Simple test.
	vector< vector<double>* >::iterator iterOuter;
	vector<double>::iterator iterInner;

	int idxRow = 0, idxCol = 0;

	cout << "Show contents." << endl;
	cout << "The size of vecOuter is " << vecOuter.size() << endl;

	for ( iterOuter = vecOuter.begin(); iterOuter != vecOuter.end(); iterOuter++ )
	{
		idxCol = 0;

		for ( iterInner = (*iterOuter)->begin(); iterInner != (*iterOuter)->end(); iterInner++ )
		{
			(*matrix)(idxRow, idxCol) = *iterInner;
			idxCol++;
		}

		idxRow++;
	}

//	cout << matrix << endl;

	// Clean up.

	for ( iterOuter = vecOuter.begin(); iterOuter != vecOuter.end(); iterOuter++ )
	{
		delete *iterOuter;
	}

	return matrix;
}

int test_read_csv_fiel_into_Eigen_matrix(void)
{
	std::string fn = "/home/yaoyu/SourceCodes/CMU/FieldRobotics/LidarSafeGuard/Debug/RangesWithNoise.csv";

	Eigen::MatrixXd* matrix = LSG_NULL;

	matrix = read_csv_into_Eigen_matrix(fn);

	if ( LSG_NULL == matrix )
	{
		std::cout << "Error!" <<std::endl;
		return -1;
	}

	std::cout << *matrix << std::endl;
	std::cout << "Done." << std::endl;

	delete matrix;

	return 0;
}

int test_naive_situation(void)
{
	// Show some information.
	std::cout << "test_naive_situation." << std::endl;

	// Dummy test data.
	const int N = 360;

	DECLARE_ALLOC_ARRAY(RP::real, angles, N);
	DECLARE_ALLOC_ARRAY(RP::real, ranges, N);
	DECLARE_ALLOC_ARRAY(int, mask, N);

	RP::real angleMin =  0.0;
	RP::real angleMax =  2.0*MY_PI;
	RP::real angleInc = ( angleMax - angleMin ) / N;

	RP::real r   = 1.0;
	RP::real ecc = 0.9;

	RP::real a = 1.0, a2 = a * 2.0;
	RP::real b = 0.0, bTemp = -2.0 * fabs(ecc);
	RP::real c = ecc * ecc - r * r;

	for ( int i=0; i<N; ++i )
	{
		angles[i] = angleMin + (i+1) * angleInc;

		b = bTemp * cos( MY_PI - angles[i] );

		ranges[i] = ( -b + sqrt( b*b - 4*a*c ) ) / a2;
//    	std::cout << angles[i] << ", " << ranges[i] << std::endl;

		mask[i] = 0;
	}

	// Test LidarMask.
	RP::LidarMask LM(angleMin, angleMax);
	RP::LidarMask::AngleSegment_t AS;
	AS.angle0 = 0;
	AS.angle1 = MY_PI / 4.0;
	LM.copy_push_segment(AS);

	AS.angle0 = MY_PI;
	AS.angle1 = MY_PI * 1.5;
	LM.copy_push_segment(AS);

	LM.put_mask(angles, N, mask);
//    PRINT_ARRAY(mask, N);

	RP::real radiusMean = 0.0;
	RP::real radiusStd  = 0.0;

	RP::InCylinderSafeGuard icsg("RPLIDARSafeGuard", ecc, 0.0);
	icsg.set_mask(&LM);
	icsg.copy_data( angles, ranges, N );

	RP::real ratioInf  = icsg.get_inf_ratio();
	RP::real ratioMask = icsg.get_mask_ratio();
	RP::LidarSafeGuard::SafetyFlag_t safeFlag;

	icsg.verify(1.0, 0.1*1.0, 0.1*1.0, &safeFlag);
	radiusMean = icsg.get_radius_mean();
	radiusStd  = icsg.get_radius_std();

	std::cout << "radiusMean =  " << radiusMean << ", "
			  << "radiusStd = " << radiusStd << "." << std::endl;
	std::cout << "ratioInf = " << ratioInf << ", "
			  << "ratioMask = " << ratioMask << "." << std::endl;

	if ( RP::LidarSafeGuard::FLAG_SAFE == safeFlag )
	{
		std::cout << "Safe." << std::endl;
	}
	else
	{
		std::cout << "Unsafe." << std::endl;
	}

	FREE_ARRAY(mask);
	FREE_ARRAY(ranges);
	FREE_ARRAY(angles);

	return 0;
}

int test_perfect_circle_with_noise(void)
{
	// Show some information.
	std::cout << __func__ << std::endl;

	// This is r = 1.0, ecc = 0.9 test data.
//	std::string fn = "/home/yaoyu/SourceCodes/CMU/FieldRobotics/LidarSafeGuard/Debug/RangesWithNoise_r1.00_ecc0.90_noiseAmp0.05.csv";
	std::string fn = "/home/huyaoyu/SourceCode/CMU/FieldRobotics/LidarSafeGuard/Debug/RangesWithNoise_r1.00_ecc0.90_noiseAmp0.10.csv";

	Eigen::MatrixXd* matrix = LSG_NULL;

	matrix = read_csv_into_Eigen_matrix(fn);

	if ( LSG_NULL == matrix )
	{
		std::cout << "Error!" <<std::endl;
		return -1;
	}

	// Have a valid Eigen matrix now.

	const int N = matrix->rows();

	DECLARE_ALLOC_ARRAY(RP::real, angles, N);
	DECLARE_ALLOC_ARRAY(RP::real, ranges, N);
	DECLARE_ALLOC_ARRAY(int, mask, N);

	RP::real ecc = 0.9;

	for ( int i=0; i<N; ++i )
	{
		angles[i] = (*matrix)(i, 0);
		ranges[i] = (*matrix)(i, 1);
		mask[i]   = 0;
	}

	RP::real radiusMean = 0.0;
	RP::real radiusStd  = 0.0;

	RP::InCylinderSafeGuard icsg("RPLIDARSafeGuard", ecc, 0.0);
	icsg.copy_data( angles, ranges, N );

	RP::real ratioInf  = icsg.get_inf_ratio();
	RP::real ratioMask = icsg.get_mask_ratio();
	RP::LidarSafeGuard::SafetyFlag_t safeFlag;

	icsg.verify(1.0, 0.1*1.0, 0.1*1.0, &safeFlag);
	radiusMean = icsg.get_radius_mean();
	radiusStd  = icsg.get_radius_std();

	std::cout << "radiusMean =  " << radiusMean << ", "
			  << "radiusStd = " << radiusStd << "." << std::endl;
	std::cout << "ratioInf = " << ratioInf << ", "
			  << "ratioMask = " << ratioMask << "." << std::endl;

	if ( RP::LidarSafeGuard::FLAG_SAFE == safeFlag )
	{
		std::cout << "Safe." << std::endl;
	}
	else
	{
		std::cout << "Unsafe." << std::endl;
	}

	delete matrix;

	FREE_ARRAY(mask);
	FREE_ARRAY(ranges);
	FREE_ARRAY(angles);

	return 0;
}

int test_data_on_launchrig_mask(void)
{
	// Show some information.
	std::cout << __func__ << std::endl;

	std::string fn = "/home/huyaoyu/SourceCode/CMU/FieldRobotics/LidarSafeGuard/Debug/20171204_DataOnTheLaunchRig.csv";

	Eigen::MatrixXd* matrix = LSG_NULL;

	matrix = read_csv_into_Eigen_matrix(fn);

	if ( LSG_NULL == matrix )
	{
		std::cout << "Error!" <<std::endl;
		return -1;
	}

	// Have a valid Eigen matrix now.

	const int N = matrix->rows();

	DECLARE_ALLOC_ARRAY(RP::real, angles, N);
	DECLARE_ALLOC_ARRAY(RP::real, ranges, N);
	DECLARE_ALLOC_ARRAY(int, mask, N);

	RP::real ecc = 0.0;

	for ( int i=0; i<N; ++i )
	{
		angles[i] = (*matrix)(i, 0);
		ranges[i] = (*matrix)(i, 1);
		mask[i]   = 0;
	}

	RP::real radiusMean = 0.0, targetRadius = 0.528;
	RP::real radiusStd  = 0.0;

	RP::LidarMask LM(-MY_PI, MY_PI);
	RP::LidarMask::AngleSegment_t AS;
	AS.angle0 = -MY_PI;
	AS.angle1 = -MY_PI / 2.0;
	LM.copy_push_segment(AS);

	AS.angle0 = MY_PI / 2.0;
	AS.angle1 = MY_PI;
	LM.copy_push_segment(AS);

	RP::InCylinderSafeGuard icsg("RPLIDARSafeGuard", ecc, 0.0);
	icsg.set_mask(&LM);
	icsg.copy_data( angles, ranges, N, 0, 1e-6 );

	RP::real ratioInf  = icsg.get_inf_ratio();
	RP::real ratioMask = icsg.get_mask_ratio();
	RP::LidarSafeGuard::SafetyFlag_t safeFlag;

	icsg.verify(targetRadius, 0.1*targetRadius, 0.1*targetRadius, &safeFlag);
	radiusMean = icsg.get_radius_mean();
	radiusStd  = icsg.get_radius_std();

	std::cout << "radiusMean =  " << radiusMean << ", "
			  << "radiusStd = " << radiusStd << "." << std::endl;
	std::cout << "ratioInf = " << ratioInf << ", "
			  << "ratioMask = " << ratioMask << "." << std::endl;

	if ( RP::LidarSafeGuard::FLAG_SAFE == safeFlag )
	{
		std::cout << "Safe." << std::endl;
	}
	else
	{
		std::cout << "Unsafe." << std::endl;
	}

	delete matrix;

	FREE_ARRAY(mask);
	FREE_ARRAY(ranges);
	FREE_ARRAY(angles);

	return 0;
}

int test_data_on_launchrig_no_mask(void)
{
	// Show some information.
	std::cout << __func__ << std::endl;

	std::string fn = "/home/huyaoyu/SourceCode/CMU/FieldRobotics/LidarSafeGuard/Debug/20171204_DataOnTheLaunchRig.csv";

	Eigen::MatrixXd* matrix = LSG_NULL;

	matrix = read_csv_into_Eigen_matrix(fn);

	if ( LSG_NULL == matrix )
	{
		std::cout << "Error!" <<std::endl;
		return -1;
	}

	// Have a valid Eigen matrix now.

	const int N = matrix->rows();

	DECLARE_ALLOC_ARRAY(RP::real, angles, N);
	DECLARE_ALLOC_ARRAY(RP::real, ranges, N);
	DECLARE_ALLOC_ARRAY(int, mask, N);

	RP::real ecc = 0.0;

	for ( int i=0; i<N; ++i )
	{
		angles[i] = (*matrix)(i, 0);
		ranges[i] = (*matrix)(i, 1);
		mask[i]   = 0;
	}

	RP::real radiusMean = 0.0, targetRadius = 0.528;
	RP::real radiusStd  = 0.0;

	RP::InCylinderSafeGuard icsg("RPLIDARSafeGuard", ecc, 0.0);
	icsg.copy_data( angles, ranges, N, 0, 1e-6 );

	RP::real ratioInf  = icsg.get_inf_ratio();
	RP::real ratioMask = icsg.get_mask_ratio();
	RP::LidarSafeGuard::SafetyFlag_t safeFlag;

	icsg.verify(targetRadius, 0.1*targetRadius, 0.1*targetRadius, &safeFlag);
	radiusMean = icsg.get_radius_mean();
	radiusStd  = icsg.get_radius_std();

	std::cout << "radiusMean =  " << radiusMean << ", "
			  << "radiusStd = " << radiusStd << "." << std::endl;
	std::cout << "ratioInf = " << ratioInf << ", "
			  << "ratioMask = " << ratioMask << "." << std::endl;

	if ( RP::LidarSafeGuard::FLAG_SAFE == safeFlag )
	{
		std::cout << "Safe." << std::endl;
	}
	else
	{
		std::cout << "Unsafe." << std::endl;
	}

	delete matrix;

	FREE_ARRAY(mask);
	FREE_ARRAY(ranges);
	FREE_ARRAY(angles);

	return 0;
}

int main(void)
{
    std::cout << "How are you doing these days C++？" << std::endl;

    std::cout << std::endl << "===" << std::endl << std::endl;

    test_naive_situation();

//    std::cout << std::endl << "===" << std::endl << std::endl;
//
//    test_read_csv_fiel_into_Eigen_matrix();

    std::cout << std::endl << "===" << std::endl << std::endl;

    test_perfect_circle_with_noise();

    std::cout << std::endl << "===" << std::endl << std::endl;

    test_data_on_launchrig_no_mask();

    std::cout << std::endl << "===" << std::endl << std::endl;

    test_data_on_launchrig_mask();

    return 0;
}
