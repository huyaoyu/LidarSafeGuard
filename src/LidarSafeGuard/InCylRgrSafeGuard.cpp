/*
 * InCylRgrSafeGuard.cpp
 *
 *  Created on: Dec 17, 2017
 *      Author: huyaoyu
 */


#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "LidarSafeGuard.hpp"

using namespace RP;

const int  InCylRgrSafeGuard::MAX_ITERS    = 20;
const real InCylRgrSafeGuard::MAX_RESIDUAL = 1e-4;
const real InCylRgrSafeGuard::RELAX        = 0.5;

InCylRgrSafeGuard::InCylRgrSafeGuard(const char* name)
:
	InCylinderSafeGuard(name, 0.0, 0.0),
	flagDebug(0),
	mMaxIters(MAX_ITERS), mMaxResidual(MAX_RESIDUAL), mRelax(RELAX),
	mIters(0), mResidual(0.0)
{

}

InCylRgrSafeGuard::InCylRgrSafeGuard(std::string& name)
:
	InCylinderSafeGuard(name, 0.0, 0.0),
	flagDebug(0),
	mMaxIters(MAX_ITERS), mMaxResidual(MAX_RESIDUAL), mRelax(RELAX),
	mIters(0), mResidual(0.0)
{

}

InCylRgrSafeGuard::~InCylRgrSafeGuard(void)
{

}

void InCylRgrSafeGuard::set_flag_debug(void)
{
	flagDebug = 1;
}

void InCylRgrSafeGuard::clear_flag_debug(void)
{
	flagDebug = 0;
}

int InCylRgrSafeGuard::get_flag_debug(void)
{
	return flagDebug;
}

void InCylRgrSafeGuard::set_max_iters(int iters)
{
	if ( iters <= 0 )
	{
		std::cout << "Wrong iters. iters = " << iters << ". mMaxIters not updated." << std::endl;
	}
	else
	{
		mMaxIters = iters;
	}
}

void InCylRgrSafeGuard::set_max_residual(real res)
{
	if ( res <= 0 )
	{
		std::cout << "Wrong res. res = " << res << ". mMaxResidual not updated." << std::endl;
	}
	else
	{
		mMaxResidual = res;
	}
}

void InCylRgrSafeGuard::set_relax(real relax)
{
	if ( relax <= 0 )
	{
		std::cout << "Wrong relax. relax = " << relax << ". mRelax not updated." << std::endl;
	}
	else
	{
		if ( relax > 1.0 )
		{
			std::cout << "Super relax specified." << std::endl;
		}

		mRelax = relax;
	}
}

int InCylRgrSafeGuard::get_max_iters(void)
{
	return mMaxIters;
}

real InCylRgrSafeGuard::get_max_residual(void)
{
	return mMaxResidual;
}

real InCylRgrSafeGuard::get_relax(void)
{
	return mRelax;
}

int InCylRgrSafeGuard::get_iters(void)
{
	return mIters;
}

real InCylRgrSafeGuard::get_residual(void)
{
	return mResidual;
}

Status_t InCylRgrSafeGuard::verify(real r, real rLimit, real stdLimit, LidarSafeGuard::SafetyFlag_t* flag)
{
	Status_t sta = FAILED;

	mFlagSafe = LidarSafeGuard::FLAG_UNSAFE;

	if ( r <= 0.0 || rLimit <= 0.0 || stdLimit <= 0.0 )
	{
		std::cout << "Invalid r or stdLimit." << std::endl
				  << "r = " << r << ", "
				  << "rLimit = " << rLimit << ", "
				  << "stdLimit" << stdLimit << "." << std::endl;

		return sta;
	}

	if ( LSG_NULL == mAngles || LSG_NULL == mRanges || 0 >= mDataLen)
	{
		std::cout << "No data has been copied yet." << std::endl;
		return sta;
	}

	if ( mDataLen != mDataLenLast )
	{
		FREE_ARRAY(mCoorX_Ecc);
		FREE_ARRAY(mCoorY_Ecc);
		FREE_ARRAY(mCoorX);
		FREE_ARRAY(mCoorY);

		ALLOC_ARRAY(real, mCoorX_Ecc, mDataLen);
		ALLOC_ARRAY(real, mCoorY_Ecc, mDataLen);
		ALLOC_ARRAY(real, mCoorX, mDataLen);
		ALLOC_ARRAY(real, mCoorY, mDataLen);

		mDataLenLast = mDataLen;
	}

	calculate_coordinates_in_ecc_frame(mAngles, mRanges, mDataLen, mCoorX_Ecc, mCoorY_Ecc);

//	translate_coordinates(mCoorX_Ecc, mCoorY_Ecc, mDataLen, mEcc, mCoorX, mCoorY);

	//calculate_mean_radius(mCoorX, mCoorY, mDataLen, mRadiusMean, mRadiusStd);

	real x0 = 0.0, y0 = 0.0;

	mRadiusMean = r;

	solve_by_least_square(mCoorX_Ecc, mCoorY_Ecc, mDataLen,
			x0, y0, mRadiusMean, mIters, mResidual,
			mMaxIters, mMaxResidual, mRelax);

	// Calculate mRadiusStd.
	radius_std(mCoorX_Ecc, mCoorY_Ecc, mDataLen, x0, y0, mRadiusMean, mRadiusStd);

	if ( fabs(mRadiusMean - r) <= rLimit && mRadiusStd <= stdLimit )
	{
		mFlagSafe = FLAG_SAFE;
	}
	else
	{
		mFlagSafe = FLAG_UNSAFE;
	}

	if ( LSG_NULL != flag )
	{
		*flag = mFlagSafe;
	}

	sta = OK;

	return sta;
}

Status_t  InCylRgrSafeGuard::solve_by_least_square(const real* coorX, const real* coorY, int n,
		real& x0, real& y0, real& r, int& iters, real& res,
		int maxIters, real maxResidual, real relax)
{
	using Eigen::MatrixXf;
	using Eigen::VectorXf;

	Status_t sta = OK;

	if ( LSG_NULL == coorX || LSG_NULL == coorY )
	{
		std::cout << "NULL input arguments. "
				  << "coorX = " << coorX << ", "
				  << "coorY = " << coorY << "." << std::endl;

		return FAILED;
	}

	if ( n <= 0 )
	{
		std::cout << "Wrong n. n = " << n << "." << std::endl;
		return FAILED;
	}

	// Create Eigen matrices and vectors.

	MatrixXf A   = MatrixXf(n, 3); // Coefficient matrix.
	VectorXf RHS = VectorXf(n);    // Right-hand side
	VectorXf C   = VectorXf::Zero(3);
	VectorXf C1  = C;

	// Solve for x0, y0, r.

	x0 = 0.0;
	y0 = 0.0;

	res = 1.0 + maxResidual;
	iters = 0;

	real x = 0.0, y = 0.0;
	real g = 0.0, g4 = 0.0;

	while ( res > maxResidual && iters <= maxIters)
	{
		// Transfer all the data into Eigen matrices.
		for ( int i=0; i < n; ++i )
		{
			x = coorX[i];
			y = coorY[i];

			g  = (x - x0) * (x - x0) + (y - y0) * ( y - y0) - r * r;
			g4 = 4.0 * g;

			A(i, 0) = -g4 * (x - x0);
			A(i, 1) = -g4 * (y - y0);
			A(i, 2) = -g4 * r;

			RHS(i) = -g * g;
		}

		C = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RHS);

		x0 = x0 + relax * C(0);
		y0 = y0 + relax * C(1);
		r  = r  + relax * C(2);

		res = (C1 - C).norm();

		C1 = C;

		iters++;
	}

//	// Debug.
//	if ( 1 == flagDebug )
//	{
//		std::ofstream ofA("InCylRgrSafeGuard_A.dat");
//		if ( ofA.is_open() )
//		{
//			ofA << A;
//		}
//
//		ofA.close();
//
//		std::ofstream ofRHS("InCylRgrSafeGuard_RHS.dat");
//		if ( ofRHS.is_open() )
//		{
//			ofRHS << RHS;
//		}
//
//		ofRHS.close();
//	}

	return sta;
}

Status_t InCylRgrSafeGuard::radius_std(const real* coorX, const real* coorY, int n, real x0, real y0, real r, real& rStd)
{
	using namespace boost::accumulators;
	accumulator_set<real, stats<tag::variance> > acc;

	Status_t sta = OK;

	if ( LSG_NULL == coorX || LSG_NULL == coorY )
	{
		std::cout << "NULL input arguments. "
				  << "coorX = " << coorX << ", "
				  << "coorY = " << coorY << "." << std::endl;

		return FAILED;
	}

	if ( n <= 0 )
	{
		std::cout << "Wrong n. n = " << n << "." << std::endl;
		return FAILED;
	}

	real temp = 0.0;

	for ( int i=0; i < n; ++i )
	{
		temp = sqrt( (coorX[i] - x0) * (coorX[i] - x0) + (coorY[i] - y0) * (coorY[i] - y0) );

		acc(temp);
	}

	rStd = sqrt( double( variance( acc ) * n / ( n - 1 ) ) );

	return sta;
}

Status_t InCylRgrSafeGuard::min_x_y(const real* coorX, const real* coorY, int n, real& minX, real& minY)
{
	Status_t sta = OK;

	if ( LSG_NULL == coorX || LSG_NULL == coorY )
	{
		std::cout << "NULL input arguments. "
				  << "coorX = " << coorX << ", "
				  << "coorY = " << coorY << "." << std::endl;

		return FAILED;
	}

	if ( n <= 0 )
	{
		std::cout << "Wrong n. n = " << n << "." << std::endl;
		return FAILED;
	}

	minX = coorX[0];
	minY = coorY[0];

	for ( int i=0; i < n; ++i )
	{
		if ( coorX[i] < minX )
		{
			minX = coorX[i];
		}

		if ( coorY[i] < minY )
		{
			minY = coorY[i];
		}
	}

	return sta;
}
