/*
 * InCylinderSafeGuard.cpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  Source file of Class InCylinderSafeGuard.
 */

#include <iostream>
#include <math.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "LidarSafeGuard.hpp"

using namespace RP;

InCylinderSafeGuard::InCylinderSafeGuard(const char* name, real ecc, real phaseAngle)
: LidarSafeGuard::LidarSafeGuard(name),
  mEcc(ecc), mPhaseAngle(phaseAngle),
  mRadiusMean(-1.0), mRadiusStd(-1.0),
  mCoorX_Ecc(LSG_NULL), mCoorY_Ecc(LSG_NULL), mCoorX(LSG_NULL), mCoorY(LSG_NULL)
{

}

InCylinderSafeGuard::InCylinderSafeGuard(std::string& name, real ecc, real phaseAngle)
: LidarSafeGuard::LidarSafeGuard(name),
  mEcc(ecc), mPhaseAngle(phaseAngle),
  mRadiusMean(-1.0), mRadiusStd(-1.0),
  mCoorX_Ecc(LSG_NULL), mCoorY_Ecc(LSG_NULL), mCoorX(LSG_NULL), mCoorY(LSG_NULL)
{

}

InCylinderSafeGuard::~InCylinderSafeGuard(void)
{
	FREE_ARRAY(mCoorY);
	FREE_ARRAY(mCoorX);
	FREE_ARRAY(mCoorY_Ecc);
	FREE_ARRAY(mCoorX_Ecc);
}

void InCylinderSafeGuard::calculate_coordinates_in_ecc_frame(const real* angles, const real* ranges, int len, real* coorX, real* coorY)
{
	real angle = 0.0;
	real r     = 0.0;

	for ( int i=0; i < len; ++i )
	{
		r     = ranges[i];
		angle = angles[i];

		coorX[i] = r * cos(angle);
		coorY[i] = r * sin(angle);
//		std::cout << coorX[i] << ", " << coorY[i] << std::endl;
	}
}

void InCylinderSafeGuard::translate_coordinates(const real* coorXFrom, const real* coorYFrom, int len, real ecc, real* coorXTo, real* coorYTo)
{
	for ( int i=0; i < len; ++i )
	{
		coorXTo[i] = coorXFrom[i] + ecc;
		coorYTo[i] = coorYFrom[i];
	}
}

Status_t InCylinderSafeGuard::verify(real r, real rLimit, real stdLimit, LidarSafeGuard::SafetyFlag_t* flag)
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

	translate_coordinates(mCoorX_Ecc, mCoorY_Ecc, mDataLen, mEcc, mCoorX, mCoorY);

	calculate_mean_radius(mCoorX, mCoorY, mDataLen, mRadiusMean, mRadiusStd);

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

void InCylinderSafeGuard::calculate_mean_radius(const real* coorX, const real* coorY, int n, real& rMean, real& rStd)
{
	using namespace boost::accumulators;
	accumulator_set<real, stats<tag::mean, tag::variance> > acc;

	if ( LSG_NULL == coorX || LSG_NULL == coorY || n <= 0)
	{
		std::cout << "No data has been processed yet." << std::endl;
		rMean = -1.0;
		rStd  = -1.0;
		return;
	}

	real r = 0.0;

	for ( int i=0; i < n; ++i )
	{
		r = sqrt( coorX[i] * coorX[i] + coorY[i] * coorY[i] ); // Possible lose of accuracy!

		acc(r);
	}

	rMean = mean( acc );
	rStd  = sqrt( double( variance( acc ) * n / ( n - 1 ) ) );
}

real InCylinderSafeGuard::get_radius_mean(void)
{
	return mRadiusMean;
}

real InCylinderSafeGuard::get_radius_std(void)
{
	return mRadiusStd;
}
