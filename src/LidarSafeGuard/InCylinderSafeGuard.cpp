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

Status_t InCylinderSafeGuard::verify(LidarSafeGuard::SafetyFlag_t* flag)
{
	Status_t sta = FAILED;

	mFlagSafe = LidarSafeGuard::FLAG_UNSAFE;

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

	if ( LSG_NULL != flag )
	{
		*flag = mFlagSafe;
	}

	sta = OK;

	return sta;
}

real InCylinderSafeGuard::get_mean_radius(void)
{
	using namespace boost::accumulators;
	accumulator_set<real, stats<tag::mean, tag::variance> > acc;

	mRadiusMean = -1.0;

	if ( LSG_NULL == mCoorX || LSG_NULL == mCoorY || mDataLen <= 0)
	{
		std::cout << "No data has been processed yet." << std::endl;
		return mRadiusMean;
	}

	real r = 0.0;

	for ( int i=0; i < mDataLen; ++i )
	{
		r = sqrt( mCoorX[i] * mCoorX[i] + mCoorY[i] * mCoorY[i] ); // Possible lose of accuracy!

		acc(r);
	}

	real tempMean = mean( acc );
	real tempStd  = sqrt( double( variance( acc ) * mDataLen / ( mDataLen - 1 ) ) );

	std::cout << "tempMean = " << tempMean << std::endl;
	std::cout << "tempStd  = " << tempStd  << std::endl;

	mRadiusMean = tempMean;
	mRadiusStd  = tempStd;

	return mRadiusMean;
}

