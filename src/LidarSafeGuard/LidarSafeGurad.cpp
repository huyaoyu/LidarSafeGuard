/*
 * LidarSafeGurad.cpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  Source file of Class LidarSafeGuard.
 */

#include "LidarSafeGuard.hpp"
#include <math.h>

using namespace RP;

LidarSafeGuard::LidarSafeGuard(const char* name)
: mName(name), mFlagSafe(LidarSafeGuard::FLAG_UNSAFE),
  mLM(LSG_NULL), mMask(LSG_NULL), mFlagNewMask(0), mMaskRatio(0.0),
  mInfRatio(0.0),
  mAngles(LSG_NULL), mRanges(LSG_NULL), mDataLen(0), mDataLenLast(0), mDataNLast(0)
{
	// Do nothing for now.
}

LidarSafeGuard::LidarSafeGuard(std::string& name)
: mName(name), mFlagSafe(LidarSafeGuard::FLAG_UNSAFE),
  mLM(LSG_NULL), mMask(LSG_NULL), mFlagNewMask(0), mMaskRatio(0.0),
  mInfRatio(0.0),
  mAngles(LSG_NULL), mRanges(LSG_NULL), mDataLen(0), mDataLenLast(0), mDataNLast(0)
{
	// Do nothing for now.
}

LidarSafeGuard::~LidarSafeGuard(void)
{
	FREE_ARRAY(mMask);
	FREE_ARRAY(mRanges);
	FREE_ARRAY(mAngles);
	mDataLenLast = 0;
	mDataLen = 0;
}

LidarSafeGuard::SafetyFlag_t LidarSafeGuard::get_safety_flag(void)
{
	return mFlagSafe;
}

std::string& LidarSafeGuard::get_name(void)
{
	return mName;
}

void LidarSafeGuard::set_mask(LidarMask* LM)
{
	mLM = LM;

	mFlagNewMask = 1;
}

LidarMask* LidarSafeGuard::get_mask(void)
{
	return mLM;
}

real LidarSafeGuard::get_inf_ratio(void)
{
	return mInfRatio;
}

real LidarSafeGuard::get_mask_ratio(void)
{
	if ( LSG_NULL != mLM )
	{
		if ( 0 == mFlagNewMask )
		{
			return mMaskRatio;
		}
		else
		{
			std::cout << "The mask has not been put. 0.0 will be returned." << std::endl;
			return real(0.0);
		}
	}
	else
	{
		return real(0.0);
	}
}

Status_t LidarSafeGuard::copy_data(const real* angles, const real* ranges, int n, int forceUpdateMask, real rangeLowLimit)
{
	Status_t sta = OK;

	if ( LSG_NULL == angles && LSG_NULL == ranges )
	{
		std::cout << "angles or ranges argument is NULL!" << std::endl;
		return FAILED;
	}

	if ( n <= 0 )
	{
		std::cout << "n should be positive. n = " << n << "." << std::endl;
		return FAILED;
	}

	if ( mDataNLast != n )
	{
		// Allocate memory.
		FREE_ARRAY(mAngles);
		FREE_ARRAY(mRanges);

		ALLOC_ARRAY(real, mAngles, n);
		ALLOC_ARRAY(real, mRanges, n);
	}

	if ( 1 == forceUpdateMask )
	{
		mFlagNewMask = 1;
	}

	if ( LSG_NULL != mLM )
	{
		if ( mDataNLast != n || LSG_NULL == mMask)
		{
			FREE_ARRAY(mMask);
			ALLOC_ARRAY(int, mMask, n)

			mFlagNewMask = 1;
		}

		if ( 1 == mFlagNewMask )
		{
			if ( FAILED == mLM->put_mask(angles, n, mMask, &mMaskRatio) )
			{
				std::cout << "Put mask failed." << std::endl;
				return FAILED;
			}
		}
	}
	else
	{
		if ( 1 == mFlagNewMask )
		{
			std::cout << "No mask fund. Could not update the mask." << std::endl;
		}
	}

	mFlagNewMask = 0;

	this->copy_data(angles, mAngles, ranges, mRanges, n, rangeLowLimit, mMask, &mDataLen, &mInfRatio);

	// Will not compress the memory after the mask and inf operation
	// since we are working with only hundreds of values.

	return sta;
}

void LidarSafeGuard::copy_data( const real* anglesFrom, real* anglesTo,
                                    const real* rangesFrom, real* rangesTo, int len,
									real rangeLowLimit,
		                            const int* mask, int* nCopied, real* infRatio )
{
	*nCopied    = 0;
	int nInf    = 0;
	int nUnMask = 0;
	real range = 0.0;

	if ( LSG_NULL == mask )
	{
		for ( int i=0; i < len; ++i )
		{
			range = rangesFrom[i];

			if ( isinf(range) == 0 && range > rangeLowLimit )
			{
				anglesTo[*nCopied] = anglesFrom[i];
				rangesTo[*nCopied] = range;

				(*nCopied)++;
			}
			else
			{
				nInf++;
			}
		}

		nUnMask = len;
	}
	else
	{
		for ( int i=0; i < len; ++i )
		{
			if ( 0 == mask[i] )
			{
				range = rangesFrom[i];

				if ( 0 == isinf(range) && range > rangeLowLimit )
				{
					anglesTo[*nCopied] = anglesFrom[i];
					rangesTo[*nCopied] = range;

					(*nCopied)++;
				}
				else
				{
					nInf++;
				}

				nUnMask++;
			}
		}
	}

	*infRatio = 1.0 * nInf / nUnMask;
}

