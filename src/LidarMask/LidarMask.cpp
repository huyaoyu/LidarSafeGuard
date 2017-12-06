/*
 * LidarMask.cpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  The source file of the Class LidarMask.
 */

#include "LidarMask.hpp"
#include <iostream>

using namespace RP;

LidarMask::LidarMask(real angleMin, real angleMax)
: mAngleMin(angleMin), mAngleMax(angleMax)
{
	if ( mAngleMin >= mAngleMax )
	{
		std::cout << "Wrong angle specification!" << std::endl;
		std::cout << "mAngleMin = " << mAngleMin
				  << "mAngleMax = " << mAngleMax
				  << std::endl;
	}
}

LidarMask::~LidarMask(void)
{
	delete_data();
}

Status_t LidarMask::is_valid(AngleSegment_t& aseg, real angleMin, real angleMax)
{
	Status_t sta = FAILED;

	if ( aseg.angle0 > aseg.angle1 )
	{
		return sta;
	}

	sta = OK;

	return sta;
}

void LidarMask::delete_data(void)
{
	if ( 0 != mSegments.size() )
	{
		std::vector<AngleSegment_t*>::iterator itr;

		for ( itr = mSegments.begin(); itr != mSegments.end(); ++itr )
		{
			if ( LSG_NULL != *itr )
			{
				delete *itr; *itr = LSG_NULL;
			}
		}
	}
}

void LidarMask::clear_data(void)
{
	delete_data();
	mSegments.clear();
}

Status_t LidarMask::copy_push_segment(AngleSegment_t& aseg)
{
	Status_t sta = OK;

	if ( FAILED == is_valid(aseg, mAngleMin, mAngleMax) )
	{
		std::cout << "Angle segment not valid. "
				  << "angle0 = " << aseg.angle0 << ", "
				  << "angle1 = " << aseg.angle1 << "." << std::endl;
		return FAILED;
	}

	// Make a copy.
	AngleSegment_t* temp = new AngleSegment_t;

	temp->angle0 = aseg.angle0;
	temp->angle1 = aseg.angle1;

	mSegments.push_back(temp);

	return sta;
}

Status_t LidarMask::check_segments(std::vector<AngleSegment_t*>& segs)
{
	Status_t sta = OK;

	for ( size_t i=0; i < segs.size() - 1; ++i )
	{
		if ( segs[i]->angle1 > segs[i+1]->angle0 )
		{
			std::cout << "The segments of index " << i << " and " << i+1 << " are not compatible." << std::endl;

			sta = FAILED;
			break;
		}
	}

	return sta;
}

int LidarMask::find_next_segment_index(std::vector<AngleSegment_t*>& segs, int currentIdx, real angle)
{
	int idx = currentIdx;

	if ( idx >= segs.size() )
	{
		return -1;
	}

	for ( ; idx < segs.size(); ++idx )
	{
		if ( segs.at(idx)->angle0 >= angle  )
		{
			return idx;
		}
	}

	return -1;
}

Status_t LidarMask::put_mask(const real* angles, int n, int* mask)
{
	Status_t sta = OK;

	if ( LSG_NULL == angles || LSG_NULL == mask )
	{
		std::cout << "The angles or mask is NULL." << std::endl;
		return FAILED;
	}

	if ( n <= 0 )
	{
		std::cout << "Wrong n. n = " << n << "." << std::endl;
		return FAILED;
	}

	if ( FAILED == check_segments(mSegments) )
	{
		std::cout << "Check segments failed." << std::endl;
		return FAILED;
	}

	int idxSeg = 0;
	real angle = 0.0;

	AngleSegment_t* pag = mSegments.at(idxSeg);

	for ( int i=0; i < n; ++i )
	{
		angle = angles[i];

		if ( angle >= pag->angle0 && angle <= pag->angle1 )
		{
			mask[i] = 1;
		}
		else
		{
			mask[i] = 0;
		}

		if ( idxSeg != -1 && angle > pag->angle1 )
		{
			idxSeg = find_next_segment_index(mSegments, idxSeg, angle);

			if ( -1 != idxSeg )
			{
				pag = mSegments.at(idxSeg);
			}
		}
	}

	return sta;
}
