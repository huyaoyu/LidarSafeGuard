/*
 * LidarMask.cpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  The source file of the Class LidarMask.
 */

#include <iostream>

#include "LidarMask.hpp"

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

	if ( aseg.angle0 < angleMin )
	{
		return sta;
	}

	if ( aseg.angle1 > angleMax )
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

	// Save the copy.
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

	// See if the current index is already out of range.
	if ( idx >= segs.size() )
	{
		return -1;
	}

	// Find the proper index.
	for ( ; idx < segs.size(); ++idx )
	{
		if ( segs.at(idx)->angle0 >= angle  )
		{
			return idx;
		}
	}

	// No index found, out of range.
	return -1;
}

Status_t LidarMask::put_mask(const real* angles, int n, int* mask, real* maskRatio)
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

	// Initial check done.

	int idxSeg  = 0;   // Index of the angle segment.
	int nMasked = 0;   // Number of masked positions.
	real angle  = 0.0; // Temporary variable.

	if ( mSegments.size() == 0 )
	{
		// No angle segments have been stacked.
		for ( int i=0; i < n; ++i )
		{
			mask[i] = 0;
		}

		nMasked = 0;
	}
	else
	{
		// Get the first angle segment.
		AngleSegment_t* pag = mSegments.at(idxSeg);

		for ( int i=0; i < n; ++i )
		{
			angle = angles[i];

			if ( angle >= pag->angle0 && angle <= pag->angle1 )
			{
				mask[i] = 1;
				nMasked++;
			}
			else
			{
				mask[i] = 0;
			}

			// Find the next possible index of the angle segment.
			// idxSeg == -1 if no further index is available.
			if ( idxSeg != -1 && angle > pag->angle1 )
			{
				idxSeg = find_next_segment_index(mSegments, idxSeg, angle);

				if ( -1 != idxSeg )
				{
					pag = mSegments.at(idxSeg);
				}
			}
		}
	} // mSegments.size() == 0

	if ( LSG_NULL != maskRatio )
	{
		*maskRatio = 1.0 * nMasked / n;
	}

	return sta;
}

