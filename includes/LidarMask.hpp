/*
 * LidarMask.hpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  Definition of Class LidarMask.
 */

#ifndef INCLUDES_LIDARMASK_HPP_
#define INCLUDES_LIDARMASK_HPP_

#include <vector>

#include "LidarHelper.hpp"

namespace RP {

class LidarMask
{
public:
	typedef struct {
		RP::real angle0;
		RP::real angle1;
	} AngleSegment_t;

public:
	LidarMask(real angleMin, real AngleMax);
	~LidarMask(void);

	Status_t copy_push_segment(AngleSegment_t& aseg);

	Status_t put_mask(const real* angles, int n, int* mask, real* maskRatio = LSG_NULL);

	void clear_data(void);

protected:
	Status_t is_valid(AngleSegment_t& aseg, real angleMin, real angleMax);
	Status_t check_segments(std::vector<AngleSegment_t*>& segs);
	void delete_data(void);
	int find_next_segment_index(std::vector<AngleSegment_t*>& segs, int currentIdx, real angle);

protected:
	real mAngleMin;
	real mAngleMax;

	std::vector<AngleSegment_t*> mSegments;
};

}



#endif /* INCLUDES_LIDARMASK_HPP_ */
