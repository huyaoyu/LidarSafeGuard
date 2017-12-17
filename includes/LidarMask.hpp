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

// C++ native headers.
#include <vector>

// Project dependent headers.
#include "LidarHelper.hpp"

namespace RP {

/** \brief A mask of measured LIDAR data.
 *
 * Class LidarMask, a mask above the array which stores the measured LIDAR data.
 *
 */
class LidarMask
{
public:
	typedef struct {
		real angle0;
		real angle1;
	} AngleSegment_t;

public:
	/** \brief Constructor.
	 *
	 * The constructor.
	 *
	 * The user has to specify angleMin and angleMax. The user should
	 * guarantee that angleMin < angleMax. It angleMin >= angleMax, an
	 * object will still be created but you will not be able to stack
	 * any angle segments.
	 *
	 * \param angleMin The allowed minimum angle, [rad].
	 * \param angleMax The allowed Maximum angle, [rad].
	 */
	LidarMask(real angleMin, real AngleMax);

	~LidarMask(void);

	/** \brief Add a new segment into the mask.
	 *
	 * A new angle segment will be copied and added into the mask.
	 *
	 * Validation is done every time a new angle segment is copied. Status FAILED will
	 * be returned if the validation is failed.
	 *
	 * Function put_mask() will check the validity of the segments that are already
	 * stacked in an LidarMask object.
	 *
	 * \param aseg A reference to an AngleSegment_t struct.
	 * \return The operation status.
	 *
	 * \sa AngleSegment_t
	 */
	Status_t copy_push_segment(AngleSegment_t& aseg);

	/** \brief Put mask numbers into an plain array.
	 *
	 * The memory of argument mask should be allocated properly before invoking this function.
	 *
	 * In array mask, a 1 means mask, a 0 means not mask.
	 *
	 * The user can provide an integer pointer which will holds the ratio of masked positions.
	 *
	 * Every value in argument angles will be compared against the angle segments stacked in
	 * the LidarMask object. Before putting values into mask, a check will be made on the angle
	 * segments. A FAILED status will be returned if the check fails.
	 *
	 * \param angles The array holds the angles, [rad].
	 * \param n The number of values in angles.
	 * \param mask The array holds the resulting mask.
	 * \param maskRatio An integer holds the resulting ratio of masked position.
	 * \return Operation status.
	 *
	 */
	Status_t put_mask(const real* angles, int n, int* mask, real* maskRatio = LSG_NULL);

	/** \brief Clear all the angle segments stacked in the LidarMask object.
	 *
	 * Clear all the angle segments stacked in the current LidarMask. After clear_data()
	 * the LidarMask object could be used as a new object.
	 *
	 */
	void clear_data(void);

protected:
	/** \brief Check the validity of an angle segment.
	 *
	 */
	Status_t is_valid(AngleSegment_t& aseg, real angleMin, real angleMax);

	/** \brief Check the validity of all the angle segments already stacked.
	 *
	 */
	Status_t check_segments(std::vector<AngleSegment_t*>& segs);

	/** \brief Utility function. Delete data in a std::vector.
	 *
	 */
	void delete_data(void);

	/** \brief This function is used when put_mask() is called.
	 *
	 */
	int find_next_segment_index(std::vector<AngleSegment_t*>& segs, int currentIdx, real angle);

protected:
	real mAngleMin; /// The minimum allowed angle, [rad].
	real mAngleMax; /// The maximum allowed angle, [rad].

	std::vector<AngleSegment_t*> mSegments; /// The vector holds the angle segments.
};

}

#endif /* INCLUDES_LIDARMASK_HPP_ */
