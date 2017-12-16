/*
 * LidarSafeGuard.hpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  Definition of the class LidarSafeGurad
 */

#ifndef INCLUDES_LIDARSAFEGUARD_HPP_
#define INCLUDES_LIDARSAFEGUARD_HPP_

#include <string>
#include <iostream>

#include "LidarHelper.hpp"
#include "LidarMask.hpp"

namespace RP {

class LidarSafeGuard
{
public:
	typedef enum {
		FLAG_SAFE,
		FLAG_UNSAFE
	}SafetyFlag_t;

public:
	LidarSafeGuard(const char* name);
	LidarSafeGuard(std::string& name);
	virtual ~LidarSafeGuard(void);
	void set_mask(LidarMask* LM);
	LidarMask* get_mask(void);
	real get_inf_ratio(void);

	SafetyFlag_t get_safety_flag(void);
	std::string& get_name(void);

	/**
	 * Copy the data from a lidar detection.
	 *
	 * \param angles An array of angles, in rad.
	 * \param ranges An array of ranges, in m.
	 * \param n The actual length of data in angles and ranges.
	 * \return LidarSafeGuard::LSG_OK for successful operation, LidarSafeGurad::LSG_FAILED for error.
	 */
	Status_t copy_data(const real* angles, const real* ranges, int n);

protected:
	void copy_data( const real* anglesFrom, real* anglesTo,
					const real* rangesFrom, real* rangesTo, int len,
					const int* mask, int* nCopied, real* infRatio );

protected:
	std::string mName;
	SafetyFlag_t mFlagSafe;
	LidarMask* mLM;
	int* mMask;

	real mInfRatio;

	real* mAngles;
	real* mRanges;
	int mDataLen;
	int mDataLenLast;
};

class InCylinderSafeGuard : public LidarSafeGuard
{
public:
	/**
	 * Constructor.
	 *
	 * For the definitions of ecc and phaseAngle, pleas refer to the documentation.
	 *
	 * \param name The name of this safe guard.
	 * \param ecc The eccentricity of the RPLIDAR.
	 * \param phaseAngle The phaseAngle of the RPLIDAR.
	 */
	InCylinderSafeGuard(const char* name, real ecc, real phaseAngle);

	/**
	 * Constructor.
	 *
	 * For the definitions of ecc and phaseAngle, pleas refer to the documentation.
	 *
	 * \param name The name of this safe guard.
	 * \param ecc The eccentricity of the RPLIDAR.
	 * \param phaseAngle The phaseAngle of the RPLIDAR.
	 */
	InCylinderSafeGuard(std::string& name, real ecc, real phaseAngle);

	~InCylinderSafeGuard(void);

	/**
	 * Verify the data.
	 *
	 * \param flag A pointer to a SatetyFlag_t variable. Served as a return value indicating whether it is safe or not. If flag == LSG_NULL, no output will inplace.
	 * \return LidarSafeGuard::LSG_OK for successful operation.
	 */
	Status_t verify(LidarSafeGuard::SafetyFlag_t* flag = LSG_NULL);

	/**
	 * Return the mean radius after verify. return negative value if an error occurred.
	 *
	 * \return The mean radius.
	 */
	real get_mean_radius(void);

protected:
	void calculate_coordinates_in_ecc_frame(const real* angles, const real* ranges, int len, real* coorX, real* coorY);

	void translate_coordinates(const real* coorXFrom, const real* coorYFrom, int len, real ecc, real* coorXTo, real* coorYTo);

protected:
	real mEcc;
	real mPhaseAngle;

	real mRadiusMean; // Sample mean.
	real mRadiusStd;  // Sample standard deviation.

	real* mCoorX_Ecc;
	real* mCoorY_Ecc;

	real* mCoorX;
	real* mCoorY;
};

}



#endif /* INCLUDES_LIDARSAFEGUARD_HPP_ */
