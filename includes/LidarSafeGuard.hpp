/*
 * LidarSafeGuard.hpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu <huyaoyu@sjtu.edu.cn>
 *
 *  Definition of the class LidarSafeGurad, InCylinderSafeGuard, and InCylRgrSafeGuard.
 */

#ifndef INCLUDES_LIDARSAFEGUARD_HPP_
#define INCLUDES_LIDARSAFEGUARD_HPP_

// C++ native headers.
#include <string>
#include <iostream>

// Project dependent headers.
#include "LidarHelper.hpp"
#include "LidarMask.hpp"

namespace RP {

/** \brief Base class LidarSafeGuard.
 *
 * This is the base class for other safe guards.
 *
 * The functionalities of LidarMask, ratio of Infs, and ratio of masked positions
 * are the main part.
 *
 */
class LidarSafeGuard
{
public:
	typedef enum {
		FLAG_SAFE,
		FLAG_UNSAFE
	}SafetyFlag_t;

public:
	/** \brief Constructor.
	 *
	 * The user should provide a name for the safe guard.
	 *
	 * \param name Name string in plain char*.
	 */
	LidarSafeGuard(const char* name);

	/** \brief Constructor.
	 *
	 * The user should provide a name for the safe guard.
	 *
	 * \param name Name string.
	 *
	 */
	LidarSafeGuard(std::string& name);
	virtual ~LidarSafeGuard(void);

	/** \brief Set up reference to an external LidarMask object.
	 *
	 * If the user specify a LidarMask object using this function
	 * the mask will take place in the later operations.
	 *
	 * Every time a new LidarMask is specified the LidarSafeGuard object
	 * will update the internal buffers automatically.
	 *
	 * The LidarSafeGuard object only holds a pointer pointing to the external
	 * LidarMask object. The user has to make sure that this object is alive during
	 * the operation and destroy the mask when it is not needed anymore.
	 *
	 * \param LM The pointer to the LidarMask object.
	 *
	 */
	void set_mask(LidarMask* LM);

	/** \brief Retrieve the referenced LidarMask object.
	 *
	 * \return The pointer to the external LidarMask object.
	 *
	 */
	LidarMask* get_mask(void);

	/** \brief Get the ratio of Infs in the LIDAR data.
	 *
	 * \return The ratio of Infs in the LIDAR data.
	 *
	 */
	real get_inf_ratio(void);

	/** \brief Get the ratio of masked positions.
	 *
	 * If no mask is supplied, it will return zero.
	 *
	 * \return The ratio of masked positions.
	 *
	 */
	real get_mask_ratio(void);

	/** \brief Get the analyzed flag of safety.
	 *
	 * NOTE: Call any verification functions that provided by any derived classes before
	 * invoke this function.
	 *
	 * The flag of safety will be returned.
	 *
	 * \return The flag of safety
	 * \sa SafetyFlag_t
	 *
	 */
	SafetyFlag_t get_safety_flag(void);

	/** \brief Get the name of this LidarSafeGuard object.
	 *
	 * \return Name of this object.
	 *
	 */
	std::string& get_name(void);

	/** \brief Copy the data from a lidar detection.
	 *
	 * The LidarSafeGuard object will copy and store the angles and ranges.
	 *
	 * In case one LidarMask is used for different set of angels and ranges, the user
	 * could specify forceUpdateMask to ensure the flush of internal mask buffers.
	 *
	 * The ratio of Infs works based on the counting of Infs in the ranges array. Some
	 * LIDAR may give 0 instead of Inf. For this case, the user could specify rangeLowLimit to
	 * facilitate the calculation of the ratio of Infs. Any range value that is lower than rangeLowLimit
	 * will be treated as Inf.
	 *
	 * \param angles An array of angles, in rad.
	 * \param ranges An array of ranges, in m.
	 * \param n The actual length of data in angles and ranges.
	 * \param forceUpdateMask 1 for force updating.
	 * \param rangeLowLimit The lower limit of valid range.
	 * \return LidarSafeGuard::LSG_OK for successful operation, LidarSafeGurad::LSG_FAILED for error.
	 */
	Status_t copy_data(const real* angles, const real* ranges, int n, int forceUpdateMask = 0, real rangeLowLimit = 0.0);

protected:
	/** \brief Utility function. Copy angle and range data.
	 *
	 */
	void copy_data( const real* anglesFrom, real* anglesTo,
					const real* rangesFrom, real* rangesTo, int len,
					real rangeLowLimit,
					const int* mask, int* nCopied, real* infRatio );

protected:
	std::string mName;      /// The name of the LidarSafeGuard object.
	SafetyFlag_t mFlagSafe; /// The flag of safety.
	LidarMask* mLM;         /// The pointer to the external LidarMask object.
	int* mMask;             /// The mask buffer.
	int  mFlagNewMask;      /// The flag indicates a flush request of the mask buffer.
	real mMaskRatio;        /// The ratio of masked positions.
	real mInfRatio;         /// The ratio of Infs.

	real* mAngles;    /// Local copy of the angles.
	real* mRanges;    /// Local copy of the ranges.
	int mDataLen;     /// Data length of mAngles and mRanges.  mDataLen and mDataLenLast is the length without any masked and Inf value.
	int mDataLenLast; /// Integer holds the length of data last time the copy_data() is invoked. mDataLen and mDataLenLast is the length without any masked and Inf value.
	int mDataNLast;   /// Integer holds the length of the data supplied to copy_data(). mDataNLast is the argument n of copy_data().
};

/** \brief Class InCylinderSafeGuard.
 *
 * InCylinderSafeGuard could be used to detect whether the LIDAR is located
 * inside a circular pipe/tube, that is a cylinder.
 *
 * This class assumes that the eccentricity between the center of the LIDAR and
 * the center of the cylinder is known.
 *
 * InCylinderSafeGuard will produce a flag of safety after the verify() function is invoked. The
 * user could use get_safety_flag() of the base class LidarSafeGuard to see the analyzed safety
 * status of the LIDAR. If InCylinderSafeGuard thinks it is inside a cylinder the flag of safety
 * will be SAFE. Otherwise, it would be UNSAFE. \sa SafteFlag_t
 *
 * The user is encouraged to examine the value from get_inf_ratio() and get_mask_ratio() to further
 * ensure whether the flag of safety could be trusted or not.
 *
 */
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

	/** \brief Verify the data.
	 *
	 * Verify the stored range data. The user supply target radius (r) and the desired
	 * absolute difference from r and the standard deviation.
	 *
	 * The flag of safety is returned by the argument flag. And it could also be obtained
	 * by calling get_safety_flag() after calling this function.
	 *
	 * The resulting radius and the standard deviation could be obtained by get_radius_mean() and
	 * get_radius_std().
	 *
	 * \param r The target radius.
	 * \param rLimit The limit of the difference from r.
	 * \param stdLimit The limit of the standard deviation.
	 * \param flag A pointer to a SatetyFlag_t variable. Served as a return value indicating whether it is safe or not. If flag == LSG_NULL, no output will inplace.
	 * \return LidarSafeGuard::LSG_OK for successful operation.
	 */
	Status_t verify(real r, real rLimit, real stdLimit, LidarSafeGuard::SafetyFlag_t* flag = LSG_NULL);

	/** \brief Return the mean radius after verifying the data.
	 *
	 * Return the mean radius after verifying the data.
	 *
	 * \return The mean radius.
	 */
	real get_radius_mean(void);

	/** \brief Return the standard deviation of the radius after verifying the data.
	 *
	 * Return the standard deviation of the radius after verifying the data.
	 *
	 * \return The standard deviation.
	 */
	real get_radius_std(void);

protected:
	/** \brief Utility function. Transfer the angels and ranges into Cartesian coordinate system.
	 *
	 * Transfer the angels and ranges into Cartesian coordinate system. The Cartesian coordinate
	 * system is the frame attached to the LIDAR. This is eccentric from the center of the cylinder.
	 *
	 */
	void calculate_coordinates_in_ecc_frame(const real* angles, const real* ranges, int len, real* coorX, real* coorY);

	/** \brief Utility function. Transfer the coordinates from the eccentric frame to a concentric frame.
	 *
	 * Transfer the coordinates from the eccentric frame ( output of calculate_coordinate_in_ecc_frame() )
	 * to a concentric frame.
	 *
	 */
	void translate_coordinates(const real* coorXFrom, const real* coorYFrom, int len, real ecc, real* coorXTo, real* coorYTo);

	/** \brief Utility function. Calculate the mean radius of the cylinder.
	 *
	 * Calculate the mean radius of the cylinder.
	 *
	 * The standard deviation is also calculated.
	 *
	 */
	void calculate_mean_radius(const real* coorX, const real* coorY, int n, real& rMean, real& rStd);

protected:
	real mEcc;        /// Installation eccentricity of the LIDAR [m].
	real mPhaseAngle; /// Installation angle of the LIDAR, [rad]. Not used currently.

	real mRadiusMean; /// Sample mean.
	real mRadiusStd;  /// Sample standard deviation.

	real* mCoorX_Ecc; /// Coordinates in the eccentric frame.
	real* mCoorY_Ecc; /// Coordinates in the eccentric frame.

	real* mCoorX;     /// Coordinates in the concentric frame.
	real* mCoorY;     /// Coordinates in the concentric frame.
};

/** \brief Class InCylRgrSafeGuard.
 *
 * Class InCylRgrSafeGuard. This class is derived from InCylinderSafeGuard with the
 * special verify() function. This class adopts the regression method to find the
 * radius and the center of the cylinder.
 *
 * For this class, in fact, the eccentricity and phase angle are no longer needed since
 * the the center coordinate in the eccentric frame could be found by the regression.
 *
 * However, the verify() function is relatively slower than that provided by its
 * base class, InCylinderSafeGuard. This class will be about an order of magnitude
 * slower. On the test data set with a mask of about 2 pi coverage, the time consumed
 * by verify() is about 3 ms (Core i5-2400 @ 3.1 GHz, Ubuntu 17.04, hosted by Windows 10 with VirtualBox)
 * with default settings.
 *
 * The regression is done in an iterating manner. The user could specify the max number of
 * iterations, the maximum residual, and the relaxation factor of the iteration process.
 *
 */
class InCylRgrSafeGuard : public InCylinderSafeGuard
{
public:
	static const int MAX_ITERS;     /// The maximum iterations for the regression calculation.
	static const real MAX_RESIDUAL; /// The maximum residual for the regression calculation.
	static const real RELAX;        /// The relaxation factor for the regression calculation.

public:
	/** \brief Constructor.
	 *
	 * No need for specifying eccentricity and phase angle.
	 *
	 */
	InCylRgrSafeGuard(const char* name);

	/** \brief Constructor.
	 *
	 */
	InCylRgrSafeGuard(std::string& name);
	~InCylRgrSafeGuard(void);

	/** \brief Set the maximum number of iterations.
	 *
	 * Set the maximum number of iterations.
	 *
	 * \param iters Number of iterations, must be positive.
	 */
	void set_max_iters(int iters);

	/** \brief Set the maximum residual of iterations.
	 *
	 * Set the maximum residual of the iterations when perform the
	 * regression calculation. The actual residual should be lower
	 * than this maximum residual to indicate a convergence.
	 *
	 * \param res The desired maximum residual. Must be positive.
	 *
	 */
	void set_max_residual(real res);

	/** \brief Set the relaxation factor.
	 *
	 * A typical relaxation factor is ranging from 0.0 to 1.0. Then the under relaxation
	 * effect will be achieved.
	 *
	 * A factor larger than 1.0 could also be specified then a super relaxation is configured.
	 * When the factor is larger than 1.0, a warning will be issued by std::cout.
	 *
	 * \param relax The under relaxation factor, must be positive.
	 *
	 */
	void set_relax(real relax);

	/** \brief Return the maximum number iterations.
	 *
	 * Return the maximum number of iterations.
	 *
	 */
	int get_max_iters(void);

	/** \brief Return the maximum residual.
	 *
	 * Return the maximum residual.
	 *
	 */
	real get_max_residual(void);

	/** \brief Return the relaxation factor.
	 *
	 * Return the relaxation factor.
	 *
	 */
	real get_relax(void);

	/** \brief Get the number of iterations after calling verify().
	 *
	 * Get the number of iterations after calling verify().
	 *
	 * In fact, it is the number of iterations of the last call of verify().
	 *
	 */
	int get_iters(void);

	/** \brief Get the residual after calling verify().
	 *
	 * Get the residual after calling verify().
	 *
	 * In fact, it is the residual of the last call of verify().
	 *
	 */
	real get_residual(void);

	/** \brief Verify the data.
	 *
	 * Verify the data. Overload function, shadows its base class InCylSafeGuard.
	 *
	 * \param r The target radius.
	 * \param rLimit The limit of the difference from r.
	 * \param stdLimit The limit of the standard deviation.
	 * \param flag A pointer to a SatetyFlag_t variable. Served as a return value indicating whether it is safe or not. If flag == LSG_NULL, no output will inplace.
	 * \return LidarSafeGuard::LSG_OK for successful operation.
	 */
	Status_t verify(real r, real rLimit, real stdLimit, LidarSafeGuard::SafetyFlag_t* flag = LSG_NULL);

	/** \brief Set flag for debugging.
	 *
	 * Set the flag for debugging. Not used right now.
	 *
	 */
	void set_flag_debug(void);

	/** \brief Clear the flag for debugging.
	 *
	 * Clear the flag for debugging. Not used right now.
	 *
	 */
	void clear_flag_debug(void);

	/** \brief Get the current value of the flag for debugging.
	 *
	 * Get the current value of the flag for debugging. Not used right now.
	 *
	 */
	int get_flag_debug(void);

protected:
	/** \brief Utility function. Solve for the coordinates of the cylinder center and the radius.
	 *
	 * As the name indicates, this function adopts the least-square method to solve for the
	 * coordinates of the center of the cylinder and its radius.
	 *
	 * \param coorX The array holds the x coordinates.
	 * \param coorY The array holds the y coordinates.
	 * \param n The number of values in coorX and coorY.
	 * \param x0 The resulting x-coordinate of the center of the cylinder.
	 * \param y0 The resulting y-coordinate of the center of the cylinder.
	 * \param r The resulting radius of the center of the cylinder.
	 * \param iters The number of iterations.
	 * \param res The residual.
	 * \param maxIters The maximum number of iterations.
	 * \param maxResidual The desired maximum residual before convergence.
	 * \param relax The relaxation factor.
	 * \return The operation status, \sa Status_t
	 *
	 */
	Status_t solve_by_least_square(const real* coorX, const real* coorY, int n,
			real& x0, real& y0, real& r, int& iters, real& res,
			int maxIters = MAX_ITERS, real maxResidual = MAX_RESIDUAL, real relax = RELAX);

	/** \brief Calculate the standard deviation after calling solve_by_least_square().
	 *
	 */
	Status_t radius_std(const real* coorX, const real* coorY, int n, real x0, real y0, real r, real& rStd);

	/** \brief Find the minimum x and y values.
	 *
	 * Not used currently.
	 *
	 */
	Status_t min_x_y(const real* coorX, const real* coorY, int n, real& minX, real& minY);

protected:
	int flagDebug; /// The flag for debugging.

	int mMaxIters;     /// The maximum number of iterations.
	real mMaxResidual; /// The maximum residual.
	real mRelax;       /// The relaxation factor.

	int mIters;        /// The number of iterations of the last call to verify.
	real mResidual;    /// The redisual of the last call to verify.
};

}

#endif /* INCLUDES_LIDARSAFEGUARD_HPP_ */
