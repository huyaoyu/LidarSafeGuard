/*
 * LidarHelper.hpp
 *
 *  Created on: Nov 28, 2017
 *      Author: yaoyu
 */

#ifndef INCLUDES_LIDARHELPER_HPP_
#define INCLUDES_LIDARHELPER_HPP_

#ifdef NULL
#define LSG_NULL NULL
#else
#define LSG_NULL (0)
#endif

#define FREE_ARRAY(x) \
	if ( LSG_NULL != x )\
	{\
		delete [] x; x = LSG_NULL;\
	}\

#define ALLOC_ARRAY(type, x, len) \
	if ( LSG_NULL != x ) \
	{\
		std::cout << "Assigning new memory to a non-NULL pointer." << std::endl << "Memory will be released first." << std::endl;\
		delete [] x;\
	}\
	x = new type[len];\

#define DECLARE_ALLOC_ARRAY(type, x, len) \
	type* x = new type[len];\

namespace RP {

typedef double real;

typedef enum {
	OK,
	FAILED
} Status_t;

}

#endif /* INCLUDES_LIDARHELPER_HPP_ */
