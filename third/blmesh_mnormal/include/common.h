#pragma once
#ifndef _COMMON_H_
#define _COMMON_H_
// text color 
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

//far node
#define FARSCALE 15 /* The distance from the far point to the centor of unit sphere */


// computation geometry setting
#define PI 3.14159265358979323846
#define ANGLE(x,y) acos((x).normalized()*(y).normalized())*180/PI

// Balance between efficiency and robustness
#define MAX_INTERATION 4 
#define SKEWNESS_THREADHOLD 0.80//only optimize node whose single skewness above this value
#define PLANE_SKEWNESS -0.1 // only ridge whose minimal cos(dihedral angle) less than this value will be count for iteration 
#define CONVEX_SKEWNESS 0.2
#define MAX_PLAIN_RIDGE 1
#define ONE_ORDER_TRI_COST  0.02 // we do not like two order element in virtual mesh because they usually cause small volume tetrahedron which do harm to solver. 
								// thus a additional cost is 
#define TWO_ORDER_TRI_COST  0.08 // we do not like two order element in virtual mesh because they usually cause small volume tetrahedron which do harm to solver. 
								// thus a additional cost is 

#define POINT_OFFSET 0.01
#define MAX_STATEGY 20

#endif
