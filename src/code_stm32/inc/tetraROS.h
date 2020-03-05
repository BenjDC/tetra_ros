/*
 * tetraROS.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef TETRAROS_H_
#define TETRAROS_H_


#ifdef __cplusplus
 extern "C" {
#endif

float const xspeed_max = 0.30f; 	// m.s
float const xacc_max = 0.10f; 		//m.s2
float const wspeed_max = 40.0f;	// dps
float const wacc_max = 30.0f;	// dps/s
float const joy_max = 0.5f; 	// m.s


void initTetraROS();
void loopTetraROS();




#ifdef __cplusplus
}
#endif


#endif /* TETRAROS_H_ */

