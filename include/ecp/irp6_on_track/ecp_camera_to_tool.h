/*
 * ecp_camera_to_tool.h
 *
 *  Created on: Apr 8, 2009
 *      Author: ghard
 */

#ifndef ECP_CAMERA_TO_TOOL_H_
#define ECP_CAMERA_TO_TOOL_H_

#include <iostream>
#include <cmath>

#include "ecp/common/ecp_matrix4x4.h"

class CameraToTool
{
	double d, a, h;
	double tpe[12];

	void fillTPE();
	double computeTPC(double[3], double[3], double[3], double[3], double[12]);

	double norm2m(double[12]);
	double norm2v(double[3]);
	void vec_prod(double[3], double[3], double[3]);

  public:
	CameraToTool(double=0.04, double=0.001, double=0.069);

	double computeTCE(double[3], double[3], double[3], double[3], double[12]);
	double computeTCE(double[12], double[12]);
};


#endif /* ECP_CAMERA_TO_TOOL_H_ */