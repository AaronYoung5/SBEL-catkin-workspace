// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dylan Hatch
// =============================================================================
//
// Helper Functions for synchrono vehicle demos.
//
// =============================================================================

#ifndef SEGMENTFOLLOWER_H
#define SEGMENTFOLLOWER_H

#include <vector>
#include <cmath>
#include <iostream>

class SegmentFollower{

public:
	SegmentFollower(float lookAhead= 1.0, float kp=0, float ki=0, float kd=0);

	void Initialize(std::vector<std::pair<double,double>> waypoints);
	void Advance(std::pair<double,double> curr_loc, double curr_rot, double curr_time);

	void SetKp(float kp);
	void SetKd(float kd);
	void SetKi(float ki);
	float GetKp();
	float GetKd();
	float GetKi();
	float GetSteering();
private:
	void UpdateSentinel();
	void UpdateSegment();
	void UpdateError(double curr_time);
	void UpdateSteeringValue();

	double Dot2D(std::pair<double,double> a, std::pair<double,double> b);
	double Cross2D(std::pair<double,double> a, std::pair<double,double> b);
	std::pair<double,double> Unit(std::pair<double,double> a);
	double Norm(std::pair<double,double> a);

	//segment parameters
	int segmentID=0;
	std::vector<std::pair<double,double>> m_waypoints;

	//error parameters
	float m_err=0;	//error of sentinel relative to target point
	float m_erri=0; //integral of error
	float m_errd=0; //derivative of error
	float m_kp;	//proportional gain
	float m_ki; //integral gain
	float m_kd; //derivative gain
	float m_lookAhead; //lookahead distance for sentinal point

	//driving parameters
	float m_steering=0; //field for storing computed steering value
	double m_time=0;	//variable holding the previous time
	std::pair<double,double> m_sentinel = std::pair<double,double>(0,0);
	double m_curr_rot = 0;	//yaw measured in radians
	std::pair<double,double> m_curr_loc = std::pair<double,double>(0,0);

};

#endif
