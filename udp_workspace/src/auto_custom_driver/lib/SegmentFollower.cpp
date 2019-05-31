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

#include "auto_custom_driver/SegmentFollower.h"

SegmentFollower::SegmentFollower(float lookAhead, float kp, float ki, float kd):
	m_lookAhead(lookAhead), m_kp(kp), m_ki(ki), m_kd(kd){

}

//load in the segments into the segment follower
void SegmentFollower::Initialize(std::vector<std::pair<double,double>> waypoints){
	m_waypoints = waypoints;
	m_err=0;
	m_erri=0;
	m_errd=0;
}

void SegmentFollower::Advance(std::pair<double,double> curr_loc, double curr_rot, double curr_time){
	m_curr_loc = curr_loc;
	m_curr_rot = curr_rot;

	UpdateSentinel();	//update the location of the sentinel point
	UpdateSegment();	//check which segment we should be using
	UpdateError(curr_time);		//compute the error and update all errors
	UpdateSteeringValue();	//update the steering according to the error

}
void SegmentFollower::SetKp(float kp){
	m_kp = kp;
}
void SegmentFollower::SetKd(float kd){
	m_kd = kd;
}
void SegmentFollower::SetKi(float ki){
	m_ki = ki;
}
float SegmentFollower::GetKp(){
	return m_kp;
}
float SegmentFollower::GetKd(){
	return m_kd;
}
float SegmentFollower::GetKi(){
	return m_ki;
}
float SegmentFollower::GetSteering(){
	return m_steering;
}

///use the current location and rotation to calculate the location of the sentinel point
void SegmentFollower::UpdateSentinel(){
	double sent_x = m_curr_loc.first + cos(m_curr_rot)*m_lookAhead;
	double sent_y = m_curr_loc.second + sin(m_curr_rot)*m_lookAhead;
	m_sentinel = std::pair<double,double>(sent_x,sent_y);
}

///use the sentinel point to figure out the id of the segment closest to that sentinel point
void SegmentFollower::UpdateSegment(){
	if(segmentID < m_waypoints.size()-1){
		//find current segment unit vector and norm
		std::pair<double,double> uCurrSeg = Unit({m_waypoints[segmentID+1].first-m_waypoints[segmentID].first, m_waypoints[segmentID+1].second-m_waypoints[segmentID].second});
		double currSegMag = Norm({m_waypoints[segmentID+1].first-m_waypoints[segmentID].first, m_waypoints[segmentID+1].second-m_waypoints[segmentID].second});
		//dot vector AB with As (s is sentinel, AB is line segment)
		double testProj = Dot2D(uCurrSeg,{m_sentinel.first-m_waypoints[segmentID].first, m_sentinel.second-m_waypoints[segmentID].second});
		//if dot product is < norm(AB), then this line segment is correct,
		if(testProj > currSegMag && segmentID < m_waypoints.size()-2) {
			segmentID++;
			std::cout<<"Updated to next line segment. ID: "<<segmentID<<std::endl;
		}
		//otherwise increment the line segment and that will be what we use
	}
	else{
		std::cout<<"SEGMENT ID NOT VALID\n";
	}
}

///use the error and gains to compute the new steering value
void SegmentFollower::UpdateSteeringValue(){
	m_steering = m_kp*m_err + m_kd*m_errd + m_ki*m_errd;
	m_steering = (m_steering < -1.0) ? -1.0 : m_steering;
	m_steering = (m_steering > 1.0) ? 1.0 : m_steering;

	std::cout<<"Steering: "<<m_steering<<", \tSegmentID: "<<segmentID<< ", \tYaw: "<<m_curr_rot<<std::endl;
}
void SegmentFollower::UpdateError(double curr_time){
	//calculate difference in time
	double step = curr_time - m_time;
	//stop if there was no change in time or went backwards somehow
	if(step < 1e-6){
		return;
	}
	else{

		//calculate error

		//compute cross product of uAB and uAs (s is sentinel, AB is line segment)
		//this should give signed error. Unit vector should be used to make sure
		//error doesn't increase as we move along the line segment. Error should only
		//depend on the perpendicularity of the two vectors
		double error = -Cross2D(Unit({m_waypoints[segmentID+1].first-m_waypoints[segmentID].first, m_waypoints[segmentID+1].second-m_waypoints[segmentID].second}),
			{m_sentinel.first-m_waypoints[segmentID].first,m_sentinel.second-m_waypoints[segmentID].second});

		m_errd = (error - m_err) / step;
		m_erri += (error+m_err)*step/2.0;
		m_err = error;
		m_time = curr_time;

		//std::cout<<"Error: "<<m_err<<", ErrorD: "<<m_errd<<", ErrorI:"<<m_erri<<std::endl;
	}

}

double SegmentFollower::Dot2D(std::pair<double,double> a, std::pair<double,double> b){
	return a.first*b.first + a.second*b.second;
}
double SegmentFollower::Cross2D(std::pair<double,double> a, std::pair<double,double> b){
	return a.first*b.second - a.second*b.first;
}
std::pair<double,double> SegmentFollower::Unit(std::pair<double,double> a){
	return std::pair<double,double>(a.first / Norm(a), a.second / Norm(a));
}
double SegmentFollower::Norm(std::pair<double,double> a){
	return std::sqrt(std::pow(a.first,2)+std::pow(a.second,2));
}
