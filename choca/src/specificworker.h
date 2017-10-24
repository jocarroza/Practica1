/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	
	
  
	float giro;
	
	const float MAX_ADV = 400.0;
	const float MAX_ROT = 0.5; 
  
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	virtual void setPick(const Pick &myPick);

private:
  InnerModel *innermodel;
  
  RoboCompLaser::TLaserData laserData;
  
  enum State {IDLE, GOTO, TURN, AVOID, END, BUG};
  State state = State::IDLE;
  

  struct Target{
	  mutable QMutex mutex;
	  float x = 0.0;
	  float z = 0.0;
	  float alpha;
	  bool empty = true;
	  
	  bool isEmpty(){
	    QMutexLocker lm(&mutex);
	    return empty;
	  }
	  
	  void setEmpty(){
	    QMutexLocker lm(&mutex);
	    empty = true;
	  }
	  
	 void insertarCoord(float _x, float _z){
	   QMutexLocker lm(&mutex);
	   empty = false;
	   x = _x;
	   z = _z;
	 }
	 
	 std::pair<float, float> extraerCoord(){
	   QMutexLocker lm(&mutex);
	   return std::make_pair(x,z);
	 }
	};
	
	Target target;
	
	float gaussian(float vr, float vx, float h);
	float sigmoid(float d);
	
	void gotoTarget();
	void bug();
	bool obstacle(RoboCompLaser::TLaserData laserData);
	bool targetAtSight(RoboCompLaser::TLaserData laserCopy);
	
};

#endif

