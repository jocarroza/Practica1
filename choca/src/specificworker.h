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
  
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	virtual void setPick(const Pick &myPick);

private:
  struct Target{
	  mutable QMutex mutex;
	  QVec coord = QVec::zeros(3);
	  float alpha;
	  bool empty = true;
	  
	  bool isEmpty(){
	    QMutexLocker lm(&mutex);
	    return empty;
	  }
	  
	  void setEmpty(bool e){
	    QMutexLocker lm(&mutex);
	    empty = e;
	  }
	  
	 void insertarCoord(float x, float z){
	   QMutexLocker lm(&mutex);
	   coord.setItem(0, x);
	   coord.setItem(1, 0);
	   coord.setItem(2, z);
	 }
	 
	 QVec extraerCoord(){
	   QMutexLocker lm(&mutex);
	   return coord;
	 }
	};
	
	Target target;
	
};

#endif

