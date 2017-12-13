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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


	innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
  
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);
  innermodel->updateTransformValues ("base",bState.x, 0, bState.z, 0, bState.alpha, 0 ); 
  
  auto tags = getapriltags_proxy->checkMarcas();
  
  if (tag.empty == true){
      int i=0;
      
      
      for (auto t : tags){
	QVec tr = innermodel->transform("world", QVec::vec3(tags[i].tx, 0, tags[i].tz), "rgbd");
	if (caja == true){
	  if (tags[i].id > 10){
	    tag.id = tags[i].id;
	    tag.x = tr.x();
	    tag.z = tr.z();
	    tag.empty = false;
	    return;
	  }
	  else{
	    i++;
	  }
	}
	if (caja == false){
	  if (tags[i].id < 10){
	    tag.id = tags[0].id;
	    tag.x = tr.x();
	    tag.z = tr.z();
	    tag.empty = false;
	    return;
	  }
	}
      }
  }
  
  switch( state )
  {
    case State::IDLE:
      
    break;
 
    case State::SEARCH:
      search();
    break;
   
    case State::GOTO:
      goPoint();
    break;
    
    case State::PICK:
      
    break;
      
    case State::RELEASE:
      
    break;
    
    case State::WAIT:
      wait();
    break;
  }

// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}


void SpecificWorker::search()
{
  gotopoint_proxy->turn(0.3);
  
  if (tag.empty == false){
    //gotopoint_proxy->stop();
    
      
    if (tag.id >=10 && caja == true){
      gotopoint_proxy->stop();
      state = State::GOTO;
    }
    else{
      if (tag.id == 0 && caja == false){
      gotopoint_proxy->stop();
      state = State::GOTO;
      }
      else{
      tag.empty = true;
      }
    }
  }
}

void SpecificWorker::goPoint()
{
  gotopoint_proxy->go("", tag.x, tag.z, 0);
  state = State::WAIT;
}


void SpecificWorker::pick_box()
{
//   gotopoint_proxy->picking_box();
  
}

void SpecificWorker::release_box()
{
//   gotopoint_proxy->releasing_box();
}


void SpecificWorker::wait()
{
  if (gotopoint_proxy->atTarget() == true){
    caja = !caja;
    std::cout<<"SIGUIENTE TAG"<<endl;
    
    if (caja == false){
      state = State::SEARCH;
      tag.empty = true;
    }
    else{
      state = State::SEARCH;
      tag.empty = true;
    }
  }
}


// void SpecificWorker::newAprilTag(const tagsList &tags)
// {
//   //std::cout<<tags[0].id<<endl;
//   
//     if (tag.empty == true){
//       int i=0;
//       
//       
//       for (auto t : tags){
// 	QVec tr = innermodel->transform("world", QVec::vec3(tags[i].tx, 0, tags[i].tz), "rgbd");
// 	if (caja == true){
// 	  if (tags[i].id > 10){
// 	    tag.id = tags[i].id;
// 	    tag.x = tr.x();
// 	    tag.z = tr.z();
// 	    tag.empty = false;
// 	    return;
// 	  }
// 	  else{
// 	    i++;
// 	  }
// 	}
// 	if (caja == false){
// 	  if (tags[i].id < 10){
// 	    tag.id = tags[0].id;
// 	    tag.x = tr.x();
// 	    tag.z = tr.z();
// 	    tag.empty = false;
// 	    return;
// 	  }
// 	}
//    }
// 	
// 	
//   }
// }






