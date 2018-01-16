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
  MotorGoalPosition pos;
  pos.name = "wrist_right_2";
//   pos.maxSpeed = 1.0;
  pos.position = 1.5;
  jointmotor_proxy->setPosition(pos);
  
  pos.name = "shoulder_right_2";
//   pos.maxSpeed = 1.0;
  pos.position = -1.0;
  jointmotor_proxy->setPosition(pos);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


  innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml");

  timer.start(Period);


  return true;
}

void SpecificWorker::compute()
{
  
  
  RoboCompDifferentialRobot::TBaseState bstate;
  
//   std::cout << coord.first << coord.second << endl;
  laserData = laser_proxy->getLaserData();
  differentialrobot_proxy->getBaseState(bstate);
  innermodel->updateTransformValues("robot", bstate.x, 0, bstate.z, 0, bstate.alpha, 0);
  
  
   
  
  switch( state )

 {

   case State::IDLE:

   if ( target.isEmpty() == false )

    state = State::GOTO;

    break;

   case State::GOTO:

    gotoTarget();

    break;

   case State::BUG:

    bug();

    break;
    
   case State::PICK:
     std::cout<<"PICKING BOX"<<endl;
     state = State::IDLE;
     usleep(2000000);
     target.setEmpty();
     break;
     
   case State::RELEASE:
     std::cout<<"RELEASING BOX"<<endl;
     state = State::IDLE;
     usleep(2000000);
     target.setEmpty();
     break;

  }
}
  
  
void SpecificWorker::gotoTarget()
{
  float vAdv;
  float vRot;
  
  std::pair<float, float> coord = target.extraerCoord();
  

  
  if(obstacle() == true)   // If ther is an obstacle ahead, then transit to Bug

   {

      state = State::BUG;

      return;

   }
  
  if (target.isEmpty() == false){
    
    auto tags = getapriltags_proxy->checkMarcas();
    
    if (tags.size() != 0){
      usleep(300);
      differentialrobot_proxy->setSpeedBase(0, 0);
      state = State::PICK;
      return;
    }
      
    
//     QVec tr = innermodel->transform("world", QVec::vec3(tags[i].tx, 0, tags[i].tz), "rgbd");
      
    QVec tr = innermodel->transform("robot", QVec::vec3(coord.first, 0, coord.second), "world");
    float d = tr.norm2();
    if (d > 400){
  //       vAdv = d;
  //       if (vAdv > MAX_ADV)
  // 	vAdv = MAX_ADV;
      
      vRot = atan2(tr.x(), tr.z());
      if (vRot > MAX_ROT)
	vRot = MAX_ROT;
    
      if (vRot < -MAX_ROT)
	vRot = -MAX_ROT;

      
      vAdv = MAX_ADV * sigmoid(d)  * gaussian(vRot, 0.5, 0.5);
      differentialrobot_proxy->setSpeedBase(vAdv,vRot);
    }
    else{
      state = State::RELEASE;
      differentialrobot_proxy->setSpeedBase(0, 0);
//       target.setEmpty();
      return;
    }
  }
}

void SpecificWorker::bug()
{
  float giro = 1.0;
  
  std::sort(laserData.begin()+8, laserData.end()-8,[](auto a,auto b){return a.dist<b.dist;});
  
  
  if (laserData[8].dist < 350 || laserData[laserData.size()-8].dist < 350){
    
    if (laserData[8].angle < 0)
      giro = 1.0;
    else
      giro = -1.0;
    
    differentialrobot_proxy->setSpeedBase(0, 0.3*giro);
  }
  else{
    differentialrobot_proxy->setSpeedBase(100, 0);
    usleep(200000);
    state = State::GOTO;
  }
  
  if (targetAtSight()){
//     differentialrobot_proxy->setSpeedBase(300, 0);
//     usleep(1000000);
    state = State::GOTO;
  }
  
  
}

bool SpecificWorker::obstacle()
{
std::sort(laserData.begin()+8, laserData.end()-8,[](auto a,auto b){return a.dist<b.dist;});

 if( laserData[8].dist < 350 || laserData[laserData.size()-8].dist < 350){
   return true;
 }
 else{
   return false;
 }
}

bool SpecificWorker::targetAtSight()
{
QPolygonF polygon;
laserData = laser_proxy->getLaserData();
for (auto l : laserData){
   QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle);
   polygon << QPointF(lr.x(), lr.z());
}

std::pair<float, float> coord = target.extraerCoord();
QVec t = QVec::vec3(coord.first, 0, coord.second);

return  polygon.containsPoint( QPointF(t.x(), t.z() ), Qt::WindingFill );
}
    
    
    
  //     differentialrobot_proxy->setSpeedBase(200,0);
  //     TLaserData data = laser_proxy->getLaserData();
  //     std::sort(data.begin()+20, data.end()-20,[](auto a,auto b){return a.dist<b.dist;});
  //     if (data[20].dist < 300){
  //       if (rand() % 2 == 0){
  // 	giro = -0.8;
  //       }
  //       else{
  // 	giro = 0.8;
  //       }
  //       
  //       differentialrobot_proxy->setSpeedBase(0, giro);
  //       int tiempo = rand() % 10 + 1;
  //       usleep(tiempo*100000);
  //     }
      
    
  //   }
  //   for(auto d:data)
  //     qDebug()<<d.dist<<d.angle;
  //   differentialrobot_proxy->setSpeedBase(100,0);
    
  //   differentialrobot_proxy->setSpeedBase(0,0);
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
  


float SpecificWorker::sigmoid(float d)
{	
  return (1.f / (1.f + exp(-d))) - 0.5;
}

float SpecificWorker::gaussian(float vr, float vx, float h)
{
  float landa = -(vx*vx)/log(h);
  return exp(-(vr*vr)/landa);
}

void SpecificWorker::go(const string& nodo, const float x, const float y, const float alpha)
{
  target.insertarCoord(x, y);
}

void SpecificWorker::turn(const float speed)
{
  differentialrobot_proxy->setSpeedBase(0, speed);
}

bool SpecificWorker::atTarget()
{
  if (target.isEmpty() == true){
    return true;
  }else{
    return false;
  }
 //Calcular distancia
  // si menor que margen
      //return true
  //si no
      //return false
}

void SpecificWorker::stop()
{
  differentialrobot_proxy->setSpeedBase(0, 0);
}

void SpecificWorker::picking_box()
{
 
}

void SpecificWorker::releasing_box()
{
//   gotopoint_proxy->releasing_box();
}


void SpecificWorker::setPick(const Pick& myPick)
{
  //std::cout << myPick.x << myPick.z << endl;
  target.insertarCoord(myPick.x, myPick.z);
}





