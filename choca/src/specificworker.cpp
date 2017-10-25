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
  
  
  RoboCompDifferentialRobot::TBaseState bstate;
  QVec auxLinea;
  
//   std::cout << coord.first << coord.second << endl;
  laserData = laser_proxy->getLaserData();
  differentialrobot_proxy->getBaseState(bstate);
  innermodel->updateTransformValues("base", bstate.x, 0, bstate.z, 0, bstate.alpha, 0);
  
  switch( state )

 {

   case State::IDLE:

    if ( target.isEmpty() == false ){
      

      auxLinea = QVec::vec3(bstate.x, 0, bstate.z);
      
      std::pair<float, float> coord = target.extraerCoord();
      QVec t = QVec::vec3(coord.first, 0, coord.second);
      
      linea = QLine2D( auxLinea, t);
      state = State::GOTO;
    }
    break;

   case State::GOTO:

    gotoTarget(laserData);

    break;

   case State::BUG:

    bug(laserData, bstate);

    break;

  }
}
  
  
void SpecificWorker::gotoTarget(const TLaserData &laserData)
{
  float vAdv;
  float vRot;
  
  std::pair<float, float> coord = target.extraerCoord();
  

  
  if(obstacle(laserData) == true)   // If ther is an obstacle ahead, then transit to Bug
   {
      state = State::BUG;
      return;
   }
    QVec tr = innermodel->transform("base", QVec::vec3(coord.first, 0, coord.second), "world");
    const float MAXADV = 400;
    const float MAXROT=0.5;
    float angulo;
    float distanciaObjetivo;
    distanciaObjetivo = tr.norm2();
    angulo = atan2 ( tr.x(),tr.z());
    
    if ( distanciaObjetivo < 50 ){
      
      state = State::IDLE;
      target.setEmpty(false);
      differentialrobot_proxy->setSpeedBase(0,0);
      return;
      
    }else{
      
      float vAdv = distanciaObjetivo;
      float vrot = angulo;
      if(vrot > MAXROT)
	vrot=MAXROT;
      if(vrot< -MAXROT)
	vrot=-MAXROT;
      vAdv = MAXADV*f1(vAdv)*f2(vrot,0.9,0.1);
      differentialrobot_proxy->setSpeedBase(vAdv,vrot);
      
  }
  
//   if (target.isEmpty() == false){
//       
//     QVec tr = innermodel->transform("base", QVec::vec3(coord.first, 0, coord.second), "world");
//     float d = tr.norm2();
//     if (d > 50){
//   //       vAdv = d;
//   //       if (vAdv > MAX_ADV)
//   // 	vAdv = MAX_ADV;
//       
//       vRot = atan2(tr.x(), tr.z());
//       if (vRot > MAX_ROT)
// 	vRot = MAX_ROT;
//     
//       if (vRot < -MAX_ROT)
// 	vRot = -MAX_ROT;
// 
//       
//       vAdv = MAX_ADV * sigmoid(d)  * gaussian(vRot, 0.3, 0.5);
//       differentialrobot_proxy->setSpeedBase(vAdv,vRot);
//     }
//     else{
//       state = State::IDLE;
//       differentialrobot_proxy->setSpeedBase(0, 0);
//       target.setEmpty(true);
//       return;
//     }
//   }
}

void SpecificWorker::bug(const TLaserData &tLaser, const TBaseState& bState)
{
//   std::sort(laserData.begin()+20, laserData.end()-20,[](auto a,auto b){return a.dist<b.dist;});
//   
//   differentialrobot_proxy->setSpeedBase(0, 0.3);
//   if (laserData[20].dist > 300){
//     state = State::IDLE;
  
  if( obstacle(tLaser) == false){
    
    const float alpha = log ( 0.1 ) /log ( 0.3 ); 
    float distanciaObstaculo = obstacleLeft(tLaser);
    float diffToline = distanceToLine(bState);
  
    
    if (targetAtSight(tLaser)){
	state = State::GOTO;
	return;
      }      
    
  
    if (distanciaAnterior < 100 and diffToline < 0){
      state = State::GOTO;
      return;
    }
    
      
    float k = 0.1; 
    float vrot =  -((1./(1. + exp(-k*(distanciaObstaculo - 450.))))-1./2.);
    float vadv = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); 		
	
    differentialrobot_proxy->setSpeedBase ( vadv ,vrot );
    
  }else{
    differentialrobot_proxy->setSpeedBase(0, 0.3); 
  }
}

bool SpecificWorker::obstacle(RoboCompLaser::TLaserData laserData)
{
std::sort(laserData.begin()+35, laserData.end()-35,[](auto a,auto b){return a.dist<b.dist;});

 if( laserData[35].dist < 350)
   return true;
 else
   return false;

}

bool SpecificWorker::targetAtSight(RoboCompLaser::TLaserData laserCopy)
{
QPolygonF polygon;
for (auto l: laserCopy){
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

void SpecificWorker::setPick(const Pick& myPick)
{
  //std::cout << myPick.x << myPick.z << endl;
  target.insertarCoord(myPick.x, myPick.z);
}

float SpecificWorker::obstacleLeft(const TLaserData &tLaser){
  
  const int laserpos = 85;
  float min = tLaser[laserpos].dist;
  for(int i=laserpos-2; i<laserpos+2;i++)
    {
      if (tLaser[i].dist < min)
	min = tLaser[i].dist;
    }
    return min;
}

float SpecificWorker::distanceToLine(const TBaseState &bState){
  
  QVec posicion = QVec::vec3(bState.x, 0., bState.z);
  float distanciaActual = fabs(linea.perpendicularDistanceToPoint(posicion));
  float diferencia = distanciaActual - distanciaAnterior;
  distanciaAnterior = distanciaActual;
  
  return diferencia;
  
}

float SpecificWorker::f1(float d)
{
  return (1/(1+exp(-d)-0.5));
}


float SpecificWorker::f2(float r,float h, float Vx)
{
  float y;
  
  y=(-pow(Vx,2))/log(h);
  return exp((-pow(r,2))/y);
  
}





