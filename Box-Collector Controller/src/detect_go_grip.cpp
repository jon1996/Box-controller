/**
 *    Desc:
 *
 *    (C) 2017, Author: EMRE OZBILGE and Pascal Bahavu Atosha
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, see <http://www.gnu.org/licenses/>
 */

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <iomanip>
#include <vector> // std::vector
#include <algorithm> // min_element, max_element
#include <numeric> // accumulate
#include <algorithm> // min_element, max_element
#include <numeric> // accumulate
#include <cstdlib> // rand(), srand()
#include <ctime> // time()
#include<unistd.h> // usleep()

using namespace PlayerCc;


/* number of pixels away from the image centre a blob can be, to be in front of the robot.
 * This is essentially the margin of error.*/
const int margin = 5;
// define motion constants
const double LV_MAX=0.5; // maximum linear speed
const double AV_MAX=0.5; // maximum angular speed
// how close the robot can approach to the blob
const double CLOSE=0.30; //cm
// define finite state machine
int state=-1;
// constants for moving to target position
const double CLOSE_TARGET=0.20; // 20 cm
const double ANG_ERR=dtor(1); //1 degree


void wandering(double& lv,double& av,Position2dProxy &pp)
{
        //pp.SetMotorEnable (true);
        std::cout << std::setprecision(2) << std::fixed;
        srand(time(NULL));
        int max_turn=90;
        int min_turn=-90;
        double max_speed=0.5;
        unsigned int microsecond = 1000000;
            //[-min_turn,max_turn] in degree
            av=dtor((rand()%(max_turn-min_turn+1))+min_turn);
            // you need to generate a number between 0 and 1
            lv=((rand()%11)/10.0)*max_speed; // [0,max_speed]
            // set robot velocities
            pp.SetSpeed(lv,av);
            // wait 1 sec then change the velocities
            usleep(microsecond);
            std::cout<<"wanderin"<<"lv:"<<lv<<" av:"<<av<<std::endl;
       
}
void goToTarget(double& lv,double& av,const std::vector<double>& tar_pos,std::vector<double>& rob_pos)
{
    
    double dist2target;
    double delta_diff, net_diff;
    
    // calculate the distance to the target
    dist2target = std::sqrt((tar_pos[0]-rob_pos[0])*(tar_pos[0]-rob_pos[0])+(tar_pos[1]-rob_pos[1])*(tar_pos[1]-rob_pos[1]));
    
    // calculate angle between target position and the robot heading
    delta_diff = std::atan2(tar_pos[1]-rob_pos[1],tar_pos[0]-rob_pos[0])-rob_pos[2];
    
    // if delta_diff is greater/less than 180 degree adjust the net angle
    if (delta_diff>M_PI)
        net_diff=delta_diff-2*M_PI;
    else if (delta_diff<-M_PI)
        net_diff=delta_diff+2*M_PI;
    else
        net_diff=delta_diff;
    
    // set angular and linear velocities
    av=AV_MAX*net_diff;
    if (std::fabs(net_diff)>ANG_ERR)
        lv=0.0; // stop
        else
            lv=LV_MAX*(1.0-std::exp(-dist2target)); // faster when away
            
            // display some paramters
            std::cout<<"target:"<<"("<<tar_pos[0]<<","<<tar_pos[1]<<")"<<" dist:"<<dist2target<<" deg_err:"<<PlayerCc::rtod(net_diff)<<std::endl;
        
        // change the state if you reach the one
        if (dist2target<CLOSE_TARGET)
        {
            lv=0.0;
            av=0.0;
            state=3;
        }
}


void moveToBox(double& lv,double& av,const int& centre,const playerc_blobfinder_blob_t& blob)
{
    // How far is the detected blob
    std::cout<<"colour(rgb):"<<blob.color<<" range:"<<blob.range<<std::endl;
    
    //change the state to grip the object
    if (blob.range<CLOSE)
    {
        lv=0.0;
        av=0.0;
        state=1;
    }
    else {
        // the closer to the object the robot moves slower
        lv = LV_MAX * (1.05-std::exp( -std::sqrt((blob.range-CLOSE)*(blob.range-CLOSE))) );
        
        if(blob.x < centre-margin) {
            std::cout<<"blob is on my left\n";
            av=0.1;
        }
        else if(blob.x > centre+margin) {
            std::cout<<"blob is on my right\n";
            av=-0.1;
        }
        else {
            std::cout<<"blob is at front\n";
            av=0.0;
        }
    }// end of else for range>close
    
}

int main() {
    try
    {
    	PlayerClient robot("localhost");
	Position2dProxy pp(&robot, 0);
	RangerProxy lp(&robot,1);
 
        // define a BlobfinderProxy proxy to communicate with robot's gripper
        BlobfinderProxy bfp(&robot, 0);
        
        // define a gripper proxy to communicate with robot's gripper
        GripperProxy gp(&robot, 0);
        
        //change client data mode to receive always latest data
        robot.SetDataMode(PLAYER_DATAMODE_PULL);
        robot.SetReplaceRule(true);
        
        // only print two decimals after the point
        std::cout << std::setprecision(2) << std::fixed;
        
        //find the largest blob
        playerc_blobfinder_blob_t blob;
        int biggestBlobArea = 0;
        int biggestBlob = 0;
        int centre;
        // current status of the gripper
        bool gripopen = true;
        // robot current velocities
        double lv=0.0, av=0.0;
        // define collection area variables
        std::vector<double> coll_pos(2);
        coll_pos[0]=6;
        coll_pos[1]=-3;
        // robot current position
        std::vector<double> rob_pos(3);
        // position of the current released box
        std::vector<double> curr_pos(3);
        double dist;
        double curr_ang_err;
        int wait_cnt=0; // timer until gripper is open
        
        // initial state
        state=0;
        
        //obstacle
        std::puts ( "Waiting Laser sensors...\n" );
        while (lp.GetRangeCount()==0) {
            robot.Read();
        }
        std::puts ( "Laser is done!\n" );
        std::vector<double> temp_readings(180);
        const int SIZE_LP=lp.GetRangeCount();
        double l_field,r_field;
        double l_min, r_min;
        std::cout << std::setprecision(2) << std::fixed;
        
          // go into read-think-act loop
        
        bool changeState = false;
        for(;;)
        {
            // this blocks until new data comes; 10Hz by default
            robot.Read();
            
            // TODO: wandering behaviour , obstacle avoidance must be added to the FSM
            // TODO: must also check if the colour of the box is the requested one, if not continue to wander
            // copy the reading to the vector for easy processing
           for (int i=0; i<SIZE_LP; ++i)
                temp_readings[i]=lp[i];
            
            // laser readings pre-processing
            r_min=std::accumulate(temp_readings.begin(), temp_readings.begin()+80, 0.0)/80.0; // [0 to 80 deg]
            l_min=std::accumulate(temp_readings.begin()+100, temp_readings.begin()+180, 0.0)/80.0; //[110 to 180 deg]    
            
            // magnitude the field vector
            l_field=(100000*r_min)/500-100; 
            r_field=(100000*l_min)/500-100; 
            
            if (l_field>100)
                l_field=100;
            if (r_field>100)
                r_field=100;
            
            if(l_min < 1 || r_min < 1){
                state = 6;
            }
         
            
            if (state==0) {
                if(bfp.GetCount()!=0) {
                    
                    for(int i=0; i<bfp.GetCount(); i++)
                    {
                        //get blob from proxy
                        playerc_blobfinder_blob_t currBlob = bfp[i];
                        // here we are checking which blob is bigger because it is possible there is two blob next to each other
                        if( std::abs((int)currBlob.area) > biggestBlobArea)
                        {
                            biggestBlob = i;
                            biggestBlobArea = currBlob.area;
                        }
                    }// end of for
                    
                    blob = bfp[biggestBlob];
                    
                    // TODO: if the box is not purple of red, do not approach it
                    if(blob.color == 16711680 || blob.color == 10494192){
                        centre = bfp.GetWidth()/2.0;              
                        moveToBox(lv,av,centre,blob);
                    }
                    else{
                        wandering(lv,av,pp);
                    }
                    // find centre of image
                }
                else
                {
                    // TODO: if no box detect, call the wandering behaviour
                    wandering(lv,av,pp);
                    //wander();
                    //lv=0.0;
                    //av=0.0;
                    
                }
            }
            else if (state==1) {
                std::cout << "Closing gripper\n";
                gripopen=false; // change gripper status
                gp.Close();
                lv=0.0;
                av=0.0;
                state=2; // move to collection area
            }
            else if (state ==2) // move to the collection area
            {
                changeState = true;
                rob_pos[0]=pp.GetXPos();
                rob_pos[1]=pp.GetYPos();
                rob_pos[2]=pp.GetYaw();
                goToTarget(lv,av,coll_pos,rob_pos);
                
            }
            else if (state==3)
            {
                // you have to release the box now, then backward to leave the collection area
                std::cout << "Opening gripper\n";
                gripopen=true; // change gripper status
                gp.Open();
                lv=0.0;
                av=0.0;
                state=4;
                curr_pos.clear();
                // this is the position where I released the box
                curr_pos[0]=pp.GetXPos();
                curr_pos[1]=pp.GetYPos();
                curr_pos[2]=pp.GetYaw();  
                
                wait_cnt=0;
                changeState = false;
                
                
                double addedValueX =((rand()%11)/10.0);
                double addedValueY = ((rand()%11)/10.0);
                std::cout<<"addedValueX:"<<addedValueX<<" addedValueY:"<<addedValueY<<std::endl;
                coll_pos[0]=6 + addedValueX;
                coll_pos[1]=-3 + addedValueY;
            }
            else if (state==4)
            {
                
                if (wait_cnt>50) // wait until gripper is open
                {
                    // distance between the disc released location and the current robot location
                    dist=std::sqrt((pp.GetXPos()-curr_pos[0])*(pp.GetXPos()-curr_pos[0])+(pp.GetYPos()-curr_pos[1])*(pp.GetYPos()-curr_pos[1]));
                    
                    curr_ang_err=std::fabs((curr_pos[2]+M_PI)-pp.GetYaw());
                    if (curr_ang_err>M_PI)
                        curr_ang_err=curr_ang_err-2*M_PI;
                    else if (curr_ang_err<-M_PI)
                        curr_ang_err=curr_ang_err+2*M_PI;
                    
                    std::cout<<"dist:"<<dist<<" ang_err:"<<curr_ang_err<<std::endl;
                    
                    if (dist<0.3)
                    {   //backward
                        lv=-0.5;
                        av=0.0;
                    }
                    else if(curr_ang_err>dtor(10)) // else if (bfp.GetCount()!=0)
                    {   //turn right
                        lv=0.0;
                        av=0.2;
                    }
                    else
                    {
                        lv=0.0;
                        av=0.0;
                        state=5;
                    }  
                }
                
                wait_cnt++;
            }
            else if (state==5)
            {
                // TODO: call wandering behaviour
                // you have to change the state to go the beginning state
                wandering(lv,av,pp);
                state = 0;
            }
            else if(state == 6){
                lv=(r_field+l_field)/500; //  some portion of summing two vector magnitude provide linear speed
                av=(r_field-l_field); // robot turns the direction of the larger vector and turning rate is the different between two vectors
                av=dtor(av);  // convert the degree to radian
                pp.SetSpeed(lv,av);
                std::cout<<"AVOIDANCE . . ."<<std::endl;
                std::cout<<"l_min:"<<l_min<<" r_min:"<<r_min<<" l_field:"<<l_field<<" r_field:"<<r_field<<std::endl;
                
                if(l_min <= 1 || r_min <= 1){
                    if(changeState){
                        state = 2;
                    }
                    else{
                        state = 0;
                    }
                    
                }
            }
            pp.SetSpeed(lv,av);
            std::cout<<"state:"<<state<<"  lv:"<<lv<<"  av:"<<av<<std::endl;
            
        } //end of sense-think-act loop
        
        
    }
    catch (PlayerCc::PlayerError & e)
    {
        std::cerr << e << std::endl;
        return -1;
    }
    
    return 0;
    
}
