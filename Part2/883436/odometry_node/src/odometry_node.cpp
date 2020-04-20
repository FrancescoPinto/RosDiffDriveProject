#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>     
#include <stdio.h>
#include <odometry_node/customOdom.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/Pose.h>
#include<sensor_msgs/Imu.h>
#include<fstream>
#include <cmath>       

class odometry_class
{
    public:
    odometry_class(double x, double y, double z, double qx, double qy, double qz, double qw, double tune_linear, double tune_angular_enc, double tune_angular_imu, bool corrupted_bag){

        //inizializzazione della posa
      //posiziona base_link rispetto ad odom, the correction has been moved to the publishTransform/publishOdom methods
        encoder_pose.pose.position.x = 0.0;//0.17;
        encoder_pose.pose.position.y = 0.0;
        encoder_pose.pose.position.z = 0.0;//-0.034;
        encoder_pose.pose.orientation.x = 0.0;
        encoder_pose.pose.orientation.y = 0.0;
        encoder_pose.pose.orientation.z = 0.0;
        encoder_pose.pose.orientation.w = 1.0;
       // timestamp inizializzarlo
        encoder_last_time = encoder_current_time = ros::Time::now();
        tf::Quaternion q(qx,qy,qz,qw);
        double roll, pitch,yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        encoder_theta = yaw;

        publishTransform();//questa chiamata si può evitare, giacché il callback di vel non viene invocato fino al movimento del robot, la trasformazione non viene ripubblicata

        // subscribe
        vel_subscriber = n.subscribe("/vel", 1000, &odometry_class::velCallback, this);
        imu_subscriber = n.subscribe("/imu",1000,&odometry_class::imuCallback, this);
        pose_subscriber = n.subscribe("/robot_markerset/pose",1000,&odometry_class::poseCallback,this); //was made for parameter tuning

        //advertise
        odom_msg_publisher = n.advertise<nav_msgs::Odometry>("/odom", 50);
        custom_odom_msg_publisher = n.advertise<odometry_node::customOdom>("/custom_odom",50);

        //parameters initialization
        tuning_linear = tune_linear;
        tuning_angular_enc = tune_angular_enc;
        tuning_angular_imu = tune_angular_imu;
        is_corrupted_bag = corrupted_bag;
       
    }

 

    void publishOdometryMessage(){
        //by default use encoders data
        ros::Time published_time;
        published_pose = encoder_pose;
        published_time = encoder_current_time;
        double angularz = encoder_angularz;
        double dx = encoder_dx;
        double dy = encoder_dy;
        double dt = encoder_dt;
        ros::param::get("/odom_source",odomType); //put param in odomType variable, default encoders if not found
        if(odomType.compare("imu") == 0){
            angularz = imu_angularz;
            //to check whether the imu data is being used!
            ROS_INFO("**************************************\n\n\n\n\nIMU****************************");
       }

       //standard odom message initialization and publishing
        odom.header.stamp = published_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link"; 

        odom.pose.pose.position.x = published_pose.pose.position.x;
        odom.pose.pose.position.y = published_pose.pose.position.y;
        odom.pose.pose.position.z = published_pose.pose.position.z;
    
        odom.pose.pose.orientation.x = published_pose.pose.orientation.x;
        odom.pose.pose.orientation.y = published_pose.pose.orientation.y;
        odom.pose.pose.orientation.z = published_pose.pose.orientation.z;
        odom.pose.pose.orientation.w = published_pose.pose.orientation.w;
    
        odom.twist.twist.angular.z = angularz;
        odom.twist.twist.linear.x = dx/dt;
        odom.twist.twist.linear.y = dy/dt;

        odom_msg_publisher.publish(odom);

        odometry_node::customOdom custom_odom_msg;
        ros::param::get("/odom_source",odomType);
        custom_odom_msg.odom_source = odomType; 
        custom_odom_msg.odometry.header.stamp = published_time;
        custom_odom_msg.odometry.header.frame_id = "odom";
        custom_odom_msg.odometry.child_frame_id = "base_link"; 

        custom_odom_msg.odometry.pose.pose.position.x = published_pose.pose.position.x;
        custom_odom_msg.odometry.pose.pose.position.y = published_pose.pose.position.y;
        custom_odom_msg.odometry.pose.pose.position.z = published_pose.pose.position.z;
    
        custom_odom_msg.odometry.pose.pose.orientation.x = published_pose.pose.orientation.x;
        custom_odom_msg.odometry.pose.pose.orientation.y = published_pose.pose.orientation.y;
        custom_odom_msg.odometry.pose.pose.orientation.z = published_pose.pose.orientation.z;
        custom_odom_msg.odometry.pose.pose.orientation.w = published_pose.pose.orientation.w;
    
        custom_odom_msg.odometry.twist.twist.angular.z = angularz;
        custom_odom_msg.odometry.twist.twist.linear.x = dx/dt;
        custom_odom_msg.odometry.twist.twist.linear.y = dy/dt;

        custom_odom_msg_publisher.publish(custom_odom_msg);
    }

    void publishTransform(){
        ros::param::get("/odom_source",odomType); //put param in odomType variable, default encoders if not found
        ros::Time published_time;

        old_published_pose = published_pose;
        //old_published_time = published_time;
        old_published_yaw = published_yaw;
        published_pose = encoder_pose;
        published_time = encoder_current_time;
        published_yaw = encoder_theta;

        tf::Transform odom_to_base_link;
        odom_to_base_link.setOrigin(tf::Vector3(published_pose.pose.position.x + 0.17,published_pose.pose.position.y,published_pose.pose.position.z-0.034));
        tf::Quaternion q1(published_pose.pose.orientation.x,published_pose.pose.orientation.y,published_pose.pose.orientation.z,published_pose.pose.orientation.w);
        //q1.setRPY(0,0,published_yaw);
        odom_to_base_link.setRotation(q1);
        transform_broadcaster.sendTransform (tf::StampedTransform(odom_to_base_link, published_time, "odom", "base_link"));
 
    }



void velCallback(const geometry_msgs::TwistStamped::ConstPtr& vel){

      //TENTATIVO, VANO, DI RISOLVERE LA CORRUZIONE DELLA BAG DELL'IMU ... CI HO RINUNCIATO 
      if(is_corrupted_bag == true && parity_discarder%2 == 0){
          parity_discarder++;
          return;
      }
       // encoder_dt = (vel->header.stamp - encoder_last_time).toSec(); 
        encoder_current_time = ros::Time::now();
        encoder_dt = (encoder_current_time - encoder_last_time).toSec(); 
       // if (encoder_dt < 0.0001)
       //     return;

        ros::param::get("/odom_source",odomType); //put param in odomType variable, default encoders if not found
        
        double angular_v = 0.0;
        double angular_dt = 0.0;
        double tuning_angular = 1.0;
        if(odomType.compare("encoders") == 0){
            angular_v = vel->twist.angular.z;
            angular_dt = encoder_dt;
            tuning_angular = tuning_angular_enc;
        }else{
            angular_v = imu_angularz;
            angular_dt = encoder_dt;
            tuning_angular = tuning_angular_imu;
            //angular_dt = imu_dt;
        }

        if (fabs(angular_v) < 1e-6){
             //runge-kutta, see old projects slides ...
            encoder_dtheta =angular_v*0.5*angular_dt;
            encoder_dx =vel->twist.linear.x*cos(encoder_theta + encoder_dtheta)*encoder_dt*tuning_linear;
            encoder_dy =vel->twist.linear.x*sin(encoder_theta + encoder_dtheta)*encoder_dt*tuning_linear;
            encoder_dtheta = angular_v*angular_dt*tuning_angular;
         }
        else
        {
            /// Exact integration 
            const double old_encoder_theta = encoder_theta;
            const double r = vel->twist.linear.x/angular_v;
            const double temp_encoder_theta = old_encoder_theta + angular_v*angular_dt*tuning_angular;
            encoder_dx = r*(sin(temp_encoder_theta)-sin(old_encoder_theta))*tuning_linear;
            encoder_dy = -r*(cos(temp_encoder_theta)-cos(old_encoder_theta))*tuning_linear;
            encoder_dtheta = angular_v*angular_dt*tuning_angular;

        }

     //   ROS_INFO("angular_v = %f, angular_dt = %f, imu_angularz = %f", angular_v, angular_dt, imu_angularz);
    /*    ROS_INFO("SMALL: dx = %f, dy = %f, dtheta = %f", 
        vel->twist.linear.x*cos(encoder_theta + encoder_dtheta)*encoder_dt*tuning_linear,
        vel->twist.linear.x*sin(encoder_theta + encoder_dtheta)*encoder_dt*tuning_linear,
         angular_v*angular_dt*tuning_angular);*/

   //     ROS_INFO("dx = %f, dy = %f, dtheta = %f", encoder_dx, encoder_dy, encoder_dtheta);

        //rimozione di valori discontinui, presenti all'avvio dei sensori!
        //in ogni caso, assumendo che il robot non faccia salti o scatti strani, è assolutamente
        //implausibile che si muova di più di 40cm in una direzione da un istante all'altro! o che
        //compia rotazioni di più di 20 radianti ...
        if(fabs(encoder_dtheta) > 20 || fabs(encoder_dx) > 0.4 || fabs(encoder_dy) > 0.4){
            encoder_dtheta = 0.0;
            encoder_dx = 0.0; 
            encoder_dy = 0.0;
            //don't update theta
            ROS_INFO("***********WRONG_SENSOR_VALUES: deleting the update");
        }else{
            encoder_theta += encoder_dtheta;
        }

        encoder_pose.pose.position.x += encoder_dx; //vettore x,y,z
        encoder_pose.pose.position.y += encoder_dy;
        
        tf::Quaternion q; 
        q.setRPY(0,0,encoder_theta);

        encoder_pose.pose.orientation.x = q.x();
        encoder_pose.pose.orientation.y = q.y();
        encoder_pose.pose.orientation.z = q.z();
        encoder_pose.pose.orientation.w = q.w();

        encoder_pose.header.stamp = encoder_current_time;
        encoder_angularz = angular_v;
  
            publishOdometryMessage();
            publishTransform();
       
        encoder_last_time = encoder_current_time;
            
      
}


//codice del tutto inutile, pensavo di usarlo per fare una stima statistica dei parametri di correzione, ma non funziona
 void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
     double true_dx = poseMsg->pose.position.x - old_pose_msg.pose.position.x;
     double odom_dx = encoder_dx;//published_pose.pose.position.x - old_published_pose.pose.position.x;
     double true_dy =  poseMsg->pose.position.y - old_pose_msg.pose.position.y;
     double odom_dy = encoder_dy;
     double odom_dtheta = encoder_dtheta;
     tf::Matrix3x3 m(tf::Quaternion(poseMsg->pose.orientation.x,poseMsg->pose.orientation.y,poseMsg->pose.orientation.z,poseMsg->pose.orientation.w));
     double roll, pitch, yaw;
     m.getRPY(roll, pitch, yaw);
     double true_dtheta = yaw-old_theta;


  //   ROS_INFO("true_dx = %f  odom_dx = %f  ratiox = %f",true_dx, odom_dx, true_dx/odom_dx);
   //  ROS_INFO("true_dy = %f  odom_dy = %f  ratioy = %f",true_dy, odom_dy, true_dy/odom_dy);
  //  ROS_INFO("true_dtheta = %f  odom_dtheta = %f  ratioy = %f",true_dtheta, odom_dtheta, true_dtheta/odom_dtheta);
    double linearX = sqrt((true_dx*true_dx + true_dy*true_dy))/sqrt((odom_dx*odom_dx + odom_dy*odom_dy));
      if(!std::isinf(linearX)){
        N++;
        linearXratio += (linearX - linearXratio)/N;
        thetaRatio += (true_dtheta/odom_dtheta - thetaRatio)/N;
      }
    //ROS_INFO("********************linearX = %f **************************",linearX);
    //ROS_INFO("********************N*linearXratio = %f **************************",N*linearXratio);
  //  ROS_INFO("********************N*linearXratio + linearX = %f **************************",N*linearXratio + linearX);
  //  ROS_INFO("********************N = %d **************************",N+1);
 //   ROS_INFO("********************linearXratio = %f **************************",linearXratio);

 //   ROS_INFO("******************angularX = %f", true_dtheta/odom_dtheta);
 //   ROS_INFO("******************angularRatio = %f", thetaRatio);
    //FARE QUALCOSA PER CALCOLARE BENE I VALORI MEDI ... se riesci a calcolarli puoi usarli per il parameter tuning e ammen!
       //   std::cout << "true_dy = " << poseMsg->pose.position.y - old_pose_msg.pose.position.y << " odom_dy = "<< published_pose.pose.position.y - old_published_pose.pose.position.y<<std::endl;

     //linearXratio = sqrt(pow(poseMsg->pose.position.x - old_pose_msg.pose.position.x,2) + pow(poseMsg->pose.position.y - old_pose_msg.pose.position.y,2))/
     //sqrt(pow(published_pose.pose.position.x - old_published_pose.pose.position.x,2) + pow(published_pose.pose.position.y - old_published_pose.pose.position.y,2));

    //write to a file
    //linearXratio = //(N*linearXratio + 
   // sqrt(pow(poseMsg->pose.position.x - true_old_x,2) + pow(poseMsg -> pose.position.y - true_old_y,2))/
  //  (sqrt(pow(encoder_dx,2)+pow(encoder_dy,2)));//)/(N+1);
   // angularZratio = //(N*angularZratio +
  //   (poseMsg->pose.orientation.z - true_old_theta)/(encoder_dtheta);///(N+1);
  
   // double linearYratio = (poseMsg -> pose.position.y - true_old_y)/(encoder_dy);
  //  std::cout << N << "LinearXRatio " << linearXratio << " AngularZRatio " << angularZratio << std::endl;
    //ratio between actual increment (obtained as difference between true new pose - true old pose) and plain odometry estimate

    old_pose_msg = *(poseMsg);
    old_theta = yaw;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){

     if(is_corrupted_bag == true && parity_discarder%2 == 0){
         parity_discarder++;
          return;
      }
   //imu_dt = (imu_current_time - imu_last_time).toSec(); 
 
   imu_angularz = -imu->angular_velocity.x - 0.31; 
   
   ROS_INFO("*********IMU, %f",imu_angularz);

  //  imu_last_time = imu_current_time;

}

   
    private:
    ros::NodeHandle n; //private node handle
    tf::TransformBroadcaster transform_broadcaster; //create the subscriber and tf broadcaster
    ros::Subscriber vel_subscriber, imu_subscriber,pose_subscriber; //todo usare MultiThread spin per gestire più subscriber!!!
    ros::Publisher odom_msg_publisher, custom_odom_msg_publisher;

    double encoder_dt = 0.0, encoder_dtheta =0.0, encoder_dx = 0.0, encoder_dy =0.0, encoder_theta =0.0, encoder_angularz = 0.0;
    double imu_dt =0.0, imu_dx =0.0, imu_dy,imu_dz =0.0, vg_x =0.0,vg_y =0.0,vg_z =0.0, imu_angularz =0.0,imu_theta = 0.0;

    std::string odomType;
    geometry_msgs::PoseStamped stamped_actual_pose_;
    geometry_msgs::PoseStamped encoder_pose, imu_pose, old_published_pose, published_pose, old_pose_msg;
    double published_yaw = 0.0;
    nav_msgs::Odometry odom; 
    //quaternion transformation matrix from local to global frame
    

    bool first_time = true, is_corrupted_bag = false;
    int parity_discarder = 1;
    ros::Time imu_last_time, encoder_last_time;
    ros::Time encoder_current_time, imu_current_time;
    double tuning_linear = 1.0, tuning_angular_enc = 1.0, tuning_angular_imu= 1.0;
    int N = 1;
    double linearXratio = 0.0, angularZratio = 0.0, old_published_yaw = 0.0, old_theta = 0.0, thetaRatio = 0.0;
    

};


int main(int argc, char **argv)
{

  for(int i = 0; i <= argc; i++){
      ROS_INFO("ARGS: %s",argv[i]);
  }

  //from 2 to 7 we have the initialization x,y,z, q.x,q.y,q.z,q.w, then the tuning params and whether the bag is the imu (corrupted) bag

  ros::init(argc, argv, "odometry_node");
  std::setprecision(14);
  double x = std::atof(argv[1]);
  double y = std::atof(argv[2]);
  double z = std::atof(argv[3]);
  double qx = std::atof(argv[4]);
  double qy = std::atof(argv[5]);
  double qz = std::atof(argv[6]);
  double qw = std::atof(argv[7]);
  double tune_linear = std::atof(argv[8]);
  double tune_angular_enc = std::atof(argv[9]);
  double tune_angular_imu = std::atof(argv[10]);
  bool corrupted_bag = true;
  if(std::string(argv[11]).compare("false") == 0){
      corrupted_bag = false;
  }
 
  static odometry_class odometry(x,y,z,qx,qy,qz,qw,tune_linear,tune_angular_enc, tune_angular_imu, corrupted_bag);
  ros::spin();
  return 0; 
  
}


 


 