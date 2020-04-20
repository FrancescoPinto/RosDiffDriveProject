#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>

namespace gazebo
{

    enum {
        RIGHT,
        LEFT,
    };

  class ModelEncoder : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      //initialize variables
      this->model = _parent;

      this->joints.resize(2);

      this->joints[LEFT] = this->model->GetJoint("left_joint");
      this->joints[RIGHT] = this->model->GetJoint("right_joint");

      //extra functionality (not required by specifications) to check the Center of Gravity automatically! <- actually we compute the Center of Mass, however in our case they are coincident
        physics::Link_V allLinks = this->model->GetLinks();
        gazebo::math::Vector3  CoG(0.0,0.0,0.0);
        double mass = 0.0;
        double nLinks = 0.0;

        for (std::vector<physics::LinkPtr>::iterator t = allLinks.begin(); t!=allLinks.end(); ++t)
        {
      
           physics::InertialPtr inertial = (**t).GetInertial();
          //  gazebo::math::Vector3 linkCoG = (*inertial).GetCoG();
         
          //by default, CoG coincides with initial relative pose
            gazebo::math::Vector3 linkCoG =  (**t).GetInitialRelativePose().pos;
            double linkMass = (*inertial).GetMass();
            mass += linkMass;
            std::cout << "---COMPONENT NAME: "<<(**t).GetName() << "------------------"<<std::endl;
            std::cout << "Mass component: " << linkMass << std::endl;
            std::cout << "Center of gravity (relative) : (" << linkCoG.x << " " << linkCoG.y << " " << linkCoG.z << ")" << std::endl;

            CoG += linkCoG*linkMass; //division will be made later
            std::cout << "Current Total cog (not normalized): ("<< CoG.x << " " << CoG.y << " " << CoG.z <<")" << std::endl;
        }

        CoG = CoG/mass;
        std::cout << "Total mass: " << mass << std::endl;
        std::cout << "Center of gravity (absolute respect to 0,0,0 : " << CoG.x << " " << CoG.y << " " << CoG.z+0.018 << std::endl; //correction of the 18 mm gap between Gazebo world origin and Inventor model origin
        
        //ideally, should get a parameter UpdatePeriod in the xml plugin tag, however, it is not required by the project specification
        //ideally: gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 20.0 );
        this->updatePeriod = 1.0/20.0; //default = 20Hz
        this->lastUpdateTime = model->GetWorld()->GetSimTime();

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelEncoder::OnUpdate,this));

        std::cout <<"Plugin lanciato"<< std::endl;

    }

    public: void OnUpdate()
    {

       common::Time currentTime = this->model->GetWorld()->GetSimTime();
       double secondsSinceLastUpdate = (currentTime - lastUpdateTime).Double();

       if (secondsSinceLastUpdate > updatePeriod) {
         double currentAngularVelocity[2];
         currentAngularVelocity[LEFT] = this->joints[LEFT]->GetVelocity(0);
         currentAngularVelocity[RIGHT] = this->joints[RIGHT]->GetVelocity(0);
                 //REMARK: i and currentTime can be used to check that there are 20 measurements/s (20Hz)
                 //WARNING: during startup, frequency might be < 20 Hz, however after startup it sticks to 20Hz (if the PC used is powerful enough)
         std::cout << "i:"<<i%20<< "|" << currentTime << " ! Left Wheel Velocity " << currentAngularVelocity[LEFT] << ", Right Wheel Velocity "
                   << currentAngularVelocity[RIGHT] << std::endl;
         this->lastUpdateTime += common::Time(updatePeriod);
         i++;
       }

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    protected: std::vector<physics::JointPtr> joints;
    protected: common::Time lastUpdateTime;
    protected: double updatePeriod;
    //counter used in order to check the frequency is actually 20Hz 
    protected: int i = 0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelEncoder)
}
