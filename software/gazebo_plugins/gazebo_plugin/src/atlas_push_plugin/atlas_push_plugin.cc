#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc/atlas_push_t.hpp"

namespace gazebo
{
	class AtlasPushPlugin : public ModelPlugin
	{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			this->model = _parent;
			this->world = _parent->GetWorld();

			this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&AtlasPushPlugin::OnUpdate, this));
			this->push_end = 0;
			this->current_time = 0;
			this->callback_queue_thread = boost::thread(boost::bind(&AtlasPushPlugin::QueueThread, this));
		}

		public: void OnUpdate()
		{
			this->current_time = this->world->GetSimTime().Double();
			// this->current_time = _info.simTime.Double();
			if (this->push_end > this->current_time) {
				this->model->SetLinearVel(this->velocity);
				// gzerr << "Trying to set linear vel" << std::endl;
				// gzerr << "Velocity: " << this->velocity.x << " " << this->velocity.y << " " << this->velocity.z << std::endl;
				// gzerr << "now: " << this->current_time << std::endl;
				// gzerr << "stop: " << this->push_end << std::endl;
			}
		}

		public: void QueueThread()
		{
			lcm::LCM lcm_subscribe_ ;
			if(!lcm_subscribe_.good()) {
				gzerr << "ERROR: lcm_subscribe_ is not good()" << std::endl;
			}

			lcm_subscribe_.subscribe("ATLAS_PUSH", &AtlasPushPlugin::on_push_msg, this);
			gzerr << "Launching AtlasPushPlugin LCM handler" << std::endl;
			while (0 == lcm_subscribe_.handle());
		}

		public: void on_push_msg(const lcm::ReceiveBuffer* buf, const std::string& channel, const drc::atlas_push_t* msg)
		{
			double duration = msg->duration;
			this->push_end = this->current_time + duration;
			this->velocity.x = msg->velocity.x;
			this->velocity.y = msg->velocity.y;
			this->velocity.z = msg->velocity.z;
		}	


		private: 
			physics::ModelPtr model;
			physics::WorldPtr world;
			math::Vector3 velocity;
		    double push_end;
		    double current_time;
		    boost::thread callback_queue_thread;
		    event::ConnectionPtr updateConnection;
	};

	GZ_REGISTER_MODEL_PLUGIN(AtlasPushPlugin)
}