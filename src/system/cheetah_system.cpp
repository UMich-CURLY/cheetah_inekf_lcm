#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <numeric>

CheetahSystem::CheetahSystem(lcm::LCM* lcm, boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer): 
    lcm_(lcm), ts_(0.05, 0.05), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx), estimator_(lcm) {
    
    std::cout << "Ready to read Yaml" << std::endl;
    // Yaml file:
    char resolved_path[PATH_MAX];
    char* p = realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_ = YAML::LoadFile(std::string(resolved_path) + "/config/settings.yaml");

    // Initialize inekf pose file printouts
    std::cout << "Reading inekf pose file names" << std::endl;
    std::string kitti_file_name_ = config_["settings"]["system_inekf_pose_filename"].as<std::string>();
    std::cout << "Reading tum inekf pose file names" << std::endl;
    std::string tum_file_name_ = config_["settings"]["system_inekf_tum_pose_filename"].as<std::string>();
    // Initialize pose publishing if requested
    std::cout << "Check enable_publisher: ";
    enable_pose_publisher_ = config_["settings"]["system_enable_pose_publisher"].as<bool>();
    std::cout << std::boolalpha << enable_pose_publisher_ << std::endl;

    // Define pose record step size for output files:
    pose_record_step_size = config_["settings"]["pose_record_step_size"].as<int>();
    step_size_count = 0;

    kitti_outfile.open(kitti_file_name_, std::ofstream::out);
    tum_outfile.open(tum_file_name_, std::ofstream::out);
    
}

CheetahSystem::~CheetahSystem() {
    kitti_outfile.close();
    tum_outfile.close();
}

void CheetahSystem::step() {
    bool hasUpdate = updateNextPacket();

    if (hasUpdate) {
        state_.set(cheetah_packet_);

        if (estimator_.enabled()) {
            estimator_.setContacts(state_);

            // estimator.update propagate and correct (if contact exists) the filter
            estimator_.update(cheetah_packet_, state_);

            if (enable_pose_publisher_) {
                poseCallback(state_);
            }

        } else {
            std::cout << "Initialized initState" << std::endl;
            if (estimator_.biasInitialized()) {
                estimator_.initState(cheetah_packet_.getTime(), cheetah_packet_, state_);
                estimator_.enableFilter();
            } else {
                estimator_.initBias(cheetah_packet_);
            }
        }
    }
}

void CheetahSystem::poseCallback(const CheetahState& state_) {
    
    if (step_size_count++ == pose_record_step_size) {
        // Writing new pose:
        kitti_outfile << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        
        // tum style
        tum_outfile << cheetah_packet_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;
        step_size_count = 0;
    }
}

// Private Functions

bool CheetahSystem::updateNextPacket() {
    //Copy data to be handled in queues (lock/unlock)
    bool hasUpdated = false;
    cdata_mtx_->lock();
    if (!cheetah_buffer_->timestamp_q.empty() &&
        !cheetah_buffer_->imu_q.empty() &&
        !cheetah_buffer_->joint_state_q.empty() &&
        !cheetah_buffer_->contact_q.empty()) 
    {
        hasUpdated = true;
        double timestamp = cheetah_buffer_->timestamp_q.front();
        cheetah_packet_.setTime(timestamp);
        cheetah_packet_.imu = cheetah_buffer_->imu_q.front();
        cheetah_packet_.joint_state = cheetah_buffer_->joint_state_q.front();
        cheetah_packet_.contact = cheetah_buffer_->contact_q.front();

        cheetah_buffer_->timestamp_q.pop();
        cheetah_buffer_->imu_q.pop();
        cheetah_buffer_->joint_state_q.pop();
        cheetah_buffer_->contact_q.pop();
    }
    cdata_mtx_->unlock();

    return hasUpdated;
}
