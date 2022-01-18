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
    
    // std::cout << "Ready to read Yaml" << std::endl;
    // Yaml file:
    char resolved_path[PATH_MAX];
    char* p = realpath("../", resolved_path);
    // std::cout << resolved_path << std::endl;
    YAML::Node config_ = YAML::LoadFile(std::string(resolved_path) + "/config/settings.yaml");

    // Initialize inekf pose file printouts
    // Initialize pose publishing if requested
    enable_pose_log_txt_ = config_["settings"]["system_enable_pose_log_txt"] ? config_["settings"]["system_enable_pose_log_txt"].as<bool>() : false;
    std::string kitti_file_name_ = config_["settings"]["system_inekf_kitti_pose_filename"].as<std::string>();
    std::string tum_file_name_ = config_["settings"]["system_inekf_tum_pose_filename"].as<std::string>();
    pose_record_step_size_ = config_["settings"]["system_pose_record_step_size"] ? config_["settings"]["system_pose_record_step_size"].as<int>() : 1;
    
    std::cout << "----Cheetah System Coniguration----"<<std::endl;
    std::cout << "system_enable_pose_log_txt: " << std::boolalpha << enable_pose_log_txt_ << std::endl;
    std::cout << "system_pose_record_step_size: "<<pose_record_step_size_<<std::endl;
    std::cout << "system_inekf_kitti_pose_filename: "<<kitti_file_name_<<std::endl;
    std::cout << "system_inekf_tum_pose_filename: "<<tum_file_name_<<std::endl;
    std::cout << "-----------------------------------"<<std::endl;


    
    step_size_count_ = 0;

    if(enable_pose_log_txt_){
        kitti_outfile_.open(kitti_file_name_, std::ofstream::out);
        tum_outfile_.open(tum_file_name_, std::ofstream::out);
    }
    
}

CheetahSystem::~CheetahSystem() {
    if(enable_pose_log_txt_){
        kitti_outfile_.close();
        tum_outfile_.close();
    }
    
}

void CheetahSystem::step() {
    bool hasUpdate = updateNextPacket();

    if (hasUpdate) {
        state_.set(cheetah_packet_);

        if (estimator_.enabled()) {
            estimator_.setContacts(state_);

            // estimator.update propagate and correct (if contact exists) the filter
            estimator_.update(cheetah_packet_, state_);

            if (enable_pose_log_txt_) {
                poseCallback(state_);
            }

        } else {
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
    
    if (step_size_count_++ == pose_record_step_size_) {
        // Writing new pose:
        kitti_outfile_ << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        
        // tum style
        tum_outfile_ << cheetah_packet_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;
        step_size_count_ = 0;
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
