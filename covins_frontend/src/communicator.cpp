#include "communicator.hpp"

namespace covins {

Communicator::Communicator(std::string server_ip, std::string port)
    : CommunicatorBase()
{
    std::cout << "--> Connect to server" << std::endl;
    std::cout << "----> Server IP: " << server_ip << std::endl;
    std::cout << "----> Port     : " << port << std::endl;
    newfd_ = ConnectToServer(server_ip.c_str(),port);
    if(newfd_ == 2){
        std::cout << "FATAL: " << "Could no establish connection - exit" << std::endl;
        exit(-1);
    }
    std::cout << "newfd_: " << newfd_ << std::endl;
    std::cout << "--> Connected" << std::endl;
}

auto Communicator::GetOdomFrame()->TransformType {
    std::unique_lock<std::mutex> lock(mtx_odom_);
    return T_odom_correction_;
}

auto Communicator::ProcessAdditional()->void {

}

auto Communicator::ProcessKeyframeMessages()->void {
//    std::unique_lock<std::mutex> lock(mtx_in_);
//    while(!buffer_keyframes_in_.empty()) {
//        MsgKeyframeBase msg = buffer_keyframes_in_.front();
//        buffer_keyframes_in_.pop_front();
//        if(msg.is_update_msg){
//            std::cout << COUTNOTICE << "Drop KF update msg" << std::endl;
//            continue;
//        } else {
//            std::cout << "Received information for KF " << msg.id << std::endl;
//            if(msg.id.second != client_id_) {
//                std::cout << COUTFATAL << "expecting data with same client ID!" << std::endl;
//                exit(-1);
//            }
//            if(msg.id_reference != std::make_pair((size_t)0,(size_t)client_id_)) {
//                std::cout << COUTFATAL << "kf_ref is not the origin frame!" << std::endl;
//                exit(-1);
//            }
//        }
//    }
}

auto Communicator::ProcessLandmarkMessages()->void {

}

auto Communicator::ProcessNewKeyframes()->void {

}

auto Communicator::ProcessNewLandmarks()->void {

}

auto Communicator::Run()->void {
    std::thread thread_recv(&Communicator::RecvMsg, this);

    while(true)
    {
        this->ProcessBufferOut();
        this->ProcessBufferIn();
        if(this->TryLock()){
            this->ProcessKeyframeMessages();
            this->ProcessLandmarkMessages();
            this->ProcessNewKeyframes();
            this->ProcessNewLandmarks();
            this->ProcessAdditional();
            this->UnLock();
        }
        if(this->ShallFinish()){
            std::cout << "Comm " << client_id_ << ": close" << std::endl;
            break;
        }
        usleep(1000);

//        sleep(1);
//        std::cout << "Comm: Iterate" << std::endl;
    }

    std::cout << "Comm " << client_id_ << ": wait for recv thread" << std::endl;
    thread_recv.join();
    std::cout << "Comm " << client_id_ << ": leave" << std::endl;
}

auto Communicator::SetOdomFrame(TransformType T_world_odom)->void {
    std::unique_lock<std::mutex> lock(mtx_odom_);
    T_odom_correction_ = T_world_odom;
}

} //end ns
