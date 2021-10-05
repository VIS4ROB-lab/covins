#include "covins_base/msgs/msg_landmark.hpp"

namespace covins {

MsgLandmark::MsgLandmark() {
    //...
}

MsgLandmark::MsgLandmark(bool filesave)
    : save_to_file(filesave)
{
    //...
}

MsgLandmark::MsgLandmark(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgLandmark::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id.first;
    msg_type[3] = id.second;
    msg_type[4] = 1;
}

auto MsgLandmark::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

} //end ns
