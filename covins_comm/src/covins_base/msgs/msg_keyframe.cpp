#include "covins_base/msgs/msg_keyframe.hpp"

namespace covins {

MsgKeyframe::MsgKeyframe() {
    //...
}

MsgKeyframe::MsgKeyframe(bool filesave)
    : save_to_file(filesave)
{
    //...
}

MsgKeyframe::MsgKeyframe(MsgTypeVector msgtype)
    : msg_type(msgtype)
{
    //...
}

auto MsgKeyframe::SetMsgType(int msg_size)->void {
    msg_type[0] = msg_size;
    msg_type[1] = (int)is_update_msg;
    msg_type[2] = id.first;
    msg_type[3] = id.second;
    msg_type[4] = 0;
}

auto MsgKeyframe::SetMsgType(MsgTypeVector msgtype)->void {
    msg_type = msgtype;
}

} //end ns
