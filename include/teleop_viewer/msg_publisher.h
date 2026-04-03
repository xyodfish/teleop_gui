#pragma once

#include <embosa.hpp>
#include <galbot/sensor_proto/joy.pb.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace omnilink::teleop_viewer {

class MsgPublisher {
   public:
    bool initRcVirtualJoyWriter(galbot::embosa::Node* node, const std::string& topic);
    bool publishRcVirtualJoyButtons(const std::vector<std::pair<std::string, int>>& buttons);
    void reset();

   private:
    static std::shared_ptr<galbot::sensor_proto::Joy> buildJoyButtonsMessage(
        const std::vector<std::pair<std::string, int>>& buttons);

    std::string rc_virtual_joy_topic_;
    std::shared_ptr<galbot::embosa::SerializationWriter<galbot::sensor_proto::Joy>> rc_virtual_joy_writer_;
};

}  // namespace omnilink::teleop_viewer

