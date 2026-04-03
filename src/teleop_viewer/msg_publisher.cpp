#include "teleop_viewer/msg_publisher.h"

#include <chrono>

namespace omnilink::teleop_viewer {

bool MsgPublisher::initRcVirtualJoyWriter(galbot::embosa::Node* node, const std::string& topic) {
    if (!node) {
        return false;
    }
    rc_virtual_joy_topic_  = topic;
    rc_virtual_joy_writer_ = node->CreateWriter<galbot::sensor_proto::Joy>(rc_virtual_joy_topic_);
    return rc_virtual_joy_writer_ != nullptr;
}

bool MsgPublisher::publishRcVirtualJoyButtons(const std::vector<std::pair<std::string, int>>& buttons) {
    if (!rc_virtual_joy_writer_) {
        return false;
    }
    if (buttons.empty()) {
        return true;
    }

    auto msg = buildJoyButtonsMessage(buttons);
    if (!msg) {
        return false;
    }

    rc_virtual_joy_writer_->Publish(msg);
    return true;
}

void MsgPublisher::reset() { rc_virtual_joy_writer_.reset(); }

std::shared_ptr<galbot::sensor_proto::Joy> MsgPublisher::buildJoyButtonsMessage(
    const std::vector<std::pair<std::string, int>>& buttons) {
    auto msg = std::make_shared<galbot::sensor_proto::Joy>();
    auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec);

    for (const auto& [name, status] : buttons) {
        auto* button = msg->add_button_list();
        auto* header = button->mutable_header();
        header->set_frame_id(name);
        auto* stamp = header->mutable_timestamp();
        stamp->set_sec(static_cast<int64_t>(sec.count()));
        stamp->set_nanosec(static_cast<uint32_t>(nsec.count()));
        button->set_status(status);
    }

    return msg;
}

}  // namespace omnilink::teleop_viewer

