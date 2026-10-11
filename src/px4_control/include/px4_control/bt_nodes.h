// 行为树节点库：把 px4_control 的飞行动作/条件封装成 BehaviorTree.CPP v3 节点。
//
// 用法（见 src/offboard_bt.cpp）：
//     BT::BehaviorTreeFactory factory;
//     px4_bt::registerNodes(factory);
//     auto blackboard = BT::Blackboard::create();
//     blackboard->set("ctx", &ctx);                  // 必须！节点靠它拿 ROS 句柄
//     BT::Tree tree = factory.createTreeFromFile(xml, blackboard);
#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>

#include <px4_control/drone_state.h>

namespace px4_bt {

/// 注册全部可用节点（节点名 = XML 里的标签名）
void registerNodes(BT::BehaviorTreeFactory& factory);

}  // namespace px4_bt
