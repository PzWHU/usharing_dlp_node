// Ported from:
//   planning_bus/data_rscl/rscl_access/rscl_access.h
// Purpose:
//   Thin RSCL access wrapper. This is the only low-level RSCL node/publisher/
//   subscriber owner used by the migrated planning port.
//
// 中文说明：
//   CRsclAccess 是对 RSCL runtime/component 的薄封装，集中管理 subscriber 和
//   publisher 生命周期。除 app/transport/rscl 外，其他层不应直接包含 RSCL 头。
//   subscribe/advertise/publish 的模板接口用于屏蔽 RSCL 具体对象保存方式。
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <ad_rscl/ad_rscl.h>
#include <ad_rscl/component/component.h>
#include <ad_rscl/component/timer_component.h>
#include <ad_rscl/task/task.h>

constexpr int kRsclDefaultQueueSize = 3;

class CRsclAccess : public senseAD::rscl::component::TimerComponent {
 public:
  static CRsclAccess* Instance();

  CRsclAccess(const CRsclAccess&) = delete;
  CRsclAccess& operator=(const CRsclAccess&) = delete;

  ~CRsclAccess() override;

  bool OnProc() override;

  int init(int argc, char** argv, const std::string& name);
  int release();

  template <typename MessageT>
  static std::shared_ptr<SendMsg<MessageT>> NewMessage() {
    return std::make_shared<SendMsg<MessageT>>();
  }

  template <class M, class T>
  int subscribe(const std::string& topic, int size,
                void (T::*fp)(const std::shared_ptr<ReceivedMsg<M>>&),
                T* obj) {
    (void)size;
    // 同一 topic 重复订阅会导致回调链难以定位，因此这里直接拒绝。
    if (subscribers_.find(topic) != subscribers_.end()) {
      return -1;
    }
    auto sub =
        InnerNode()->CreateSubscriber<M>(topic,
                                         std::bind(fp, obj,
                                                   std::placeholders::_1));
    subscribers_[topic] = sub;
    return 0;
  }

  template <class M>
  int subscribe(
      const std::string& topic, int size,
      std::function<void(const std::shared_ptr<ReceivedMsg<M>>&)> cb_func) {
    (void)size;
    if (subscribers_.find(topic) != subscribers_.end()) {
      return -1;
    }
    auto sub = InnerNode()->CreateSubscriber<M>(topic, std::move(cb_func));
    subscribers_[topic] = sub;
    return 0;
  }

  template <class M>
  int advertise(const std::string& topic, int size = kRsclDefaultQueueSize) {
    (void)size;
    // 发布者也按 topic 去重，保持 topics_cfg 与实际 publisher 一一对应。
    if (publishers_.find(topic) != publishers_.end()) {
      return -1;
    }
    auto pub = InnerNode()->CreatePublisher<M>(topic);
    publishers_[topic] = pub;
    return 0;
  }

  template <class M>
  int publish(const std::string& topic,
              const std::shared_ptr<SendMsg<M>>& msg) {
    auto iter = publishers_.find(topic);
    if (iter == publishers_.end()) {
      return -1;
    }
    return iter->second->Publish(msg) ? 0 : -1;
  }

  template <typename M>
  int publish(const std::string& topic, const M& msg) const {
    auto iter = publishers_.find(topic);
    if (iter == publishers_.end()) {
      return -1;
    }
    return iter->second->Publish(msg) ? 0 : -1;
  }

 private:
  CRsclAccess();

  senseAD::rscl::Runtime* runtime_{nullptr};
  std::map<std::string, senseAD::rscl::comm::PublisherBase::Ptr> publishers_;
  std::map<std::string, senseAD::rscl::comm::SubscriberBase::Ptr>
      subscribers_;
};
