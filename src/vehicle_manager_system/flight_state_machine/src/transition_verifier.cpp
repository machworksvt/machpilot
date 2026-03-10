#include "transition_verifier.hpp"

TransitionVerifier::TransitionVerifier(rclcpp::Node &parrent){
    for (size_t i=0;i<SERVICES_COUNT;i++){
        std::string name(SERVICE_NAMES[i].data(),SERVICE_NAMES[i].size());
        this->services[i] = parrent.create_client<flight_state_machine_interface::srv::TransitionRequest>(std::move(name));
    }
}

bool TransitionVerifier::request_transition(std::chrono::milliseconds timeout,StateEnum new_state){

    if (SERVICES_COUNT == 0) return true;

    // Setup primitives
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<size_t> responses_received{0};
    std::atomic<bool> failed{false};
    std::array<int64_t,SERVICES_COUNT> request_ids;

    
    // This prevents a race condition where a callback notifies before we reach wait_for.
    std::unique_lock<std::mutex> lock(mtx);

    for (size_t i = 0; i < SERVICES_COUNT; ++i) {
        auto request = std::make_shared<flight_state_machine_interface::srv::TransitionRequest::Request>();
    
        request->new_state_id=static_cast<int>(new_state);

        //create a call back that will wake up this thread if the transition is denied responces or everything has sent an approve responce
        auto result = this->services[i]->async_send_request(request,
            [&, i](rclcpp::Client<flight_state_machine_interface::srv::TransitionRequest>::SharedFuture future) {
            
            if (failed.load()) return;

            auto response = future.get();

            if (!response->transition_allowed) {
                failed.store(true);
                
                std::lock_guard<std::mutex> notify_lock(mtx);
                cv.notify_one();
            }else if (++responses_received == SERVICES_COUNT) {
                std::lock_guard<std::mutex> notify_lock(mtx);
                cv.notify_one();
            }
        });

      request_ids[i] = result.request_id;
    }

    bool finished = cv.wait_for(lock, timeout, [&]() {
      return failed.load() || responses_received.load() == SERVICES_COUNT;
    });

    bool success = finished && !failed.load();

    if (!success) {
      // Cancel all pending requests
      for (size_t i = 0; i < SERVICES_COUNT; ++i) {
        this->services[i]->remove_pending_request(request_ids[i]);
      }
    }

    return success;
}