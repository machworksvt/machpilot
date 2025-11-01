// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/Statistics
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/statistics.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_Statistics_common : public MATLABROS2MsgInterface<interfaces::msg::Statistics> {
  public:
    virtual ~ros2_interfaces_msg_Statistics_common(){}
    virtual void copy_from_struct(interfaces::msg::Statistics* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::Statistics* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_Statistics_common::copy_from_struct(interfaces::msg::Statistics* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'header' is wrong type; expected a struct.");
    }
    try {
        //runs_ok
        const matlab::data::TypedArray<uint32_t> runs_ok_arr = arr["runs_ok"];
        msg->runs_ok = runs_ok_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'runs_ok' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'runs_ok' is wrong type; expected a uint32.");
    }
    try {
        //runs_aborted
        const matlab::data::TypedArray<uint32_t> runs_aborted_arr = arr["runs_aborted"];
        msg->runs_aborted = runs_aborted_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'runs_aborted' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'runs_aborted' is wrong type; expected a uint32.");
    }
    try {
        //total_runtime
        const matlab::data::TypedArray<uint64_t> total_runtime_arr = arr["total_runtime"];
        msg->total_runtime = total_runtime_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'total_runtime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'total_runtime' is wrong type; expected a uint64.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_Statistics_common::get_arr(MDFactory_T& factory, const interfaces::msg::Statistics* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","runs_ok","runs_aborted","total_runtime"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/Statistics");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // runs_ok
    auto currentElement_runs_ok = (msg + ctr)->runs_ok;
    outArray[ctr]["runs_ok"] = factory.createScalar(currentElement_runs_ok);
    // runs_aborted
    auto currentElement_runs_aborted = (msg + ctr)->runs_aborted;
    outArray[ctr]["runs_aborted"] = factory.createScalar(currentElement_runs_aborted);
    // total_runtime
    auto currentElement_total_runtime = (msg + ctr)->total_runtime;
    outArray[ctr]["total_runtime"] = factory.createScalar(currentElement_total_runtime);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_Statistics_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_Statistics_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_Statistics_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::Statistics,ros2_interfaces_msg_Statistics_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_Statistics_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::Statistics,ros2_interfaces_msg_Statistics_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_Statistics_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::Statistics>();
    ros2_interfaces_msg_Statistics_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_Statistics_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_Statistics_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::Statistics*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_Statistics_common, MATLABROS2MsgInterface<interfaces::msg::Statistics>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_Statistics_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER