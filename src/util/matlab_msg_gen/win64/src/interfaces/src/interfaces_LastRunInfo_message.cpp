// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/LastRunInfo
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
#include "interfaces/msg/last_run_info.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_LastRunInfo_common : public MATLABROS2MsgInterface<interfaces::msg::LastRunInfo> {
  public:
    virtual ~ros2_interfaces_msg_LastRunInfo_common(){}
    virtual void copy_from_struct(interfaces::msg::LastRunInfo* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::LastRunInfo* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_LastRunInfo_common::copy_from_struct(interfaces::msg::LastRunInfo* msg, const matlab::data::Struct& arr,
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
        //last_runtime
        const matlab::data::TypedArray<uint16_t> last_runtime_arr = arr["last_runtime"];
        msg->last_runtime = last_runtime_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'last_runtime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'last_runtime' is wrong type; expected a uint16.");
    }
    try {
        //last_off_rpm
        const matlab::data::TypedArray<uint16_t> last_off_rpm_arr = arr["last_off_rpm"];
        msg->last_off_rpm = last_off_rpm_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'last_off_rpm' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'last_off_rpm' is wrong type; expected a uint16.");
    }
    try {
        //last_off_egt
        const matlab::data::TypedArray<uint16_t> last_off_egt_arr = arr["last_off_egt"];
        msg->last_off_egt = last_off_egt_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'last_off_egt' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'last_off_egt' is wrong type; expected a uint16.");
    }
    try {
        //last_off_pump_power
        const matlab::data::TypedArray<float> last_off_pump_power_arr = arr["last_off_pump_power"];
        msg->last_off_pump_power = last_off_pump_power_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'last_off_pump_power' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'last_off_pump_power' is wrong type; expected a single.");
    }
    try {
        //last_off_state
        const matlab::data::TypedArray<uint8_t> last_off_state_arr = arr["last_off_state"];
        msg->last_off_state = last_off_state_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'last_off_state' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'last_off_state' is wrong type; expected a uint8.");
    }
    try {
        //last_off_state_str
        const matlab::data::CharArray last_off_state_str_arr = arr["last_off_state_str"];
        msg->last_off_state_str = last_off_state_str_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'last_off_state_str' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'last_off_state_str' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_LastRunInfo_common::get_arr(MDFactory_T& factory, const interfaces::msg::LastRunInfo* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","last_runtime","last_off_rpm","last_off_egt","last_off_pump_power","last_off_state","last_off_state_str"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/LastRunInfo");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // last_runtime
    auto currentElement_last_runtime = (msg + ctr)->last_runtime;
    outArray[ctr]["last_runtime"] = factory.createScalar(currentElement_last_runtime);
    // last_off_rpm
    auto currentElement_last_off_rpm = (msg + ctr)->last_off_rpm;
    outArray[ctr]["last_off_rpm"] = factory.createScalar(currentElement_last_off_rpm);
    // last_off_egt
    auto currentElement_last_off_egt = (msg + ctr)->last_off_egt;
    outArray[ctr]["last_off_egt"] = factory.createScalar(currentElement_last_off_egt);
    // last_off_pump_power
    auto currentElement_last_off_pump_power = (msg + ctr)->last_off_pump_power;
    outArray[ctr]["last_off_pump_power"] = factory.createScalar(currentElement_last_off_pump_power);
    // last_off_state
    auto currentElement_last_off_state = (msg + ctr)->last_off_state;
    outArray[ctr]["last_off_state"] = factory.createScalar(currentElement_last_off_state);
    // last_off_state_str
    auto currentElement_last_off_state_str = (msg + ctr)->last_off_state_str;
    outArray[ctr]["last_off_state_str"] = factory.createCharArray(currentElement_last_off_state_str);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_LastRunInfo_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_LastRunInfo_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_LastRunInfo_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::LastRunInfo,ros2_interfaces_msg_LastRunInfo_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_LastRunInfo_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::LastRunInfo,ros2_interfaces_msg_LastRunInfo_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_LastRunInfo_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::LastRunInfo>();
    ros2_interfaces_msg_LastRunInfo_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_LastRunInfo_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_LastRunInfo_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::LastRunInfo*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_LastRunInfo_common, MATLABROS2MsgInterface<interfaces::msg::LastRunInfo>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_LastRunInfo_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER