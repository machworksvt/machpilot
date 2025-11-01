// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/VoltageCurrent
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
#include "interfaces/msg/voltage_current.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_VoltageCurrent_common : public MATLABROS2MsgInterface<interfaces::msg::VoltageCurrent> {
  public:
    virtual ~ros2_interfaces_msg_VoltageCurrent_common(){}
    virtual void copy_from_struct(interfaces::msg::VoltageCurrent* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::VoltageCurrent* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_VoltageCurrent_common::copy_from_struct(interfaces::msg::VoltageCurrent* msg, const matlab::data::Struct& arr,
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
        //battery_voltage
        const matlab::data::TypedArray<float> battery_voltage_arr = arr["battery_voltage"];
        msg->battery_voltage = battery_voltage_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'battery_voltage' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'battery_voltage' is wrong type; expected a single.");
    }
    try {
        //battery_current
        const matlab::data::TypedArray<float> battery_current_arr = arr["battery_current"];
        msg->battery_current = battery_current_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'battery_current' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'battery_current' is wrong type; expected a single.");
    }
    try {
        //flags
        const matlab::data::TypedArray<uint8_t> flags_arr = arr["flags"];
        msg->flags = flags_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'flags' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'flags' is wrong type; expected a uint8.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_VoltageCurrent_common::get_arr(MDFactory_T& factory, const interfaces::msg::VoltageCurrent* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","battery_voltage","battery_current","flags"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/VoltageCurrent");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // battery_voltage
    auto currentElement_battery_voltage = (msg + ctr)->battery_voltage;
    outArray[ctr]["battery_voltage"] = factory.createScalar(currentElement_battery_voltage);
    // battery_current
    auto currentElement_battery_current = (msg + ctr)->battery_current;
    outArray[ctr]["battery_current"] = factory.createScalar(currentElement_battery_current);
    // flags
    auto currentElement_flags = (msg + ctr)->flags;
    outArray[ctr]["flags"] = factory.createScalar(currentElement_flags);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_VoltageCurrent_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_VoltageCurrent_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_VoltageCurrent_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::VoltageCurrent,ros2_interfaces_msg_VoltageCurrent_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_VoltageCurrent_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::VoltageCurrent,ros2_interfaces_msg_VoltageCurrent_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_VoltageCurrent_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::VoltageCurrent>();
    ros2_interfaces_msg_VoltageCurrent_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_VoltageCurrent_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_VoltageCurrent_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::VoltageCurrent*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_VoltageCurrent_common, MATLABROS2MsgInterface<interfaces::msg::VoltageCurrent>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_VoltageCurrent_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER