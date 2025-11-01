// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/SystemInfo2
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
#include "interfaces/msg/system_info2.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_SystemInfo2_common : public MATLABROS2MsgInterface<interfaces::msg::SystemInfo2> {
  public:
    virtual ~ros2_interfaces_msg_SystemInfo2_common(){}
    virtual void copy_from_struct(interfaces::msg::SystemInfo2* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::SystemInfo2* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_SystemInfo2_common::copy_from_struct(interfaces::msg::SystemInfo2* msg, const matlab::data::Struct& arr,
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
        //ecu_hw_serial_sumber
        const matlab::data::TypedArray<uint64_t> ecu_hw_serial_sumber_arr = arr["ecu_hw_serial_sumber"];
        msg->ecu_hw_serial_sumber = ecu_hw_serial_sumber_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ecu_hw_serial_sumber' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ecu_hw_serial_sumber' is wrong type; expected a uint64.");
    }
    try {
        //eiu_sw_version
        const matlab::data::TypedArray<uint32_t> eiu_sw_version_arr = arr["eiu_sw_version"];
        msg->eiu_sw_version = eiu_sw_version_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'eiu_sw_version' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'eiu_sw_version' is wrong type; expected a uint32.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_SystemInfo2_common::get_arr(MDFactory_T& factory, const interfaces::msg::SystemInfo2* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","ecu_hw_serial_sumber","eiu_sw_version"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/SystemInfo2");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // ecu_hw_serial_sumber
    auto currentElement_ecu_hw_serial_sumber = (msg + ctr)->ecu_hw_serial_sumber;
    outArray[ctr]["ecu_hw_serial_sumber"] = factory.createScalar(currentElement_ecu_hw_serial_sumber);
    // eiu_sw_version
    auto currentElement_eiu_sw_version = (msg + ctr)->eiu_sw_version;
    outArray[ctr]["eiu_sw_version"] = factory.createScalar(currentElement_eiu_sw_version);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_SystemInfo2_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_SystemInfo2_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_SystemInfo2_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::SystemInfo2,ros2_interfaces_msg_SystemInfo2_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_SystemInfo2_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::SystemInfo2,ros2_interfaces_msg_SystemInfo2_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_SystemInfo2_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::SystemInfo2>();
    ros2_interfaces_msg_SystemInfo2_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_SystemInfo2_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_SystemInfo2_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::SystemInfo2*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_SystemInfo2_common, MATLABROS2MsgInterface<interfaces::msg::SystemInfo2>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_SystemInfo2_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER