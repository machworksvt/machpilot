// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/SystemInfo
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
#include "interfaces/msg/system_info.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_SystemInfo_common : public MATLABROS2MsgInterface<interfaces::msg::SystemInfo> {
  public:
    virtual ~ros2_interfaces_msg_SystemInfo_common(){}
    virtual void copy_from_struct(interfaces::msg::SystemInfo* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::SystemInfo* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_SystemInfo_common::copy_from_struct(interfaces::msg::SystemInfo* msg, const matlab::data::Struct& arr,
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
        //serial_number
        const matlab::data::TypedArray<uint16_t> serial_number_arr = arr["serial_number"];
        msg->serial_number = serial_number_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'serial_number' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'serial_number' is wrong type; expected a uint16.");
    }
    try {
        //fw_version
        const matlab::data::TypedArray<uint16_t> fw_version_arr = arr["fw_version"];
        msg->fw_version = fw_version_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'fw_version' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'fw_version' is wrong type; expected a uint16.");
    }
    try {
        //engine_type
        const matlab::data::TypedArray<uint8_t> engine_type_arr = arr["engine_type"];
        msg->engine_type = engine_type_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'engine_type' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'engine_type' is wrong type; expected a uint8.");
    }
    try {
        //engine_subtype
        const matlab::data::TypedArray<uint8_t> engine_subtype_arr = arr["engine_subtype"];
        msg->engine_subtype = engine_subtype_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'engine_subtype' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'engine_subtype' is wrong type; expected a uint8.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_SystemInfo_common::get_arr(MDFactory_T& factory, const interfaces::msg::SystemInfo* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","serial_number","fw_version","engine_type","engine_subtype"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/SystemInfo");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // serial_number
    auto currentElement_serial_number = (msg + ctr)->serial_number;
    outArray[ctr]["serial_number"] = factory.createScalar(currentElement_serial_number);
    // fw_version
    auto currentElement_fw_version = (msg + ctr)->fw_version;
    outArray[ctr]["fw_version"] = factory.createScalar(currentElement_fw_version);
    // engine_type
    auto currentElement_engine_type = (msg + ctr)->engine_type;
    outArray[ctr]["engine_type"] = factory.createScalar(currentElement_engine_type);
    // engine_subtype
    auto currentElement_engine_subtype = (msg + ctr)->engine_subtype;
    outArray[ctr]["engine_subtype"] = factory.createScalar(currentElement_engine_subtype);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_SystemInfo_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_SystemInfo_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_SystemInfo_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::SystemInfo,ros2_interfaces_msg_SystemInfo_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_SystemInfo_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::SystemInfo,ros2_interfaces_msg_SystemInfo_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_SystemInfo_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::SystemInfo>();
    ros2_interfaces_msg_SystemInfo_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_SystemInfo_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_SystemInfo_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::SystemInfo*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_SystemInfo_common, MATLABROS2MsgInterface<interfaces::msg::SystemInfo>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_SystemInfo_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER