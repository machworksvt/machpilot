// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/EngineData
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
#include "interfaces/msg/engine_data.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_EngineData_common : public MATLABROS2MsgInterface<interfaces::msg::EngineData> {
  public:
    virtual ~ros2_interfaces_msg_EngineData_common(){}
    virtual void copy_from_struct(interfaces::msg::EngineData* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::EngineData* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_EngineData_common::copy_from_struct(interfaces::msg::EngineData* msg, const matlab::data::Struct& arr,
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
        //set_rpm
        const matlab::data::TypedArray<uint32_t> set_rpm_arr = arr["set_rpm"];
        msg->set_rpm = set_rpm_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'set_rpm' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'set_rpm' is wrong type; expected a uint32.");
    }
    try {
        //real_rpm
        const matlab::data::TypedArray<uint32_t> real_rpm_arr = arr["real_rpm"];
        msg->real_rpm = real_rpm_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'real_rpm' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'real_rpm' is wrong type; expected a uint32.");
    }
    try {
        //egt
        const matlab::data::TypedArray<float> egt_arr = arr["egt"];
        msg->egt = egt_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'egt' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'egt' is wrong type; expected a single.");
    }
    try {
        //state
        const matlab::data::TypedArray<uint8_t> state_arr = arr["state"];
        msg->state = state_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'state' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'state' is wrong type; expected a uint8.");
    }
    try {
        //pump_power
        const matlab::data::TypedArray<float> pump_power_arr = arr["pump_power"];
        msg->pump_power = pump_power_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'pump_power' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'pump_power' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_EngineData_common::get_arr(MDFactory_T& factory, const interfaces::msg::EngineData* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","set_rpm","real_rpm","egt","state","pump_power"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/EngineData");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // set_rpm
    auto currentElement_set_rpm = (msg + ctr)->set_rpm;
    outArray[ctr]["set_rpm"] = factory.createScalar(currentElement_set_rpm);
    // real_rpm
    auto currentElement_real_rpm = (msg + ctr)->real_rpm;
    outArray[ctr]["real_rpm"] = factory.createScalar(currentElement_real_rpm);
    // egt
    auto currentElement_egt = (msg + ctr)->egt;
    outArray[ctr]["egt"] = factory.createScalar(currentElement_egt);
    // state
    auto currentElement_state = (msg + ctr)->state;
    outArray[ctr]["state"] = factory.createScalar(currentElement_state);
    // pump_power
    auto currentElement_pump_power = (msg + ctr)->pump_power;
    outArray[ctr]["pump_power"] = factory.createScalar(currentElement_pump_power);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_EngineData_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_EngineData_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_EngineData_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::EngineData,ros2_interfaces_msg_EngineData_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_EngineData_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::EngineData,ros2_interfaces_msg_EngineData_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_EngineData_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::EngineData>();
    ros2_interfaces_msg_EngineData_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_EngineData_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_EngineData_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::EngineData*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_EngineData_common, MATLABROS2MsgInterface<interfaces::msg::EngineData>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_EngineData_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER