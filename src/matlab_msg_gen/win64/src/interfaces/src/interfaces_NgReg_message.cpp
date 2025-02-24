// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/NgReg
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
#include "interfaces/msg/ng_reg.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_NgReg_common : public MATLABROS2MsgInterface<interfaces::msg::NgReg> {
  public:
    virtual ~ros2_interfaces_msg_NgReg_common(){}
    virtual void copy_from_struct(interfaces::msg::NgReg* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::NgReg* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_NgReg_common::copy_from_struct(interfaces::msg::NgReg* msg, const matlab::data::Struct& arr,
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
        //integrator
        const matlab::data::TypedArray<float> integrator_arr = arr["integrator"];
        msg->integrator = integrator_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'integrator' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'integrator' is wrong type; expected a single.");
    }
    try {
        //windup
        const matlab::data::TypedArray<float> windup_arr = arr["windup"];
        msg->windup = windup_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'windup' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'windup' is wrong type; expected a single.");
    }
    try {
        //error
        const matlab::data::TypedArray<float> error_arr = arr["error"];
        msg->error = error_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'error' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'error' is wrong type; expected a single.");
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
  MDArray_T ros2_interfaces_msg_NgReg_common::get_arr(MDFactory_T& factory, const interfaces::msg::NgReg* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","integrator","windup","error","pump_power"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/NgReg");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // integrator
    auto currentElement_integrator = (msg + ctr)->integrator;
    outArray[ctr]["integrator"] = factory.createScalar(currentElement_integrator);
    // windup
    auto currentElement_windup = (msg + ctr)->windup;
    outArray[ctr]["windup"] = factory.createScalar(currentElement_windup);
    // error
    auto currentElement_error = (msg + ctr)->error;
    outArray[ctr]["error"] = factory.createScalar(currentElement_error);
    // pump_power
    auto currentElement_pump_power = (msg + ctr)->pump_power;
    outArray[ctr]["pump_power"] = factory.createScalar(currentElement_pump_power);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_NgReg_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_NgReg_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_NgReg_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::NgReg,ros2_interfaces_msg_NgReg_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_NgReg_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::NgReg,ros2_interfaces_msg_NgReg_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_NgReg_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::NgReg>();
    ros2_interfaces_msg_NgReg_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_NgReg_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_NgReg_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::NgReg*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_NgReg_common, MATLABROS2MsgInterface<interfaces::msg::NgReg>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_NgReg_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER