// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/FuelAmbient
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
#include "interfaces/msg/fuel_ambient.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_FuelAmbient_common : public MATLABROS2MsgInterface<interfaces::msg::FuelAmbient> {
  public:
    virtual ~ros2_interfaces_msg_FuelAmbient_common(){}
    virtual void copy_from_struct(interfaces::msg::FuelAmbient* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::FuelAmbient* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_FuelAmbient_common::copy_from_struct(interfaces::msg::FuelAmbient* msg, const matlab::data::Struct& arr,
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
        //fuel_flow
        const matlab::data::TypedArray<uint32_t> fuel_flow_arr = arr["fuel_flow"];
        msg->fuel_flow = fuel_flow_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'fuel_flow' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'fuel_flow' is wrong type; expected a uint32.");
    }
    try {
        //fuel_consumed
        const matlab::data::TypedArray<uint32_t> fuel_consumed_arr = arr["fuel_consumed"];
        msg->fuel_consumed = fuel_consumed_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'fuel_consumed' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'fuel_consumed' is wrong type; expected a uint32.");
    }
    try {
        //engine_box_pressure
        const matlab::data::TypedArray<float> engine_box_pressure_arr = arr["engine_box_pressure"];
        msg->engine_box_pressure = engine_box_pressure_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'engine_box_pressure' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'engine_box_pressure' is wrong type; expected a single.");
    }
    try {
        //ambient_temperature
        const matlab::data::TypedArray<int16_t> ambient_temperature_arr = arr["ambient_temperature"];
        msg->ambient_temperature = ambient_temperature_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ambient_temperature' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ambient_temperature' is wrong type; expected a int16.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_FuelAmbient_common::get_arr(MDFactory_T& factory, const interfaces::msg::FuelAmbient* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","fuel_flow","fuel_consumed","engine_box_pressure","ambient_temperature"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/FuelAmbient");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // fuel_flow
    auto currentElement_fuel_flow = (msg + ctr)->fuel_flow;
    outArray[ctr]["fuel_flow"] = factory.createScalar(currentElement_fuel_flow);
    // fuel_consumed
    auto currentElement_fuel_consumed = (msg + ctr)->fuel_consumed;
    outArray[ctr]["fuel_consumed"] = factory.createScalar(currentElement_fuel_consumed);
    // engine_box_pressure
    auto currentElement_engine_box_pressure = (msg + ctr)->engine_box_pressure;
    outArray[ctr]["engine_box_pressure"] = factory.createScalar(currentElement_engine_box_pressure);
    // ambient_temperature
    auto currentElement_ambient_temperature = (msg + ctr)->ambient_temperature;
    outArray[ctr]["ambient_temperature"] = factory.createScalar(currentElement_ambient_temperature);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_FuelAmbient_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_FuelAmbient_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_FuelAmbient_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::FuelAmbient,ros2_interfaces_msg_FuelAmbient_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_FuelAmbient_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::FuelAmbient,ros2_interfaces_msg_FuelAmbient_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_FuelAmbient_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::FuelAmbient>();
    ros2_interfaces_msg_FuelAmbient_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_FuelAmbient_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_FuelAmbient_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::FuelAmbient*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_FuelAmbient_common, MATLABROS2MsgInterface<interfaces::msg::FuelAmbient>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_FuelAmbient_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER