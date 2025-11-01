// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for interfaces/GlowPlugs
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
#include "interfaces/msg/glow_plugs.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class INTERFACES_EXPORT ros2_interfaces_msg_GlowPlugs_common : public MATLABROS2MsgInterface<interfaces::msg::GlowPlugs> {
  public:
    virtual ~ros2_interfaces_msg_GlowPlugs_common(){}
    virtual void copy_from_struct(interfaces::msg::GlowPlugs* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const interfaces::msg::GlowPlugs* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_interfaces_msg_GlowPlugs_common::copy_from_struct(interfaces::msg::GlowPlugs* msg, const matlab::data::Struct& arr,
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
        //glow_plug_v
        const matlab::data::TypedArray<float> glow_plug_v_arr = arr["glow_plug_v"];
        size_t nelem = 2;
        	std::copy(glow_plug_v_arr.begin(), glow_plug_v_arr.begin()+nelem, msg->glow_plug_v.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'glow_plug_v' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'glow_plug_v' is wrong type; expected a single.");
    }
    try {
        //glow_plug_i
        const matlab::data::TypedArray<float> glow_plug_i_arr = arr["glow_plug_i"];
        size_t nelem = 2;
        	std::copy(glow_plug_i_arr.begin(), glow_plug_i_arr.begin()+nelem, msg->glow_plug_i.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'glow_plug_i' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'glow_plug_i' is wrong type; expected a single.");
    }
    try {
        //sekevence
        const matlab::data::TypedArray<uint16_t> sekevence_arr = arr["sekevence"];
        msg->sekevence = sekevence_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'sekevence' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'sekevence' is wrong type; expected a uint16.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_interfaces_msg_GlowPlugs_common::get_arr(MDFactory_T& factory, const interfaces::msg::GlowPlugs* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","glow_plug_v","glow_plug_i","sekevence"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("interfaces/GlowPlugs");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // glow_plug_v
    auto currentElement_glow_plug_v = (msg + ctr)->glow_plug_v;
    outArray[ctr]["glow_plug_v"] = factory.createArray<interfaces::msg::GlowPlugs::_glow_plug_v_type::const_iterator, float>({currentElement_glow_plug_v.size(), 1}, currentElement_glow_plug_v.begin(), currentElement_glow_plug_v.end());
    // glow_plug_i
    auto currentElement_glow_plug_i = (msg + ctr)->glow_plug_i;
    outArray[ctr]["glow_plug_i"] = factory.createArray<interfaces::msg::GlowPlugs::_glow_plug_i_type::const_iterator, float>({currentElement_glow_plug_i.size(), 1}, currentElement_glow_plug_i.begin(), currentElement_glow_plug_i.end());
    // sekevence
    auto currentElement_sekevence = (msg + ctr)->sekevence;
    outArray[ctr]["sekevence"] = factory.createScalar(currentElement_sekevence);
    }
    return std::move(outArray);
  } 
class INTERFACES_EXPORT ros2_interfaces_GlowPlugs_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_interfaces_GlowPlugs_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_interfaces_GlowPlugs_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<interfaces::msg::GlowPlugs,ros2_interfaces_msg_GlowPlugs_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_interfaces_GlowPlugs_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<interfaces::msg::GlowPlugs,ros2_interfaces_msg_GlowPlugs_common>>();
  }
  std::shared_ptr<void> ros2_interfaces_GlowPlugs_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<interfaces::msg::GlowPlugs>();
    ros2_interfaces_msg_GlowPlugs_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_interfaces_GlowPlugs_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_interfaces_msg_GlowPlugs_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (interfaces::msg::GlowPlugs*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_msg_GlowPlugs_common, MATLABROS2MsgInterface<interfaces::msg::GlowPlugs>)
CLASS_LOADER_REGISTER_CLASS(ros2_interfaces_GlowPlugs_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER